from abc import ABC, abstractmethod
import numpy as np
from models import get_initial_state, conv_Sphe2Cart, h_nonlinear, compute_jacobian



class UpdateStrategy(ABC):

    @abstractmethod
    def apply(self, tracker, z_meas, z_pred, mask, R, config, chi2_thresholds):
        pass



class StandardGatingStrategy(UpdateStrategy):
    """Handles dynamic truncation and standard Mahalanobis gating. Delegates the actual update logic to subclasses if gating succeeds."""

    def apply(self, tracker, z_meas, z_pred, mask, R, config, chi2_thresholds):

        # ---| Dynamic truncation |--- #
        
        z_meas_part = z_meas[mask]          # Only keeping valid measures
        z_pred_part = z_pred[mask]          # Only keeping the associated predicted measures
        R_part = R[np.ix_(mask, mask)]      # Only keeping the lines and columns associated to the valid measures

        # Calculating jacobian for predicated state
        H_full = compute_jacobian(h_nonlinear, tracker.ekf.x, config.radar_pos)
        # Truncating jacobian accordingly to available measurements
        H_part = H_full[mask, :]
        # Calculating truncated covariance innovation 
        S_part = tracker.ekf.covariance_innovation_S(H_part, R_part)

        # ---| Gating |--- #

        # Calculating truncated mahalanobis parameters
        D2 = calculate_mahalanobis(z_meas_part, z_pred_part, S_part)
        dof = np.sum(mask)
        gating_threshold = chi2_thresholds[dof]

        if D2 <= gating_threshold:
            # Gating passed: calling the specific update of the child class
            return self.perform_update(tracker, mask, z_meas_part, z_pred_part, H_part, S_part, config)
        else:
            # Measurement rejected
            return False

    @abstractmethod
    def perform_update(self, tracker, mask, z_meas_part, z_pred_part, H_part, S_part, config):
        """Must be implemented by child classes to define what happens after gating succeeds."""
        pass



class StandardUpdate(StandardGatingStrategy):
    """Using the traditional update mechanism of the standard EKF."""
    
    def perform_update(self, tracker, mask, z_meas_part, z_pred_part, H_part, S_part, config):
        # Standard EKF Update
        tracker.ekf.update(z_meas_part, z_pred_part, H_part, S_part)
        return True


        
class HeuristicResetUpdate(StandardGatingStrategy):
    """Restarts the track (stops using the information inertia of the system) if the comparison 
    of the predicted and initialisation covariance matrices crosses a threshold."""

    def perform_update(self, tracker, mask, z_meas_part, z_pred_part, H_part, S_part, config):

        # Evaluating restart if measurement is complete
        if np.all(mask):

            # Fetching initial state vector and covariance matrix in case of restart
            X_init, P_init = get_initial_state(z_meas_part, config) 
            
            # Calculation of the ratio of traces
            trace_pred = np.trace(tracker.ekf.p)
            trace_init = np.trace(P_init)
            ratio = trace_pred / trace_init
            
            # Threshold evaluation
            if ratio > config.reset_threshold:
                # RESET : The prediction is overwritten by the new initialization.
                tracker.ekf.x = X_init
                tracker.ekf.p = P_init
                return True
                
        # Fallback : If partial measurement or ratio under threshold 
        tracker.ekf.update(z_meas_part, z_pred_part, H_part, S_part)
        return True

        

class HybridResetUpdate(UpdateStrategy):
    """Partially restarts the track (stops using the position information inertia of the system) if the comparison of the predicted and 
    initialisation position covariance matrices crosses a threshold. The update method keeps the velocity information inertia of the system."""

    def perform_update(self, tracker, mask, z_meas_part, z_pred_part, H_part, S_part, config):

        # Evaluating restart if measurement is complete
        if np.all(mask):

            # Fetching initial state vector and covariance matrix in case of restart
            X_init, P_init = get_initial_state(z_meas_part, config) 
            
            # Calculation of the ratio of position traces
            trace_pred_pos = np.trace(tracker.ekf.p[0:3, 0:3])
            trace_init_pos = np.trace(P_init[0:3, 0:3])
            ratio = trace_pred_pos / trace_init_pos
            
            # Threshold evaluation
            if ratio > config.reset_threshold:
                # RESET: Hybrid state fusion
                
                # Stacking position from Init and velocity from Prediction
                tracker.ekf.x = np.vstack((X_init[0:3], tracker.ekf.x[3:6]))
                
                # Creating a block diagonal matrix (Zeroing cross-correlations)
                P_reset = np.zeros((6, 6))
                P_reset[0:3, 0:3] = P_init[0:3, 0:3]
                P_reset[3:6, 3:6] = tracker.ekf.p[3:6, 3:6]
                tracker.ekf.p = P_reset
                return True

        # Fallback : If partial measurement or ratio under threshold 
        tracker.ekf.update(z_meas_part, z_pred_part, H_part, S_part)
        return True

        


class CMKF(UpdateStrategy):
    """Uses a Centered Measurement Kalman Filter. The discrepency between prediction and measurement is calculated in the cartesian coordonates system.
    This filter induces two (partial) updates: position then velocity."""

    def __init__(self):
        # COMPOSITION : CMKF has a fallback startegy to treat partial mesaurements
        self.fallback_strategy = StandardUpdate()

    def apply(self, tracker, z_meas, z_pred, mask, R, config, chi2_thresholds):

        # Transactional Backup (Rollback) 
        x_backup = np.copy(tracker.ekf.x)
        p_backup = np.copy(tracker.ekf.p)

        # ---| 1. Position Update |--- #

        if np.all(mask):

            Z_cart_pos = np.array(conv_Sphe2Cart(z_meas[[0, 2, 3]].flatten())).reshape(3, 1)      # Removing speed component
            H_lin = np.eye(3, 6)  
            X_pred_pos = H_lin @ tracker.ekf.x                                                    # Linear matrix only keeping cartesian coordonates of the position
            Y_cart = Z_cart_pos - X_pred_pos                                                      # Calculating the discrepency between prediction position and measurement position

            R_pol_pos = R[np.ix_([0, 2, 3], [0, 2, 3])]
            z_meas_pos = z_meas[[0, 2, 3]]
            J_s2c_pos = compute_jacobian(conv_Sphe2Cart, z_meas_pos)
            R_cart = J_s2c_pos @ R_pol_pos @ J_s2c_pos.T
            S_cart = H_lin @ tracker.ekf.p @ H_lin.T + R_cart
            K_cart = tracker.ekf.p @ H_lin.T @ np.linalg.solve(S_cart, np.eye(S_cart.shape[0]))

            # --- Cartesian Position Gating --- #

            D2 = calculate_mahalanobis(Z_cart_pos, X_pred_pos, S_cart)
            gating_threshold = chi2_thresholds[3]
    
            if D2 <= gating_threshold:

                tracker.ekf.x = tracker.ekf.x + K_cart @ Y_cart                                 # Updating position
                tracker.ekf.p = (np.eye(K_cart.shape[0]) - K_cart @ H_lin) @ tracker.ekf.p      # Updating covariance matrix: including position-velocity covariances

            else:
                # Measurement rejected: no update at all
                return False

        # ---| 2. Velocity Update |--- #

            z_pred_full_new = h_nonlinear(tracker.ekf.x, tracker.radar_pos)
            H_full_new = compute_jacobian(h_nonlinear, tracker.ekf.x, tracker.radar_pos)

            # Extracting only the radial velocity components (index 1)
            v_rad_meas = z_meas[1].reshape(1, 1)
            v_rad_pred = z_pred_full_new[1].reshape(1, 1)
            H_v_rad = H_full_new[1, :].reshape(1, 6)
 
            # Recomputing S for the radial velocity with the NEW covariance matrix
            R_v_rad = R[1, 1].reshape(1, 1)
            S_v_rad = H_v_rad @ tracker.ekf.p @ H_v_rad.T + R_v_rad

            # --- Radial Velocity Gating --- #
            
            D2_vel = calculate_mahalanobis(v_rad_meas, v_rad_pred, S_v_rad)
            gating_threshold = chi2_thresholds[1]

            if D2_vel <= gating_threshold:
        
                # Second Update (Using standard EKF math for the 1D velocity)
                tracker.ekf.update(v_rad_meas, v_rad_pred, H_v_rad, S_v_rad)
                return True

            else:
                # Measurement rejected: Rollback
                tracker.ekf.x = x_backup
                tracker.ekf.p = p_backup
                return False
                
        else:
            # Fallback : If partial measurement 
            return self.fallback_strategy.apply(tracker, z_meas, z_pred, mask, R, config, chi2_thresholds)



class UKF(UpdateStrategy):
    """Uses an Unscented Kalman Filter. The filter by passes the jacobian, fondamentally it is the same as a Monte Carlo simultion,
      but by chosing wisely a limited number of points (sigma points)."""

    def apply(self, tracker, z_meas, z_pred, mask, R, config, chi2_thresholds):

        # ---| 1. Unscented Transform Hyper Parameters |--- #

        n = 6                               # Input dimension
        alpha = 1e-3                        # Dispersion of the sigma points around the mean
        k = 0                               # Secondary scaling factor
        beta = 2                            # Distribution information (2 for gaussian distribution)
        Lambda = (alpha ** 2) * (n + k) - n

        # ---| 2. Initialization |--- #

        sigma_points_cart = np.zeros((n, 2*n+1))
        sigma_points_pol = np.zeros((4, 2*n+1))
        weights_mean = np.zeros(2*n+1)
        weights_cov = np.zeros(2*n+1)

        # ---| 3. Creating Sigma Points |--- #

        L = np.linalg.cholesky((n+Lambda)*tracker.ekf.p)    # P = symmetric positive-definite matrix -> L = lower triangular

        # Central point
        sigma_points_cart[:, 0] = tracker.ekf.x.flatten()
        weights_mean[0] = Lambda/(n+Lambda)
        weights_cov[0] = weights_mean[0] + (1.0 - (alpha ** 2) + beta)

        # Peripheral points
        for i in range(n):
            sigma_points_cart[:, i+1] = tracker.ekf.x.flatten() + L[:, i]
            weights_mean[i+1] = 1/(2*(n+Lambda))
            weights_cov[i+1] = 1/(2*(n+Lambda))
            sigma_points_cart[:, i+1+n] = tracker.ekf.x.flatten() - L[:, i]
            weights_mean[i+1+n] = 1/(2*(n+Lambda))
            weights_cov[i+1+n] = 1/(2*(n+Lambda))

        # ---| 4. Non-Linear Projection : State -> Measurement |--- #
        
        for i in range(2*n+1):
            sigma_points_pol[:, i] = h_nonlinear(sigma_points_cart[:, i], tracker.radar_pos).flatten()

        z_pred_ukf = np.zeros(4)
        for i in range(2*n+1):
            z_pred_ukf += weights_mean[i] * sigma_points_pol[:, i]       # Weighted average

        # Innovation
        y = z_meas[mask] - z_pred_ukf[mask].reshape(-1, 1)
        
        # ---| 5. Non-Linear Projection : Uncertainty Matrices |--- #

        # Initialization
        S = np.copy(R[np.ix_(mask, mask)] )          
        P_xz = np.zeros((n, np.sum(mask)))

        for i in range(2*n+1):
            # Polar discrepency
            z_diff = sigma_points_pol[mask, i].reshape(-1, 1) - z_pred_ukf[mask].reshape(-1, 1)
            
            # Cartesian difference
            x_diff = sigma_points_cart[:, i].reshape(6, 1) - tracker.ekf.x
            
            # Innovation Covariance Matrix (replaces H * P * H^T + R)
            S += weights_cov[i] * (z_diff @ z_diff.T)
            
            # Crossed Covariance P_xz (replaces P * H^T)
            P_xz += weights_cov[i] * (x_diff @ z_diff.T)

        # Kalman Gain
        K = P_xz @ np.linalg.inv(S)

        # ---| 6. Gating |--- #

        D2 = calculate_mahalanobis(z_meas[mask], z_pred_ukf[mask].reshape(-1, 1), S)
        gating_threshold = chi2_thresholds[np.sum(mask)]
        
        if D2 <= gating_threshold:

            # State & Covariance Update
            tracker.ekf.x = tracker.ekf.x + K @ y
            tracker.ekf.p = tracker.ekf.p - K @ S @ K.T
            # Gating passed
            return True

        else:
            # Measurement rejected
            return False


def calculate_mahalanobis(meas: np.ndarray, pred: np.ndarray, S: np.ndarray) -> float:
        """
        Computes the squared Mahalanobis distance between the actual measurement 
        and the predicted measurement.
        
        Args:
            z_meas (np.ndarray): The radar measurement vector.
            z_pred (np.ndarray): The converted prediction into measurement.
            S (np.ndarray): Covariance Innovation (plausible innovation)
        Returns:
            D2 (float) : Mahalanobis distance
        """

        innovation = meas - pred
        D2 = innovation.T @ np.linalg.solve(S, innovation)      # Same as : inov.T x inv(S) x inov

        return D2.item()