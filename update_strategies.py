from abc import ABC, abstractmethod
import numpy as np
from models import get_initial_state, conv_Sphe2Cart, h_nonlinear, compute_jacobian



class UpdateStrategy(ABC):
    @abstractmethod
    def apply(self, tracker, z_meas, mask, z_meas_part, z_pred_part, H_part, S_part, R_part, config):
        pass

class StandardUpdate(UpdateStrategy):
    """Using the traditional update mechanism of the standard EKF."""

    def apply(self, tracker, z_meas, mask, z_meas_part, z_pred_part, H_part, S_part, R_part, config):

        tracker.ekf.update(z_meas_part, z_pred_part, H_part, S_part)

class HeuristicResetUpdate(UpdateStrategy):
    """Restarts the track (stops using the information inertia of the system) if the comparison 
    of the predicted and initialisation covariance matrices crosses a threshold."""

    def apply(self, tracker, z_meas, mask, z_meas_part, z_pred_part, H_part, S_part, R_part, config):

        # Checking measurement is complete
        if np.all(mask):

            # Fetching initial state vector and covariance matrix in case of restart
            X_init, P_init = get_initial_state(z_meas, config) 
            
            # Calculation of the ratio of traces
            trace_pred = np.trace(tracker.ekf.p)
            trace_init = np.trace(P_init)
            ratio = trace_pred / trace_init
            
            # Threshold evaluation
            if ratio > config.reset_threshold:
                # RESET : The prediction is overwritten by the new initialization.
                tracker.ekf.x = X_init
                tracker.ekf.p = P_init
                return  

        #elif np.nan(z_meas[2]) and np.nan(z_meas[3]):

                
        # Fallback : If partial measurement or ratio under threshold 
        tracker.ekf.update(z_meas_part, z_pred_part, H_part, S_part)

class HybridResetUpdate(UpdateStrategy):
    """Partially restarts the track (stops using the position information inertia of the system) if the comparison of the predicted and 
    initialisation position covariance matrices crosses a threshold. The update method keeps the velocity information inertia of the system."""

    def apply(self, tracker, z_meas, mask, z_meas_part, z_pred_part, H_part, S_part, R_part, config):

        # Checking measurement is complete
        if np.all(mask):

            # Fetching initial state vector and covariance matrix in case of restart
            X_init, P_init = get_initial_state(z_meas, config) 
            
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
                return 

        # Fallback : If partial measurement or ratio under threshold 
        tracker.ekf.update(z_meas_part, z_pred_part, H_part, S_part)

class CMKF(UpdateStrategy):
    """Uses a Centered Measurement Kalman Filter. The discrepency between prediction and measurement is calculated in the cartesian coordonates system.
    This filter induces two (partial) updates: position then velocity."""

    def apply(self, tracker, z_meas, mask, z_meas_part, z_pred_part, H_part, S_part, R_part, config):

        # ---| 1. Position Update |--- #

        if np.all(mask):

            Z_cart_pos = np.array(conv_Sphe2Cart(z_meas[[0, 2, 3]].flatten())).reshape(3, 1)     # Removing speed component
            H_lin = np.eye(3, 6)                                                                 # Linear matrix only keeping cartesian coordonates of the position
            Y_cart = Z_cart_pos - H_lin @ tracker.ekf.x                                          # Calculating the discrepency between prediction position and measurement position

            R_pol_pos = R_part[np.ix_([0, 2, 3], [0, 2, 3])]
            z_meas_pos = z_meas[[0, 2, 3]]
            J_s2c_pos = compute_jacobian(conv_Sphe2Cart, z_meas_pos)
            R_cart = J_s2c_pos @ R_pol_pos @ J_s2c_pos.T
            S_cart = H_lin @ tracker.ekf.p @ H_lin.T + R_cart
            K_cart = tracker.ekf.p @ H_lin.T @ np.linalg.solve(S_cart, np.eye(S_cart.shape[0]))

            tracker.ekf.x = tracker.ekf.x + K_cart @ Y_cart                                 # Updating position
            tracker.ekf.p = (np.eye(K_cart.shape[0]) - K_cart @ H_lin) @ tracker.ekf.p      # Updating covariance matrix: including position-velocity covariances

        # ---| 2. Velocity Update |--- #

            z_pred_full_new = h_nonlinear(tracker.ekf.x, tracker.radar_pos)
            H_full_new = compute_jacobian(h_nonlinear, tracker.ekf.x, tracker.radar_pos)

            # Extracting only the radial velocity components (index 1)
            v_rad_meas = z_meas[1].reshape(1, 1)
            v_rad_pred = z_pred_full_new[1].reshape(1, 1)
            H_v_rad = H_full_new[1, :].reshape(1, 6)
 
            # Recomputing S for the radial velocity with the NEW covariance matrix
            R_v_rad = R_part[1, 1].reshape(1, 1)
            S_v_rad = H_v_rad @ tracker.ekf.p @ H_v_rad.T + R_v_rad
        
            # Second Update (Using standard EKF math for the 1D velocity)
            tracker.ekf.update(v_rad_meas, v_rad_pred, H_v_rad, S_v_rad)

        else:
            # Fallback : If partial measurement 
            tracker.ekf.update(z_meas_part, z_pred_part, H_part, S_part)