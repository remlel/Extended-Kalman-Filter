import numpy as np
from ekf import ExtendedKalmanFilter
from models import get_F_CV, get_Q_CV, h_nonlinear, compute_jacobian

class RadarTracker:
    """
    Manages the lifecycle of a target track, handling missed detections (NaNs),
    outlier rejection (Gating), and orchestrating the EKF mathematical engine.
    """

    def __init__(self, initial_state: np.ndarray, initial_covariance: np.ndarray, max_missed_detect: float, dt: float, 
                 radar_pos: np.ndarray = np.array([0.0, 0.0, 0.0]), chi2_thresholds: dict = {1: 3.84, 2: 5.99, 3: 7.81, 4: 9.49}):
        
        # Mathematical engine
        self.ekf = ExtendedKalmanFilter(initial_state, initial_covariance)
        
        # Parameters
        self.dt = dt
        self.max_missed_detect = max_missed_detect
        self.radar_pos = radar_pos
        self.chi2_thresholds = chi2_thresholds    
        
        # Track lifecycle logic
        self.missed_detections = 0
        self.is_alive = True


    def process_measurement(self, z_meas: np.ndarray, R: np.ndarray, process_noise_var: float) -> None:
        """
        Main entry point called by main.py at each time step.
        Executes the logic: Predict -> Gating (Mahalanobis) -> Update (or Coasting).
        
        Args:
            z_meas (np.ndarray): The radar measurement vector (can contain NaNs).
            R (np.ndarray): Measurement noise covariance matrix.
            process_noise_var (float): Process noise variance for Q matrix.
        Returns:
            bool : False if track is dead, True otherwise
        """
        # 1. Prediction (always)
        F = get_F_CV(self.dt)
        Q = get_Q_CV(self.dt, process_noise_var)
        self.ekf.predict(F, Q)

        # 2. Converting prediction into a measurement
        z_pred_full = h_nonlinear(self.ekf.x, self.radar_pos)
        H_full = compute_jacobian(h_nonlinear, self.ekf.x, self.radar_pos)

        # 3. Creation of a Boolean mask (True -> measure, False -> NaN)
        mask = ~np.isnan(z_meas).flatten()
        
        # 4. Degrees of freedom (number of valid measurements)
        dof = np.sum(mask)

        # 5. Case where no measurement at all
        if dof == 0:
            self.missed_detections += 1

        # 6. Case where there is at least one measurement    
        else:
            # Dynamic truncation
            z_meas_part = z_meas[mask]           # Only keeping valid measures
            z_pred_part = z_pred_full[mask]      # Only keeping the associated predicted measures
            H_part = H_full[mask, :]             # Only keeping the lines associated to the valid measures
            R_part = R[np.ix_(mask, mask)]       # Only keeping the lines and columns associated to the valid measures
            
            # 6.1. Calculating covariance innovation and Mahalanobis' distance
            S_part = self.ekf.covariance_innovation_S(H_part, R_part)
            D2 = self._calculate_mahalanobis(z_meas_part, z_pred_part, S_part)
            
            # 6.2. Fetching the gating threshold associated with the degree of freedom
            gating_threshold = self.chi2_thresholds[dof]
            
            
            if D2 <= gating_threshold:
                # 6.3. Updating
                self.ekf.update(z_meas_part, z_pred_part, H_part, S_part)
                self.missed_detections = 0
          
            else:
                # 6.4. Rejecting measurement
                self.missed_detections += 1

        # 7. Handling track's death if to many missed detections
        if self.missed_detections == self.max_missed_detect:
            return False
        else:
            return True
        

    def _calculate_mahalanobis(self, z_meas: np.ndarray, z_pred: np.ndarray, S: np.ndarray) -> float:
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

        innovation = z_meas - z_pred
        D2 = innovation.T @ np.linalg.solve(S, innovation)      # Same as : inov.T x inv(S) x inov

        return D2.item()