import numpy as np
from ekf import ExtendedKalmanFilter
from models import get_F_CV, get_Q_CV, h_nonlinear, compute_jacobian
from config import TrackerConfig
from update_strategies import *



class RadarTracker:
    """
    Manages the lifecycle of a target track, handling missed detections (NaNs),
    outlier rejection (Gating), and orchestrating the EKF mathematical engine.
    """

    def __init__(self, initial_state: np.ndarray, initial_covariance: np.ndarray, config: TrackerConfig):
        
        # Mathematical engine
        self.ekf = ExtendedKalmanFilter(initial_state, initial_covariance)
        
        # Parameters
        self.dt = config.dt
        self.max_missed_detect = config.max_missed_detect
        self.radar_pos = config.radar_pos
        self.chi2_thresholds = config.chi2_thresholds  
        self.allow_partial_update = config.allow_partial_update  
        self.config = config
        
        # Track lifecycle logic
        self.missed_detections = 0
        if self.config.update_strategy.lower() == "standard":
            self.updater = StandardUpdate()
        elif self.config.update_strategy.lower() == "heuristic_reset":
            self.updater = HeuristicResetUpdate()
        elif self.config.update_strategy.lower() == "hybrid_heuristic_reset":
            self.updater = HybridResetUpdate()
        elif self.config.update_strategy.lower() == "cmkf":
            self.updater = CMKF()
        elif self.config.update_strategy.lower() == "ukf":
            self.updater = UKF()
        else:
            raise ValueError(f"Unknown Strategy: {self.config.update_strategy}")


    def process_measurement(self, z_meas: np.ndarray) -> None:
        """
        Main entry point called by main.py at each time step.
        Executes the logic: Predict -> Gating (Mahalanobis) -> Update (or Coasting).
        
        Args:
            z_meas (np.ndarray): The radar measurement vector (can contain NaNs).
        Returns:
            bool : False if track is dead, True otherwise
        """

        # ---| 1. Prediction (always) |---#

        F = get_F_CV(self.dt)
        Q = get_Q_CV(self.dt, self.config.sigma_acc)
        self.ekf.predict(F, Q)

        # ---| 2. Update (not always) |---#

        # 1. Determining R according to the measurement
        R_current = self.config.R_elementary    # Currently using the same basic measurement noise matrix

        # 2. Converting prediction into a measurement
        z_pred = h_nonlinear(self.ekf.x, self.radar_pos)

        # 3. Creation of a Boolean mask (True -> measure, False -> NaN)
        mask = ~np.isnan(z_meas).flatten()

        # 4. Evaluating detection accordingly to the config (allow_partial_update)
        is_missing = not np.any(mask) if self.allow_partial_update else not np.all(mask)
        if is_missing:
            self.missed_detections += 1

        # 5. Case where an update is possible    
        else:

            # 5.1. Launching Updating Process
            is_updated = self.updater.apply(self, z_meas, z_pred, mask, R_current, self.config, self.chi2_thresholds)

            if is_updated:
                # 5.2. Restarting counter
                self.missed_detections = 0
            else:
                # 5.3. Rejecting measurement
                self.missed_detections += 1

        # 6. Handling track's death if to many missed detections
        if self.missed_detections == self.max_missed_detect:
            return False
        else:
            return True