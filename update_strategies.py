from abc import ABC, abstractmethod
import numpy as np
from models import get_initial_state



class UpdateStrategy(ABC):
    @abstractmethod
    def apply(self, tracker, z_meas, mask, z_meas_part, z_pred_part, H_part, S_part, config):
        pass

class StandardUpdate(UpdateStrategy):
    def apply(self, tracker, z_meas, mask, z_meas_part, z_pred_part, H_part, S_part, config):
        # L'update classique de l'EKF
        tracker.ekf.update(z_meas_part, z_pred_part, H_part, S_part)

class HeuristicResetUpdate(UpdateStrategy):
    def apply(self, tracker, z_meas, mask, z_meas_part, z_pred_part, H_part, S_part, config):

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
    def apply(self, tracker, z_meas, mask, z_meas_part, z_pred_part, H_part, S_part, config):

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