import numpy as np
from config import TrackerConfig
from models import get_initial_state
from tracker import RadarTracker



def run_single_scenario(scenario_data: np.ndarray, estimated_states: np.ndarray, config: TrackerConfig):
    """ 
    
    """

    # Global index for the entire scenario
    k = 0  
    num_measures, num_states = np.shape(estimated_states)
    
    while k < num_measures:
        
        # ---| PHASE 1 : SEARCH (Initialization) |--- #

        # Moving on until finding a measurement with at least one value
        while k < num_measures and np.all(np.isnan(scenario_data[k])):
            estimated_states[k, :] = np.nan
            k += 1

        # Moving on to next scenario when the end is reached    
        if k == num_measures:
            break  

        # Fetching initial measurement    
        Z_init = scenario_data[k]
    
        X_init, P_init = get_initial_state(Z_init, config.sigma_array, config.sigma_vel, config.antenna_array, config.N, config.radar_pos)
        
        tracker = RadarTracker(X_init, P_init, config.max_missed_detect, config.dt, config.radar_pos, config.chi2_thresholds)
        estimated_states[k, :] = tracker.ekf.x.flatten()
        k += 1  # Moving to next measurement
        
        # ---| PHASE 2 : Tracking |--- #

        while k < num_measures:
            z_meas = scenario_data[k].reshape(-1, 1)

            # Determining R according to the measurement
            R_current = config.R_elementary 
            
            track_alive = tracker.process_measurement(z_meas, R_current, config.sigma_acc)
            estimated_states[k, :] = tracker.ekf.x.flatten()
            k += 1
            
            if not track_alive:
                # Clearing predictions
                start_idx = max(0, k - config.max_missed_detect)
                estimated_states[start_idx : k ] = np.nan
                # Launching new initialization
                break

    return estimated_states