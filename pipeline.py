import numpy as np
from config import TrackerConfig
from models import get_initial_state
from tracker import RadarTracker



def run_single_scenario(scenario_data: np.ndarray, estimated_states: np.ndarray, covariance_matrices: np.ndarray, config: TrackerConfig):
    """ 
    Applies the tracking pipeline: 
    1) Initialization of a track. 
    2) Tracking until death of the track.
    3) Repeating first two steps until end of scenario. to the input scenario. 
    The output of this function is an array of the associated estimated states.
    Args:
        scenario_data (np.ndarray): The 4 measurements for each time step.
        estimated_states (np.ndarray): Nan array to be filled with estimated states (6).
        covariance_matrices (np.ndarray): Nan array to be filled with covariance matrices (6x6).
        config (TrackerConfig): The user config of the scenario.
    Returns:
        estimated_states (np.ndarray): Filled array.
        covariance_matrices (np.ndarray): Filled array.
    """
 
    # ---| Initialization Ablation Study |--- #
    if config.force_degraded_init:
        # Searching first non-all Nan measurement 
        detection_idx = np.where(~np.all(np.isnan(scenario_data), axis=1))[0][0]
        # Removing both angles (failed MUSIC)
        scenario_data[detection_idx, 2:4] = np.nan        
    #------------------------------------------#

    # Global index for the entire scenario
    k = 0  
    num_measures, num_states = np.shape(estimated_states)
    
    while k < num_measures:
        
        # ---| PHASE 1 : SEARCH (Initialization) |--- #

        # Moving on until finding a measurement verifying initialization criteria
        isnan_condition = np.all if config.allow_partial_init else np.any
        while k < num_measures and isnan_condition(np.isnan(scenario_data[k])):
            estimated_states[k, :]    = np.nan
            covariance_matrices[k, :, :] = np.nan
            k += 1

        # Moving on to next scenario when the end is reached    
        if k == num_measures:
            break  

        # Fetching initial measurement    
        Z_init = scenario_data[k]
    
        X_init, P_init = get_initial_state(Z_init, config.sigma_array, config.sigma_vel, config.antenna_array, config.N, config.radar_pos)
        
        tracker = RadarTracker(X_init, P_init, config.max_missed_detect, config.dt, config.radar_pos, config.chi2_thresholds)
        estimated_states[k, :] = tracker.ekf.x.flatten()
        covariance_matrices[k, :, :] = tracker.ekf.p
        k += 1  # Moving to next measurement
        
        # ---| PHASE 2 : Tracking |--- #

        while k < num_measures:
            z_meas = scenario_data[k].reshape(-1, 1)

            # Determining R according to the measurement
            R_current = config.R_elementary 
            
            track_alive = tracker.process_measurement(z_meas, R_current, config.sigma_acc)
            estimated_states[k, :] = tracker.ekf.x.flatten()
            covariance_matrices[k, :, :] = tracker.ekf.p
            k += 1
            
            if not track_alive:
                # Clearing predictions
                start_idx = max(0, k - config.max_missed_detect)
                estimated_states[start_idx : k ] = np.nan
                covariance_matrices[start_idx : k, :, :] = np.nan
                # Launching new initialization
                break

    return estimated_states, covariance_matrices