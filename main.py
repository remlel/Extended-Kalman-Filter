import numpy as np
import scipy.io as sio
from tracker import RadarTracker
from models import get_initial_state
from display_perf import evaluate_tracker_performance, plot_single_scenario

# ====| 1. Loading Dataset |==== #

mat_data = sio.loadmat('_.mat')

dataset = mat_data['_']       
num_scenarios, num_measures, num_features = dataset.shape

dataset[:, :, 2] = np.radians(dataset[:, :, 2])     # Converting azimuth and elevation angles to radians
dataset[:, :, 3] = np.radians(dataset[:, :, 3])

# Creating array for Kalman output states
estimated_states_array = np.zeros((num_scenarios, num_measures, 6))


#=================================================================#
#---------------| 2. Parameters to be set by User |---------------#
#=================================================================#

# Dynamic Gating Threshold (Chi-2 law at 95%)
chi2_thresholds = {1: 3.84, 2: 5.99, 3: 7.81, 4: 9.49}

#---| Dataset Parameters |---

# Standard Deviation of radar measuremnts
sigma_r = 1                 # Using the 1 m range resolution
sigma_v_rad = 0.76          # Using the 0.76 m/s velocity resolution
sigma_az = np.radians(3)    # Using the 3° standard deviation 
sigma_el = np.radians(3)    # Using the 3° standard deviation
sigma_array = np.array([sigma_r, sigma_v_rad, sigma_az, sigma_el])

# Measurement Noise Covariance Matrix (Data-Driven Tuning)
R_elementary = np.diag([sigma_r**2, sigma_v_rad**2, sigma_az**2, sigma_el**2])

# Process Noise Standard Deviation (Data-Driven Tuning) [m/s²]
sigma_acc = 6.79    # Square of the trace of the covariance matrix

# Speed Standard Deviation (Data-Driven Tuning) [m/s]
sigma_vel = 8.75    # Square of the trace of the covariance matrix

# Time Step
dt = 0.1

# Radar Position (Boresight direction = x axis)
radar_pos = np.array([0.0, 0.0, 0.0])

# Antenna
az_max = np.radians(45)
az_min = np.radians(45)
el_max = np.radians(45)
el_min = np.radians(45)
Rmax = 150      # max detection range on the boresight axis
antenna_array = np.array([az_max, az_min, el_max, el_min, Rmax])

# Number of generated points for Monte Carlo simulation
N = 1000

#=================================================================#


# ====| 3. Filtering Loop |==== #

for j in range(num_scenarios):

    scenario_data = dataset[j]

    # Global index for the entire scenario
    k = 0  
    
    while k < num_measures:
        
        # ---| PHASE 1 : SEARCH (Initialization) |--- #

        # Moving on until finding a measurement with at least one value
        while k < num_measures and np.all(np.isnan(scenario_data[k])):
            estimated_states_array[j, k, :] = np.nan
            k += 1

        # Moving on to next scenario when the end is reached    
        if k == num_measures:
            break  

        # Fetching initial measurement    
        Z_init = scenario_data[k]
    
        X_init, P_init = get_initial_state(Z_init, sigma_array, sigma_vel, antenna_array, N, radar_pos)
        
        tracker = RadarTracker(X_init, P_init, dt, radar_pos, chi2_thresholds)
        estimated_states_array[j, k, :] = tracker.ekf.x.flatten()
        k += 1  # Moving to next measurement
        
        # ---| PHASE 2 : Tracking |--- #

        while k < num_measures:
            z_meas = scenario_data[k]

            # Determining R according to the measurement
            R_current = R_elementary 
            
            track_alive = tracker.process_measurement(z_meas, R_current, sigma_acc)
            estimated_states_array[j, k, :] = tracker.ekf.x.flatten()
            k += 1
            
            if not track_alive:
                # Launching new initialization
                break


# ====| 4. Evaluating Filter Output |==== #

# 1. Global Performance
truth_data = dataset[:, :, 4:10]
evaluate_tracker_performance(estimated_states_array, truth_data)

# 2. Detailed View
i = np.random.randint(0, num_scenarios)
plot_single_scenario(estimated_states_array[i], truth_data[i], dt, radar_pos)