import numpy as np
import scipy.io as sio
from display_perf import evaluate_tracker_performance, plot_single_scenario
from config import TrackerConfig
from pipeline import run_single_scenario



# ====| 1. Data |==== #

dataset_files = [
    'Dataset/Radar_Dataset_dt_0.1_1.mat',
    'Dataset/Radar_Dataset_dt_0.1_3.mat'
]


# ====| 2. User Parameters |==== #

config = TrackerConfig()


# ====| 3.1 Iterating on Datasets |==== #

if 0:

    all_estimated_states = []
    all_ground_truth = []

    for file in dataset_files:

        # 1. Loading dataset
        mat = sio.loadmat(file)
        dataset = mat['dataset']
        dataset[:, :, 2] = np.radians(dataset[:, :, 2])
        dataset[:, :, 3] = np.radians(dataset[:, :, 3])
        num_scenarios, num_measures, _ = dataset.shape

        # 2. Fetching time step of current dataset
        dt = float(mat['dt'].item())
        config.dt = dt

        # 3. Running pipeline scenario after scenario
        for i in range(num_scenarios):

            estimated_states_array = np.full((num_measures, 6), np.nan)
            estimated_states_array = run_single_scenario(dataset[i, :, 0:4], estimated_states_array, config)

            # 4. Adding new filled estimated states array & associated ground truth
            all_estimated_states.append(estimated_states_array)
            all_ground_truth.append(dataset[i, :, 4:10])


    # ====| 4.1 Displaying results |==== #

    evaluate_tracker_performance(all_estimated_states, all_ground_truth)


# ====| 3.2 Filtering Specific Scenarios |==== #

if 1:

    # 1. Fetching global dataset
    all_data = []

    for file in dataset_files:   
        mat = sio.loadmat(file)
        dataset = mat['dataset']
        dt = float(mat['dt'].item())
        num_scenarios = dataset.shape[0]

        for i in range(num_scenarios):
            all_data.append((dataset[i], dt))

    # 2. Selecting a random scenario
    scenario_idx = np.random.randint(0, len(all_data))
    scenario, current_dt = all_data[scenario_idx] 
    num_measures = scenario.shape[0]
    scenario[:, 2] = np.radians(scenario[:, 2])
    scenario[:, 3] = np.radians(scenario[:, 3])

    # 3. Assigning correct dt to config
    config.dt = current_dt

    # 4. Running & Plotting
    estimated_states_array = np.full((num_measures, 6), np.nan)
    estimated_states = run_single_scenario(scenario[:, 0:4], estimated_states_array, config)

    plot_single_scenario(estimated_states, scenario[:, 4:10], config.dt, config.radar_pos)
