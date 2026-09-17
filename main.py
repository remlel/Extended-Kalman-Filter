import numpy as np
import scipy.io as sio
from display_perf import *
from config import TrackerConfig
from pipeline import run_single_scenario



def run_full_estimation(dataset_files: list, config: TrackerConfig):
    """
    Uses all datasets files to evaluate the Kalman Filter at a global scale.
    The datasets can have different time steps and/or number of measurements (2nd dimension).
    Args:
        dataset_files (list): Contains .mat datatset files
        config (TrackerConfig): The user config of the scenario.
    Returns:
        all_estimated_states (list[np.ndarray]): Contains all the arrays of estimated states for each scenario. 
        all_covariance_matrices (list[np.ndarray]): Contains all the estimated covariance matrices for each scenario.
        all_ground_truth (list[np.ndarray]): Contains all the arrays of truth states for each scenario. 
    """

    all_estimated_states    = []
    all_covariance_matrices = []
    all_ground_truth        = []

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
            covariance_matrices_array = np.full((num_measures, 6, 6), np.nan)
            estimated_states_array, covariance_matrices_array = run_single_scenario(dataset[i, :, 0:4],
                estimated_states_array, covariance_matrices_array, config)

            # 4. Adding new filled estimated states array & associated covariance matrices & associated ground truth
            all_estimated_states.append(estimated_states_array)
            all_covariance_matrices.append(covariance_matrices_array)
            all_ground_truth.append(dataset[i, :, 4:10])

    return all_estimated_states, all_covariance_matrices, all_ground_truth



def run_random_visualization(dataset_files: list, config: TrackerConfig):
    """
        Randomly picks a scenario among all datasets files and displays a plot of the target trajectory and estimated positions.
        The datasets can have different time steps and/or number of measurements (2nd dimension).
        """

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
    covariance_matrices_array = np.full((num_measures, 6, 6), np.nan)
    estimated_states, covariance_matrices = run_single_scenario(scenario[:, 0:4], estimated_states_array, covariance_matrices_array, config)

    plot_single_scenario(estimated_states, scenario[:, 4:10], config.dt, config.radar_pos)
    ellipsoids_plot_single_scenario(estimated_states, covariance_matrices, scenario[:, 4:10], config.radar_pos)



# ====| Main Execution |==== #

if __name__ == "__main__":
    
    # ---| 1. "Switches" |--- #
    
    DO_EVALUATION = False
    DO_VISUALIZATION = False
    DO_BENCHMARK = True


    # ---| 2. Setup |--- #

    dataset_files = [
        'Dataset/Radar_Dataset_dt_0.1_1.mat',
        'Dataset/Radar_Dataset_dt_0.1_2.mat'
    ]
    my_config = TrackerConfig()


    # ---| 3. Execution |--- #

    if DO_EVALUATION:

        print("\n--- Running Global Evaluation ---")
        all_estimated_states, all_covariance_matrices, all_ground_truth = run_full_estimation(dataset_files, my_config)
        evaluate_tracker_performance(all_estimated_states, all_covariance_matrices, all_ground_truth)
        
    if DO_VISUALIZATION:

        print("\n--- Running Random Scenario Visualization ---")
        run_random_visualization(dataset_files, my_config)

    if DO_BENCHMARK:

        print("\n--- Running Benchmark ---")

        # ---| Common configuration |--- #
        my_config.force_degraded_init = True

        # ---| Run 1 |--- #

        # ======> Chose Title <====== #
        run1_title = "ukf (Init_Part & ~Update_Part)"
        # ======> Chose desired config <====== #
        my_config.allow_partial_init = True
        my_config.allow_partial_update = False
        my_config.update_strategy = "ukf"
        #======================================#

        states_run1, cov_matrices_run1, truth = run_full_estimation(dataset_files, my_config)
        array_err_pos_run1, array_err_vel_run1, array_D2_run1, array_NLL_run1 = evaluate_tracker_performance(states_run1, cov_matrices_run1, truth, benchmark=True)
        
        # ---| Run 2 |--- #

        # ======> Chose Title <====== #
        run2_title = "heuristic_reset (Init_Part & ~Update_Part)"
        # ======> Chose desired config <====== #
        my_config.allow_partial_init = True
        my_config.allow_partial_update = False
        my_config.update_strategy = "heuristic_reset"
        #======================================#

        states_run2, cov_matrices_run2, _ = run_full_estimation(dataset_files, my_config)
        array_err_pos_run2, array_err_vel_run2, array_D2_run2, array_NLL_run2 = evaluate_tracker_performance(states_run2, cov_matrices_run2, truth, benchmark=True)

        # ---| Final Fair Intersection Comparaison |--- #

        benchmark_title = "Run 1: " + run1_title + " VS " + "Run 2: " + run2_title
        evaluate_benchmark(array_err_pos_run2, array_err_vel_run2, array_D2_run2, array_NLL_run2,
                            array_err_pos_run1, array_err_vel_run1, array_D2_run1, array_NLL_run1, benchmark_title)