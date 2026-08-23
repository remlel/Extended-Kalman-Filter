import numpy as np
import matplotlib.pyplot as plt



def evaluate_tracker_performance(all_estimated_states: list, all_covariance_matrices: list, all_truth_states: list, benchmark: bool=False):
    """
    Evaluates and displays the performance of the Kalman Filter on the entire dataset.
    This function takes into account partial state filter output : position and velocity.
    
    Args:
        estimated_states (list[np.ndarray]): Resulting states, scenarios x (time_steps, 6 state variables).
        all_covariance_matrices (list[np.ndarray]): Resulting covariance matrices, scenarios x (time_steps, 6x6 covariance matrix) 
        truth_states (list[np.ndarray]): Ground truth, scenarios x (time_steps, 6 state variables).
    """

    total_num_scenarios = len(all_truth_states)
    total_num_states = 0
    all_err_pos = []
    all_err_vel = []
    all_diff    = []
    
    # ---| 1. Calculation Euclidean Errors |--- #

    for i in range(total_num_scenarios):

        err_pos = np.sqrt(np.sum((all_estimated_states[i][:, 0:3] - all_truth_states[i][:, 0:3])**2, axis=1))
        err_vel = np.sqrt(np.sum((all_estimated_states[i][:, 3:6] - all_truth_states[i][:, 3:6])**2, axis=1))
        all_err_pos.append(err_pos)
        all_err_vel.append(err_vel)
        total_num_states += all_truth_states[i].shape[0]
    
    array_err_pos = np.concatenate(all_err_pos)
    array_err_vel = np.concatenate(all_err_vel)

    # ---| 2. Calculation Mahalanobis & NLL Error |--- #

    for i in range(total_num_scenarios):

        diff = all_estimated_states[i] - all_truth_states[i]
        all_diff.append(diff)

    array_diff = np.concatenate(all_diff)
    array_covariance_matrices = np.concatenate(all_covariance_matrices)

    # Filtering diff with no Nan at all
    valid_mask = ~np.any(np.isnan(array_diff[:, :6]), axis=1)

    # Initialization of arrays
    array_D2 = np.full(len(array_diff), np.nan)
    array_NLL = np.full(len(array_diff), np.nan)

    # Fetching elements associated with valid_mask
    diff_valid = array_diff[valid_mask]
    P_valid = array_covariance_matrices[valid_mask]

    # Mahalanobis error
    invP_diff = np.linalg.solve(P_valid, diff_valid[:, :, np.newaxis]).squeeze(-1)      # Adding fake column for calculatio, them removing it
    array_D2[valid_mask] = np.sum(diff_valid * invP_diff, axis=1)                       # Same as: diff_valid @ invP_diff

    # NLL error
    det_P = np.linalg.det(2 * np.pi * P_valid)
    array_NLL[valid_mask] = 0.5 * np.log(det_P) + 0.5 * array_D2[valid_mask]

    # ---| 3. Individual Statistical Evaluation |--- #    
    if not benchmark:

        # 2. Global Statistics
        mean_pos_err = np.nanmean(array_err_pos)
        p95_pos_err  = np.nanpercentile(array_err_pos, 95)
        mean_vel_err = np.nanmean(array_err_vel)
        p95_vel_err  = np.nanpercentile(array_err_vel, 95)
        mean_D2_err  = np.nanmean(array_D2)
        p95_D2_err   = np.nanpercentile(array_D2, 95)
        mean_NLL_err  = np.nanmean(array_NLL)
        p95_NLL_err   = np.nanpercentile(array_NLL, 95)

        # 3. Availability Ratio
        total_expected = total_num_states * 2
        total_valid_err_pos = np.sum(~np.isnan(array_err_pos))
        total_valid_err_vel = np.sum(~np.isnan(array_err_vel))
        total_valid = total_valid_err_pos + total_valid_err_vel
        availability = (total_valid / total_expected) * 100
    
        print("=== OVERALL PERFORMANCE ===")
        print(f"Track availability  : {availability:.2f}%")
        print(f"Mean Position Error : {mean_pos_err:.2f} m (95% < {p95_pos_err:.2f} m)")
        print(f"Mean Velocity Error : {mean_vel_err:.2f} m/s (95% < {p95_vel_err:.2f} m/s)")
        print(f"Mean Mahalanobis Error : {mean_D2_err:.2f} (95% < {p95_D2_err:.2f})")
        print(f"Mean NLL Error : {mean_NLL_err:.2f} (95% < {p95_NLL_err:.2f})\n")

    else:
        return array_err_pos, array_err_vel, array_D2, array_NLL
    


def plot_single_scenario(est_state: np.ndarray, true_state: np.ndarray, dt: float, radar_pos: np.ndarray = np.array([0.0, 0.0, 0.0])):
    """ Displays the 3D trajectory and temporal error for ONE scenario. """

    err_pos_scenario = np.sqrt(np.sum((est_state[:, 0:3] - true_state[:, 0:3])**2, axis=1))

    time_axis = np.arange(len(est_state)) * dt
    
    fig = plt.figure(figsize=(14, 6))
    
    # --- Plot 1 : 3D Trajectory --- #
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    ax1.plot(true_state[:, 0], true_state[:, 1], true_state[:, 2], 
             label="Ground Truth", color='green', linewidth=2)
    ax1.plot(est_state[:, 0], est_state[:, 1], est_state[:, 2], 
             label="Kalman (EKF)", color='red', linestyle='dashed')
    
    # Scattering Radar Position
    x0, y0, z0 = radar_pos
    ax1.scatter(x0, y0, z0, color='black', marker='^', s=100, label="Radar")
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title("Comparison of 3D Trajectories")
    ax1.legend()
    
    # --- Plot 2 : Temporal Error --- #
    ax2 = fig.add_subplot(1, 2, 2)
    ax2.plot(time_axis, err_pos_scenario, color='blue', label='Erreur Position (m)')
    ax2.axhline(y=np.nanmean(err_pos_scenario), color='orange', linestyle='--', label='Moyenne du scénario')
    
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Euclidean Error (m)')
    ax2.set_title("Position error over time")
    ax2.grid(True)
    ax2.legend()
    
    plt.tight_layout()
    plt.show()



def evaluate_benchmark(err_pos_full: np.ndarray, err_vel_full: np.ndarray,err_pos_part: np.ndarray, err_vel_part: np.ndarray):
    """
    """
    
    # 1. Shared masks
    common_mask_pos = ~np.isnan(err_pos_full) & ~np.isnan(err_pos_part)
    common_mask_vel = ~np.isnan(err_vel_full) & ~np.isnan(err_vel_part)

    # 2. Applying masks
    fair_pos_full = err_pos_full[common_mask_pos]
    fair_pos_part = err_pos_part[common_mask_pos]
    
    fair_vel_full = err_vel_full[common_mask_vel]
    fair_vel_part = err_vel_part[common_mask_vel]

    # 3. Display
    print("\n=== BENCHMARK: PARTIAL vs FULL INIT ===")

    print("\n-- Position Error --")
    print(f"Full Init   : {np.mean(fair_pos_full):.2f} m (95% < {np.percentile(fair_pos_full, 95):.2f} m)")
    print(f"Partial Init  : {np.mean(fair_pos_part):.2f} m (95% < {np.percentile(fair_pos_part, 95):.2f} m)")
    
    print("\n-- Velocity Error --")
    print(f"Full Init   : {np.mean(fair_vel_full):.2f} m/s (95% < {np.percentile(fair_vel_full, 95):.2f} m/s)")
    print(f"Partial Init  : {np.mean(fair_vel_part):.2f} m/s (95% < {np.percentile(fair_vel_part, 95):.2f} m/s)\n")