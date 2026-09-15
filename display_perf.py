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
    invP_diff = np.linalg.solve(P_valid, diff_valid[:, :, np.newaxis]).squeeze(-1)      # Adding fake column for calculation, them removing it
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
    """ Displays the 3D trajectory and temporal error for ONE scenario."""

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
    ax2.plot(time_axis, err_pos_scenario, color='blue', label='Position Error (m)')
    ax2.axhline(y=np.nanmean(err_pos_scenario), color='orange', linestyle='--', label='Mean Error')
    
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Euclidean Error (m)')
    ax2.set_title("Position error over time")
    ax2.grid(True)
    ax2.legend()
    
    plt.tight_layout()
    plt.show()



def ellipsoids_plot_single_scenario(est_states: np.ndarray, cov_matrices: np.ndarray, truth_states: np.ndarray, 
                                    radar_pos: np.ndarray = np.array([0.0, 0.0, 0.0]), step: int = 10, confidence_sigma: float = 2.8):
    """
    Displays the 3D trajectories and the confidence ellipsoids associated to the estimated states.
    
    Args:
        est_states: Array of shape (N, 6) containing estimated states.
        cov_matrices: Array of shape (N, 6, 6) containing covariance matrices.
        truth_states: Array of shape (N, 6) containing ground truth.
        radar_pos: 3D coordinates of the radar.
        step: Plots an ellipsoid every 'step' measurements to avoid visual clutter.
        confidence_sigma: Scale factor for the ellipsoids. Follows X_2 distribution with 3 degrees of freedom (e.g., 2.8 for 95% confidence).
    """

    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # 1. Plot trajectories
    ax.plot(truth_states[:, 0], truth_states[:, 1], truth_states[:, 2], 'g-', linewidth=2, label='Ground Truth')
    
    ax.plot(est_states[:, 0], est_states[:, 1], est_states[:, 2], 'r--', linewidth=2, label='Kalman (EKF)')
    ax.scatter(*radar_pos, c='k', marker='^', s=150, label='Radar')

    # 2. Base unit sphere generation
    u = np.linspace(0, 2 * np.pi, 20)
    v = np.linspace(0, np.pi, 20)
    x_sph = np.outer(np.cos(u), np.sin(v))
    y_sph = np.outer(np.sin(u), np.sin(v))
    z_sph = np.outer(np.ones_like(u), np.cos(v))
    sphere_coords = np.vstack((x_sph.flatten(), y_sph.flatten(), z_sph.flatten()))

    # 3. Plotting ellipsoids every 'step'
    for k in range(0, len(est_states), step):
        if np.any(np.isnan(est_states[k, :6])) or np.any(np.isnan(cov_matrices[k, :, :])):
            continue

        pos = est_states[k, 0:3]
        cov_pos = cov_matrices[k, 0:3, 0:3]

        # Diagonalization: calculating eigenvalues and eigenvectors
        eigvals, eigvecs = np.linalg.eigh(cov_pos)
        
        # Safety for numerical stability (prevent negative extremely close to zero eigenvalues)
        eigvals = np.maximum(eigvals, 0)

        # Scale and rotate the sphere
        semi_axes = np.sqrt(eigvals) * confidence_sigma
        scaling_matrix = np.diag(semi_axes)
        
        ellipsoid_coords = eigvecs @ scaling_matrix @ sphere_coords

        # Translate to the estimated position
        X = ellipsoid_coords[0, :].reshape(20, 20) + pos[0]
        Y = ellipsoid_coords[1, :].reshape(20, 20) + pos[1]
        Z = ellipsoid_coords[2, :].reshape(20, 20) + pos[2]

        # Plot as a wireframe so we can see the trajectory inside
        ax.plot_wireframe(X, Y, Z, color='blue', alpha=0.15)

    ax.set_title('3D Trajectories with Confidence Ellipsoids')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.legend()
    
    plt.tight_layout()
    plt.show()



def evaluate_benchmark(err_pos_run2: np.ndarray, err_vel_run2: np.ndarray, D2_run2: np.ndarray, NLL_run2: np.ndarray,
                        err_pos_run1: np.ndarray, err_vel_run1: np.ndarray, D2_run1: np.ndarray, NLL_run1: np.ndarray,
                         title: str = "PARTIAL vs FULL"):
    """Allows to compare fairly the performance of two different models. It displays statistics regarding common estimated states
    and their performance."""
    
    # 1. Shared masks
    mask_run2 = ~np.isnan(err_pos_run2)
    mask_run1 = ~np.isnan(err_pos_run1)

    common_mask_pos = ~np.isnan(err_pos_run2) & ~np.isnan(err_pos_run1)
    common_mask_vel = ~np.isnan(err_vel_run2) & ~np.isnan(err_vel_run1)
    common_mask_D2 = ~np.isnan(D2_run2) & ~np.isnan(D2_run1)
    common_mask_NLL = ~np.isnan(NLL_run2) & ~np.isnan(NLL_run1)

    # 2. Analysing common estimations
    total_run2 = np.sum(mask_run2)
    total_run1 = np.sum(mask_run1)
    total_common = np.sum(common_mask_pos)
    
    only_run2 = np.sum(mask_run2 & ~mask_run1)
    only_run1 = np.sum(~mask_run2 & mask_run1)
    
    pct_run2_shared = (total_common / total_run2 * 100) if total_run2 > 0 else 0
    pct_run1_shared = (total_common / total_run1 * 100) if total_run1 > 0 else 0

    # 2. Applying masks
    fair_pos_run2 = err_pos_run2[common_mask_pos]
    fair_pos_run1 = err_pos_run1[common_mask_pos]
    
    fair_vel_run2 = err_vel_run2[common_mask_vel]
    fair_vel_run1 = err_vel_run1[common_mask_vel]

    fair_D2_run2 = D2_run2[common_mask_D2]
    fair_D2_run1 = D2_run1[common_mask_D2]

    fair_NLL_run2 = NLL_run2[common_mask_NLL]
    fair_NLL_run1 = NLL_run1[common_mask_NLL]

    # 3. Display
    print(f"\n=== BENCHMARK: {title} ===")

    print("\n-- Track Survival Intersection --")
    print(f"Total survived updates Run 1 : {total_run1}")
    print(f"Total survived updates Run 2 : {total_run2}")
    print(f"Common survival (Intersection)   : {total_common}")
    print(f"  -> Run 1 shares {pct_run1_shared:.1f}% of its life with Run 2")
    print(f"  -> Run 2 shares {pct_run2_shared:.1f}% of its life with Run 1")
    print(f"Unique to Run 1 : {only_run1} updates")
    print(f"Unique to Run 2 : {only_run2} updates")

    print("\n-- Position Error --")
    print(f"Run 1 : {np.mean(fair_pos_run1):.2f} m (95% < {np.percentile(fair_pos_run1, 95):.2f} m)")
    print(f"Run 2 : {np.mean(fair_pos_run2):.2f} m (95% < {np.percentile(fair_pos_run2, 95):.2f} m)")
    
    print("\n-- Velocity Error --")
    print(f"Run 1 : {np.mean(fair_vel_run1):.2f} m/s (95% < {np.percentile(fair_vel_run1, 95):.2f} m/s)")
    print(f"Run 2 : {np.mean(fair_vel_run2):.2f} m/s (95% < {np.percentile(fair_vel_run2, 95):.2f} m/s)")

    print("\n-- Mahalanobis Error --")
    print(f"Run 1 : {np.mean(fair_D2_run1):.2f} (95% < {np.percentile(fair_D2_run1, 95):.2f})")
    print(f"Run 2 : {np.mean(fair_D2_run2):.2f} (95% < {np.percentile(fair_D2_run2, 95):.2f})")

    print("\n-- NLL --")
    print(f"Run 1 : {np.mean(fair_NLL_run1):.2f} (95% < {np.percentile(fair_NLL_run1, 95):.2f})")
    print(f"Run 2 : {np.mean(fair_NLL_run2):.2f} (95% < {np.percentile(fair_NLL_run2, 95):.2f})")