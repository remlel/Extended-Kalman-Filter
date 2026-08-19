import numpy as np
import matplotlib.pyplot as plt

def evaluate_tracker_performance(all_estimated_states: list, all_truth_states: list):
    """
    Evaluates and displays the performance of the Kalman Filter on the entire dataset.
    This function takes into account partial state filter output : position and velocity.
    
    Args:
        estimated_states: Resulting states 3D array (scenarios, time_steps, 6 state variables).
        truth_states: Ground truth 3D array (scenarios, time_steps, 6 state variables).
    """

    total_num_scenarios = len(all_truth_states)
    total_num_states = 0
    all_err_pos = []
    all_err_vel = []
    
    # 1. Calculation of Euclidean errors (ignoring NaNs for the calculations)
    for i in range(total_num_scenarios):

        err_pos = np.sqrt(np.sum((all_estimated_states[i][:, 0:3] - all_truth_states[i][:, 0:3])**2, axis=1))
        err_vel = np.sqrt(np.sum((all_estimated_states[i][:, 3:6] - all_truth_states[i][:, 3:6])**2, axis=1))
        all_err_pos.append(err_pos)
        all_err_vel.append(err_vel)
        total_num_states += all_truth_states[i].shape[0]
    
    # 2. Global Statistics  
    array_err_pos = np.concatenate(all_err_pos)
    mean_pos_err  = np.nanmean(array_err_pos)
    p95_pos_err   = np.nanpercentile(array_err_pos, 95)

    array_err_vel = np.concatenate(all_err_vel)
    mean_vel_err  = np.nanmean(array_err_vel)
    p95_vel_err   = np.nanpercentile(array_err_vel, 95)
    
    # Availability Ratio
    total_expected = total_num_states * 2
    total_valid_err_pos = np.sum(~np.isnan(array_err_pos))
    total_valid_err_vel = np.sum(~np.isnan(array_err_vel))
    total_valid = total_valid_err_pos + total_valid_err_vel
    availability = (total_valid / total_expected) * 100
    
    print("=== OVERALL PERFORMANCE ===")
    print(f"Track availability  : {availability:.2f}%")
    print(f"Mean Position Error : {mean_pos_err:.2f} m (95% < {p95_pos_err:.2f} m)")
    print(f"Mean Velocity Error : {mean_vel_err:.2f} m/s (95% < {p95_vel_err:.2f} m/s)\n")
    


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