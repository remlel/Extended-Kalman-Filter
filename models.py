import numpy as np
import math

def get_F_CV(dt: float) -> np.ndarray:
    """
    Computes the State Transition Matrix (F) for a Constant Velocity (CV) model in 3D.
    The state vector is assumed to be X = [x, y, z, vx, vy, vz]^T.
    
    Args:
        dt (float): Time step between two measurements (in seconds).

    Returns:
        np.ndarray: A 6x6 transition matrix.
    """

    F = np.array([
        [1, 0, 0, dt,  0,  0],
        [0, 1, 0,  0, dt,  0],
        [0, 0, 1,  0,  0, dt],
        [0, 0, 0,  1,  0,  0],
        [0, 0, 0,  0,  1,  0],
        [0, 0, 0,  0,  0,  1]
    ])

    return F


def get_Q_CV(dt: float, process_noise_var: float) -> np.ndarray:
    """
    Computes the Process Noise Covariance Matrix (Q) for a Constant Velocity (CV) model.
    This matrix models the uncertainty in the constant velocity assumption 
    (e.g., target maneuvers or accelerations). We use the discrete-time 
    white noise acceleration model.
    
    Args:
        dt (float): Time step (in seconds).
        process_noise_var (float): The variance of the unknown acceleration (sigma^2).
                                                                
    Returns:
        np.ndarray: A 6x6 process noise covariance matrix.
    """

    Q = process_noise_var * np.array([
        [(dt**4)/4,         0,         0, (dt**3)/2,         0,         0],
        [        0, (dt**4)/4,         0,         0, (dt**3)/2,         0],
        [        0,         0, (dt**4)/4,         0,         0, (dt**3)/2],
        [(dt**3)/2,         0,         0,     dt**2,         0,         0],
        [        0, (dt**3)/2,         0,         0,     dt**2,         0],
        [        0,         0, (dt**3)/2,         0,         0,     dt**2],
    ])

    return Q


def h_nonlinear(state: np.ndarray, radar_pos: np.ndarray = np.array([0.0, 0.0, 0.0])) -> np.ndarray:
    """
    Non-linear measurement function h(x).
    Converts the 3D Cartesian state into the 3D Spherical radar measurement space.
    
    Args:
        state (np.ndarray): The predicted state vector [x, y, z, vx, vy, vz]^T (6x1).
        
    Returns:
        np.ndarray: The predicted measurement vector [range, doppler, azimuth, elevation]^T (4x1).
                    - Range (r) is the Euclidean distance.
                    - Doppler (r_dot) is the radial velocity.
                    - Azimuth (theta) in radians (usually atan2).
                    - Elevation (phi) in radians (usually arcsin or atan2).
    """

    state = state.flatten()         # Ensures state = column vector
    x, y, z, vx, vy, vz = state

    # Relative coordonates to radar
    dx = x - radar_pos[0]
    dy = y - radar_pos[1]
    dz = z - radar_pos[2]

    # Expression of target's range to radar
    range = math.sqrt(dx**2 + dy**2 + dz**2)

    # Handling case where target position = radar position
    if range < 1e-6:
        return np.zeros((4, 1))

    # Expression of measured velocity (positive when target moving towards radar)
    speed = - (dx*vx + dy*vy + dz*vz) / range

    # Expressions of target's azimuth and elevation (handles zero denominator)
    azimuth = np.arctan2(dy, dx)
    elevation = np.arctan2(dz, np.sqrt(dx**2 + dy**2))

    h_x = np.array([
        [range],
        [speed],
        [azimuth],
        [elevation]
    ])

    return h_x
    


def compute_jacobian(func, state: np.ndarray, *args) -> np.ndarray:
    """
    Dynamically calculates the Jacobian of any function (deterministic).
    func: transformation function (must take an array as first argument)
    state: input vector (1D)
    *args: any additional arguments (ex: radar_pos)
    """

    inf_step = 1e-6
    
    # Nominal evaluation (and ensuring it is a 1D vector)
    state_1d = np.array(state).flatten()
    val_nominal = np.array(func(state_1d, *args)).flatten()
    
    J = np.zeros((len(val_nominal), len(state_1d)))
    
    for i in range(len(state_1d)):
        state_perturbed = np.copy(state_1d).astype(float)
        state_perturbed[i] += inf_step
        
        val_perturbed = np.array(func(state_perturbed, *args)).flatten()
        J[:, i] = (val_perturbed - val_nominal) / inf_step

    return J



def get_initial_state(Z_init: np.ndarray, sigma_array: np.ndarray, sigma_vel: float,
                       antenna_array: np.ndarray, N: int, radar_pos: np.ndarray = np.array([0.0, 0.0, 0.0])):
    """
    Calculates an initial state vector and initial covariance matrix. 
    This function handles various cases, regarding the number of measurements in the measurement vector.
    Full measurements scenario is handled with a change-of-basis matrix and Jacobian matrix. 
    Partial measurements are handled with a Monte Carlo generation, since the transformation is non-linear.

    Args:
        Z_init : initial measurement vector.
        sigma_array : standard deviation of measured quantities.
        antenna_array : characterizes the radar surveillance zone.
        radar_pos : position of the radar in the cartesian coordinate system.
    Returns:
        X_init (np.ndarray) : initial state vector.
        P_init (np.ndarray) : initial covariance matrix.
    """

    r, v_rad, az, el = Z_init
    sigma_r, sigma_v_rad, sigma_az, sigma_el = sigma_array
    az_max, az_min, el_max, el_min, Rmax = antenna_array


    # --- Local coordonate system --- #
    
    # Choice : no tangential speed -> no information used
    v_u = v_rad     
    v_v = 0.0        
    v_w = 0.0

    # Distribution of kinematic uncertainty
    sigma2_vu = sigma_v_rad**2
    sigma2_vv = (sigma_vel**2 - sigma2_vu) / 2      # Assumption : equiprobability of the velocity components along v and w
    sigma2_vw = (sigma_vel**2 - sigma2_vu) / 2

    # ------------------------------- #


    # ====| Measurement : full |==== #

    if not np.any(np.isnan(Z_init)):
        
        # ---| State Vector |--- #

        # 1. Initial Variables
        position_measure = np.array([r, az, el])
        speed_measure = np.array([v_u, v_v, v_w])
        
        # 2. State Variables
        x, y, z = conv_Sphe2Cart(position_measure, radar_pos)
        R = building_local_basis(np.array([x, y, z]), radar_pos)      # Transforms local vectors into global vectors
        vx, vy, vz = R @ speed_measure

        X_init = np.array([[x], [y], [z], [vx], [vy], [vz]])

        # ---| Covariance Matrix |--- #
        
        # Fetching operating point
        vars_0 = np.array([r, az, el, v_u, v_v, v_w])

        # 3. Calculating Jacobian matrix 6D (all dependencies)
        J_full = compute_jacobian(full_state_mapping, vars_0, radar_pos)

        # 4. Covariance matrix of initial variables (assumption : independence)
        Cov_inputs = np.diag([sigma_r**2, sigma_az**2, sigma_el**2, sigma2_vu, sigma2_vv, sigma2_vw])

        # 5. Non-linear uncertainty propagation
        P_init = J_full @ Cov_inputs @ J_full.T

        return X_init, P_init


    # ====| Measurement : range & radial velocity |==== #

    if np.all(~np.isnan([r, v_rad])) and np.all(np.isnan([az, el])):

        # ---| Monte Carlo |--- #

        # Monte Carlo generation
        r_sample   = np.random.normal(r, sigma_r, N)
        az_sample  = np.random.uniform(az_min, az_max, N)
        el_sample  = np.random.uniform(el_min, el_max, N)
        v_u_sample = np.random.normal(v_rad, sigma_v_rad, N)
        v_v_sample = np.random.normal(0, np.sqrt(sigma2_vv), N)
        v_w_sample = np.random.normal(0, np.sqrt(sigma2_vw), N)

        X_init, P_init = monte_carlo_post_process(N, r_sample, az_sample, el_sample, v_u_sample, v_v_sample, v_w_sample, radar_pos)
        
        return X_init, P_init


    # ====| Measurement : range & radial velocity & azimuth |==== #

    if np.all(~np.isnan([r, v_rad, az])) and np.isnan(el):

        # ---| Monte Carlo |--- #
        
        # Monte Carlo generation
        r_sample   = np.random.normal(r, sigma_r, N)
        az_sample  = np.random.normal(az, sigma_az, N)
        el_sample  = np.random.uniform(el_min, el_max, N)
        v_u_sample = np.random.normal(v_rad, sigma_v_rad, N)
        v_v_sample = np.random.normal(0, np.sqrt(sigma2_vv), N)
        v_w_sample = np.random.normal(0, np.sqrt(sigma2_vw), N)

        X_init, P_init = monte_carlo_post_process(N, r_sample, az_sample, el_sample, v_u_sample, v_v_sample, v_w_sample, radar_pos)
        
        return X_init, P_init
    

    # ====| Measurement : range & radial velocity & elevation |==== #

    if np.all(~np.isnan([r, v_rad, el])) and np.isnan(az):

         # ---| Monte Carlo |--- #
        
        # Monte Carlo generation
        r_sample   = np.random.normal(r, sigma_r, N)
        az_sample  = np.random.uniform(az_min, az_max, N)
        el_sample  = np.random.normal(el, sigma_el, N)
        v_u_sample = np.random.normal(v_rad, sigma_v_rad, N)
        v_v_sample = np.random.normal(0, np.sqrt(sigma2_vv), N)
        v_w_sample = np.random.normal(0, np.sqrt(sigma2_vw), N)

        X_init, P_init = monte_carlo_post_process(N, r_sample, az_sample, el_sample, v_u_sample, v_v_sample, v_w_sample, radar_pos)
        
        return X_init, P_init


    # ====| Measurement : azimuth & elevation |==== #

    if np.all(~np.isnan([az, el])) and np.all(np.isnan([r, v_rad])):

        # ---| Monte Carlo |--- #

        # No speed information at all : equiprobability
        sigma2_isotrope = sigma_vel**2 / 3
         
        # Monte Carlo generation
        r_sample   = np.random.uniform(3, Rmax, N)              # using Rmax(az, el) from the antenna pattern would be more precise
        az_sample  = np.random.normal(az, sigma_az, N)
        el_sample  = np.random.normal(el, sigma_el, N)
        v_u_sample = np.random.normal(0, np.sqrt(sigma2_isotrope), N)              
        v_v_sample = np.random.normal(0, np.sqrt(sigma2_isotrope), N)
        v_w_sample = np.random.normal(0, np.sqrt(sigma2_isotrope), N)

        X_init, P_init = monte_carlo_post_process(N, r_sample, az_sample, el_sample, v_u_sample, v_v_sample, v_w_sample, radar_pos)
        
        return X_init, P_init


def conv_Sphe2Cart(meas: np.ndarray, radar_pos: np.ndarray = np.array([0.0, 0.0, 0.0])):
    """Conversion from the spherical coordonate system to the cartesian one."""

    r, az, el = meas
    x0, y0, z0 = radar_pos

    x = x0 + r * np.cos(el) * np.cos(az)
    y = y0 + r * np.cos(el) * np.sin(az)
    z = z0 + r * np.sin(el)

    return x, y, z



def building_local_basis(pos: np.ndarray, radar_pos: np.ndarray = np.array([0.0, 0.0, 0.0])):
    """
    Builds a local basis based on a given position.
    
    Args:
        pos : Global position, cartesian coordinate system ([x, y, z]).
        radar_pos : Position of the radar in the cartesian coordinate system.
    Returns:
        R (np.array) : The associated change-of-basis matrix.
    """

    # 1. Defining target-radar vector
    vector = radar_pos - pos
    u = vector/np.linalg.norm(vector)

    # 2. Building an orthonormal basis 
    if abs(u[0]) < 0.9:
        tmp = np.array([1.0, 0.0, 0.0])
    else:
        tmp = np.array([0.0, 1.0, 0.0])

    v = np.cross(u, tmp)
    v /= np.linalg.norm(v)

    w = np.cross(u, v)

    # 3. Building rotation matrix
    R = np.column_stack((u, v, w))

    return R



def full_state_mapping(vars, radar_pos: np.ndarray = np.array([0.0, 0.0, 0.0])):
    """Linking initial variables to state variables."""
    _r, _az, _el, _vu, _vv, _vw = vars

    # Recalculation of the (noisy) position
    _x, _y, _z = conv_Sphe2Cart(np.array([_r, _az, _el]), radar_pos)
    
    # Recalculation of the rotation matrix (depends on the noisy angles)
    _R_mat = building_local_basis(np.array([_x, _y, _z]), radar_pos)
    
    # Velocity projection 
    _vx, _vy, _vz = _R_mat @ np.array([_vu, _vv, _vw])
    
    return np.array([_x, _y, _z, _vx, _vy, _vz])



def monte_carlo_post_process(N: int, r_sample, az_sample, el_sample, v_u_sample, v_v_sample, v_w_sample, 
                radar_pos: np.ndarray = np.array([0.0, 0.0, 0.0])):
    """Processes the samples of a Monte Carlo simulation. 
        It converts the generated points into state vectors, and calculates the mean state and covariance matrix."""

    state_array = np.zeros((N, 6))

    for i in range(N):
        r_gen  = r_sample[i]
        az_gen = az_sample[i]
        el_gen = el_sample[i]
        vu_gen = v_u_sample[i]
        vv_gen = v_v_sample[i]
        vw_gen = v_w_sample[i]

        vars_gen = np.array([r_gen, az_gen, el_gen, vu_gen, vv_gen, vw_gen])
        X_init_gen = full_state_mapping(vars_gen, radar_pos)
        state_array[i] = X_init_gen.flatten()

    # Calculating mean state vector and covariance matrix
    X_init = np.mean(state_array, axis=0).reshape(6, 1)
    P_init = np.cov(state_array, rowvar=False)

    return X_init, P_init