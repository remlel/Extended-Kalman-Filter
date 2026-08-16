import numpy as np

class ExtendedKalmanFilter:
    """
    Generic mathematical engine of an Extended Kalman Filter (EKF). 
    This class contains no business logic (radar, drone, etc.), only the state and covariance equations.
    """

    def __init__(self, initial_state: np.ndarray, initial_covariance: np.ndarray):
        """
        Defines initial state and uncertainties.
        
        Args:
            initial_state (np.ndarray): Initial state vector (ex: 6x1)
            initial_covariance (np.ndarray): Initial covariance matrix P (ex: 6x6)
        """

        self.x = initial_state
        self.p = initial_covariance


    def predict(self, F: np.ndarray, Q: np.ndarray) -> None:
        """
        Phase 1 : Prediction (Time Update).
        Advances the state and covariance over time.
        
        Args:
            F (np.ndarray): State transition matrix. 
            Q (np.ndarray): Covariance matrix of process noise.
            
        """

        # Prediction of the next state
        self.x = F @ self.x

        # Prediction of the next covariance matrix
        self.p = F @ self.p @ F.T + Q
        

    def update(self, z: np.ndarray, h_x: np.ndarray, H: np.ndarray, S: np.ndarray) -> None:
        """
        Phase 2 : Correction (Measurement Update).
        Corrects the prediction with the latest radar measurement.
        
        Args:
            z (np.ndarray): Vector of the radar measurement (ex: 4x1).
            h_x (np.ndarray): The prediction associated with the measurement (State projected 
                              in the measurment space through the non linear function).
            H (np.ndarray): Jacobian matrix evaluated at the predicted state.
            S (np.ndarray): Covariance innovation matrix.
        """

        # Measurement Innovation 
        y = z - h_x  

        # Kalman Gain
        K = self.p @ H.T @ np.linalg.inv(S)

        # State Update
        self.x = self.x + K @ y

        # Covariance Update
        I = np.eye(self.p.shape[0])
        self.p = (I - K @ H) @ self.p


    def covariance_innovation_S(self, H: np.ndarray, R: np.ndarray):
        """
        Calculates the covariance innovation : indicator of the plausible innovation measurements.

        Args:
            H (np.ndarray): Jacobian matrix evaluated at the predicted state.
            R (np.ndarray): Covariance matrix of the measurement noise.
        Returns:
            S (np.ndarray): Covariance innovation matrix.
        """

        S = H @ self.p @ H.T + R

        return S