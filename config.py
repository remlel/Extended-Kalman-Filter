from dataclasses import dataclass, field
import numpy as np

@dataclass
class TrackerConfig:
    """
    Dataclass containing every user input parameters. 
    Those parameters are either tracking parameters or dataset/scenario related parameters.
    """

    # ---| Tracking Parameters |--- #

    # Dynamic Gating Threshold (Chi-2 law at 95%)
    chi2_thresholds  : dict = field(default_factory=lambda: {1: 3.84, 2: 5.99, 3: 7.81, 4: 9.49})
    # Number of generated points for Monte Carlo simulation
    N                : int  = 1000
    # Maximal number of missed detections before killing track
    max_missed_detect: int  = 10

    # ---| Dataset Parameters |--- #

    # Time Step
    dt         : float     | None = None 
    # Standard Deviation of radar measuremnts
    sigma_r    : float      = 1                      # Using the 1 m range resolution
    sigma_v_rad: float      = 0.76                   # Using the 0.76 m/s velocity resolution
    sigma_az   : float      = np.radians(3)          # Using the 3° standard deviation 
    sigma_el   : float      = np.radians(3)          # Using the 3° standard deviation
    # Process Noise Standard Deviation (Data-Driven Tuning) [m/s²]
    sigma_acc  : float      = 6.79                   # Square of the trace of the covariance matrix
    # Speed Standard Deviation (Data-Driven Tuning) [m/s]
    sigma_vel  : float      = 8.75                   # Square of the trace of the covariance matrix
    # Radar Position (Boresight direction = x axis)
    radar_pos  : np.ndarray = field(default_factory=lambda:np.array([0.0, 0.0, 0.0]))
    # Antenna
    az_max     : float      = np.radians(45)
    az_min     : float      = np.radians(-45)
    el_max     : float      = np.radians(45)
    el_min     : float      = np.radians(-45)
    Rmax       : float      = 150                    # max detection range on the boresight axis
     

    @property
    def sigma_array(self) -> np.ndarray:
        return np.array([
            self.sigma_r,
            self.sigma_v_rad,
            self.sigma_az,
            self.sigma_el
        ])

    @property
    def R_elementary(self) -> np.ndarray:
        """ Measurement Noise Covariance Matrix"""
        return np.diag([
            self.sigma_r**2, 
            self.sigma_v_rad**2, 
            self.sigma_az**2, 
            self.sigma_el**2
        ])

    @property
    def antenna_array(self) -> np.ndarray:
        return np.array([
            self.az_max, 
            self.az_min, 
            self.el_max, 
            self.el_min, 
            self.Rmax
        ])