import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

class KalmanFilterWrapper:
    """
    A simple wrapper for a 1D Kalman Filter.
    It tracks the position and velocity of a single coordinate.
    """
    def __init__(self, R=5, Q_var=0.1):
        """
        Initializes the Kalman Filter.
        :param R: Measurement noise covariance. How much we trust the measurement.
                  Lower R means more trust in the raw detected value.
        :param Q_var: Process noise variance. How much we expect the object's acceleration to vary.
        """
        # Create a Kalman Filter for a 1D system
        # State variables are [position, velocity], so dim_x = 2
        # We only measure position, so dim_z = 1
        self.kf = KalmanFilter(dim_x=2, dim_z=1)

        # State Transition Matrix [F]
        # x_k = F * x_{k-1}
        # [pos_k] = [1, 1] * [pos_{k-1}]
        # [vel_k]   [0, 1]   [vel_{k-1}]
        # Assumes constant velocity model with dt=1
        self.kf.F = np.array([[1., 1.],
                              [0., 1.]])

        # Measurement Function [H]
        # z_k = H * x_k
        # It maps the state space into the measurement space.
        # We only measure position, so H = [1, 0]
        self.kf.H = np.array([[1., 0.]])

        # Measurement Noise Covariance [R]
        # Represents the uncertainty of the measurement.
        self.kf.R = np.array([[R]])

        # Process Noise Covariance [Q]
        # Represents the uncertainty in the process model (e.g., unexpected acceleration)
        self.kf.Q = Q_discrete_white_noise(dim=2, dt=1.0, var=Q_var)

        # Initial State Covariance [P]
        # High initial uncertainty.
        self.kf.P *= 1000.

    def predict(self):
        """
        Predict the next state.
        """
        self.kf.predict()

    def update(self, measurement):
        """
        Update the filter with a new measurement and return the filtered position.
        :param measurement: The new raw measurement (e.g., detected center_x).
        :return: The filtered position.
        """
        self.kf.update(np.array([measurement]))
        # The filtered position is the first element of the state vector 'x'
        return self.kf.x[0, 0]
