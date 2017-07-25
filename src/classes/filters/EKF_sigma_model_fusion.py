import math
import numpy as np


class EKF_sigma_model_fusion(object):
    """ Implements an EKF to bicycle model
    Args:
        wheel_distance: Distance between wheels
        dt: sample time
        alpha: fading memory filter (a = 1 to disable)
    """

    def __init__(self, wheel_distance=1.2, dt=0.1, alpha=1.0):
        self.w = wheel_distance  # Set the distance between the wheels
        self.dt = dt
        self.alpha = alpha

        self.xs = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Set the initial state
        # Initial covariance matrix
        self.P = np.eye(6) * 100
        self.K = np.zeros((6, 6))  # Kalman gain

        self.P_filter = self.P
        self.y = np.zeros((6, 1))

        # process noise covariance Q -----------------------------------------------------------------------
        # Maximum change (acceleration) for given dataset
        max_acc_v = 4.26354980469
        max_acc_phi_dot = 0.201111111111
        max_acc_delta_dot = 0.122222222223

        sigma_v = (max_acc_v * self.dt) ** 2
        sigma_phi_dot = (max_acc_phi_dot * self.dt) ** 2
        sigma_delta_dot = (max_acc_delta_dot * self.dt) ** 2

        self.Q_std = [sigma_v, sigma_phi_dot, sigma_delta_dot]  # v, phi_dot, delta_dot

        # Set the process noise covariance
        self.Q = np.diag([self.Q_std[0],  # v
                          self.Q_std[1],  # phi_dot
                          self.Q_std[2]  # delta_dot
                          ])

        # measurement noise covariance R ---------------------------------------------------------------------
        self.R_std = [0.1 ** 2, 0.1 ** 2,  # x
                      0.1 ** 2, 0.1 ** 2,  # y
                      0.1 ** 2, 0.1 ** 2, 0.1 ** 2,  # z
                      0.001 ** 2, 0.001 ** 2, 0.005 ** 2]  # delta - psi - phi

        # Set the measurement noise covariance
        self.R = np.diag([self.R_std[0],  # xf
                          self.R_std[1],  # xr
                          self.R_std[2],  # yf
                          self.R_std[3],  # yr
                          self.R_std[4],  # zf
                          self.R_std[5],  # zr
                          self.R_std[6],  # za
                          self.R_std[7],  # sigma
                          self.R_std[8],  # psi
                          self.R_std[9]])  # phi

        # Linear relationship H -  z = Hx
        self.H = np.zeros((10, 6))  # 10 measurements x 6 state variables
        [self.H[0, 0], self.H[1, 0]] = [1.0, 1.0]  # x
        [self.H[2, 1], self.H[3, 1]] = [1.0, 1.0]  # y
        [self.H[4, 2], self.H[5, 2], self.H[6, 2]] = [1.0, 1.0, 1.0]  # z
        [self.H[7, 3], self.H[8, 4], self.H[9, 5]] = [1.0, 1.0, 1.0]  # sigma - psi - phi

    def get_state(self):
        """ Returns filter state """
        return self.xs

    def reset_filter(self):
        self.xs = self.xs * 0.0
        self.K = np.zeros((6, 6))
        self.P = np.eye(6) * 100.0

    def Fx(self, xs, u):
        """ Linearize the system with the Jacobian of the x """
        F_result = np.eye(len(xs))

        v = u[0]
        phi_dot = u[1]
        delta_dot = u[2]

        sigma = xs[3]
        psi = xs[4]
        phi = xs[5]
        t = self.dt

        F04 = -t * v * np.sin(psi)
        F14 = t * v * np.cos(psi)
        F33 = (2 * t * delta_dot * sigma * self.w) + 1
        F43 = (t * v) / np.cos(phi)
        F45 = t * sigma * v * np.sin(phi) / np.cos(phi) ** 2

        F_result[0, 4] = F04
        F_result[1, 4] = F14
        F_result[3, 3] = F33
        F_result[4, 3] = F43
        F_result[4, 5] = F45

        return F_result

    def Fu(self, xs, u):
        """ Linearize the system with the Jacobian of the u """
        v = u[0]
        phi_dot = u[1]
        delta_dot = u[2]

        sigma = xs[3]
        psi = xs[4]
        phi = xs[5]
        t = self.dt

        V_result = np.zeros((len(xs), len(u)))

        V00 = t * np.cos(psi)
        V10 = t * np.sin(psi)
        V32 = (t / self.w) * ((sigma ** 2) * (self.w ** 2) + 1)
        V40 = t * sigma / np.cos(phi)
        V51 = t

        V_result[0, 0] = V00
        V_result[1, 0] = V10
        V_result[3, 2] = V32
        V_result[4, 0] = V40
        V_result[5, 1] = V51

        return V_result

    def f(self, xs, u):
        """ Estimate the non-linear state of the system """
        v = u[0]
        phi_dot = u[1]
        delta_dot = u[2]

        sigma = xs[3]
        psi = xs[4]
        phi = xs[5]
        t = self.dt

        fxu_result = np.zeros((len(xs), 1))

        fxu_result[0] = xs[0] + t * v * np.cos(psi)
        fxu_result[1] = xs[1] + t * v * np.sin(psi)
        fxu_result[2] = xs[2]
        fxu_result[3] = xs[3] + (t * phi_dot / self.w) * ((sigma ** 2) * (self.w ** 2) + 1)
        fxu_result[4] = xs[4] + t * v * sigma / np.cos(phi)
        fxu_result[5] = xs[5] + t * phi_dot

        return fxu_result

    def h(self, x):
        """ takes a state variable and returns the measurement
        that would correspond to that state. """
        sensor_out = np.zeros((10, 1))
        sensor_out[0] = x[0]
        sensor_out[1] = x[0]
        sensor_out[2] = x[1]
        sensor_out[3] = x[1]
        sensor_out[4] = x[2]
        sensor_out[5] = x[2]
        sensor_out[6] = x[2]
        sensor_out[7] = x[3]  # sigma
        sensor_out[8] = x[4]  # psi
        sensor_out[9] = x[5]  # phi

        return sensor_out

    def prediction(self, u):
        """ Execute prediction step of filter using input u """
        x_ = self.xs
        P_ = self.P
        self.xs = self.f(x_, u)
        self.P_filter = self.Fx(x_, u).dot(P_).dot((self.Fx(x_, u)).T) + \
                        self.Fu(x_, u).dot(self.Q).dot((self.Fu(x_, u)).T)

        self.P = (self.alpha ** 2.0) * self.P_filter

    def update(self, z):
        """ Update the Kalman Prediction using the measurement z """
        self.y = z - self.h(self.xs)
        self.K = self.P.dot(self.H.T).dot(np.linalg.inv(self.H.dot(self.P).dot(self.H.T) + self.R))

        self.xs = self.xs + self.K.dot(self.y)
        self.P = (np.eye(len(self.xs)) - self.K.dot(self.H)).dot(self.P)
