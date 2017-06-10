import math
import numpy as np


class EKF_sigma_model(object):
    """Implements an EKF to bicycle model"""

    def __init__(self, xs, P, R_std, Q_std, wheel_distance=1.2, dt=0.1, alpha=None):
        self.w = wheel_distance  # Set the distance between the wheels
        self.xs = xs * 0.0  # Set the initial state
        self.P = P  # Set the initial Covariance
        self.P_filter = P
        self.dt = dt
        self.R_std = R_std
        self.Q_std = Q_std
        self.alpha = alpha
        self.K = np.zeros((6, 6))  # Kalman gain
        self.e = 0.0
        self.S = []
        self.y = np.zeros((6, 1))

        # Set the process noise covariance
        self.Q = np.diag([self.Q_std[0],  # v
                          self.Q_std[1],  # phi_dot
                          self.Q_std[2]  # delta_dot
                          ])

        # Set the measurement noise covariance
        self.R = np.diag([self.R_std[0],  # x
                          self.R_std[1],  # y
                          self.R_std[2],  # z
                          self.R_std[3],  # sigma
                          self.R_std[4],  # psi
                          self.R_std[5]])  # phi

        self.update_R(R_std)
        self.update_Q(Q_std)

        # Linear relationship H -  z = Hx
        self.H = np.eye(6)

    def update_R(self, R_std):
        self.R_std = R_std

        # Set the measurement noise covariance
        self.R = np.diag([self.R_std[0],  # x
                          self.R_std[1],  # y
                          self.R_std[2],  # z
                          self.R_std[3],  # sigma
                          self.R_std[4],  # psi
                          self.R_std[5]])  # phi

    def update_Q(self, Q_std):
        self.Q_std = Q_std

        # Set the process noise covariance
        self.Q = np.diag([self.Q_std[0],  # v
                          self.Q_std[1],  # phi_dot
                          self.Q_std[2]  # delta_dot
                          ])

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
        F33 = (2.0 * t * delta_dot * sigma * self.w) + 1.0
        F43 = (t * v) / np.cos(phi)
        F45 = t * sigma * v * np.sin(phi) / np.cos(phi) ** 2.0

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
        V32 = (t / self.w) * ((sigma ** 2.0) * (self.w ** 2.0) + 1.0)
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
        fxu_result[3] = xs[3] + (t * delta_dot / self.w) * ((sigma ** 2.0) * (self.w ** 2.0) + 1)
        fxu_result[4] = xs[4] + t * v * sigma / np.cos(phi)
        fxu_result[5] = xs[5] + t * phi_dot

        return fxu_result

    def h(self, x):
        """ takes a state variable and returns the measurement
        that would correspond to that state. """
        ident_matrix = np.eye(len(x))
        sensor_out = ident_matrix.dot(x)

        return sensor_out

    def prediction(self, u):
        x_ = self.xs
        P_ = self.P
        self.xs = self.f(x_, u)
        self.P_filter = self.Fx(x_, u).dot(P_).dot((self.Fx(x_, u)).T) + \
                        self.Fu(x_, u).dot(self.Q).dot((self.Fu(x_, u)).T)

        self.P = (self.alpha**2.0) * self.P_filter

    def update(self, z):
        """Update the Kalman Prediction using the measurement z"""
        self.y = z - self.h(self.xs)

        self.S = self.H.dot(self.P_filter).dot(self.H.T) + self.R
        self.e = self.y.T.dot(np.linalg.inv(self.S)).dot(self.y)

        self.K = self.P.dot(self.H.T).dot(np.linalg.inv(self.H.dot(self.P).dot(self.H.T) + self.R))

        self.xs = self.xs + self.K.dot(self.y)
        self.P = (np.eye(len(self.xs)) - self.K.dot(self.H)).dot(self.P)
