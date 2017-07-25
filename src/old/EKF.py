import math
import numpy as np
from numpy.random import randn


class EKF(object):
    """Implements an EKF to bicycle model"""

    def __init__(self, xs, P, M, std, wheel_distance=0.2, dt=0.1):
        self.L = wheel_distance  # Set the distance between the wheels
        self.xs = xs  # Set the initial state
        self.P = P  # Set the initial Covariance
        self.M = M  # Set the process noise covariance
        self.dt = dt
        self.std = std

        # Set the measurement noise covariance
        self.R = np.diag([self.std[0] * randn(),  # x
                          self.std[1] * randn(),  # y
                          self.std[2] * randn(),  # z
                          self.std[3] * randn(),  # v
                          self.std[4] * randn(),  # psi
                          self.std[5] * randn(),  # phi
                          self.std[6] * randn()])  # delta

        # Linear relationship H -  z = Hx
        self.H = np.eye(len(self.xs))

    def Fx(self, xs, u):
        """ Linearize the system with the Jacobian of the x """
        a = u[0]
        beta = u[1] + 0.00001  # prevent zero division

        v = xs[3]
        psi = xs[4]
        phi = xs[5]
        delta = xs[6]

        d = v * self.dt
        R = d / beta

        F_result = np.eye(len(xs))

        dt_beta = self.dt / beta

        F03 = -dt_beta * math.sin(psi) + dt_beta * math.sin(psi + beta)
        F04 = -R * np.cos(psi) + R * np.cos(psi + beta)
        F13 = dt_beta * np.cos(psi) - dt_beta * np.cos(psi + beta)
        F14 = -R * np.sin(psi) + R * np.sin(psi + beta)

        F_result[0, 3] = F03
        F_result[0, 4] = F04
        F_result[1, 3] = F13
        F_result[1, 4] = F14

        return F_result

    def Fu(self, xs, u):
        """ Linearize the system with the Jacobian of the u """
        a = u[0]
        beta = u[1] + 0.00001  # prevent zero division

        v = xs[3]
        psi = xs[4]
        phi = xs[5]
        delta = xs[6]

        t = self.dt
        d = v * t
        R = d / beta

        V_result = np.zeros((7, 4))

        V01 = (t * v / beta) * np.cos(psi + beta) + (t * v / beta ** 2) * np.sin(psi) - (t * v / beta ** 2) * np.sin(
            psi + beta)
        V11 = (t * v / beta) * np.cos(psi + beta) - (t * v / beta ** 2) * np.sin(psi) + (t * v / beta ** 2) * np.sin(
            psi + beta)
        V30 = t
        V41 = 1.0
        V52 = t
        V63 = t

        V_result[0, 1] = V01
        V_result[1, 1] = V11
        V_result[3, 0] = V30
        V_result[4, 1] = V41
        V_result[5, 2] = V52
        V_result[6, 3] = V63

        return V_result

    def f(self, xs, u):
        """ Estimate the non-linear state of the system """
        a = u[0]
        beta = u[1] + 0.00001  # prevent zero division
        phi_dot = u[2]
        delta_dot = u[3]

        v = xs[3]
        psi = xs[4]
        phi = xs[5]
        delta = xs[6]

        t = self.dt
        d = v * t
        R = d / beta

        fxu_result = np.zeros((7, 1))

        fxu_result[0] = xs[0] - R * np.sin(psi) + R * np.sin(psi + beta)
        fxu_result[1] = xs[1] + R * np.cos(psi) - R * np.cos(psi + beta)
        fxu_result[2] = xs[2]
        fxu_result[3] = xs[3] + a * t
        fxu_result[4] = xs[4] + beta
        fxu_result[5] = xs[5] + phi_dot * t
        fxu_result[6] = xs[6] + delta_dot * t

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
        self.P = self.Fx(x_, u).dot(P_).dot((self.Fx(x_, u)).T) + \
                 self.Fu(x_, u).dot(self.M).dot((self.Fu(x_, u)).T)

    def update(self, z):
        """Update the Kalman Prediction using the meazurement z"""
        y = z - self.h(self.xs)
        K = self.P.dot(self.H.T).dot(np.linalg.inv(self.H.dot(self.P).dot(self.H.T) + self.R))
        self.xs = self.xs + K.dot(y)
        self.P = (np.eye(len(self.xs)) - K.dot(self.H)).dot(self.P)

    def normalize_angle(self, x):
        x = x % (2 * np.pi)  # force in range [0, 2 pi)
        # if x > np.pi:          # move to [-pi, pi)
        #    x -= 2 * np.pi
        return x
