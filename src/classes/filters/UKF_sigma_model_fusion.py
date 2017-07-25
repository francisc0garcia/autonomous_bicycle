import math
import numpy as np
from filterpy.kalman import unscented_transform, MerweScaledSigmaPoints
from filterpy.kalman import UnscentedKalmanFilter as UKF


class UKF_sigma_model_fusion(object):
    """ Implements an UKF to bicycle model
    Args:
        dt: sample time
        w: Distance between wheels
    """

    def __init__(self, dt=0.25, w=1.0):
        self.fx_filter_vel = 0.0
        self.fy_filter_vel = 0.0
        self.fz_filter_vel = 0.0
        self.fsigma_filter_vel = 0.0
        self.fpsi_filter_vel = 0.0
        self.fphi_filter_vel = 0.0
        self.U_init = []

        self.w = w
        self.dt = dt
        self.t = 0
        self.number_state_variables = 6

        # Q Process Noise Matrix [x, y, z, sigma, psi, phi]
        self.Q = np.diag([0.5 ** 2, 0.5 ** 2, 0.5 ** 2, 0.1 ** 2, 0.1 ** 2, 0.05 ** 2])

        # R Measurement Noise Matrix [xf, xr, yf, yr, zf, zr, za, delta, psi, phi]
        self.R = np.diag([3.5 ** 2, 3.5 ** 2,  # x
                          3.5 ** 2, 3.5 ** 2,  # y
                          1.5 ** 2, 1.5 ** 2, 3.5 ** 2,  # z
                          0.05 ** 2, 0.05 ** 2, 0.5 ** 2])  # delta - psi - phi

        # Init Sigma points
        self.sigma = [self.alpha, self.beta, self.kappa] = [0.7, 2.0, -2.0]

        self.points = MerweScaledSigmaPoints(n=self.number_state_variables,
                                             alpha=self.alpha, beta=self.beta, kappa=self.kappa)

        # Create UKF filter based on filterpy library
        self.kf = UKF(dim_x=self.number_state_variables, dim_z=10, dt=self.dt,
                      fx=self.f_bicycle, hx=self.H_bicycle, points=self.points)

        # Q Process Noise Matrix
        self.kf.Q = self.Q

        # R Measurement Noise Matrix
        self.kf.R = self.R
        self.kf.P = np.eye(self.number_state_variables) * 10  # Covariance matrix
        self.P = self.kf.P

        # only for consistency
        self.K = np.zeros((6, 6))  # Kalman gain
        self.y = np.zeros((6, 1))  # residual
        self.H = np.zeros((10, 6))  # 10 measurements x 6 state variables

    def get_state(self):
        """ Returns filter state """
        return self.kf.x

    def reset_filter(self):
        self.kf.x = np.zeros((1, self.number_state_variables))
        # self.kf.k = np.zeros((6, 6))
        self.kf.P = np.eye(self.number_state_variables) * 10
        self.P = self.kf.P

    def prediction(self, U):
        """ Execute prediction step of filter using input u """
        self.kf.predict(fx_args=U)
        self.P = self.kf.P

    def update(self, Z):
        """ Update the Kalman Prediction using the measurement z """
        Z_t = np.zeros((1, 10))
        Z_t[0, 0] = Z[0]  # xf
        Z_t[0, 1] = Z[1]  # xr
        Z_t[0, 2] = Z[2]  # yf
        Z_t[0, 3] = Z[3]  # yr
        Z_t[0, 4] = Z[4]  # zf
        Z_t[0, 5] = Z[5]  # zr
        Z_t[0, 6] = Z[6]  # za
        Z_t[0, 7] = Z[7]  # sigma
        Z_t[0, 8] = Z[8]  # psi
        Z_t[0, 9] = Z[9]  # phi

        self.kf.update(Z_t[0])
        self.P = self.kf.P

    def fx_filter(self, x, t):
        return self.fx_filter_vel

    def fy_filter(self, y, t):
        return self.fy_filter_vel

    def fz_filter(self, y, t):
        return self.fz_filter_vel

    def fsigma_filter(self, y, t):
        return self.fsigma_filter_vel

    def fpsi_filter(self, yaw, t):
        return self.fpsi_filter_vel

    def fphi_filter(self, yaw, t):
        return self.fphi_filter_vel

    def rk4(self, y, x, dx, f):
        k1 = dx * f(y, x)
        k2 = dx * f(y + 0.5 * k1, x + 0.5 * dx)
        k3 = dx * f(y + 0.5 * k2, x + 0.5 * dx)
        k4 = dx * f(y + k3, x + dx)

        return y + (k1 + 2 * k2 + 2 * k3 + k4) / 6.

    def f_bicycle(self, x, dt, U=None):
        """ Compute next bicycle state based on current state X and inputs U """
        if U is None:
            U = self.U_init

        x_out = x
        [x_ini, y_ini, z_ini, sigma_ini, psi_ini, phi_ini] = x

        v_ini = U[0]
        phi_dot = U[1]
        delta_dot = U[2]

        # Solve diff equation by approximation
        x = self.rk4(x_ini, self.t, self.dt, self.fx_filter)
        y = self.rk4(y_ini, self.t, self.dt, self.fy_filter)
        z = self.rk4(z_ini, self.t, self.dt, self.fz_filter)
        sigma = self.rk4(sigma_ini, self.t, self.dt, self.fsigma_filter)
        psi = self.rk4(psi_ini, self.t, self.dt, self.fpsi_filter)
        phi = self.rk4(phi_ini, self.t, self.dt, self.fphi_filter)

        self.fx_filter_vel = math.cos(psi) * v_ini
        self.fy_filter_vel = math.sin(psi) * v_ini
        self.fz_filter_vel = 0
        self.fsigma_filter_vel = (phi_dot / self.w) * (1 + (self.w ** 2) * (sigma_ini ** 2))
        self.fpsi_filter_vel = (v_ini * sigma_ini) / math.cos(phi_ini)
        self.fphi_filter_vel = phi_dot

        x_out[0] = x
        x_out[1] = y
        x_out[2] = z
        x_out[3] = sigma
        x_out[4] = psi
        x_out[5] = phi

        return x_out

    def H_bicycle(self, x):
        """ takes a state variable and returns the measurement
        that would correspond to that state. """
        sensor_out = self.H.dot(x)

        return sensor_out
