import math
import numpy as np
from filterpy.kalman import unscented_transform, MerweScaledSigmaPoints
from filterpy.kalman import UnscentedKalmanFilter as UKF


class UKF_sigma_model(object):
    def __init__(self, dt=0.25, w=1.0):
        self.fx_filter_vel = 0.0
        self.fy_filter_vel = 0.0
        self.fz_filter_vel = 0.0
        self.fsigma_filter_vel = 0.0
        self.fpsi_filter_vel = 0.0
        self.fphi_filter_vel = 0.0

        # Q Process Noise Matrix
        self.Q = np.diag([0.5 ** 2, 0.5 ** 2, 0.5 ** 2, 0.1 ** 2, 0.1 ** 2, 0.05 ** 2])  # [x, y, z, sigma, psi, phi]

        # R Measurement Noise Matrix
        self.R = np.diag([8.5 ** 2, 8.5 ** 2, 8.5 ** 2, 1.8 ** 2, 8.5 ** 2, 1.8 ** 2])  # [x, y, z, sigma, psi, phi]

        # Sigma points
        self.sigma = [self.alpha, self.beta, self.kappa] = [0.9, 0.5, -2.0]

        self.w = w
        self.dt = dt
        self.t = 0
        self.number_state_variables = 6

        self.points = MerweScaledSigmaPoints(n=self.number_state_variables,
                                             alpha=self.alpha, beta=self.beta, kappa=self.kappa)

        self.kf = UKF(dim_x=self.number_state_variables, dim_z=self.number_state_variables,
                      dt=self.dt, fx=self.f_bicycle, hx=self.H_bicycle, points=self.points)

        # Q Process Noise Matrix
        self.kf.Q = self.Q

        # R Measurement Noise Matrix
        self.kf.R = self.R

        # H identity
        self.H = np.eye(6)

        self.K = np.zeros((6, 6))  # Kalman gain
        self.y = np.zeros((6, 1))  # residual

        self.kf.x = np.zeros((1, self.number_state_variables))  # Initial state
        self.kf.P = np.eye(self.number_state_variables) * 10  # Covariance matrix

    def get_state(self):
        return self.kf.x

    def reset_filter(self):
        self.kf.x = np.zeros((1, self.number_state_variables))
        # self.kf.k = np.zeros((6, 6))
        self.kf.P = np.eye(self.number_state_variables) * 10

    def prediction(self, U):
        self.kf.predict(fx_args=U)

    def update(self, Z):
        Z_t = np.zeros((1, 6))
        Z_t[0, 0] = Z[0]  # x
        Z_t[0, 1] = Z[1]  # y
        Z_t[0, 2] = Z[2]  # z
        Z_t[0, 3] = Z[3]  # sigma
        Z_t[0, 4] = Z[4]  # psi
        Z_t[0, 5] = Z[5]  # phi

        self.kf.update(Z_t[0])
        #self.K = self.kf.k

    def update_R(self, R_std):
        # TODO: Implement method
        pass

    def update_Q(self, Q_std):
        # TODO: Implement method
        pass

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

    def f_bicycle(self, x, dt, U):
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
        # ident_matrix = np.eye(len(x))
        sensor_out = self.H.dot(x)

        return sensor_out
