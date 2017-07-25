import math
import numpy as np
from numpy.random import randn, uniform

from filterpy.monte_carlo import systematic_resample, residual_resample
from filterpy.monte_carlo import stratified_resample, multinomial_resample


class ParticleFilter_sigma_model(object):
    """Implements a particle filter to bicycle kinematic model
    Args:
        dt: sample time
        w: Distance between wheels
    """

    def __init__(self, dt=0.25, w=1.02):
        ang = 2 * np.pi
        initial_mean_x = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        initial_std_x = [0.5, 0.5, 0.05, 0.5 * ang, 0.5 * ang, 0.5 * ang]
        process_std = [0.3, 0.3, 0.1, 0.005 * ang, 0.005 * ang, 0.005 * ang]

        self.n_particles = 1000
        self.n_states = 6
        self.p = np.zeros((self.n_particles, self.n_states))
        self.weights = np.zeros(self.n_particles)

        self.dt = dt
        self.w = w
        self.process_std = process_std
        self.R = 0.03

        # compatibility only
        self.K = np.zeros((6, 6))  # Kalman gain
        self.y = np.zeros((6, 1))  # residual
        self.H = np.eye(6)

        # create particles and weights
        if initial_mean_x is not None and initial_std_x is not None:
            self.create_gaussian_particles(initial_mean_x=initial_mean_x,
                                           initial_std_x=initial_std_x,
                                           n_particles=self.n_particles)
        else:
            min_range = [-0.2, -0.2, -0.2, -0.02, -0.02, -0.02]
            max_range = [0.2, 0.2, 0.2, 0.5, 0.02, 0.02]

            self.create_uniform_particles(min_range=min_range, max_range=max_range, n_particles=self.n_particles)

    def get_state(self):
        [mean, var] = self.estimate()
        return mean

    def reset_filter(self):
        self.p = np.zeros((self.n_particles, self.n_states))
        self.weights = np.zeros(self.n_particles)

    def create_gaussian_particles(self, initial_mean_x, initial_std_x, n_particles=100):
        N = n_particles
        self.p = np.empty((N, 6))

        self.p[:, 0] = initial_mean_x[0] + (randn(N) * initial_std_x[0])  # x
        self.p[:, 1] = initial_mean_x[1] + (randn(N) * initial_std_x[1])  # y
        self.p[:, 2] = initial_mean_x[2] + (randn(N) * initial_std_x[2])  # z
        self.p[:, 3] = initial_mean_x[3] + (randn(N) * initial_std_x[3])  # sigma
        self.p[:, 4] = initial_mean_x[4] + (randn(N) * initial_std_x[4])  # psi
        self.p[:, 5] = initial_mean_x[5] + (randn(N) * initial_std_x[5])  # phi

        # Constrain angles between 0 and 2*pi
        self.p[:, 4] %= 2 * np.pi
        self.p[:, 5] %= 2 * np.pi

    def create_uniform_particles(self, min_range, max_range, n_particles=100):
        N = n_particles

        self.p = np.empty((N, 6))

        self.p[:, 0] = uniform(min_range[0], max_range[0], size=N)  # x
        self.p[:, 1] = uniform(min_range[1], max_range[1], size=N)  # y
        self.p[:, 2] = uniform(min_range[2], max_range[2], size=N)  # z
        self.p[:, 3] = uniform(min_range[3], max_range[3], size=N)  # sigma
        self.p[:, 4] = uniform(min_range[4], max_range[4], size=N)  # psi
        self.p[:, 5] = uniform(min_range[5], max_range[5], size=N)  # phi

        # Constrain angles between 0 and 2*pi
        self.p[:, 4] %= 2 * np.pi
        self.p[:, 5] %= 2 * np.pi

    def prediction(self, U):
        """ move according to control input u (heading change, velocity)
        with noise Q (std heading change, std velocity)`"""
        N = len(self.p)

        v = U[0]
        phi_dot = U[1]
        delta_dot = U[2]

        # add process noise
        nx = (randn(N) * self.process_std[0])  # x
        ny = (randn(N) * self.process_std[1])  # y
        nz = (randn(N) * self.process_std[2])  # z
        nsigma = (randn(N) * self.process_std[3])  # sigma
        npsi = (randn(N) * self.process_std[4])  # psi
        nphi = (randn(N) * self.process_std[5])  # phi

        # move particles
        self.p[:, 0] += (self.dt * v * np.cos(self.p[:, 4])) + nx
        self.p[:, 1] += (self.dt * v * np.sin(self.p[:, 4])) + ny
        self.p[:, 1] += 0 + nz
        self.p[:, 3] += self.dt * (delta_dot / self.w) * (1 + (self.w ** 2) * (self.p[:, 3] ** 2)) + nsigma
        self.p[:, 4] += (self.dt * (v * self.p[:, 3]) / np.cos(self.p[:, 5])) + npsi
        self.p[:, 5] += self.dt * phi_dot + nphi

    def update(self, z):
        N = len(self.p)

        self.weights = np.ones(N)
        diff = self.p - z.T

        # if no GPS measurement, avoid penalize particle position (x, y, z)
        diff[:, 0] = 0.0 if self.H[0, 0] == 0 else diff[:, 0]
        diff[:, 1] = 0.0 if self.H[1, 1] == 0 else diff[:, 1]
        diff[:, 2] = 0.0 if self.H[2, 2] == 0 else diff[:, 2]

        error = np.linalg.norm(diff, axis=1)

        self.weights = (1 / np.sqrt(2.0 * np.pi * (self.R ** 2))) * np.exp(-(error.T ** 2) / (self.R ** 2) / 2.0)

        self.weights += 1.e-300  # avoid round-off to zero
        self.weights /= sum(self.weights)  # normalize

        # Resample
        if self.neff() < self.n_particles / 2:
            # indexes = residual_resample(self.weights)
            # indexes = stratified_resample(self.weights)
            # indexes = systematic_resample(self.weights)
            indexes = multinomial_resample(self.weights)
            # indexes = self.resampling_wheel()
            self.resample_from_index(indexes)

    def estimate(self):
        """returns mean and variance of the weighted particles"""
        pos = self.p[:, 0:6]
        mean = np.average(pos, weights=self.weights, axis=0)
        var = np.average((pos - mean) ** 2, weights=self.weights, axis=0)
        return mean, var

    def resample_from_index(self, indexes):
        self.p[:] = self.p[indexes]
        self.weights[:] = self.weights[indexes]
        self.weights /= np.sum(self.weights)

    def resampling_wheel(self):
        N = len(self.p)
        index_final = []
        index = int(np.random.random() * N)
        beta = 0.0
        mw = np.max(self.weights)
        for i in range(N):
            beta += np.random.random() * 2.0 * mw
            while beta > self.weights[index]:
                beta -= self.weights[index]
                index = (index + 1) % N
            index_final.append(index)
        return index_final

    def neff(self):
        return 1. / np.sum(np.square(self.weights))

    def normalize_angle(self, x):
        x = x % (2 * np.pi)  # force in range [0, 2 pi)
        # if x > np.pi:          # move to [-pi, pi)
        #    x -= 2 * np.pi
        return x
