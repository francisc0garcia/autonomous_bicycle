#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
@author: Francisco Javier Garcia Rosas 2017
"""
import math
import numpy as np
from numpy.random import randn
import copy
import matplotlib.pyplot as plt
import BicycleUtils as bu

plt.rcParams['figure.figsize'] = (12, 7)


def rk4(y, x, dx, f):
    """computes 4th order Runge-Kutta for dy/dx.
    y is the initial value for y
    x is the initial value for x
    dx is the difference in x (e.g. the time step)
    f is a callable function (y, x) that you supply to 
      compute dy/dx for the specified values.
    """
    k1 = dx * f(y, x)
    k2 = dx * f(y + 0.5 * k1, x + 0.5 * dx)
    k3 = dx * f(y + 0.5 * k2, x + 0.5 * dx)
    k4 = dx * f(y + k3, x + dx)

    return y + (k1 + 2 * k2 + 2 * k3 + k4) / 6.


class BicycleInput(object):
    """
    Container for input variables
    """

    def __init__(self):
        self.v = 0.0
        self.phi_dot = 0.0
        self.delta_dot = 0.0


class BicycleState(object):
    """
    Container for state variables
    """

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.delta = 0.0
        self.psi = 0.0
        self.phi = 0.0


class BicycleMeasurement(object):
    """
    Container for measurements
    """

    def __init__(self):
        [self.xf, self.xr] = [0.0, 0.0]
        [self.yf, self.yr] = [0.0, 0.0]
        [self.zf, self.zr, self.za] = [0.0, 0.0, 0.0]
        self.delta = 0.0
        self.psi = 0.0
        self.phi = 0.0


class BicycleTrajectory2D(object):
    """
    Simulation of bicycle motion using kinematic and 4th order Runge-Kutta discretization
    """

    def __init__(self, X_init, U_init, w=1.12, noise=np.zeros(10)):
        # X_init = [x, y, z, psi, phi, delta]
        # U_init = [v, phi_dot, delta_dot]
        self.t = 0
        self.w = w
        self.noise = np.array(noise)
        self.X = X_init

        self.bicycle_state = BicycleState()
        self.bicycle_measurement = BicycleMeasurement()
        self.bicycle_input = BicycleInput()

        # define inputs
        self.bicycle_input.v = U_init[0]
        self.bicycle_input.phi_dot = U_init[1]
        self.bicycle_input.delta_dot = U_init[2]

        # update x init
        self.bicycle_state.x = X_init[0]
        self.bicycle_state.y = X_init[1]
        self.bicycle_state.z = X_init[2]
        self.bicycle_state.delta = X_init[3]
        self.bicycle_state.psi = X_init[4]
        self.bicycle_state.phi = X_init[5]

        self.fx_dot = self.bicycle_input.v * math.cos(self.bicycle_state.psi)
        self.fy_dot = self.bicycle_input.v * math.sin(self.bicycle_state.psi)
        self.fpsi_dot = (self.bicycle_input.v / self.w) * (
        math.tan(self.bicycle_state.delta) / math.cos(self.bicycle_state.phi))
        self.fphi_dot = self.bicycle_input.phi_dot
        self.fdelta_dot = self.bicycle_input.delta_dot

    def step(self, dt):
        self.bicycle_state.x = rk4(self.bicycle_state.x, self.t, dt, self.fx)
        self.bicycle_state.y = rk4(self.bicycle_state.y, self.t, dt, self.fy)
        self.bicycle_state.z = self.bicycle_state.z  # TODO: add function dependen on epsilon
        self.bicycle_state.delta = rk4(self.bicycle_state.delta, self.t, dt, self.fdelta)
        self.bicycle_state.psi = rk4(self.bicycle_state.psi, self.t, dt, self.fpsi)
        self.bicycle_state.phi = rk4(self.bicycle_state.phi, self.t, dt, self.fphi)

        self.fx_dot = self.bicycle_input.v * math.cos(self.bicycle_state.psi)
        self.fy_dot = self.bicycle_input.v * math.sin(self.bicycle_state.psi)
        self.fpsi_dot = (self.bicycle_input.v / self.w) * (
        math.tan(self.bicycle_state.delta) / math.cos(self.bicycle_state.phi))
        self.fphi_dot = self.bicycle_input.phi_dot
        self.fdelta_dot = self.bicycle_input.delta_dot

        # Update measurement vector
        _noise = np.multiply(self.noise, randn(1, 10)).T

        # print self.bicycle_state.y

        self.bicycle_measurement.xf = self.bicycle_state.x + _noise[0]
        self.bicycle_measurement.xr = self.bicycle_state.x + _noise[1]
        self.bicycle_measurement.yf = self.bicycle_state.y + _noise[2]
        self.bicycle_measurement.yr = self.bicycle_state.y + _noise[3]
        self.bicycle_measurement.zf = self.bicycle_state.z + _noise[4]
        self.bicycle_measurement.zr = self.bicycle_state.z + _noise[5]
        self.bicycle_measurement.za = self.bicycle_state.z + _noise[6]
        self.bicycle_measurement.delta = self.bicycle_state.delta + _noise[7]
        self.bicycle_measurement.psi = self.bicycle_state.psi + _noise[8]
        self.bicycle_measurement.phi = self.bicycle_state.phi + _noise[9]

        self.t += dt

        return (copy.copy(self.bicycle_state), copy.copy(self.bicycle_measurement))

    def simulate_path(self, N, dt):
        zs_gt = []
        zs_sim = []
        time = np.zeros((N, 1))
        t = 0

        for i in range(N):
            (xs, zs) = self.step(dt)
            zs_gt.append(xs)
            zs_sim.append(zs)
            time[i] = t
            t += dt

        return (zs_gt, zs_sim, time)

    def fx(self, x, t):
        return self.fx_dot

    def fy(self, y, t):
        return self.fy_dot

    def fv(self, yaw, t):
        return self.fv_dot

    def fpsi(self, yaw, t):
        return self.fpsi_dot

    def fphi(self, yaw, t):
        return self.fphi_dot

    def fdelta(self, yaw, t):
        return self.fdelta_dot


def test_bicycle_kinematic():
    # test_bicycle_kinematic() # plot bike position and yaw
    N = 100

    X_init = [0.0, 0.0, 0.0, math.radians(8), 0.0, 0.0]  # initial X_init = [x, y, z, delta, psi, phi]
    U_init = [1.0, 0.0, 0.0]  # initial condition U_init = [v, phi_dot, delta_dot]

    # noise = [xf, xr, yf, yr, zf, zr, za, delta, psi, phi]
    noise = [0.5, 0.5, 0.5, 0.5, 0.1, 0.1, 0.01, 0.01, 0.01, 0.01]

    # noise = [0.2, 0.2, 0.1, 0.1, 0.05, 0.05, 0.05]

    bike = BicycleTrajectory2D(X_init=X_init, U_init=U_init, noise=noise)

    (gt_sim, zs_sim, time) = bike.simulate_path(N=N, dt=0.25)

    bu.plot_results(xs=[], zs_sim=zs_sim, gt_sim=gt_sim, time=time, plot_xs=False)

    # plt.axis('equal');
