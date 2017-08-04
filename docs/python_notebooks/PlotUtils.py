#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
@author: Francisco Javier Garcia Rosas 2017
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from BicycleUtils import *
from FormatUtils import *


def plot_simulate_state_variables(gt, sim, time, file_name):
    # Path x-y ------------------------------------------------------------------------
    file_name_sp_xy = file_name + "_x_y"
    fig_xy = plt.figure(figsize=(fig_x, fig_y))
    sp_xy = plt.subplot(111)
    [xmin, xmax] = [np.min(gt[:, 0]), np.max(gt[:, 0]) * 1.02]
    [ymin, ymax] = [np.min(sim[:, 2]), np.max(sim[:, 2]) * 1.02]
    sp_xy = set_format_subplot(sp_xy, title="Path $x$ - $y$", xlabel="$x$ [m]", ylabel="$y$ [m]",
                               xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_xy.plot(gt[:, 0], gt[:, 1], color=color_gt, label='Reference', linewidth=1.0)
    sp_xy.scatter(sim[:, 0], sim[:, 2], color=color_noise_1, label='GPS Front', linewidth=1.0, s=10)
    sp_xy.scatter(sim[:, 1], sim[:, 3], color=color_noise_2, label='GPS Rear', linewidth=1.0, s=10)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_xy + ".svg", dpi=dpi, transparent=True)

    # Altitude Z --------------------------------------------------------------------------
    file_name_sp_z = file_name + "_z"
    fig_z = plt.figure(figsize=(fig_x, fig_y))
    sp_z = plt.subplot(111)

    [xmin, xmax] = [np.min(time), np.max(time) * 1.02]
    [ymin, ymax] = [np.min(sim[:, 5]), np.max(sim[:, 5]) * 1.02]
    sp_z = set_format_subplot(sp_z, title="Altitude $z$", xlabel="time [s]", ylabel="$z$ [m]",
                              xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_z.plot(time, gt[:, 2], color=color_gt, label='Reference', linewidth=1.0)
    sp_z.scatter(time, sim[:, 4], color=color_noise_1, label='GPS Front', linewidth=1.0, s=10)
    sp_z.scatter(time, sim[:, 5], color=color_noise_2, label='GPS Rear', linewidth=1.0, s=10)
    sp_z.scatter(time, sim[:, 6], color=color_noise_3, label='Altimeter', linewidth=1.0, s=10)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_z + ".svg", dpi=dpi, transparent=True)

    # Steering delta --------------------------------------------------------------------------
    file_name_sp_delta = file_name + "_delta"
    fig_delta = plt.figure(figsize=(fig_x, fig_y))
    sp_delta = plt.subplot(111)

    [xmin, xmax] = [np.min(time), np.max(time) * 1.02]
    [ymin, ymax] = [np.min(sim[:, 7]), np.max(sim[:, 7]) * 1.02]
    sp_delta = set_format_subplot(sp_delta, title="Effective steering $\delta$", xlabel="time [s]",
                                  ylabel="Angle [rad]",
                                  xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_delta.plot(time, gt[:, 3], color=color_gt, label='Reference', linewidth=1.0)
    sp_delta.scatter(time, sim[:, 7], color=color_noise_1, label='Imu $\delta$', linewidth=1.0, s=10)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_delta + ".svg", dpi=dpi, transparent=True)

    # Orientation psi --------------------------------------------------------------------------
    file_name_sp_psi = file_name + "_psi"
    fig_psi = plt.figure(figsize=(fig_x, fig_y))
    sp_psi = plt.subplot(111)

    [xmin, xmax] = [np.min(time), np.max(time) * 1.02]
    [ymin, ymax] = [np.min(gt[:, 4]), np.max(gt[:, 4]) * 1.02]
    sp_psi = set_format_subplot(sp_psi, title="Orientation $\psi$", xlabel="time [s]", ylabel="Angle [rad]",
                                xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_psi.plot(time, gt[:, 4], color=color_gt, label='Reference', linewidth=1.0)
    sp_psi.scatter(time, sim[:, 8], color=color_noise_1, label='Imu $\psi$', linewidth=1.0, s=10)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_psi + ".svg", dpi=dpi, transparent=True)

    # Lean phi --------------------------------------------------------------------------
    file_name_sp_phi = file_name + "_phi"
    fig_phi = plt.figure(figsize=(fig_x, fig_y))
    sp_phi = plt.subplot(111)

    [xmin, xmax] = [np.min(time), np.max(time) * 1.02]
    [ymin, ymax] = [np.min(sim[:, 9]), np.max(sim[:, 9]) * 1.02]
    sp_phi = set_format_subplot(sp_phi, title="Lean $\phi$", xlabel="time [s]", ylabel="Angle [rad]",
                                xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_phi.plot(time, gt[:, 5], color=color_gt, label='Reference', linewidth=1.0)
    sp_phi.scatter(time, sim[:, 9], color=color_noise_1, label='Imu $\phi$', linewidth=1.0, s=10)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_phi + ".svg", dpi=dpi, transparent=True)
    # --------------------------------------------------------------------------

    plt.show()


def plot_results(xs, zs_sim, gt_sim, time, plot_xs=True, plot_all=True, convert_obj2array=True, plot_gt=True):
    if convert_obj2array:
        (gt, zs) = convert_object_to_array(gt_sim, zs_sim)
    else:
        (gt, zs) = (gt_sim, zs_sim)

    xmin = np.min(time)
    xmax = np.max(time)

    plt.figure(figsize=(13, 15))
    G = gridspec.GridSpec(4, 2)
    axes_x_y = plt.subplot(G[0, 0])
    axes_x_y.set_title('x-y')

    axes_psi = plt.subplot(G[0, 1])
    axes_psi.set_title('psi [heading]')
    axes_psi.set_xlim([xmin, xmax])

    axes_phi = plt.subplot(G[2, 0])
    axes_phi.set_title('phi [lean]')
    axes_phi.set_xlim([xmin, xmax])

    axes_delta = plt.subplot(G[2, 1])
    axes_delta.set_title('delta [steering]')
    axes_delta.set_xlim([xmin, xmax])

    axes_x = plt.subplot(G[1, 0])
    axes_x.set_title('x')
    axes_x.set_xlim([xmin, xmax])

    axes_y = plt.subplot(G[1, 1])
    axes_y.set_title('y')
    axes_y.set_xlim([xmin, xmax])

    axes_z = plt.subplot(G[3, 0])
    axes_z.set_title('z')
    axes_z.set_xlim([xmin, xmax])

    if plot_gt: axes_x_y.plot(gt[:, 0], gt[:, 1], color='red', label='ref')
    axes_x_y.scatter(zs[:, 0], zs[:, 2], color='black', label='sim_f')
    axes_x_y.scatter(zs[:, 1], zs[:, 3], color='gray', label='sim_r')

    if plot_gt: axes_psi.plot(time, gt[:, 4], color='red', label='ref')
    axes_psi.plot(time, zs[:, 8], color='black', label='sim')

    if plot_all:
        if plot_gt: axes_phi.plot(time, gt[:, 5], color='red', label='ref')
        axes_phi.plot(time, zs[:, 9], color='black', label='sim')

        if plot_gt: axes_delta.plot(time, gt[:, 3], color='red', label='ref')
        axes_delta.plot(time, zs[:, 7], color='black', label='sim')

        if plot_gt: axes_x.plot(time, gt[:, 0], color='red', label='ref')
        axes_x.plot(time, zs[:, 0], color='black', label='sim_f')
        axes_x.plot(time, zs[:, 1], color='gray', label='sim_r')

        if plot_gt: axes_y.plot(time, gt[:, 1], color='red', label='ref')
        axes_y.plot(time, zs[:, 2], color='black', label='sim_f')
        axes_y.plot(time, zs[:, 3], color='gray', label='sim_r')

        if plot_gt: axes_z.plot(time, gt[:, 2], color='red', label='ref')
        axes_z.plot(time, zs[:, 4], color='black', label='sim_f')
        axes_z.plot(time, zs[:, 5], color='gray', label='sim_r')
        axes_z.plot(time, zs[:, 6], color='silver', label='sim_a')

        if plot_xs:
            axes_phi.plot(time, xs[:, 5], color='blue', label='pred')
            axes_delta.plot(time, xs[:, 3], color='blue', label='pred')
            axes_x.plot(time, xs[:, 0], color='blue', label='pred')
            axes_y.plot(time, xs[:, 1], color='blue', label='pred')
            axes_z.plot(time, xs[:, 2], color='blue', label='pred')

    if plot_xs:
        axes_x_y.plot(xs[:, 0], xs[:, 1], color='blue', label='pred')
        axes_psi.plot(time, xs[:, 4], color='blue', label='pred')

    axes_x_y.legend(loc='best', shadow=True)

    axes_phi.legend(loc='best', shadow=True)
    axes_delta.legend(loc='best', shadow=True)
    axes_psi.legend(loc='best', shadow=True)

    axes_x.legend(loc='best', shadow=True)
    axes_y.legend(loc='best', shadow=True)
    axes_z.legend(loc='best', shadow=True)

    plt.tight_layout()
    plt.show()
