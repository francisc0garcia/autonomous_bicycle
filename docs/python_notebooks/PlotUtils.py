#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
@author: Francisco Javier Garcia Rosas 2017
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import os
from BicycleUtils import *
from FormatUtils import *


def check_directory(input_dir):
    if not os.path.exists(input_dir):
        os.makedirs(input_dir)


def plot_simulate_state_variables(gt, sim, time, file_name, scale_max=1.02, autoscale_axis=False):
    check_directory(file_name)  # Check if a directory exists and create one if not

    # Path x-y ------------------------------------------------------------------------
    [xmin, xmax, ymin, ymax] = [None, None, None, None]
    file_name_sp_xy = file_name + "x_y"
    fig_xy = plt.figure(figsize=(fig_x, fig_y))
    sp_xy = plt.subplot(111)
    if not autoscale_axis: [xmin, xmax] = [np.min(gt[:, 0]), np.max(gt[:, 0]) * scale_max]
    if not autoscale_axis: [ymin, ymax] = [np.min(sim[:, 2]), np.max(sim[:, 2]) * scale_max]
    sp_xy = set_format_subplot(sp_xy, title="Path $x$ - $y$", xlabel="$x$ [m]", ylabel="$y$ [m]",
                               xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_xy.plot(gt[:, 0], gt[:, 1], color=color_gt, label='Reference', linewidth=1.0)
    sp_xy.scatter(sim[:, 0], sim[:, 2], color=color_noise_1, label='GPS Front', linewidth=1.0, s=10)
    sp_xy.scatter(sim[:, 1], sim[:, 3], color=color_noise_2, label='GPS Rear', linewidth=1.0, s=10)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_xy + ".svg", dpi=dpi, transparent=True)

    # Altitude Z --------------------------------------------------------------------------
    [xmin, xmax, ymin, ymax] = [None, None, None, None]
    file_name_sp_z = file_name + "z"
    fig_z = plt.figure(figsize=(fig_x, fig_y))
    sp_z = plt.subplot(111)

    if not autoscale_axis: [xmin, xmax] = [np.min(time), np.max(time) * scale_max]
    if not autoscale_axis: [ymin, ymax] = [np.min(sim[:, 5]), np.max(sim[:, 5]) * scale_max]
    sp_z = set_format_subplot(sp_z, title="Altitude $z$", xlabel="time [s]", ylabel="$z$ [m]",
                              xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_z.plot(time, gt[:, 2], color=color_gt, label='Reference', linewidth=1.0)
    sp_z.scatter(time, sim[:, 4], color=color_noise_1, label='GPS Front', linewidth=1.0, s=10)
    sp_z.scatter(time, sim[:, 5], color=color_noise_2, label='GPS Rear', linewidth=1.0, s=10)
    sp_z.scatter(time, sim[:, 6], color=color_noise_3, label='Altimeter', linewidth=1.0, s=10)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_z + ".svg", dpi=dpi, transparent=True)

    # Steering delta --------------------------------------------------------------------------
    [xmin, xmax, ymin, ymax] = [None, None, None, None]
    file_name_sp_delta = file_name + "delta"
    fig_delta = plt.figure(figsize=(fig_x, fig_y))
    sp_delta = plt.subplot(111)

    if not autoscale_axis: [xmin, xmax] = [np.min(time), np.max(time) * scale_max]
    if not autoscale_axis: [ymin, ymax] = [np.min(sim[:, 7]), np.max(sim[:, 7]) * scale_max]
    sp_delta = set_format_subplot(sp_delta, title="Effective steering $\delta$", xlabel="time [s]",
                                  ylabel="Angle [rad]",
                                  xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_delta.plot(time, gt[:, 3], color=color_gt, label='Reference', linewidth=1.0)
    sp_delta.scatter(time, sim[:, 7], color=color_noise_1, label='Imu $\delta$', linewidth=1.0, s=10)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_delta + ".svg", dpi=dpi, transparent=True)

    # Orientation psi --------------------------------------------------------------------------
    [xmin, xmax, ymin, ymax] = [None, None, None, None]
    file_name_sp_psi = file_name + "psi"
    fig_psi = plt.figure(figsize=(fig_x, fig_y))
    sp_psi = plt.subplot(111)

    if not autoscale_axis: [xmin, xmax] = [np.min(time), np.max(time) * scale_max]
    if not autoscale_axis: [ymin, ymax] = [np.min(gt[:, 4]), np.max(gt[:, 4]) * scale_max]
    sp_psi = set_format_subplot(sp_psi, title="Orientation $\psi$", xlabel="time [s]", ylabel="Angle [rad]",
                                xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_psi.plot(time, gt[:, 4], color=color_gt, label='Reference', linewidth=1.0)
    sp_psi.scatter(time, sim[:, 8], color=color_noise_1, label='Imu $\psi$', linewidth=1.0, s=10)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_psi + ".svg", dpi=dpi, transparent=True)

    # Lean phi --------------------------------------------------------------------------
    [xmin, xmax, ymin, ymax] = [None, None, None, None]
    file_name_sp_phi = file_name + "phi"
    fig_phi = plt.figure(figsize=(fig_x, fig_y))
    sp_phi = plt.subplot(111)

    if not autoscale_axis: [xmin, xmax] = [np.min(time), np.max(time) * scale_max]
    if not autoscale_axis: [ymin, ymax] = [np.min(sim[:, 9]), np.max(sim[:, 9]) * scale_max]
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


def plot_filter_results(xs, gt, sim, time, file_name, filter_name, scale_max=1.02, autoscale_axis=False):
    check_directory(file_name)  # Check if a directory exists and create one if not

    # Path x-y ------------------------------------------------------------------------
    [xmin, xmax, ymin, ymax] = [None, None, None, None]
    file_name_sp_xy = file_name + "x_y"
    fig_xy = plt.figure(figsize=(fig_x, fig_y))
    sp_xy = plt.subplot(111)
    if not autoscale_axis: [xmin, xmax] = [np.min(xs[:, 0]), np.max(sim[:, 0]) * scale_max]
    if not autoscale_axis: [ymin, ymax] = [np.min(xs[:, 2]), np.max(sim[:, 2]) * scale_max]
    sp_xy = set_format_subplot(sp_xy, title="Path $x$ - $y$", xlabel="$x$ [m]", ylabel="$y$ [m]",
                               xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_xy.plot(gt[:, 0], gt[:, 1], color=color_gt, label='Reference', linewidth=2.0)
    sp_xy.plot(xs[:, 0], xs[:, 1], color=color_prediction, label=filter_name, linewidth=2.0)
    sp_xy.scatter(sim[:, 0], sim[:, 2], color=color_noise_1, label='GPS Front', linewidth=1.0, s=10)
    sp_xy.scatter(sim[:, 1], sim[:, 3], color=color_noise_2, label='GPS Rear', linewidth=1.0, s=10)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_xy + ".svg", dpi=dpi, transparent=True)

    # Position X --------------------------------------------------------------------------
    [xmin, xmax, ymin, ymax] = [None, None, None, None]
    file_name_sp_x = file_name + "x"
    fig_xp = plt.figure(figsize=(fig_x, fig_y))
    sp_x = plt.subplot(111)

    if not autoscale_axis: [xmin, xmax] = [np.min(time), np.max(time) * scale_max]
    if not autoscale_axis: [ymin, ymax] = [np.min(xs[:, 0]), np.max(sim[:, 0]) * scale_max]
    sp_x = set_format_subplot(sp_x, title="Position $x$", xlabel="time [s]", ylabel="$x$ [m]",
                              xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_x.plot(time, gt[:, 0], color=color_gt, label='Reference', linewidth=2.0)
    sp_x.plot(time, xs[:, 0], color=color_prediction, label=filter_name, linewidth=2.0)
    sp_x.scatter(time, sim[:, 0], color=color_noise_1, label='GPS Front', linewidth=1.0, s=10)
    sp_x.scatter(time, sim[:, 1], color=color_noise_2, label='GPS Rear', linewidth=1.0, s=10)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_x + ".svg", dpi=dpi, transparent=True)

    # Position Y --------------------------------------------------------------------------
    [xmin, xmax, ymin, ymax] = [None, None, None, None]
    file_name_sp_y = file_name + "y"
    fig_yp = plt.figure(figsize=(fig_x, fig_y))
    sp_y = plt.subplot(111)

    if not autoscale_axis: [xmin, xmax] = [np.min(time), np.max(time) * scale_max]
    if not autoscale_axis: [ymin, ymax] = [np.min(xs[:, 1]), np.max(sim[:, 2]) * scale_max]
    sp_y = set_format_subplot(sp_y, title="Position $y$", xlabel="time [s]", ylabel="$y$ [m]",
                              xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_y.plot(time, gt[:, 1], color=color_gt, label='Reference', linewidth=2.0)
    sp_y.plot(time, xs[:, 1], color=color_prediction, label=filter_name, linewidth=2.0)
    sp_y.scatter(time, sim[:, 2], color=color_noise_1, label='GPS Front', linewidth=1.0, s=10)
    sp_y.scatter(time, sim[:, 3], color=color_noise_2, label='GPS Rear', linewidth=1.0, s=10)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_y + ".svg", dpi=dpi, transparent=True)

    # Altitude Z --------------------------------------------------------------------------
    [xmin, xmax, ymin, ymax] = [None, None, None, None]
    file_name_sp_z = file_name + "z"
    fig_z = plt.figure(figsize=(fig_x, fig_y))
    sp_z = plt.subplot(111)

    if not autoscale_axis: [xmin, xmax] = [np.min(time), np.max(time) * scale_max]
    if not autoscale_axis: [ymin, ymax] = [np.min(sim[:, 5]), np.max(sim[:, 5]) * scale_max]
    sp_z = set_format_subplot(sp_z, title="Altitude $z$", xlabel="time [s]", ylabel="$z$ [m]",
                              xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_z.plot(time, gt[:, 2], color=color_gt, label='Reference', linewidth=2.0)
    sp_z.plot(time, xs[:, 2], color=color_prediction, label=filter_name, linewidth=2.0)
    sp_z.scatter(time, sim[:, 4], color=color_noise_1, label='GPS Front', linewidth=1.0, s=10)
    sp_z.scatter(time, sim[:, 5], color=color_noise_2, label='GPS Rear', linewidth=1.0, s=10)
    sp_z.scatter(time, sim[:, 6], color=color_noise_3, label='Altimeter', linewidth=1.0, s=10)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_z + ".svg", dpi=dpi, transparent=True)

    # Steering delta --------------------------------------------------------------------------
    [xmin, xmax, ymin, ymax] = [None, None, None, None]
    file_name_sp_delta = file_name + "delta"
    fig_delta = plt.figure(figsize=(fig_x, fig_y))
    sp_delta = plt.subplot(111)

    if not autoscale_axis: [xmin, xmax] = [np.min(time), np.max(time) * scale_max]
    if not autoscale_axis: [ymin, ymax] = [np.min(sim[:, 7]), np.max(sim[:, 7]) * scale_max]
    sp_delta = set_format_subplot(sp_delta, title="Effective steering $\delta$", xlabel="time [s]",
                                  ylabel="Angle [rad]",
                                  xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_delta.plot(time, gt[:, 3], color=color_gt, label='Reference', linewidth=2.0)
    sp_delta.plot(time, xs[:, 3], color=color_prediction, label=filter_name, linewidth=2.0)
    sp_delta.scatter(time, sim[:, 7], color=color_noise_1, label='Imu $\delta$', linewidth=1.0, s=10)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_delta + ".svg", dpi=dpi, transparent=True)

    # Orientation psi --------------------------------------------------------------------------
    [xmin, xmax, ymin, ymax] = [None, None, None, None]
    file_name_sp_psi = file_name + "psi"
    fig_psi = plt.figure(figsize=(fig_x, fig_y))
    sp_psi = plt.subplot(111)

    if not autoscale_axis: [xmin, xmax] = [np.min(time), np.max(time) * scale_max]
    if not autoscale_axis: [ymin, ymax] = [np.min(gt[:, 4]), np.max(gt[:, 4]) * scale_max]
    sp_psi = set_format_subplot(sp_psi, title="Orientation $\psi$", xlabel="time [s]", ylabel="Angle [rad]",
                                xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_psi.plot(time, gt[:, 4], color=color_gt, label='Reference', linewidth=2.0)
    sp_psi.plot(time, xs[:, 4], color=color_prediction, label=filter_name, linewidth=2.0)
    sp_psi.scatter(time, sim[:, 8], color=color_noise_1, label='Imu $\psi$', linewidth=1.0, s=10)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_psi + ".svg", dpi=dpi, transparent=True)

    # Lean phi --------------------------------------------------------------------------
    [xmin, xmax, ymin, ymax] = [None, None, None, None]
    file_name_sp_phi = file_name + "phi"
    fig_phi = plt.figure(figsize=(fig_x, fig_y))
    sp_phi = plt.subplot(111)

    if not autoscale_axis: [xmin, xmax] = [np.min(time), np.max(time) * scale_max]
    if not autoscale_axis: [ymin, ymax] = [np.min(sim[:, 9]), np.max(sim[:, 9]) * scale_max]
    sp_phi = set_format_subplot(sp_phi, title="Lean $\phi$", xlabel="time [s]", ylabel="Angle [rad]",
                                xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_phi.plot(time, gt[:, 5], color=color_gt, label='Reference', linewidth=2.0)
    sp_phi.plot(time, xs[:, 5], color=color_prediction, label=filter_name, linewidth=2.0)
    sp_phi.scatter(time, sim[:, 9], color=color_noise_1, label='Imu $\phi$', linewidth=1.0, s=10)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_phi + ".svg", dpi=dpi, transparent=True)
    # --------------------------------------------------------------------------

    plt.show()


def plot_EKF_gain_covariance(time, KU, PU, file_name, autoscale_axis=False, format_file="svg"):
    check_directory(file_name)  # Check if a directory exists and create one if not

    # Kalman gain --------------------------------------------------------------------------
    [xmin, xmax, ymin, ymax] = [None, None, None, None]
    file_name_sp_ku = file_name + "KU"
    fig_ku = plt.figure(figsize=(fig_x, fig_y))
    sp_ku = plt.subplot(111)
    if not autoscale_axis: [xmin, xmax] = [np.min(time), np.max(time) * 1.02]
    if not autoscale_axis: [ymin, ymax] = [-0.05, np.max(KU[:, 0]) * 1.02]
    sp_ku = set_format_subplot(sp_ku, title="Kalman gain", xlabel="time [s]", ylabel="Gain",
                               xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_ku.plot(time, KU[:, 0], label='$x$')
    sp_ku.plot(time, KU[:, 1], label='$y$')
    sp_ku.plot(time, KU[:, 2], label='$z$')
    sp_ku.plot(time, KU[:, 3], label='$\sigma$')
    sp_ku.plot(time, KU[:, 4], label='$\psi$')
    sp_ku.plot(time, KU[:, 5], label='$\phi$')

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon,
               bbox_to_anchor=(-0.01, 0.85, 1., .06), ncol=9, borderaxespad=0., prop={'size': 14})

    plt.savefig(file_name_sp_ku + "." + format_file, dpi=dpi, transparent=True)

    # Process covariance ---------------------------------------------------------------------
    [xmin, xmax, ymin, ymax] = [None, None, None, None]
    file_name_sp_pu = file_name + "PU"
    fig_pu = plt.figure(figsize=(fig_x, fig_y))
    sp_pu = plt.subplot(111)
    if not autoscale_axis: [xmin, xmax] = [np.min(time), np.max(time) * 1.02]
    if not autoscale_axis: [ymin, ymax] = [None, None]
    sp_pu = set_format_subplot(sp_pu, title="Process covariance", xlabel="time [s]", ylabel="Covariance",
                               xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_pu.semilogy(time, PU[:, 0], label='$x$')
    sp_pu.step(time, PU[:, 1], label='$y$')
    sp_pu.step(time, PU[:, 2], label='$z$')
    sp_pu.step(time, PU[:, 3], label='$\sigma$')
    sp_pu.step(time, PU[:, 4], label='$\psi$')
    sp_pu.step(time, PU[:, 5], label='$\phi$')

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon,
               bbox_to_anchor=(-0.01, 0.85, 1., .06), ncol=9, borderaxespad=0., prop={'size': 14})

    plt.savefig(file_name_sp_pu + "." + format_file, dpi=dpi, transparent=True)

    plt.show()


def plot_real_data_state_variables(U, sim, time, file_name, dpi, format='png'):
    check_directory(file_name)  # Check if a directory exists and create one if not

    [xmin, xmax, ymin, ymax] = [None, None, None, None]

    # Path x-y ------------------------------------------------------------------------
    file_name_sp_xy = file_name + "x_y"
    fig_xy = plt.figure(figsize=(fig_x, fig_y))
    sp_xy = plt.subplot(111)
    sp_xy = set_format_subplot(sp_xy, title="Path $x$ - $y$", xlabel="$x$ [m]", ylabel="$y$ [m]",
                               xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_xy.plot(sim[:, 0], sim[:, 2], label='GPS Front', linewidth=2.0)
    sp_xy.plot(sim[:, 1], sim[:, 3], label='GPS Rear', linewidth=2.0)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_xy + "." + format, dpi=dpi, transparent=True)

    # Altitude Z --------------------------------------------------------------------------
    file_name_sp_z = file_name + "z"
    fig_z = plt.figure(figsize=(fig_x, fig_y))
    sp_z = plt.subplot(111)

    sp_z = set_format_subplot(sp_z, title="Altitude $z$", xlabel="time [s]", ylabel="$z$ [m]",
                              xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_z.plot(time, sim[:, 4], label='GPS Front', linewidth=2.0)
    sp_z.plot(time, sim[:, 5], label='GPS Rear', linewidth=2.0)
    sp_z.plot(time, sim[:, 6], label='Altimeter', linewidth=2.0)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_z + "." + format, dpi=dpi, transparent=True)

    # Angles --------------------------------------------------------------------------
    file_name_sp_angles = file_name + "angles"
    fig_angles = plt.figure(figsize=(fig_x, fig_y))
    sp_angles = plt.subplot(111)

    sp_angles = set_format_subplot(sp_angles, title="Angles", xlabel="time [s]",
                                  ylabel="Angle [rad]",
                                  xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_angles.plot(time, sim[:, 7], label='Imu $\delta$', linewidth=2.0)
    sp_angles.plot(time, sim[:, 8], label='Imu $\psi$', linewidth=2.0)
    sp_angles.plot(time, sim[:, 9], label='Imu $\phi$', linewidth=2.0)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_angles + "." + format, dpi=dpi, transparent=True)

    # Inputs --------------------------------------------------------------------------
    file_name_sp_inputs = file_name + "inputs"
    fig_inputs, axes = plt.subplots(nrows=1, ncols=2, figsize=(fig_x*2.0, fig_y))

    axes[0] = set_format_subplot(axes[0], title="Input velocity", xlabel="time [s]",
                                   ylabel="Speed [m/s]",
                                   xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    axes[0].plot(time, U[:, 0], label='Velocity $v$', linewidth=2.0)

    axes[1] = set_format_subplot(axes[1], title="Input $\dot \delta$ and $\dot \phi$", xlabel="time [s]",
                                 ylabel="Angle [rad]",
                                 xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    axes[1].plot(time, U[:, 1], label='$\dot \delta$', linewidth=1.0)
    axes[1].plot(time, U[:, 2], label='$\dot \phi$', linewidth=1.0)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_inputs + "." + format, dpi=dpi, transparent=True)

    plt.show()


def plot_filter_results_real_data(xs, sim, time, file_name, filter_name, format='png', dpi=100):
    check_directory(file_name)  # Check if a directory exists and create one if not

    [xmin, xmax, ymin, ymax] = [None, None, None, None]

    # Path x-y ------------------------------------------------------------------------
    file_name_sp_xy = file_name + "x_y"
    fig_xy = plt.figure(figsize=(fig_x, fig_y))
    sp_xy = plt.subplot(111)
    sp_xy = set_format_subplot(sp_xy, title="Path $x$ - $y$", xlabel="$x$ [m]", ylabel="$y$ [m]",
                               xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_xy.plot(xs[:, 0], xs[:, 1], color=color_prediction, label=filter_name, linewidth=2.0)
    sp_xy.plot(sim[:, 0], sim[:, 2], color=color_noise_1, label='GPS Front', linewidth=1.0)
    sp_xy.plot(sim[:, 1], sim[:, 3], color=color_noise_2, label='GPS Rear', linewidth=1.0)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_xy + "." + format, dpi=dpi, transparent=True)

    # Position X --------------------------------------------------------------------------
    file_name_sp_x = file_name + "x"
    fig_xp = plt.figure(figsize=(fig_x, fig_y))
    sp_x = plt.subplot(111)

    sp_x = set_format_subplot(sp_x, title="Position $x$", xlabel="time [s]", ylabel="$x$ [m]",
                              xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_x.plot(time, xs[:, 0], color=color_prediction, label=filter_name, linewidth=2.0)
    sp_x.plot(time, sim[:, 0], color=color_noise_1, label='GPS Front', linewidth=1.0)
    sp_x.plot(time, sim[:, 1], color=color_noise_2, label='GPS Rear', linewidth=1.0)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_x + "." + format, dpi=dpi, transparent=True)

    # Position Y --------------------------------------------------------------------------
    file_name_sp_y = file_name + "y"
    fig_yp = plt.figure(figsize=(fig_x, fig_y))
    sp_y = plt.subplot(111)

    sp_y = set_format_subplot(sp_y, title="Position $y$", xlabel="time [s]", ylabel="$y$ [m]",
                              xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_y.plot(time, xs[:, 1], color=color_prediction, label=filter_name, linewidth=2.0)
    sp_y.plot(time, sim[:, 2], color=color_noise_1, label='GPS Front', linewidth=1.0)
    sp_y.plot(time, sim[:, 3], color=color_noise_2, label='GPS Rear', linewidth=1.0)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_y + "." + format, dpi=dpi, transparent=True)

    # Altitude Z --------------------------------------------------------------------------
    file_name_sp_z = file_name + "z"
    fig_z = plt.figure(figsize=(fig_x, fig_y))
    sp_z = plt.subplot(111)

    sp_z = set_format_subplot(sp_z, title="Altitude $z$", xlabel="time [s]", ylabel="$z$ [m]",
                              xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_z.plot(time, xs[:, 2], color=color_prediction, label=filter_name, linewidth=2.0)
    sp_z.plot(time, sim[:, 4], color=color_noise_1, label='GPS Front', linewidth=2.0)
    sp_z.plot(time, sim[:, 5], color=color_noise_2, label='GPS Rear', linewidth=2.0)
    sp_z.plot(time, sim[:, 6], color=color_noise_3, label='Altimeter', linewidth=2.0)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_z + "." + format, dpi=dpi, transparent=True)

    # Steering delta --------------------------------------------------------------------------
    file_name_sp_delta = file_name + "delta"
    fig_delta = plt.figure(figsize=(fig_x, fig_y))
    sp_delta = plt.subplot(111)

    sp_delta = set_format_subplot(sp_delta, title="Effective steering $\delta$", xlabel="time [s]",
                                  ylabel="Angle [rad]",
                                  xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_delta.plot(time, xs[:, 3], color=color_prediction, label=filter_name, linewidth=2.0)
    sp_delta.plot(time, sim[:, 7], color=color_noise_1, label='Imu $\delta$', linewidth=1.0)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_delta + "." + format, dpi=dpi, transparent=True)

    # Orientation psi --------------------------------------------------------------------------
    file_name_sp_psi = file_name + "psi"
    fig_psi = plt.figure(figsize=(fig_x, fig_y))
    sp_psi = plt.subplot(111)

    sp_psi = set_format_subplot(sp_psi, title="Orientation $\psi$", xlabel="time [s]", ylabel="Angle [rad]",
                                xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_psi.plot(time, xs[:, 4], color=color_prediction, label=filter_name, linewidth=2.0)
    sp_psi.plot(time, sim[:, 8], color=color_noise_1, label='Imu $\psi$', linewidth=1.0)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_psi + "." + format, dpi=dpi, transparent=True)

    # Lean phi --------------------------------------------------------------------------
    file_name_sp_phi = file_name + "phi"
    fig_phi = plt.figure(figsize=(fig_x, fig_y))
    sp_phi = plt.subplot(111)

    sp_phi = set_format_subplot(sp_phi, title="Lean $\phi$", xlabel="time [s]", ylabel="Angle [rad]",
                                xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_phi.plot(time, xs[:, 5], color=color_prediction, label=filter_name, linewidth=2.0)
    sp_phi.plot(time, sim[:, 9], color=color_noise_1, label='Imu $\phi$', linewidth=1.0)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_phi + "." + format, dpi=dpi, transparent=True)
    # --------------------------------------------------------------------------

    plt.show()


def plot_comparison_real_data(xs_ekf, xs_ukf, sim, time, file_name, format='png', dpi=100):
    check_directory(file_name)  # Check if a directory exists and create one if not

    [xmin, xmax, ymin, ymax] = [None, None, None, None]

    # Path x-y ------------------------------------------------------------------------
    file_name_sp_xy = file_name + "x_y"
    fig_xy = plt.figure(figsize=(fig_x, fig_y))
    sp_xy = plt.subplot(111)
    sp_xy = set_format_subplot(sp_xy, title="Path $x$ - $y$", xlabel="$x$ [m]", ylabel="$y$ [m]",
                               xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_xy.plot(xs_ekf[:, 0], xs_ekf[:, 1], color=color_prediction_ekf, label='EKF', linewidth=1.0)
    sp_xy.plot(xs_ukf[:, 0], xs_ukf[:, 1], color=color_prediction_ukf, label='UKF', linewidth=1.0)
    sp_xy.plot(sim[:, 0], sim[:, 2], color=color_noise_1, label='GPS Front', linewidth=1.0)
    sp_xy.plot(sim[:, 1], sim[:, 3], color=color_noise_2, label='GPS Rear', linewidth=1.0)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_xy + "." + format, dpi=dpi, transparent=True)

    # Position X --------------------------------------------------------------------------
    file_name_sp_x = file_name + "x"
    fig_xp = plt.figure(figsize=(fig_x, fig_y))
    sp_x = plt.subplot(111)

    sp_x = set_format_subplot(sp_x, title="Position $x$", xlabel="time [s]", ylabel="$x$ [m]",
                              xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_x.plot(time, xs_ekf[:, 0], color=color_prediction_ekf, label='EKF', linewidth=1.0)
    sp_x.plot(time, xs_ukf[:, 0], color=color_prediction_ukf, label='UKF', linewidth=1.0)
    sp_x.plot(time, sim[:, 0], color=color_noise_1, label='GPS Front', linewidth=1.0)
    sp_x.plot(time, sim[:, 1], color=color_noise_2, label='GPS Rear', linewidth=1.0)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_x + "." + format, dpi=dpi, transparent=True)

    # Position Y --------------------------------------------------------------------------
    file_name_sp_y = file_name + "y"
    fig_yp = plt.figure(figsize=(fig_x, fig_y))
    sp_y = plt.subplot(111)

    sp_y = set_format_subplot(sp_y, title="Position $y$", xlabel="time [s]", ylabel="$y$ [m]",
                              xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_y.plot(time, xs_ekf[:, 1], color=color_prediction_ekf, label='EKF', linewidth=1.0)
    sp_y.plot(time, xs_ukf[:, 1], color=color_prediction_ukf, label='UKF', linewidth=1.0)
    sp_y.plot(time, sim[:, 2], color=color_noise_1, label='GPS Front', linewidth=1.0)
    sp_y.plot(time, sim[:, 3], color=color_noise_2, label='GPS Rear', linewidth=1.0)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_y + "." + format, dpi=dpi, transparent=True)

    # Altitude Z --------------------------------------------------------------------------
    file_name_sp_z = file_name + "z"
    fig_z = plt.figure(figsize=(fig_x, fig_y))
    sp_z = plt.subplot(111)

    sp_z = set_format_subplot(sp_z, title="Altitude $z$", xlabel="time [s]", ylabel="$z$ [m]",
                              xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_z.plot(time, xs_ekf[:, 2], color=color_prediction_ekf, label='EKF', linewidth=1.0)
    sp_z.plot(time, xs_ukf[:, 2], color=color_prediction_ukf, label='UKF', linewidth=1.0)
    sp_z.plot(time, sim[:, 4], color=color_noise_1, label='GPS Front', linewidth=2.0)
    sp_z.plot(time, sim[:, 5], color=color_noise_2, label='GPS Rear', linewidth=2.0)
    sp_z.plot(time, sim[:, 6], color=color_noise_3, label='Altimeter', linewidth=2.0)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_z + "." + format, dpi=dpi, transparent=True)

    # Steering delta --------------------------------------------------------------------------
    file_name_sp_delta = file_name + "delta"
    fig_delta = plt.figure(figsize=(fig_x, fig_y))
    sp_delta = plt.subplot(111)

    sp_delta = set_format_subplot(sp_delta, title="Effective steering $\delta$", xlabel="time [s]",
                                  ylabel="Angle [rad]",
                                  xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_delta.plot(time, xs_ekf[:, 3], color=color_prediction_ekf, label='EKF', linewidth=1.0)
    sp_delta.plot(time, xs_ukf[:, 3], color=color_prediction_ukf, label='UKF', linewidth=1.0)
    sp_delta.plot(time, sim[:, 7], color=color_noise_1, label='Imu $\delta$', linewidth=1.0)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_delta + "." + format, dpi=dpi, transparent=True)

    # Orientation psi --------------------------------------------------------------------------
    file_name_sp_psi = file_name + "psi"
    fig_psi = plt.figure(figsize=(fig_x, fig_y))
    sp_psi = plt.subplot(111)

    sp_psi = set_format_subplot(sp_psi, title="Orientation $\psi$", xlabel="time [s]", ylabel="Angle [rad]",
                                xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_psi.plot(time, xs_ekf[:, 4], color=color_prediction_ekf, label='EKF', linewidth=1.0)
    sp_psi.plot(time, xs_ukf[:, 4], color=color_prediction_ukf, label='UKF', linewidth=1.0)
    sp_psi.plot(time, sim[:, 8], color=color_noise_1, label='Imu $\psi$', linewidth=1.0)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_psi + "." + format, dpi=dpi, transparent=True)

    # Lean phi --------------------------------------------------------------------------
    file_name_sp_phi = file_name + "phi"
    fig_phi = plt.figure(figsize=(fig_x, fig_y))
    sp_phi = plt.subplot(111)

    sp_phi = set_format_subplot(sp_phi, title="Lean $\phi$", xlabel="time [s]", ylabel="Angle [rad]",
                                xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_phi.plot(time, xs_ekf[:, 5], color=color_prediction_ekf, label='EKF', linewidth=1.0)
    sp_phi.plot(time, xs_ukf[:, 5], color=color_prediction_ukf, label='UKF', linewidth=1.0)
    sp_phi.plot(time, sim[:, 9], color=color_noise_1, label='Imu $\phi$', linewidth=1.0)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_phi + "." + format, dpi=dpi, transparent=True)
    # --------------------------------------------------------------------------

    plt.show()


def plot_real_data_delta_comparison(sim_rotatory, sim_imu, time, file_name, dpi, format='png'):
    check_directory(file_name)  # Check if a directory exists and create one if not

    [xmin, xmax, ymin, ymax] = [None, None, None, None]

    # Angles --------------------------------------------------------------------------
    file_name_sp_angles = file_name + "imu_vs_rotatory"
    fig_angles = plt.figure(figsize=(fig_x, fig_y))
    sp_angles = plt.subplot(111)

    sp_angles = set_format_subplot(sp_angles, title="IMU vs Rotatory delta", xlabel="time [s]",
                                   ylabel="Angle [rad]",
                                   xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)

    sp_angles.plot(time, sim_imu[:, 7], label='Imu $\delta$', linewidth=2.0)
    sp_angles.plot(time, sim_rotatory[:, 7], label='Rotatory $\delta$', linewidth=2.0)

    plt.legend(loc='best', shadow=legend_shadow, frameon=legend_frameon)
    plt.savefig(file_name_sp_angles + "." + format, dpi=dpi, transparent=True)

    plt.show()
