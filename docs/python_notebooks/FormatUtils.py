#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
@author: Francisco Javier Garcia Rosas 2017
"""
import numpy as np

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
import matplotlib.ticker as mtick

# Set global format
font = {'family': 'sans-serif', 'weight': 'normal', 'size': 22}
matplotlib.rc('font', **font)

dpi = 100
[fig_x, fig_y] = [10, 6]
[legend_frameon, legend_shadow] = [True, False]
[color_gt, color_prediction, color_noise_1, color_noise_2, color_noise_3] = ['red', 'blue', 'black', 'gray', 'silver']

params = {'legend.fontsize': 'xx-small',
          'figure.figsize': (fig_x, fig_y),
          'axes.titlesize': 'medium',
          'axes.labelsize': 'small',
          'xtick.labelsize': 'x-small',
          'ytick.labelsize': 'x-small'}

pylab.rcParams.update(params)


def set_format_subplot(sp, title, xlabel, ylabel, xmin=None, xmax=None, ymin=None, ymax=None):
    sp.set_title(title, fontweight='bold')

    sp.set_xlabel(xlabel)
    sp.set_ylabel(ylabel)
    [x_div, y_div] = [5, 5]

    if xmin is not None and xmax is not None:
        sp.set_xlim([xmin, xmax])
        plt.xticks(np.linspace(xmin, xmax, x_div, endpoint=True))

    if ymin is not None and ymax is not None:
        sp.set_ylim([ymin, ymax])
        plt.yticks(np.linspace(ymin, ymax, y_div, endpoint=True))

    sp.yaxis.set_major_formatter(mtick.FormatStrFormatter('%.2f'))
    sp.xaxis.set_major_formatter(mtick.FormatStrFormatter('%.2f'))

    # if xmax > 0.0: sp.xaxis.set_major_locator(plt.MultipleLocator(xmax / x_div))
    # if xmax > 0.0: sp.xaxis.set_minor_locator(plt.MultipleLocator(xmax / (2 * x_div)))
    # if ymax > 0.0: sp.yaxis.set_major_locator(plt.MultipleLocator(ymax / y_div))
    # if ymax > 0.0: sp.yaxis.set_minor_locator(plt.MultipleLocator(ymax / (2 * y_div)))

    sp.grid(True)

    return sp
