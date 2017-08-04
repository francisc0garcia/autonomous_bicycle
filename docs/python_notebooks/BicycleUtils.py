#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
@author: Francisco Javier Garcia Rosas 2017
"""

from math import radians, sin, cos
import math
import numpy as np
from numpy.random import randn
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from filterpy.stats import plot_covariance_ellipse
import pandas as pd


def convert_object_to_array(gt, zs):
    gt_array = np.zeros((len(gt), 6))
    zs_array = np.zeros((len(zs), 10))

    for i in range(len(gt)):
        gt_array[i] = [gt[i].x, gt[i].y, gt[i].z, gt[i].delta, gt[i].psi, gt[i].phi]

    for i in range(len(zs)):
        zs_array[i] = [zs[i].xf, zs[i].xr,
                       zs[i].yf, zs[i].yr,
                       zs[i].zf, zs[i].zr, zs[i].za,
                       zs[i].delta, zs[i].psi, zs[i].phi]

    return (gt_array, zs_array)
