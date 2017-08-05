#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat May 27 10:54:41 2017

@author: pach0
"""
import matplotlib.pyplot as plt
import pandas as pd
from numpy import linalg as LA


class DatasetImporter(object):
    def __init__(self, filename, fill_na=False, filter_data=True):
        self.filename = filename
        self.fill_na = fill_na

        self.time = 'time'
        self.gt_x = '_bicycle_odom_rear_wheel.pose.pose.position.x'
        self.gt_y = '_bicycle_odom_rear_wheel.pose.pose.position.y'
        self.gt_z = '_bicycle_odom_rear_wheel.pose.pose.position.z'
        self.gt_v = '_bicycle_odom_rear_wheel.twist.twist.linear.x'
        self.gt_psi = '_imu_lean_yaw.data'
        self.gt_phi = '_imu_lean_pitch.data'
        self.gt_delta = '_imu_steering_yaw.data'

        self.sim_xf = '_bicycle_odom_gps_front.pose.pose.position.x'
        self.sim_xr = '_bicycle_odom_gps_rear.pose.pose.position.x'
        self.sim_yf = '_bicycle_odom_gps_front.pose.pose.position.y'
        self.sim_yr = '_bicycle_odom_gps_rear.pose.pose.position.y'
        self.sim_zf = '_bicycle_odom_gps_front.pose.pose.position.z'
        self.sim_zr = '_bicycle_odom_gps_rear.pose.pose.position.z'
        self.sim_za = '_bicycle_altitude.altitude'
        self.sim_v_x = '_bicycle_gps_rear_velocity.vector.x'
        self.sim_v_y = '_bicycle_gps_rear_velocity.vector.y'
        self.sim_v = 'velocity'
        self.sim_psi = '_imu_lean_noise_yaw.data'
        self.sim_phi = '_imu_lean_noise_pitch.data'
        self.sim_delta = '_imu_steering_noise_yaw.data'

        self.linear_a_x = '_bicycle_imu_1.linear_acceleration.x'
        self.linear_a_y = '_bicycle_imu_1.linear_acceleration.y'
        self.linear_a = 'linear_a'

        self.angular_vel_phi = '_bicycle_imu_1.angular_velocity.y'
        self.angular_vel_delta = '_bicycle_imu_steering.angular_velocity.y'

        self.data = pd.read_csv(self.filename, na_filter=True, parse_dates=[self.time]) \
            .drop_duplicates(subset=[self.time])

        if filter_data:
            self.data = self.data.filter(items=[self.time, self.gt_x, self.gt_y, self.gt_z, self.gt_v,
                                                self.gt_psi, self.gt_phi, self.gt_delta,
                                                self.sim_xf, self.sim_xr, self.sim_yf, self.sim_yr,
                                                self.sim_zf, self.sim_zr, self.sim_za,
                                                self.sim_v_x, self.sim_v_y,
                                                self.sim_psi, self.sim_phi, self.sim_delta,
                                                self.linear_a_x, self.linear_a_y, self.angular_vel_phi,
                                                self.angular_vel_delta])

        # if self.fill_na:
        self.data = self.data.fillna(method='pad')  # fill forward
        self.data = self.data.fillna(method='bfill')  # fill backward

        self.data[self.sim_v] = self.data.apply(self.compute_velocity, axis=1)

        # Set time as index in dataset
        self.data['time_index'] = pd.to_datetime(self.data['time'])
        self.data = self.data.set_index('time_index', drop=True, verify_integrity=True)
        self.data['time'] = self.data.index

    def compute_velocity(self, row):
        return LA.norm([row[self.sim_v_x], row[self.sim_v_y]])

    def plot_dataset(self):
        fig, axes = plt.subplots(nrows=3, ncols=2, figsize=(15, 15))

        self.data.plot(x=self.gt_x, y=self.gt_y, ax=axes[0, 0], kind='scatter', use_index=False, legend=False)
        self.data.plot(x=self.sim_xf, y=self.sim_yf, ax=axes[0, 0], kind='scatter', use_index=False, legend=False)
        self.data.plot(x=self.sim_xr, y=self.sim_yr, ax=axes[0, 0], kind='scatter', use_index=False, legend=False)
        axes[0, 0].set_title('x-y (path)')

        self.data.plot(x=self.time, y=self.sim_psi, ax=axes[0, 1], legend=True)
        self.data.plot(x=self.time, y=self.sim_phi, ax=axes[0, 1], legend=True)
        self.data.plot(x=self.time, y=self.sim_delta, ax=axes[0, 1], legend=True)
        axes[0, 1].set_title('angles')

        self.data.plot(x=self.time, y=self.gt_x, ax=axes[1, 0], legend=True)
        self.data.plot(x=self.time, y=self.sim_xf, ax=axes[1, 0], legend=True)
        self.data.plot(x=self.time, y=self.sim_xr, ax=axes[1, 0], legend=True)
        self.data.plot(x=self.time, y=self.gt_y, ax=axes[1, 0], legend=True)
        self.data.plot(x=self.time, y=self.sim_yf, ax=axes[1, 0], legend=True)
        self.data.plot(x=self.time, y=self.sim_yr, ax=axes[1, 0], legend=True)
        axes[1, 0].set_title('x-y (time)')

        self.data.plot(x=self.time, y=self.gt_v, ax=axes[1, 1], legend=True)
        self.data.plot(x=self.time, y=self.sim_v, ax=axes[1, 1], legend=True)
        axes[1, 1].set_title('v')

        self.data.plot(x=self.time, y=self.linear_a, ax=axes[2, 0], legend=True)
        axes[2, 0].set_title('linear a')

        self.data.plot(x=self.time, y=self.angular_vel_phi, ax=axes[2, 1], legend=True)
        self.data.plot(x=self.time, y=self.angular_vel_delta, ax=axes[2, 1], legend=True)
        axes[2, 1].set_title('angular vel')
