#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
@author: Francisco Garcia
"""
import matplotlib.pyplot as plt


class SimulationDatasetHelper(object):
    def __init__(self, data):
        self.data = data
        self.time = 'time'

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

    def plot_dataset(self):
        fig, axes = plt.subplots(nrows=3, ncols=2, figsize=(15, 15))

        self.data.plot(x=self.real_xf, y=self.real_yf, ax=axes[0, 0], kind='scatter', use_index=False, legend=False)
        self.data.plot(x=self.real_xr, y=self.real_yr, ax=axes[0, 0], kind='scatter', use_index=False, legend=False)
        axes[0, 0].set_title('x-y (path)')

        self.data.plot(x=self.time, y=self.real_psi, ax=axes[0, 1], legend=True)
        self.data.plot(x=self.time, y=self.real_phi, ax=axes[0, 1], legend=True)
        self.data.plot(x=self.time, y=self.real_delta, ax=axes[0, 1], legend=True)
        axes[0, 1].set_title('angles')

        self.data.plot(x=self.time, y=self.real_xf, ax=axes[1, 0], legend=True)
        self.data.plot(x=self.time, y=self.real_xr, ax=axes[1, 0], legend=True)
        self.data.plot(x=self.time, y=self.real_yf, ax=axes[1, 0], legend=True)
        self.data.plot(x=self.time, y=self.real_yr, ax=axes[1, 0], legend=True)
        axes[1, 0].set_title('x-y (time)')

        self.data.plot(x=self.time, y=self.real_v, ax=axes[1, 1], legend=True)
        axes[1, 1].set_title('v')

        self.data.plot(x=self.time, y=self.linear_a, ax=axes[2, 0], legend=True)
        axes[2, 0].set_title('linear a')

        self.data.plot(x=self.time, y=self.angular_vel_phi, ax=axes[2, 1], legend=True)
        self.data.plot(x=self.time, y=self.angular_vel_delta, ax=axes[2, 1], legend=True)
        axes[2, 1].set_title('angular vel')


class RealDatasetHelper(object):
    def __init__(self, data):
        self.data = data
        self.time = 'time'

        self.real_lat_f = '_bicycle_gps_front.latitude'
        self.real_lat_r = '_bicycle_gps_rear.latitude'
        self.real_lon_f = '_bicycle_gps_front.longitude'
        self.real_lon_r = '_bicycle_gps_rear.longitude'
        self.gps_zone_number = 'gps_zone_number'
        self.gps_zone_letter = 'gps_zone_letter'
        self.predicted_lat = 'predicted_lat'
        self.predicted_lon = 'predicted_lon'

        self.real_xf = '_bicycle_odom_gps_front.pose.pose.position.x'
        self.real_xr = '_bicycle_odom_gps_rear.pose.pose.position.x'
        self.real_yf = '_bicycle_odom_gps_front.pose.pose.position.y'
        self.real_yr = '_bicycle_odom_gps_rear.pose.pose.position.y'
        self.real_zf = '_bicycle_gps_front.altitude'
        self.real_zr = '_bicycle_gps_rear.altitude'
        self.real_za = '_bicycle_altitude.data'
        self.real_v_x = '_bicycle_gps_front_velocity.twist.linear.x'
        self.real_v_y = '_bicycle_gps_front_velocity.twist.linear.y'
        self.real_v = '_bicycle_velocity.data'
        self.real_v_gps = 'real_v_gps'
        self.real_psi_gps = 'real_psi_gps'
        self.real_psi = '_imu_lean_yaw.data'
        self.real_phi = '_imu_lean_pitch.data'
        self.real_delta = 'real_steering_delta'
        self.real_delta_imu = '_imu_steering_yaw.data'

        self.imu_lean_x = '_bicycle_imu_1.orientation.x'
        self.imu_lean_y = '_bicycle_imu_1.orientation.y'
        self.imu_lean_z = '_bicycle_imu_1.orientation.z'
        self.imu_lean_w = '_bicycle_imu_1.orientation.w'

        self.imu_steering_x = '_bicycle_imu_steering.orientation.x'
        self.imu_steering_y = '_bicycle_imu_steering.orientation.y'
        self.imu_steering_z = '_bicycle_imu_steering.orientation.z'
        self.imu_steering_w = '_bicycle_imu_steering.orientation.w'

        self.linear_a_x = '_bicycle_imu_1.linear_acceleration.x'
        self.linear_a_y = '_bicycle_imu_1.linear_acceleration.y'
        self.linear_a = 'linear_a'

        self.angular_vel_phi = '_bicycle_imu_1.angular_velocity.y'
        self.angular_vel_delta = '_bicycle_imu_steering.angular_velocity.y'

    def plot_dataset(self):
        fig, axes = plt.subplots(nrows=3, ncols=2, figsize=(15, 15))

        self.data.plot(x=self.real_xf, y=self.real_yf, ax=axes[0, 0], kind='scatter', use_index=False, legend=False)
        self.data.plot(x=self.real_xr, y=self.real_yr, ax=axes[0, 0], kind='scatter', use_index=False, legend=False)
        axes[0, 0].set_title('x-y (path)')

        self.data.plot(x=self.time, y=self.real_psi, ax=axes[0, 1], legend=True)
        self.data.plot(x=self.time, y=self.real_phi, ax=axes[0, 1], legend=True)
        self.data.plot(x=self.time, y=self.real_delta, ax=axes[0, 1], legend=True)
        axes[0, 1].set_title('angles')

        self.data.plot(x=self.time, y=self.real_xf, ax=axes[1, 0], legend=True)
        self.data.plot(x=self.time, y=self.real_xr, ax=axes[1, 0], legend=True)
        self.data.plot(x=self.time, y=self.real_yf, ax=axes[1, 0], legend=True)
        self.data.plot(x=self.time, y=self.real_yr, ax=axes[1, 0], legend=True)
        axes[1, 0].set_title('x-y (time)')

        self.data.plot(x=self.time, y=self.real_v, ax=axes[1, 1], legend=True)
        axes[1, 1].set_title('v')

        self.data.plot(x=self.time, y=self.linear_a, ax=axes[2, 0], legend=True)
        axes[2, 0].set_title('linear a')

        self.data.plot(x=self.time, y=self.angular_vel_phi, ax=axes[2, 1], legend=True)
        self.data.plot(x=self.time, y=self.angular_vel_delta, ax=axes[2, 1], legend=True)
        axes[2, 1].set_title('angular vel')

