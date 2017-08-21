#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
@author: Francisco Javier Garcia
"""
import matplotlib.pyplot as plt
import pandas as pd
from numpy import linalg as LA
import utm
import transforms3d


class RealDatasetImporter(object):
    def __init__(self, filename, filter_data=True):
        self.filename = filename
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
        self.real_delta = '_imu_steering_yaw.data'

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

        self.data = pd.read_csv(self.filename, na_filter=True, parse_dates=[self.time]) \
            .drop_duplicates(subset=[self.time])

        if filter_data:
            self.data = self.data.filter(items=[self.time,
                                                self.real_zf, self.real_zr, self.real_za,
                                                self.real_v_x, self.real_v_y, self.real_v,
                                                self.linear_a_x, self.linear_a_y, self.angular_vel_phi,
                                                self.angular_vel_delta,
                                                self.real_lat_f, self.real_lat_r, self.real_lon_f, self.real_lon_r,
                                                self.imu_lean_x, self.imu_lean_y, self.imu_lean_z, self.imu_lean_w,
                                                self.imu_steering_x, self.imu_steering_y, self.imu_steering_z,
                                                self.imu_steering_w])

        self.data = self.data.fillna(method='pad')  # fill forward
        self.data = self.data.fillna(method='bfill')  # fill backward

        # compute variables
        self.data = self.data.apply(self.compute_variables, axis=1)

        # Set time as index in datasetreal_v_gps
        self.data['time_index'] = pd.to_datetime(self.data['time'])
        self.data = self.data.set_index('time_index', drop=True, verify_integrity=True)
        self.data['time'] = self.data.index

    def compute_variables(self, row):
        # Convert GPS into UTM --------------------------------------------------
        utm_front = utm.from_latlon(row[self.real_lat_f], row[self.real_lon_f])
        row[self.real_xf] = utm_front[0]
        row[self.real_yf] = utm_front[1]
        row[self.gps_zone_number] = utm_front[2]
        row[self.gps_zone_letter] = utm_front[3]

        utm_rear = utm.from_latlon(row[self.real_lat_r], row[self.real_lon_r])
        row[self.real_xr] = utm_rear[0]
        row[self.real_yr] = utm_rear[1]
        # ------------------------------------------------------------------------

        # Update linear acceleration
        row[self.linear_a] = LA.norm([row[self.linear_a_x], row[self.linear_a_y]])

        # Use GPS velocity
        row[self.real_v_gps] = LA.norm([row[self.real_v_x], row[self.real_v_y]])

        # Update angles 
        [lx, ly, lz, lw] = [row[self.imu_lean_x], row[self.imu_lean_y],
                            row[self.imu_lean_z], row[self.imu_lean_w]]
        angles_lean = transforms3d.euler.quat2euler([lw, lx, ly, lz])
        row[self.real_phi] = angles_lean[1]

        [sx, sy, sz, sw] = [row[self.imu_steering_x], row[self.imu_steering_y],
                            row[self.imu_steering_z], row[self.imu_steering_w]]
        angles_steering = transforms3d.euler.quat2euler([sw, sx, sy, sz])
        row[self.real_delta] = angles_steering[2] - angles_lean[2]

        # Use IMU psi
        row[self.real_psi] = (angles_lean[2] if angles_lean[2] >= 0 else -angles_lean[2])

        return row

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
