#!/usr/bin/env python

import rospy
import numpy as np
import math
import time
import datetime
from geometry_msgs.msg import Quaternion, Point, Pose
import std_msgs.msg
from tf.transformations import quaternion_from_euler
from tf import TransformBroadcaster
from autonomous_bicycle.msg import EKF_variables, bicycle_state

from classes.EKF_sigma_model import *
from classes.UKF_sigma_model import *
from classes.PF_sigma_model import *

from classes.OdometryHandler import *
from classes.ImuHandler import *
from classes.TwistHandler import *
from classes.Vector3Handler import *
from classes.FloatHandler import *

rad2degrees = 180.0/math.pi
degrees2rad = math.pi / 180.0


class RobotPoseEstimatorReal:
    def __init__(self):
        self.degrees2rad = math.pi / 180.0
        self.variables = EKF_variables()

        # Get values from launch file
        self.rate = rospy.get_param('~rate', 50.0)
        self.load_rosbag = rospy.get_param('~load_rosbag', False)
        self.parent_frame = rospy.get_param('~parent_frame', "world")
        self.child_frame_filter = rospy.get_param('~child_frame_filter', "estimated_odom_filter")
        self.child_frame_measurement = rospy.get_param('~child_frame_measurement', "estimated_odom_measurement")

        self.dt = 1.0/self.rate
        self.wheel_distance = 1.2
        self.number_state_variables = 6

        # [x, y, z, sigma, psi, phi]
        self.X_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.Z = np.zeros((self.number_state_variables, 1))
        self.Z_plot = np.zeros((self.number_state_variables, 1))
        self.X = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # fading memory filter ( a = 1 to disable)
        self.alpha = 1.01

        # static gains of inputs
        self.U_v_static = 0.0
        self.U_phi_dot_static = 0.0
        self.U_delta_dot_static = 0.0
        self.U = [0.0, 0.0, 0.0]

        self.max_pos = 1e5
        self.max_angle = 500.0  # 2*np.pi

        # Extended kalman filter
        self.filter = EKF_sigma_model(wheel_distance=self.wheel_distance, dt=self.dt, alpha=self.alpha)

        # Uncented kalman filter
        # self.filter = UKF_sigma_model(dt=self.dt, w=self.wheel_distance)

        # Particle filter
        # self.filter = ParticleFilter_sigma_model(dt=self.dt, w=self.wheel_distance)

        self.topic_imu_1 = "/bicycle/imu_1"
        self.topic_imu_2 = "/bicycle/imu_2"
        self.topic_imu_steering = "/bicycle/imu_steering"
        self.topic_gps_front = "/bicycle/odom_gps_front"
        self.topic_gps_rear = "/bicycle/odom_gps_rear"
        self.topic_odom = "/bicycle/odom"
        self.topic_velocity_twist = "/bicycle/gps_front_velocity"
        self.topic_velocity_vector3 = "/bicycle/gps_front_velocity"
        self.topic_velocity_wheel = "/bicycle/velocity"

        self.seq = 0

        # create publishers
        self.pub_estimated_odom = rospy.Publisher("/bicycle/odom_filter", Odometry, queue_size=10)
        self.pub_EKF_variables = rospy.Publisher("/bicycle/ekf_data", EKF_variables, queue_size=10)

        self.pub_bicycle_state_measurement = rospy.Publisher("/bicycle/bicycle_state_measurement", bicycle_state, queue_size=10)
        self.pub_bicycle_state_filter = rospy.Publisher("/bicycle/bicycle_state_filter", bicycle_state, queue_size=10)

        self.bicycle_state_measurement = bicycle_state()
        self.bicycle_state_filter = bicycle_state()

        self.estimated_odom_br = TransformBroadcaster()

        self.velocity_twist = TwistHandler(topic_name=self.topic_velocity_twist)
        self.velocity_wheel = FloatHandler(topic_name=self.topic_velocity_wheel)

        self.imu_1 = ImuHandler(topic_name=self.topic_imu_1)
        # self.imu_2 = ImuHandler(topic_name=self.topic_imu_2)

        self.imu_steering = ImuHandler(topic_name=self.topic_imu_steering)

        self.psi_computed = 0.0

        # self.odom_real = OdometryHandler(topic_name=self.topic_odom)
        # self.odom_gps_front = OdometryHandler(topic_name=self.topic_gps_front)
        self.odom_gps_rear = OdometryHandler(topic_name=self.topic_gps_rear)
        [self.prev_x, self.prev_y] = [0.0, 0.0]

        self.last_time = rospy.Time.now()
        self.real_dt = 0.01

        # Main while loop.
        while not rospy.is_shutdown():
            init_time = datetime.datetime.now()
            self.current_time = rospy.Time.now()

            # update real dt on filter
            self.filter.dt = self.real_dt

            # update variables for filter
            self.update_variables()

            # Update H based on available GPS data
            self.H = np.eye(6)
            if abs(self.Z[0]) == 0 or abs(self.Z[1]) == 0:
                self.H[0, 0] = 0.0  # x
                self.H[1, 1] = 0.0  # y
                self.H[2, 2] = 0.0  # z

            self.filter.H = self.H

            # Prediction step
            self.filter.prediction(self.U)

            # Update step
            self.filter.update(self.Z)

            # send odometry filter
            self.publish_estimated_odometry(self.filter.get_state(), self.current_time, self.child_frame_filter)

            # send odometry Measurement
            if (abs(self.Z[0]) > 0 or abs(self.Z[1]) > 0) or not self.load_rosbag:
                self.publish_estimated_odometry(self.Z, self.current_time, self.child_frame_measurement)

            self.publish_bicycle_state()
            self.publish_efk_data()

            if self.last_time > self.current_time:
                self.filter.reset_filter()
                rospy.loginfo("Resetting filter")

            self.last_time = self.current_time

            time.sleep(self.dt)
            final_time = datetime.datetime.now()

            # Compute real delta time
            self.real_dt = (final_time - init_time).seconds

    def publish_efk_data(self):
        # update Header
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        self.variables.header = h

        xs = self.filter.get_state()

        mp = self.max_pos
        ma = self.max_angle

        # Update state vector
        self.variables.xs_x = self.bound_limit(xs[0], -mp, mp)
        self.variables.xs_y = self.bound_limit(xs[1], -mp, mp)
        self.variables.xs_z = self.bound_limit(xs[2], -mp, mp)
        self.variables.xs_sigma = self.bound_limit(xs[3], -ma, ma)
        self.variables.xs_psi = self.bound_limit(xs[4], -ma, ma)
        self.variables.xs_phi = self.bound_limit(xs[5], -ma, ma)

        # Update measurement vector
        self.variables.z_x = self.bound_limit(self.Z_plot[0], -mp, mp)
        self.variables.z_y = self.bound_limit(self.Z_plot[1], -mp, mp)
        self.variables.z_z = self.bound_limit(self.Z_plot[2], -mp, mp)
        self.variables.z_sigma = self.bound_limit(self.Z_plot[3], -ma, ma)
        self.variables.z_psi = self.bound_limit(self.Z_plot[4], -ma, ma)
        self.variables.z_phi = self.bound_limit(self.Z_plot[5], -ma, ma)

        # Update input vector
        self.variables.u_v = self.U[0]
        self.variables.u_phi_dot = self.U[1]
        self.variables.u_delta_dot = self.U[2]

        # update kalman gain
        self.variables.k_x = self.bound_limit(self.filter.K[0, 0], -mp, mp)
        self.variables.k_y = self.bound_limit(self.filter.K[1, 1], -mp, mp)
        self.variables.k_z = self.bound_limit(self.filter.K[2, 2], -mp, mp)
        self.variables.k_sigma = self.bound_limit(self.filter.K[3, 3], -ma, ma)
        self.variables.k_psi = self.bound_limit(self.filter.K[4, 4], -ma, ma)
        self.variables.k_phi = self.bound_limit(self.filter.K[5, 5], -ma, ma)

        # update process covariance
        self.variables.p_x = self.bound_limit(self.filter.P[0, 0], -mp, mp)
        self.variables.p_y = self.bound_limit(self.filter.P[1, 1], -mp, mp)
        self.variables.p_z = self.bound_limit(self.filter.P[2, 2], -mp, mp)
        self.variables.p_sigma = self.bound_limit(self.filter.P[3, 3], -ma, ma)
        self.variables.p_psi = self.bound_limit(self.filter.P[4, 4], -ma, ma)
        self.variables.p_phi = self.bound_limit(self.filter.P[5, 5], -ma, ma)

        self.pub_EKF_variables.publish(self.variables)

    def publish_bicycle_state(self):
        mp = self.max_pos
        ma = self.max_angle

        # Publish filtered state
        xs = self.filter.get_state()
        self.bicycle_state_filter.x = self.bound_limit(xs[0], -mp, mp)
        self.bicycle_state_filter.y = self.bound_limit(xs[1], -mp, mp)
        self.bicycle_state_filter.z = self.bound_limit(xs[2], -mp, mp)
        self.bicycle_state_filter.heading_psi = self.bound_limit(xs[4], -ma, ma)
        delta = np.arctan2(xs[3], 1/self.wheel_distance)
        self.bicycle_state_filter.steering_delta = self.bound_limit(delta, -ma, ma)
        self.bicycle_state_filter.lean_phi = self.bound_limit(xs[5], -ma, ma)
        self.pub_bicycle_state_filter.publish(self.bicycle_state_filter)

        # Publish measurement state
        self.bicycle_state_measurement.x = self.bound_limit(self.Z[0], -mp, mp)
        self.bicycle_state_measurement.y = self.bound_limit(self.Z[1], -mp, mp)
        self.bicycle_state_measurement.z = self.bound_limit(self.Z[2], -mp, mp)
        self.bicycle_state_measurement.heading_psi = self.bound_limit(self.Z[4], -ma, ma)
        delta = np.arctan2(self.Z[3], 1/self.wheel_distance)
        self.bicycle_state_measurement.steering_delta = self.bound_limit(delta, -ma, ma)
        self.bicycle_state_measurement.lean_phi = self.bound_limit(self.Z[5], -ma, ma)
        self.pub_bicycle_state_measurement.publish(self.bicycle_state_measurement)

    def update_variables(self):
        # Read variables
        # [odom_real_x, odom_real_y, odom_real_z] = self.odom_real.get_value()
        # [gps_front_x, gps_front_y, gps_front_z] = self.odom_gps_front.get_value()
        [gps_rear_x, gps_rear_y, gps_rear_z] = self.odom_gps_rear.get_value()

        imu_1_data = self.imu_1.get_value()
        # imu_2_data = self.imu_2.get_value()
        imu_steering_data = self.imu_steering.get_value()

        if abs(gps_rear_x) > 0 and abs(gps_rear_y) > 0:
            delta_x = gps_rear_x - self.prev_x
            delta_y = gps_rear_y - self.prev_y

            self.psi_computed = np.arctan2(delta_y, delta_x)
            [self.prev_x, self.prev_y] = [gps_rear_x, gps_rear_y]

        # lean_phi_avg = -((imu_1_data[1] + offset_lean_imu1) + (imu_steering_data[1] + offset_lean_steer))/2
        lean_phi_avg = imu_1_data[1] + 0.0  # -0.27

        delta = imu_steering_data[2]
        sigma = np.tan(delta) / self.wheel_distance  # sigma
        # sigma = imu_steering_data[2] #+ np.pi
        # fix upper/lower limits
        # sigma = self.bound_limit(sigma, -2*np.pi, 2*np.pi)
        # lean_phi_avg = self.bound_limit(lean_phi_avg, -2*np.pi, 2*np.pi)
        self.psi_computed = self.bound_limit(self.psi_computed, -2*np.pi, 2*np.pi)

        # Update measurements
        self.Z[0] = gps_rear_x   # x
        self.Z[1] = gps_rear_y   # y
        self.Z[2] = gps_rear_z   # z
        self.Z[3] = sigma
        self.Z[4] = self.psi_computed  # psi
        self.Z[5] = lean_phi_avg  # phi

        if abs(gps_rear_x) > 0.0:
            self.Z_plot[0] = gps_rear_x   # x
        if abs(gps_rear_y) > 0.0:
            self.Z_plot[1] = gps_rear_y   # y
        if abs(gps_rear_z) > 0.0:
            self.Z_plot[2] = gps_rear_z   # z

        self.Z_plot[3] = sigma  # sigma
        self.Z_plot[4] = self.psi_computed  # psi
        self.Z_plot[5] = lean_phi_avg  # phi

        [vx, vy, vz] = self.velocity_twist.get_value()

        # Update inputs
        #if abs(vx) > 0 or abs(vy) > 0:
        #    velocity = np.sqrt(vx**2.0 + vy**2.0)
        #else:
        #    velocity = self.velocity_wheel.get_value()*0.23

        velocity = self.velocity_wheel.get_value()*0.23

        self.U[0] = velocity + self.U_v_static
        self.U[1] = imu_1_data[4] + self.U_phi_dot_static  # angular_y
        self.U[2] = imu_steering_data[4] + self.U_delta_dot_static  # angular_y

    def publish_estimated_odometry(self, x, current_time, child_frame):
        odom_quat = quaternion_from_euler(0.0, 0.0, 0.0)

        # check if odom is valid
        if not np.isnan(x[0]) and not np.isnan(x[1]) and not np.isnan(x[2]):
            # publish the transform over tf
            self.estimated_odom_br.sendTransform(
                (x[0], x[1], x[2]), odom_quat, current_time, child_frame,
                self.parent_frame
            )

            # publish the odometry message
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = self.parent_frame
            odom.pose.pose = Pose(Point(x[0], x[1], x[2]), Quaternion(*odom_quat))
            odom.child_frame_id = child_frame
            self.pub_estimated_odom.publish(odom)

    def normalize_angle(self, x):
        x = x % (2 * np.pi)    # force in range [0, 2 pi)
        # if x > np.pi:          # move to [-pi, pi)
        #    x -= 2 * np.pi
        return x

    def bound_limit(self, n, minn, maxn):
        return max(min(maxn, n), minn)

# Main function.
if __name__ == '__main__':
    rospy.loginfo('Starting RobotPoseEstimatorReal')

    # Initialize the node and name it.
    rospy.init_node('RobotPoseEstimatorReal')

    obj_temp = RobotPoseEstimatorReal()

