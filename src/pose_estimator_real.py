#!/usr/bin/env python

import datetime
import time
from numpy import linalg as LA

# ROS dependencies
import rospy
import tf
from tf import TransformBroadcaster
import std_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Pose

# Import filters
from classes.filters.EKF_sigma_model_fusion import *
from classes.filters.UKF_sigma_model_fusion import *
from autonomous_bicycle.msg import EKF_variables, bicycle_state

# Import handlers
from classes.handlers.ImuHandler import *
from classes.handlers.OdometryHandler import *
from classes.handlers.TwistHandler import *
from classes.handlers.Vector3Handler import *
from classes.handlers.FloatHandler import *

rad2degrees = 180.0/math.pi
degrees2rad = math.pi / 180.0


class RobotPoseEstimatorReal:
    """
    Pose estimation of bicycle based on EKF and UKF and Real data
    Subscribe to measurement topics and publish estimated bicycle state X
    """

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

        self.Z = np.zeros((10, 1))  # [xf, xr, yf, yr, zf, zr, za, delta, psi, phi]
        self.Z_plot = np.zeros((10, 1))  # [xf, xr, yf, yr, zf, zr, za, delta, psi, phi]
        self.X = np.array([0.0, 0.0, 0.0, 0.0, np.pi, 0.0])  # [x, y, z, sigma, psi, phi]

        # fading memory filter ( a = 1 to disable)
        self.alpha = 1.01

        self.U = [0.0, 0.0, 0.0]

        self.max_pos = 1e3
        self.max_angle = 2*np.pi

        # Extended kalman filter
        self.filter = EKF_sigma_model_fusion(wheel_distance=self.wheel_distance, dt=self.dt, alpha=self.alpha)

        # Uncented kalman filter
        # self.filter = UKF_sigma_model_fusion(dt=self.dt, w=self.wheel_distance)

        # Particle filter
        # self.filter = ParticleFilter_sigma_model(dt=self.dt, w=self.wheel_distance)

        self.topic_imu_1 = "/bicycle/imu_1"
        self.topic_imu_steering = "/bicycle/imu_steering"
        self.topic_gps_front = "/bicycle/odom_gps_front"
        self.topic_gps_rear = "/bicycle/odom_gps_rear"
        self.topic_odom = "/bicycle/odom"
        self.topic_velocity_twist = "/bicycle/gps_front_velocity"
        self.topic_velocity_vector3 = "/bicycle/gps_front_velocity"
        self.topic_velocity_wheel = "/bicycle/velocity"
        self.topic_altitude = "/bicycle/altitude"
        self.topic_angle_steering = "/bicycle/steering_angle"

        self.offset_za = 0.0

        self.seq = 0

        # create publishers
        self.pub_estimated_odom = rospy.Publisher("/bicycle/odom_filter", Odometry, queue_size=10)
        self.pub_EKF_variables = rospy.Publisher("/bicycle/ekf_data", EKF_variables, queue_size=10)

        self.pub_bicycle_state_measurement = rospy.Publisher("/bicycle/bicycle_state_measurement", bicycle_state, queue_size=10)
        self.pub_bicycle_state_filter = rospy.Publisher("/bicycle/bicycle_state_filter", bicycle_state, queue_size=10)

        self.bicycle_state_measurement = bicycle_state()
        self.bicycle_state_filter = bicycle_state()

        self.estimated_odom_br = TransformBroadcaster()

        # self.velocity_twist = TwistHandler(topic_name=self.topic_velocity_twist)
        self.velocity_wheel = FloatHandler(topic_name=self.topic_velocity_wheel)
        self.altitude_bar = FloatHandler(topic_name=self.topic_altitude)
        self.steering_angle = FloatHandler(topic_name=self.topic_angle_steering, buffer_size=1000)
        self.imu_1 = ImuHandler(topic_name=self.topic_imu_1)
        self.imu_steering = ImuHandler(topic_name=self.topic_imu_steering)
        self.odom_gps_front = OdometryHandler(topic_name=self.topic_gps_front)
        self.odom_gps_rear = OdometryHandler(topic_name=self.topic_gps_rear)

        self.last_time = rospy.Time.now()
        self.real_dt = self.dt

        # Main while loop.
        while not rospy.is_shutdown():
            init_time = datetime.datetime.now()
            self.current_time = rospy.Time.now()

            # update real dt on filter
            self.filter.dt = self.real_dt

            # update variables for filter
            self.update_variables()

            # Update H based on available data
            self.H = np.zeros((10, 6))  # 10 measurements x 6 state variables
            self.H[0, 0] = 1.0 if abs(self.Z[0]) > 0 else 0.0  # xf
            self.H[1, 0] = 1.0 if abs(self.Z[1]) > 0 else 0.0  # xr
            self.H[2, 1] = 1.0 if abs(self.Z[2]) > 0 else 0.0  # yf
            self.H[3, 1] = 1.0 if abs(self.Z[3]) > 0 else 0.0  # yr
            self.H[4, 2] = 1.0 if abs(self.Z[4]) > 0 else 0.0  # zf
            self.H[5, 2] = 1.0 if abs(self.Z[5]) > 0 else 0.0  # zr
            self.H[6, 2] = 1.0 if abs(self.Z[6]) > 0 else 0.0  # za
            self.H[7, 3] = 1.0 if abs(self.Z[7]) > 0 else 0.0  # sigma
            self.H[8, 4] = 1.0 if abs(self.Z[8]) > 0 else 0.0  # psi
            self.H[9, 5] = 1.0 if abs(self.Z[9]) > 0 else 0.0  # phi
            self.filter.H = self.H

            # Prediction step
            # for i in range(5):
            self.filter.prediction(self.U)

            # Update step
            self.filter.update(self.Z)

            self.publish_bicycle_state()
            self.publish_efk_data()

            # send odometry filter
            self.publish_estimated_odometry(self.filter.get_state(), self.current_time, self.child_frame_filter)

            # send odometry Measurement
            if (abs(self.Z[0]) > 0 or abs(self.Z[2]) > 0) or not self.load_rosbag:
                self.publish_estimated_odometry(self.Z, self.current_time, self.child_frame_measurement)

            if self.last_time > self.current_time:
                self.filter.reset_filter()
                rospy.loginfo("Resetting filter")

            self.last_time = self.current_time

            time.sleep(self.dt)
            final_time = datetime.datetime.now()

            # Compute real delta time
            self.real_dt = (final_time - init_time).seconds

    def publish_efk_data(self):
        """ Takes filtered output data and publish it into ROS network """
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
        self.variables.z_xf = self.bound_limit(self.Z_plot[0], -mp, mp)
        self.variables.z_xr = self.bound_limit(self.Z_plot[1], -mp, mp)
        self.variables.z_yf = self.bound_limit(self.Z_plot[2], -mp, mp)
        self.variables.z_yr = self.bound_limit(self.Z_plot[3], -mp, mp)
        self.variables.z_zf = self.bound_limit(self.Z_plot[4], -mp, mp)
        self.variables.z_zr = self.bound_limit(self.Z_plot[5], -mp, mp)
        self.variables.z_za = self.bound_limit(self.Z_plot[6], -mp, mp)
        self.variables.z_sigma = self.bound_limit(self.Z_plot[7], -ma, ma)
        self.variables.z_psi = self.bound_limit(self.Z_plot[8], -ma, ma)
        self.variables.z_phi = self.bound_limit(self.Z_plot[9], -ma, ma)

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
        """ Publish marker with bicycle state and measurement"""
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
        self.bicycle_state_measurement.y = self.bound_limit(self.Z[2], -mp, mp)
        self.bicycle_state_measurement.z = self.bound_limit(self.Z[4], -mp, mp)
        delta = np.arctan2(self.Z[7], 1/self.wheel_distance)
        self.bicycle_state_measurement.steering_delta = self.bound_limit(delta, -ma, ma)
        self.bicycle_state_measurement.heading_psi = self.bound_limit(self.Z[8], -ma, ma)
        self.bicycle_state_measurement.lean_phi = self.bound_limit(self.Z[9], -ma, ma)
        self.pub_bicycle_state_measurement.publish(self.bicycle_state_measurement)

    def update_variables(self):
        """ Update measurements and inputs (self.Z, self.Z_plot and self.U) based on subscribed data"""

        [gps_front_x, gps_front_y, gps_front_z] = self.odom_gps_front.get_value()
        [gps_rear_x, gps_rear_y, gps_rear_z] = self.odom_gps_rear.get_value()

        if not abs(self.offset_za) > 0:
            self.offset_za = self.altitude_bar.get_value()

        z_bar = self.altitude_bar.get_value() - self.offset_za
        imu_1_data = self.imu_1.get_value()
        imu_steering_data = self.imu_steering.get_value()

        # Read steering angle from IMU
        # delta = imu_steering_data[2] - imu_1_data[2]

        # Read steering angle from rotary encoder
        delta = self.steering_angle.get_value()
        delta *= degrees2rad

        sigma = np.tan(delta) / self.wheel_distance
        # psi = (imu_1_data[2] if imu_1_data[2] >= 0 else -imu_1_data[2])  # yaw
        psi = imu_1_data[2]
        # phi = (imu_1_data[1] if imu_1_data[2] >= 0 else -imu_1_data[1])  # pitch
        phi = imu_1_data[1]  # pitch

        angular_vel_phi = imu_1_data[4]  # angular_y
        angular_vel_delta = imu_steering_data[4]  # angular_y

        # Update measurements
        self.Z[0] = gps_front_x  # x
        self.Z[1] = gps_rear_x  # x
        self.Z[2] = gps_front_y  # y
        self.Z[3] = gps_rear_y  # y
        self.Z[4] = gps_front_z  # z
        self.Z[5] = gps_rear_z  # z
        self.Z[6] = z_bar  # z
        self.Z[7] = sigma  # sigma
        self.Z[8] = psi  # psi
        self.Z[9] = phi  # phi

        self.Z_plot[0] = gps_front_x if abs(gps_front_x) > 0.0 else self.Z_plot[0]
        self.Z_plot[1] = gps_rear_x if abs(gps_rear_x) > 0.0 else self.Z_plot[1]
        self.Z_plot[2] = gps_front_y if abs(gps_front_y) > 0.0 else self.Z_plot[2]
        self.Z_plot[3] = gps_rear_y if abs(gps_rear_y) > 0.0 else self.Z_plot[3]
        self.Z_plot[4] = gps_front_z if abs(gps_front_z) > 0.0 else self.Z_plot[4]
        self.Z_plot[5] = gps_rear_z if abs(gps_rear_z) > 0.0 else self.Z_plot[5]
        self.Z_plot[6] = z_bar if abs(z_bar) > 0.0 else self.Z_plot[6]
        self.Z_plot[7] = sigma  # sigma
        self.Z_plot[8] = psi  # psi
        self.Z_plot[9] = phi  # phi

        # [vx, vy, vz] = self.velocity_twist.get_value()
        # velocity_gps = LA.norm([vx, vy])
        velocity = self.velocity_wheel.get_value()*0.23

        self.U[0] = velocity
        self.U[1] = angular_vel_phi
        self.U[2] = angular_vel_delta

    def publish_estimated_odometry(self, x, current_time, child_frame):
        """ Publish estimated bicycle state (Odometry and TF)"""

        odom_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)

        # check if odom is valid
        if not np.isnan(x[0]) and not np.isnan(x[1]) and not np.isnan(x[2]):
            x[2] = 0.0  # TODO: Change it! only for testing
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

    def bound_limit(self, n, minn, maxn):
        return max(min(maxn, n), minn)

# Main function.
if __name__ == '__main__':
    rospy.loginfo('Starting RobotPoseEstimatorReal')

    # Initialize the node and name it.
    rospy.init_node('RobotPoseEstimatorReal')

    obj_temp = RobotPoseEstimatorReal()

