#!/usr/bin/env python

# System dependencies
import datetime
import time

# ROS dependencies
import rospy
import std_msgs.msg
import tf
from autonomous_bicycle.msg import EKF_variables
from geometry_msgs.msg import Quaternion, Point, Pose
from nav_msgs.msg import Odometry

# Import filters
from classes.filters.EKF_sigma_model_fusion import *
from classes.filters.UKF_sigma_model_fusion import *

# Import handlers
from classes.handlers.ImuHandler import *
from classes.handlers.OdometryHandler import *
from classes.handlers.TwistHandler import *
from classes.handlers.Vector3Handler import *


class RobotPoseEstimator:
    """
    Pose estimation of bicycle based on EKF and UKF and simulated data
    Subscribe to measurement topics and publish estimated bicycle state X
    """

    def __init__(self):
        self.variables = EKF_variables()

        # Get values from launch file
        self.rate = rospy.get_param('~rate', 50.0)
        self.parent_frame = "world"
        self.child_frame = "estimated_odom"

        self.dt = 1.0 / self.rate
        self.wheel_distance = 1.2
        self.number_state_variables = 6

        self.Z = np.zeros((10, 1))  # [xf, xr, yf, yr, zf, zr, za, delta, psi, phi]
        self.Z_plot = np.zeros((10, 1))  # [xf, xr, yf, yr, zf, zr, za, delta, psi, phi]
        self.X = np.array([0.0, 0.0, 0.0, 0.0, np.pi, 0.0])  # [x, y, z, sigma, psi, phi]

        # fading memory filter ( a = 1 to disable)
        self.alpha = 1.01

        # Initial input
        self.U = [0.0, 0.0, 0.0]

        # Extended kalman filter
        # self.filter = EKF_sigma_model(wheel_distance=self.wheel_distance, dt=self.dt, alpha=self.alpha)
        # self.filter = EKF_sigma_model_fusion(wheel_distance=self.wheel_distance, dt=self.dt, alpha=self.alpha)
        self.filter = EKF_sigma_model_fusion()
        # Uncented kalman filter
        # self.filter = UKF_sigma_model(dt=self.dt, w=self.wheel_distance)
        self.filter = UKF_sigma_model_fusion(dt=self.dt, w=self.wheel_distance)

        # Particle filter
        # self.filter = ParticleFilter_sigma_model(dt=self.dt, w=self.wheel_distance)

        self.topic_imu_1 = "/bicycle/imu_1"
        self.topic_imu_steering = "/bicycle/imu_steering"
        self.topic_gps_front = "/bicycle/odom_gps_front"
        self.topic_gps_rear = "/bicycle/odom_gps_rear"
        self.topic_odom = "/bicycle/odom"
        self.topic_velocity_twist = "/bicycle/gps_front_velocity"
        self.topic_velocity_vector3 = "/bicycle/gps_front_velocity"

        self.seq = 0

        # create publishers
        self.pub_estimated_odom = rospy.Publisher("/bicycle/odom_filter", Odometry, queue_size=1)
        self.pub_EKF_variables = rospy.Publisher("/bicycle/ekf_data", EKF_variables, queue_size=1)

        self.estimated_odom_br = tf.TransformBroadcaster()

        # Init Handlers
        self.velocity_vector3 = Vector3Handler(topic_name=self.topic_velocity_vector3)
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
            for i in range(1):
                self.filter.prediction(self.U)

            # Update step
            self.filter.update(self.Z)

            # Send output of filter as odometry message
            self.publish_estimated_odometry(self.filter.get_state(), self.current_time)

            self.publish_efk_data()

            if self.last_time > self.current_time:
                self.filter.reset_filter()
                rospy.loginfo("Resetting filter EKF_sigma_model")

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

        xs = self.filter.get_state()  # update state

        # Update state vector
        self.variables.xs_x = xs[0]
        self.variables.xs_y = xs[1]
        self.variables.xs_z = xs[2]
        self.variables.xs_sigma = xs[3]
        self.variables.xs_psi = xs[4]
        self.variables.xs_phi = xs[5]

        # Update measurement vector
        self.variables.z_xf = self.Z_plot[0]
        self.variables.z_xr = self.Z_plot[1]
        self.variables.z_yf = self.Z_plot[2]
        self.variables.z_yr = self.Z_plot[3]
        self.variables.z_zf = self.Z_plot[4]
        self.variables.z_zr = self.Z_plot[5]
        self.variables.z_za = self.Z_plot[6]
        self.variables.z_sigma = self.Z_plot[7]
        self.variables.z_psi = self.Z_plot[8]
        self.variables.z_phi = self.Z_plot[9]

        # Update input vector
        self.variables.u_v = self.U[0]
        self.variables.u_phi_dot = self.U[1]
        self.variables.u_delta_dot = self.U[2]

        # update kalman gain
        self.variables.k_x = self.filter.K[0, 0]
        self.variables.k_y = self.filter.K[1, 1]
        self.variables.k_z = self.filter.K[2, 2]
        self.variables.k_sigma = self.filter.K[3, 3]
        self.variables.k_psi = self.filter.K[4, 4]
        self.variables.k_phi = self.filter.K[5, 5]

        self.variables.p_x = self.filter.y[0]
        self.variables.p_y = self.filter.y[1]
        self.variables.p_z = self.filter.y[2]
        self.variables.p_sigma = self.filter.y[3]
        self.variables.p_psi = self.filter.y[4]
        self.variables.p_phi = self.filter.y[5]

        self.pub_EKF_variables.publish(self.variables)

    def update_variables(self):
        """ Update measurements and inputs (self.Z, self.Z_plot and self.U) based on subscribed data"""
        [gps_front_x, gps_front_y, gps_front_z] = self.odom_gps_front.get_value()
        [gps_rear_x, gps_rear_y, gps_rear_z] = self.odom_gps_rear.get_value()

        imu_1_data = self.imu_1.get_value()
        imu_steering_data = self.imu_steering.get_value()

        psi = imu_1_data[2]  # yaw
        phi = imu_1_data[1] + np.pi / 2  # pitch
        delta = imu_steering_data[2]  # yaw

        # Update measurements
        self.Z[0] = gps_front_x  # x
        self.Z[1] = gps_rear_x  # x
        self.Z[2] = gps_front_y  # y
        self.Z[3] = gps_rear_y  # y
        self.Z[4] = gps_front_z  # z
        self.Z[5] = gps_rear_z  # z
        self.Z[6] = 0  # bar_altitude  # z
        self.Z[7] = np.tan(delta) / self.wheel_distance  # sigma
        self.Z[8] = psi  # psi
        self.Z[9] = phi  # phi

        self.Z_plot[0] = gps_front_x if abs(gps_front_x) > 0.0 else self.Z_plot[0]
        self.Z_plot[1] = gps_rear_x if abs(gps_rear_x) > 0.0 else self.Z_plot[1]
        self.Z_plot[2] = gps_front_y if abs(gps_front_y) > 0.0 else self.Z_plot[2]
        self.Z_plot[3] = gps_rear_y if abs(gps_rear_y) > 0.0 else self.Z_plot[3]
        self.Z_plot[4] = gps_front_z if abs(gps_front_z) > 0.0 else self.Z_plot[4]
        self.Z_plot[5] = gps_rear_z if abs(gps_rear_z) > 0.0 else self.Z_plot[5]
        self.Z_plot[7] = np.tan(delta) / self.wheel_distance  # sigma
        self.Z_plot[8] = psi  # psi
        self.Z_plot[9] = phi  # phi

        [vx, vy, vz] = self.velocity_vector3.get_value()

        # Update inputs
        self.U[0] = np.sqrt(vx ** 2.0 + vy ** 2.0)  # Velocity
        self.U[1] = imu_1_data[4]  # angular_y
        self.U[2] = imu_steering_data[4]  # angular_y

    def publish_estimated_odometry(self, x, current_time):
        """ Publish estimated bicycle state (Odometry and TF)"""
        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)

        # first, we'll publish the transform over tf
        self.estimated_odom_br.sendTransform(
            (-x[0], -x[1], x[2]),
            odom_quat,
            current_time,
            self.parent_frame,
            self.child_frame
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.parent_frame

        # set the position
        odom.pose.pose = Pose(Point(-x[0], -x[1], x[2]), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = self.child_frame

        # publish the message
        self.pub_estimated_odom.publish(odom)


# Main function.
if __name__ == '__main__':
    rospy.loginfo('Starting RobotPoseEstimator')

    # Initialize the node and name it.
    rospy.init_node('RobotPoseEstimator')

    try:
        obj_temp = RobotPoseEstimator()
    except rospy.ROSInterruptException:
        pass
