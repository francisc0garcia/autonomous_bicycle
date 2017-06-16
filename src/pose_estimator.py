#!/usr/bin/env python

import rospy
import numpy as np
import time
import datetime
from geometry_msgs.msg import Quaternion, Point, Pose
import std_msgs.msg
import tf
from autonomous_bicycle.msg import EKF_variables
from autonomous_bicycle.cfg import pose_estimation_interactionConfig

from dynamic_reconfigure.server import Server

from classes.EKF_sigma_model import *
from classes.UKF_sigma_model import *
from classes.PF_sigma_model import *

from classes.OdometryHandler import *
from classes.ImuHandler import *
from classes.TwistHandler import *
from classes.Vector3Handler import *

rad2degrees = 180.0/math.pi
degrees2rad = math.pi / 180.0

class RobotPoseEstimator:
    def __init__(self):
        self.degrees2rad = math.pi / 180.0
        self.variables = EKF_variables()

        # Get values from launch file
        self.rate = rospy.get_param('~rate', 50.0)
        self.parent_frame = "world"
        self.child_frame = "estimated_odom"

        self.dt = 1.0/self.rate
        self.wheel_distance = 1.2
        self.number_state_variables = 6

        # [x, y, z, v, psi, phi, delta]
        self.X_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.Z = np.zeros((self.number_state_variables, 1))  # [x, y, z, sigma, psi, phi]
        self.Z_plot = np.zeros((self.number_state_variables, 1))
        self.X = np.array([0.0, 0.0, 0.0, 0.0, np.pi, 0.0])  # [x, y, z, sigma, psi, phi]

        # fading memory filter ( a = 1 to disable)
        self.alpha = 1.01

        # static gains of inputs
        self.U_v_static = 0.0  # 1.0
        self.U_phi_dot_static = 0.0  # 1.58
        self.U_delta_dot_static = 0.0  # 0.05

        self.U = [0.0, 0.0, 0.0]

        # Extended kalman filter
        # self.filter = EKF_sigma_model(wheel_distance=self.wheel_distance, dt=self.dt, alpha=self.alpha)

        # Uncented kalman filter
        self.filter = UKF_sigma_model(dt=self.dt, w=self.wheel_distance)

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

        # define dynamic_reconfigure callback
        self.srv = Server(pose_estimation_interactionConfig, self.reconfig_callback)

        self.seq = 0

        # create publishers
        self.pub_estimated_odom = rospy.Publisher("/bicycle/odom_filter", Odometry, queue_size=1)
        self.pub_EKF_variables = rospy.Publisher("/bicycle/ekf_data", EKF_variables, queue_size=1)

        self.estimated_odom_br = tf.TransformBroadcaster()

        # self.velocity_twist = TwistHandler(topic_name=self.topic_velocity_twist)
        self.velocity_vector3 = Vector3Handler(topic_name=self.topic_velocity_vector3)

        self.imu_1 = ImuHandler(topic_name=self.topic_imu_1)
        # self.imu_2 = ImuHandler(topic_name=self.topic_imu_2)
        self.imu_steering = ImuHandler(topic_name=self.topic_imu_steering)

        # self.odom_real = OdometryHandler(topic_name=self.topic_odom)
        # self.odom_gps_front = OdometryHandler(topic_name=self.topic_gps_front)
        self.odom_gps_rear = OdometryHandler(topic_name=self.topic_gps_rear)

        rospy.on_shutdown(self.shutdown_node)
        # rate = rospy.Rate(self.rate)

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

    def reconfig_callback(self, config, level):
        # process noise covariance Q ------------------
        self.Q_std = [config['sigma_v'], config['sigma_phi_dot'], config['sigma_delta_dot']]  # v, phi_dot, delta_dot
        self.filter.update_Q(self.Q_std)

        # measurement noise covariance R --------------
        self.R_std = [config['R_x']**2.0,  # x
                      config['R_y']**2.0,  # y
                      config['R_z']**2.0,  # z
                      config['R_sigma']**2.0,  # sigma
                      config['R_psi']**2.0,  # psi
                      config['R_phi']**2.0]  # phi

        rospy.loginfo("EKF parameters changed: Q_std: " + str(self.Q_std) + " - R_std: " + str(self.R_std))
        self.filter.update_R(self.R_std)

        self.filter.reset_filter()

        rospy.loginfo("Filter updated! ")

        return config

    def publish_efk_data(self):
        # update Header
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        self.variables.header = h

        xs = self.filter.get_state()

        # Update state vector
        self.variables.xs_x = xs[0]
        self.variables.xs_y = xs[1]
        self.variables.xs_z = xs[2]
        self.variables.xs_sigma = xs[3]
        self.variables.xs_psi = xs[4]
        self.variables.xs_phi = xs[5]

        # Update measurement vector
        self.variables.z_x = self.Z_plot[0]
        self.variables.z_y = self.Z_plot[1]
        self.variables.z_z = self.Z_plot[2]
        self.variables.z_sigma = self.Z_plot[3]
        self.variables.z_psi = self.Z_plot[4]
        self.variables.z_phi = self.Z_plot[5]

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

        # update process covariance
        # self.variables.p_x = self.filter.e # self.filter.P[0, 0]
        # self.variables.p_y = self.filter.P[1, 1]
        # self.variables.p_z = self.filter.P[2, 2]
        # self.variables.p_sigma = self.filter.P[3, 3]
        # self.variables.p_psi = self.filter.P[4, 4]
        # self.variables.p_phi = self.filter.P[5, 5]

        self.variables.p_x = self.filter.y[0]
        self.variables.p_y = self.filter.y[1]
        self.variables.p_z = self.filter.y[2]
        self.variables.p_sigma = self.filter.y[3]
        self.variables.p_psi = self.filter.y[4]
        self.variables.p_phi = self.filter.y[5]

        self.pub_EKF_variables.publish(self.variables)

    def update_variables(self):
        # Read variables
        # [odom_real_x, odom_real_y, odom_real_z] = self.odom_real.get_value()
        #[gps_front_x, gps_front_y, gps_front_z] = self.odom_gps_front.get_value()
        [gps_rear_x, gps_rear_y, gps_rear_z] = self.odom_gps_rear.get_value()

        imu_1_data = self.imu_1.get_value()
        # imu_2_data = self.imu_2.get_value()
        imu_steering_data = self.imu_steering.get_value()

        psi = imu_1_data[2]  # yaw
        phi = imu_1_data[1] + np.pi/2  # pitch
        delta = imu_steering_data[2]  # yaw

        # Update measurements
        self.Z[0] = gps_rear_x   # x
        self.Z[1] = gps_rear_y   # y
        self.Z[2] = gps_rear_z   # z
        self.Z[3] = np.tan(delta) / self.wheel_distance  # sigma
        self.Z[4] = psi  # psi
        self.Z[5] = phi  # phi

        # self.Z[3] = self.Z[3] % 2*np.pi
        # self.Z[4] = self.Z[4] % 2*np.pi
        # self.Z[5] = self.Z[5] % 2*np.pi

        if abs(gps_rear_x) > 0.0:
            self.Z_plot[0] = gps_rear_x   # x
        if abs(gps_rear_y) > 0.0:
            self.Z_plot[1] = gps_rear_y   # y
        if abs(gps_rear_z) > 0.0:
            self.Z_plot[2] = gps_rear_z   # z

        self.Z_plot[3] = np.tan(delta) / self.wheel_distance  # sigma
        self.Z_plot[4] = psi  # psi
        self.Z_plot[5] = phi  # phi

        [vx, vy, vz] = self.velocity_vector3.get_value()
        # [vx, vy, vz] = self.velocity_twist.get_value()

        # Update inputs
        self.U[0] = np.sqrt(vx**2.0 + vy**2.0) + self.U_v_static
        self.U[1] = imu_1_data[4] + self.U_phi_dot_static  # angular_y
        self.U[2] = imu_steering_data[4] + self.U_delta_dot_static  # angular_y

        '''
        # update R
        s_x = self.filter.R[0, 0]
        s_y = self.filter.R[1, 1]
        s_z = self.filter.R[2, 2]
        s_sigma = self.filter.R[3, 3]
        s_psi = self.filter.R[4, 4]
        s_phi = self.filter.R[5, 5]

        rho = 2000.0
        gamma = 500.0
        #s_sigma = (rho + gamma*imu_steering_data[7])**2
        s_psi = (rho + gamma*imu_1_data[7])**2
        #s_phi = (rho + gamma*imu_1_data[7])**2

        #self.R_std = [s_x, s_y, s_z, s_sigma, s_psi, s_phi]

        # rospy.loginfo("R_std: " + str(self.R_std))
        self.filter.update_R(self.R_std)
        '''

    def publish_estimated_odometry(self, x, current_time):
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

    def shutdown_node(self):
        rospy.loginfo("Turning off node: RobotPoseEstimator")


# Main function.
if __name__ == '__main__':
    rospy.loginfo('Starting RobotPoseEstimator')

    # Initialize the node and name it.
    rospy.init_node('RobotPoseEstimator')

    try:
        obj_temp = RobotPoseEstimator()
    except rospy.ROSInterruptException:
        pass
