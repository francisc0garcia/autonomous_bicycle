#!/usr/bin/env python

import math
import rospy
import numpy as np
from geometry_msgs.msg import Quaternion, Point, Pose
import tf
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry

from classes.EKF import *


class RobotPoseEstimator:
    def __init__(self):
        self.degrees2rad = math.pi/180.0
        self.bicycle_odom = Odometry()

        self.rate = rospy.get_param('~rate', 80.0)  # the rate at which to publish the transform

        self.dt = 1/self.rate
        self.wheel_distance = 1.02
        self.number_state_variables = 7

        # [x, y, z, v, psi, phi, delta]
        self.X_init = np.array([0.0, 0.0, 0.0, 0.1, 0.0, 0.1, math.radians(8)])

        self.Z = np.zeros((1, self.number_state_variables)).T
        self.X = np.zeros((1, self.number_state_variables)).T

        self.P = np.eye(self.number_state_variables)
        self.M = np.eye(4)
        [self.M[0, 0], self.M[1, 1], self.M[2, 2], self.M[3, 3]] = \
            [1.9**2, 0.1**2, 0.005**2, 0.01**2]  # a, beta, phi_dot, delta_dot

        std = [0.01, 0.01, 0.01, 0.01, 0.02, 0.02, 0.02]
        self.filter = EKF(self.X_init, self.P, self.M, std,
                          wheel_distance=self.wheel_distance, dt=self.dt)

        self.filter.dt = self.dt

        # Get values from launch file
        self.topic_imu_lean_1 = "/bicycle/imu_1"
        self.topic_imu_lean_2 = "/bicycle/imu_2"
        self.topic_imu_steering = "/bicycle/imu_steering"
        self.topic_gps_1 = "/bicycle/gps_front"
        self.topic_gps_2 = "/bicycle/gps_rear"
        self.topic_odom = "/bicycle/odom"

        # define properties
        self.lean_1_angle = 0.0
        self.heading_1_angle = 0.0
        self.lean_1_static_transform = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.lean_2_angle = 0.0
        self.lean_2_static_transform = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.steering_angle = 0.0
        self.steering_static_transform = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.gps_1_static_transform = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.gps_1_odom = Odometry()
        self.gps_1_x = 0.0
        self.gps_1_y = 0.0

        self.gps_2_static_transform = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.gps_2_odom = Odometry()
        self.gps_2_x = 0.0
        self.gps_2_y = 0.0

        self.z = 0.0
        self.seq = 0

        # create publishers
        self.pub_estimated_odom = rospy.Publisher("/bicycle/odom_filter", Odometry, queue_size=1)
        self.estimated_odom_br = tf.TransformBroadcaster()

        # Create subscribers:
        self.sub_imu_lean_1 = rospy.Subscriber(self.topic_imu_lean_1, Imu, self.callback_imu_lean_1, queue_size=1)
        self.sub_imu_lean_2 = rospy.Subscriber(self.topic_imu_lean_2, Imu, self.callback_imu_lean_2, queue_size=1)
        self.sub_imu_steering = rospy.Subscriber(self.topic_imu_steering, Imu, self.callback_imu_steering, queue_size=1)

        self.sub_odom_real = rospy.Subscriber(self.topic_odom, Odometry, self.callback_odometry_real, queue_size=1)
        self.sub_odom_gps_1 = rospy.Subscriber(self.topic_odom, Odometry, self.callback_odometry_gps_1, queue_size=1)
        self.sub_odom_gps_2 = rospy.Subscriber(self.topic_odom, Odometry, self.callback_odometry_gps_2, queue_size=1)

        rospy.on_shutdown(self.shutdown_node)
        rate = rospy.Rate(self.rate)

        self.alpha = 0.0
        self.beta = 0.0
        self.phi_dot = 0.0
        self.delta_dot = 0.0

        self.U = [self.alpha, self.beta, self.phi_dot, self.delta_dot]

        # Main while loop.
        while not rospy.is_shutdown():
            self.current_time = rospy.Time.now()

            # update U
            v = self.filter.xs[3]
            phi = self.filter.xs[5]
            delta = self.filter.xs[6]
            self.U[1] = self.filter.normalize_angle( (v*self.dt/self.wheel_distance)*(np.tan(delta)/np.cos(phi)))

            self.filter.prediction(self.U)
            self.update_z()
            self.filter.update(self.Z)
            self.publish_estimated_odometry(self.filter.xs, self.current_time)

            rate.sleep()

    def update_z(self):
        self.Z[0] = self.gps_1_odom.pose.pose.position.x
        self.Z[1] = self.gps_1_odom.pose.pose.position.y
        self.Z[2] = 0
        self.Z[3] = self.gps_1_odom.twist.twist.linear.x
        self.Z[4] = self.heading_1_angle
        self.Z[5] = self.lean_1_angle
        self.Z[6] = self.steering_angle

    def publish_estimated_odometry(self, x, current_time):
        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, x[4])

        # first, we'll publish the transform over tf
        self.estimated_odom_br.sendTransform(
            (x[0], x[1], x[2]),
            odom_quat,
            current_time,
            "gazebo_world",
            "estimated_odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "gazebo_world"

        # set the position
        odom.pose.pose = Pose(Point(x[0], x[1], x[2]), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "estimated_odom"

        # publish the message
        self.pub_estimated_odom.publish(odom)

    def callback_odometry_real(self, odom_msg):
        self.bicycle_odom = odom_msg

    def callback_odometry_gps_1(self, odom_msg):
        self.gps_1_odom = odom_msg

    def callback_odometry_gps_2(self, odom_msg):
        self.gps_2_odom = odom_msg

    def callback_imu_lean_1(self, msg):
        quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        self.lean_1_angle = roll
        self.heading_1_angle = yaw

    def callback_imu_lean_2(self, msg):
        quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        self.lean_2_angle = roll

    def callback_imu_steering(self, msg):
        quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        self.steering_angle = roll

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
