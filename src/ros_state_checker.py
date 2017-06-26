#!/usr/bin/env python

import rospy
import numpy as np
import time
import subprocess

from std_msgs.msg import Int8MultiArray, Float32
from sensor_msgs.msg import CompressedImage, NavSatFix, Imu
from autonomous_bicycle.srv import turn_off_msg


class RosStateChecker:
    def __init__(self):
        # Get values from launch file
        self.rate = rospy.get_param('~rate', 1.0)
        self.qz = 10

        self.topic_imu_1 = '/bicycle/imu_1'
        self.topic_imu_steering = '/bicycle/imu_steering'
        self.topic_camera = '/bicycle/image_raw/compressed'
        self.topic_altitude = '/bicycle/altitude'
        self.topic_gps_front = '/bicycle/gps_front'
        self.topic_gps_rear = '/bicycle/gps_rear'

        self.imu_1_counter = 0
        self.imu_steering_counter = 0
        self.camera_counter = 0
        self.altitude_counter = 0
        self.gps_front_counter = 0
        self.gps_rear_counter = 0

        self.state_msg = Int8MultiArray()
        self.state_vector = []

        # create publisher
        self.pub_state_system = rospy.Publisher("/bicycle/state_system", Int8MultiArray, queue_size=1)

        # create service to turn-off system
        self.service_turn_off_script = rospy.get_param('~service_turn_off_script', "/tmp")
        self.srv_calibration = rospy.Service("/bicycle/turn_off", turn_off_msg, self.callback_service)

        rospy.on_shutdown(self.shutdown_node)

        # create subscribers
        self.sub_imu_1 = rospy.Subscriber(self.topic_imu_1, Imu, self.callback_imu_1, queue_size=self.qz)
        self.sub_steer = rospy.Subscriber(self.topic_imu_steering, Imu, self.callback_imu_steering, queue_size=self.qz)
        self.sub_camera = rospy.Subscriber(self.topic_camera, CompressedImage, self.callback_camera, queue_size=self.qz)
        self.sub_altitude = rospy.Subscriber(self.topic_altitude, Float32, self.callback_altitude, queue_size=self.qz)
        self.sub_gps_f = rospy.Subscriber(self.topic_gps_front, NavSatFix, self.callback_gps_front, queue_size=self.qz)
        self.sub_gps_r = rospy.Subscriber(self.topic_gps_rear, NavSatFix, self.callback_gps_rear, queue_size=self.qz)

        self.rate_timer = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            self.update_state_system()
            self.rate_timer.sleep()

    def callback_service(self, req):
        try:
            if req.turn_off:
                rospy.loginfo("Ready to execute script: " + self.service_turn_off_script)
                rc = subprocess.call(self.service_turn_off_script)
                rospy.loginfo("Finish execution of: " + self.service_turn_off_script)
                return True
            else:
                return False
        except:
            return False

    def update_state_system(self):
        self.state_vector = []

        # Check if new message were received
        imu_1_active = 1 if self.imu_1_counter > 0 else 0
        imu_steering_active = 1 if self.imu_steering_counter > 0 else 0
        camera_active = 1 if self.camera_counter > 0 else 0
        altitude_active = 1 if self.altitude_counter > 0 else 0
        gps_front_active = 1 if self.gps_front_counter > 0 else 0
        gps_rear_active = 1 if self.gps_rear_counter > 0 else 0

        self.state_vector.append(imu_1_active)  # IMU Lean
        self.state_vector.append(imu_steering_active)  # IMU Steering
        self.state_vector.append(camera_active)  # Camera
        self.state_vector.append(altitude_active)  # Altimeter
        self.state_vector.append(gps_front_active)  # GPS front
        self.state_vector.append(gps_rear_active)  # GPS rear

        self.state_msg.data = self.state_vector
        self.pub_state_system.publish(self.state_msg)

        # clean counters
        self.imu_1_counter = 0
        self.imu_steering_counter = 0
        self.camera_counter = 0
        self.altitude_counter = 0
        self.gps_front_counter = 0
        self.gps_rear_counter = 0

    def callback_imu_1(self, msg):
        self.imu_1_counter += 1

    def callback_imu_steering(self, msg):
        self.imu_steering_counter += 1

    def callback_camera(self, msg):
        self.camera_counter += 1

    def callback_altitude(self, msg):
        self.altitude_counter += 1

    def callback_gps_front(self, msg):
        self.gps_front_counter += 1

    def callback_gps_rear(self, msg):
        self.gps_rear_counter += 1

    def shutdown_node(self):
        rospy.loginfo("Turning off node: RobotPoseEstimator")


# Main function.
if __name__ == '__main__':
    rospy.loginfo('Starting RosStateChecker')

    # Initialize the node and name it.
    rospy.init_node('RosStateChecker')

    try:
        obj_temp = RosStateChecker()
    except rospy.ROSInterruptException:
        pass
