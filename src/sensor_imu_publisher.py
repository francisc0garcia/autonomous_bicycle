#!/usr/bin/env python

import time
import rospy

from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Imu
from classes.ImuDriver import *
from autonomous_bicycle.srv import calibration_msg
from autonomous_bicycle.msg import calibration_status


class RobotImuPublisherNode:
    def __init__(self):
        self.degrees2rad = math.pi / 180.0

        # Get values from launch file
        self.rate = rospy.get_param('~rate', 100.0)  # the rate at which to publish the transform
        # Static transform between sensor and fixed frame: x, y, z, roll, pitch, yaw
        # <rosparam param="static_transform">[0, 0, 0, 0, 0, 0]</rosparam>
        self.static_transform = rospy.get_param('~static_transform', [0, 0, 0, 0, 0, 0])
        self.serial_port = rospy.get_param('~serial_port', "/dev/ttyUSB0")
        self.topic_name = rospy.get_param('~topic_name', "/imu")
        self.calib_topic_name = rospy.get_param('~calib_topic_name', "/imu_calib")
        self.fixed_frame = rospy.get_param('~fixed_frame', "world")
        self.frame_name = rospy.get_param('~frame_name', "imu")
        self.publish_transform = rospy.get_param('~publish_transform', False)
        self.calibration_data = rospy.get_param('~calibration_data', [0, 0, 0, 0, 0, 0, 0,
                                                                      0, 0, 0, 0, 0, 0, 0,
                                                                      0, 0, 0, 0, 0, 0, 0, 0])

        self.service_name = rospy.get_param('~calibration_service', 'calibration')
        self.srv_calibration = rospy.Service(self.service_name, calibration_msg, self.callback_service)

        self.imu = ImuDriver(serial_port=self.serial_port, calibration_vector=self.calibration_data)
        self.imu.init_imu()

        # Create a publisher for imu message
        self.pub_imu = rospy.Publisher(self.topic_name, Imu, queue_size=10)
        self.odomBroadcaster_imu = TransformBroadcaster()

        # Create a publisher for IMU calibration status
        self.pub_calib = rospy.Publisher(self.calib_topic_name, calibration_status, queue_size=10)

        self.imu_msg = Imu()
        self.imu_msg.orientation_covariance[0] = -1
        self.imu_msg.angular_velocity_covariance[0] = -1
        self.imu_msg.linear_acceleration_covariance[0] = -1

        self.current_time = rospy.get_time()
        self.last_time = rospy.get_time()

        self.seq = 0

        rate = rospy.Rate(self.rate)

        rospy.loginfo("Ready for publishing imu:" + self.serial_port)

        # Main while loop.
        self.imu.update_offset()

        while not rospy.is_shutdown():
            self.current_time = rospy.get_time()

            # Read IMU
            [status, message] = self.imu.read()

            if status:
                if self.publish_transform:
                    quaternion = quaternion_from_euler(self.static_transform[3] * self.degrees2rad,
                                                       self.static_transform[4] * self.degrees2rad,
                                                       self.static_transform[5] * self.degrees2rad)

                    # send static transformation tf between imu and fixed frame
                    self.odomBroadcaster_imu.sendTransform(
                        (self.static_transform[0], self.static_transform[1], self.static_transform[2]),
                        (quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
                        rospy.Time.now(), self.frame_name, self.fixed_frame
                    )

                # publish imu message
                self.publish_info(imu=self.imu)
            else:
                rospy.loginfo("Imu publisher: " + message)

            # Publish calibration data if flag is active
            if self.imu.enable_calibration_status_reading:
                self.publish_calibration_info()

            rate.sleep()

    def callback_service(self, req):
        self.imu.enable_load_calibration = req.load_calibration
        self.imu.enable_set_offset = req.set_offset
        self.imu.enable_calibration_status_reading = req.enable_calibration_status
        self.imu.enable_print_calibration_vector = req.print_calibration_vector
        self.imu.enable_reset_imu = req.reset_imu

        return True

    def publish_calibration_info(self):
        calib_msg = calibration_status()
        calib_msg.header.stamp = rospy.Time.now()
        calib_msg.header.frame_id = "calibration_" + self.frame_name
        calib_msg.cal_accelerometer = self.imu.cal_accel
        calib_msg.cal_gyroscope = self.imu.cal_gyro
        calib_msg.cal_magnetometer = self.imu.cal_mag
        calib_msg.cal_system = self.imu.cal_sys

        self.pub_calib.publish(calib_msg)

        if self.imu.enable_print_calibration_vector:
            rospy.loginfo("Calibration " + self.frame_name + ": " + str(self.imu.real_calibration_vector))

    def publish_info(self, imu):
        self.imu_msg = Imu()
        self.imu_msg.linear_acceleration.x = imu.linear_acceleration_x
        self.imu_msg.linear_acceleration.y = imu.linear_acceleration_y
        self.imu_msg.linear_acceleration.z = imu.linear_acceleration_z

        self.imu_msg.angular_velocity.x = imu.angular_velocity_x
        self.imu_msg.angular_velocity.y = imu.angular_velocity_y
        self.imu_msg.angular_velocity.z = imu.angular_velocity_z

        self.imu_msg.orientation.x = imu.orientation_x
        self.imu_msg.orientation.y = imu.orientation_y
        self.imu_msg.orientation.z = imu.orientation_z
        self.imu_msg.orientation.w = imu.orientation_w

        self.imu_msg.header.stamp = rospy.Time.now()
        self.imu_msg.header.frame_id = self.frame_name
        self.imu_msg.header.seq = self.seq

        self.pub_imu.publish(self.imu_msg)
        self.seq += 1

# Main function.
if __name__ == '__main__':
    rospy.loginfo('Starting RobotImuPublisherNode')

    # Initialize the node and name it.
    rospy.init_node('sensor_imu_publisher')

    obj_temp = RobotImuPublisherNode()
