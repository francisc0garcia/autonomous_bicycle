#!/usr/bin/env python

# ROS Dependencies
import rospy
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler

# Project dependencies
from classes.handlers.ImuHandler import *
from classes.handlers.OdometryHandler import *


class BicycleMarkerGeneratorNode:
    def __init__(self):

        # Get values from launch file
        self.rate = rospy.get_param('~rate', 80.0)  # the rate at which to publish the transform
        self.model_main_frame = rospy.get_param('~model_main_frame', "")
        self.model_steer_frame = rospy.get_param('~model_steer_frame', "")

        self.pub_marker_main = rospy.Publisher("/bicycle_marker_main", Marker, queue_size=1)
        self.pub_marker_steering = rospy.Publisher("/bicycle_marker_steering", Marker, queue_size=1)
        self.pub_psi = rospy.Publisher("/bicycle/psi", Float32, queue_size=1)
        self.count = 0

        self.data_imu_steering = []
        self.data_imu_lean = []

        self.topic_imu_1 = "/bicycle/imu_1"
        self.topic_imu_steering = "/bicycle/imu_steering"
        self.topic_gps_rear = "/bicycle/odom_gps_rear"

        self.psi = 0.0
        self.prev_x = 0.0
        self.prev_y = 0.0

        self.imu_1 = ImuHandler(topic_name=self.topic_imu_1, queue_size=10)
        self.imu_steering = ImuHandler(topic_name=self.topic_imu_steering, queue_size=10)
        self.odom_gps_rear = OdometryHandler(topic_name=self.topic_gps_rear, queue_size=10)

        rospy.on_shutdown(self.shutdown_node)
        rate = rospy.Rate(self.rate)

        # Main while loop.
        while not rospy.is_shutdown():
            self.data_imu_steering = self.imu_steering.get_value()
            self.data_imu_lean = self.imu_1.get_value()
            self.gps_data = self.odom_gps_rear.get_value()

            self.estimate_psi()

            self.publish_markers()

            rate.sleep()

    def normalize_angle(self, x):
        x = x % (2 * np.pi)  # force in range [0, 2 pi)
        if x > np.pi:  # move to [-pi, pi)
            x -= 2 * np.pi
        return x

    def estimate_psi(self):
        if abs(self.gps_data[0]) > 0 and abs(self.gps_data[1]) > 0:
            delta_x = self.gps_data[0] - self.prev_x
            delta_y = self.gps_data[1] - self.prev_y

            self.psi = np.arctan2(delta_y, delta_x)

            self.prev_x = self.gps_data[0]
            self.prev_y = self.gps_data[1]

            self.psi = self.normalize_angle(self.psi) + np.pi / 2

            msg_data = Float32()
            msg_data.data = self.psi
            self.pub_psi.publish(msg_data)

    def publish_markers(self):
        ellipse_main = Marker()
        ellipse_main.header.frame_id = "odom_gps_rear"
        ellipse_main.header.stamp = rospy.Time.now()
        ellipse_main.id = 1
        ellipse_main.action = Marker.ADD
        ellipse_main.type = Marker.MESH_RESOURCE
        ellipse_main.mesh_resource = self.model_main_frame

        ellipse_main.pose.position.x = 0
        ellipse_main.pose.position.y = 0
        ellipse_main.pose.position.z = 2.0

        offset_lean_imu1 = 0.35
        offset_lean_steer = -0.4

        lean_avg = -((self.data_imu_lean[1] + offset_lean_imu1) + (self.data_imu_steering[1] + offset_lean_steer)) / 2

        # lean_avg = -(self.data_imu_lean[1] + offset_lean_imu1)

        roll = 0.0
        pitch = lean_avg
        yaw = self.psi + np.pi
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        ellipse_main.pose.orientation.x = quaternion[0]
        ellipse_main.pose.orientation.y = quaternion[1]
        ellipse_main.pose.orientation.z = quaternion[2]
        ellipse_main.pose.orientation.w = quaternion[3]

        ellipse_main.scale.x = 1.0
        ellipse_main.scale.y = 1.0
        ellipse_main.scale.z = 1.0

        ellipse_main.color.a = 1.0
        ellipse_main.color.r = 1.0
        ellipse_main.color.g = 0.5
        ellipse_main.color.b = 0.0

        # Publish the MarkerArray
        self.pub_marker_main.publish(ellipse_main)

        ellipse = Marker()
        ellipse.header.frame_id = "odom_gps_rear"
        ellipse.header.stamp = rospy.Time.now()
        ellipse.id = 1
        ellipse.action = Marker.ADD
        ellipse.type = Marker.MESH_RESOURCE
        ellipse.mesh_resource = self.model_steer_frame

        ellipse.pose.position.x = 0
        ellipse.pose.position.y = 0
        ellipse.pose.position.z = 2.1

        roll = 0.0
        pitch = lean_avg
        yaw = -self.data_imu_steering[2] - 0.2 * np.pi  # 0.0 + 0.8 * np.pi  # self.data_imu_steering[2]
        quaternion = quaternion_from_euler(roll, pitch, yaw)

        ellipse.pose.orientation.x = quaternion[0]
        ellipse.pose.orientation.y = quaternion[1]
        ellipse.pose.orientation.z = quaternion[2]
        ellipse.pose.orientation.w = quaternion[3]

        ellipse.scale.x = 1.0
        ellipse.scale.y = 1.0
        ellipse.scale.z = 1.0

        ellipse.color.a = 1.0
        ellipse.color.r = 0.8
        ellipse.color.g = 0.4
        ellipse.color.b = 0.0

        # Publish the MarkerArray
        self.pub_marker_steering.publish(ellipse)

    def shutdown_node(self):
        rospy.loginfo("Turning off node: BicycleMarkerGeneratorNode")


# Main function.
if __name__ == '__main__':
    rospy.loginfo('Starting BicycleMarkerGeneratorNode')

    # Initialize the node and name it.
    rospy.init_node('BicycleMarkerGeneratorNode')

    try:
        obj_temp = BicycleMarkerGeneratorNode()
    except rospy.ROSInterruptException:
        pass
