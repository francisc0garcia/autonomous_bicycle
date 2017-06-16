#!/usr/bin/env python

import rospy
import numpy as np
from visualization_msgs.msg import Marker


class BicycleMarkerGeneratorNode:
    def __init__(self):

        # Get values from launch file
        self.rate = rospy.get_param('~rate', 80.0)  # the rate at which to publish the transform
        self.model_main_frame = rospy.get_param('~model_main_frame', "")
        self.model_steer_frame = rospy.get_param('~model_steer_frame', "")

        self.pub_marker_main = rospy.Publisher("/bicycle_marker_main", Marker, queue_size=1)
        self.pub_marker_steering = rospy.Publisher("/bicycle_marker_steering", Marker, queue_size=1)
        self.count = 0

        rospy.on_shutdown(self.shutdown_node)
        rate = rospy.Rate(self.rate)

        # Main while loop.
        while not rospy.is_shutdown():
            ellipse_main = Marker()
            ellipse_main.header.frame_id = "odom_gps_rear"
            ellipse_main.header.stamp = rospy.Time.now()
            ellipse_main.id = 1
            ellipse_main.action = Marker.ADD
            ellipse_main.type = Marker.MESH_RESOURCE
            ellipse_main.mesh_resource = self.model_main_frame
            ellipse_main.pose.position.x = 0
            ellipse_main.pose.position.y = 0
            ellipse_main.pose.position.z = 0
            ellipse_main.pose.orientation.x = 0
            ellipse_main.pose.orientation.y = 0
            ellipse_main.pose.orientation.z = 0
            ellipse_main.pose.orientation.w = 1
            ellipse_main.scale.x = 1.0
            ellipse_main.scale.y = 1.0
            ellipse_main.scale.z = 1.0
            ellipse_main.color.a = 1.0
            ellipse_main.color.r = 0.0
            ellipse_main.color.g = 0.0
            ellipse_main.color.b = 0.5

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
            ellipse.pose.position.z = 0
            ellipse.pose.orientation.x = 0
            ellipse.pose.orientation.y = 0
            ellipse.pose.orientation.z = 0
            ellipse.pose.orientation.w = 1
            ellipse.scale.x = 1.0
            ellipse.scale.y = 1.0
            ellipse.scale.z = 1.0
            ellipse.color.a = 1.0
            ellipse.color.r = 0.0
            ellipse.color.g = 0.0
            ellipse.color.b = 0.3

            # Publish the MarkerArray
            self.pub_marker_steering.publish(ellipse)

            rate.sleep()

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
