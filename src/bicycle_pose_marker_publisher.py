#!/usr/bin/env python

import time

# ROS dependencies
import rospy
import numpy as np
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler

# Project dependencies
from autonomous_bicycle.msg import bicycle_state


class BicyclePoseMarkerPublisherNode:
    def __init__(self):
        # Get values from launch file
        self.rate = rospy.get_param('~rate', 50.0)  # the rate at which to publish the transform
        self.alpha = rospy.get_param('~alpha', 1.0)
        self.model_main_frame = rospy.get_param('~model_main_frame', "")
        self.model_steer_frame = rospy.get_param('~model_steer_frame', "")
        self.topic_bicycle_state = rospy.get_param('~topic_bicycle_state', "/bicycle/bicycle_state")
        self.tf_main_frame_name = rospy.get_param('~tf_main_frame_name', "odom_gps_rear")
        self.tf_steering_frame_name = rospy.get_param('~tf_steering_frame_name', "odom_gps_rear")
        self.marker_name = rospy.get_param('~marker_name', "/bicycle/filter")

        # storage current bicycle state (x, y, z, steering, heading, lean)
        self.bicycle_state = bicycle_state()

        # create publishers for main frame and steering frame markers
        self.pub_marker_main = rospy.Publisher(self.marker_name + "_main", Marker, queue_size=10)
        self.pub_marker_steering = rospy.Publisher(self.marker_name + "_steering", Marker, queue_size=10)

        # subscribe to bicycle state topic
        self.sub_bic_state = rospy.Subscriber(self.topic_bicycle_state, bicycle_state,
                                              self.bicycle_state_callback, queue_size=10)

        # Main while loop.
        while not rospy.is_shutdown():
            self.publish_markers()
            time.sleep(1.0 / self.rate)

    def bicycle_state_callback(self, msg):
        self.bicycle_state = msg

    def publish_markers(self):
        offset_main_psi = -np.pi*0.70
        offset_steer_psi = -np.pi*0.70
        # create main frame marker
        marker_main_frame = Marker()
        marker_main_frame.header.frame_id = self.tf_steering_frame_name
        marker_main_frame.header.stamp = rospy.Time.now()
        marker_main_frame.id = 1
        marker_main_frame.action = Marker.ADD
        marker_main_frame.type = Marker.MESH_RESOURCE
        marker_main_frame.mesh_resource = self.model_main_frame

        marker_main_frame.pose.position.x = 0.0
        marker_main_frame.pose.position.y = 0.0
        marker_main_frame.pose.position.z = 0.0

        roll = 0.0
        pitch = -self.bicycle_state.lean_phi
        yaw = self.bicycle_state.heading_psi + offset_main_psi

        quaternion = quaternion_from_euler(roll, pitch, yaw)
        marker_main_frame.pose.orientation.x = quaternion[0]
        marker_main_frame.pose.orientation.y = quaternion[1]
        marker_main_frame.pose.orientation.z = quaternion[2]
        marker_main_frame.pose.orientation.w = quaternion[3]

        marker_main_frame.scale.x = 1.0
        marker_main_frame.scale.y = 1.0
        marker_main_frame.scale.z = 1.0

        marker_main_frame.color.a = self.alpha
        marker_main_frame.color.r = 1.0
        marker_main_frame.color.g = 0.5
        marker_main_frame.color.b = 0.0

        self.pub_marker_main.publish(marker_main_frame)

        # create steering frame marker
        maker_steering_frame = Marker()
        maker_steering_frame.header.frame_id = self.tf_steering_frame_name
        maker_steering_frame.header.stamp = rospy.Time.now()
        maker_steering_frame.id = 1
        maker_steering_frame.action = Marker.ADD

        maker_steering_frame.type = Marker.MESH_RESOURCE
        maker_steering_frame.mesh_resource = self.model_steer_frame
        maker_steering_frame.scale.x = 1.0
        maker_steering_frame.scale.y = 1.0
        maker_steering_frame.scale.z = 1.0

        maker_steering_frame.pose.position.x = 0.0
        maker_steering_frame.pose.position.y = 0.0
        maker_steering_frame.pose.position.z = 0.0

        roll = 0.0
        pitch = -self.bicycle_state.lean_phi
        yaw = offset_steer_psi + self.bicycle_state.heading_psi + self.bicycle_state.steering_delta
        quaternion = quaternion_from_euler(roll, pitch, yaw)

        maker_steering_frame.pose.orientation.x = quaternion[0]
        maker_steering_frame.pose.orientation.y = quaternion[1]
        maker_steering_frame.pose.orientation.z = quaternion[2]
        maker_steering_frame.pose.orientation.w = quaternion[3]

        maker_steering_frame.color.a = self.alpha
        maker_steering_frame.color.r = 0.8
        maker_steering_frame.color.g = 0.4
        maker_steering_frame.color.b = 0.0

        self.pub_marker_steering.publish(maker_steering_frame)


# Main function.
if __name__ == '__main__':
    rospy.loginfo('Starting BicyclePoseMarkerPublisherNode')

    # Initialize the node and name it.
    rospy.init_node('BicyclePoseMarkerPublisherNode')

    try:
        obj_temp = BicyclePoseMarkerPublisherNode()
    except rospy.ROSInterruptException:
        rospy.logerr("Node BicyclePoseMarkerPublisherNode fail")
