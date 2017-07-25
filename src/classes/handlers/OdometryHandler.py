import numpy as np
import rospy
from nav_msgs.msg import Odometry


class OdometryHandler(object):
    """
    Handler for ROS topics of type: nav_msgs/Odometry
    Args:
        topic_name: Name of ROS topic to be subscribed
        buffer_size: Variable buffer, depend on frame rate of topic, default: 500
        queue_size: Subscriber queue_size
    """

    def __init__(self, topic_name, buffer_size=500, queue_size=10):
        self.odom = Odometry()
        # mean values
        [self.x, self.y, self.z] = [0.0, 0.0, 0.0]

        # instant values
        [self.x_t, self.y_t, self.z_t] = [0.0, 0.0, 0.0]

        self.topic_odom = topic_name
        self.queue_size = queue_size
        self.buffer_size = buffer_size
        self.counter = 0

        self.buffer = np.zeros([self.buffer_size, 3])

        self.sub = rospy.Subscriber(self.topic_odom, Odometry, self.callback, queue_size=self.queue_size)

    def callback(self, msg):
        self.odom = msg
        [self.x_t, self.y_t, self.z_t] = [self.odom.pose.pose.position.x,
                                          self.odom.pose.pose.position.y,
                                          self.odom.pose.pose.position.z]

        if self.counter < self.buffer_size:
            self.buffer[self.counter] = [self.x_t, self.y_t, self.z_t]
        else:
            rospy.loginfo("OdometryHandler for: " + self.topic_odom + " has reach buffer size.")

        self.counter += 1

    def get_value(self):
        if self.counter > 0:
            self.x = np.sum(self.buffer[:, 0]) / self.counter
            self.y = np.sum(self.buffer[:, 1]) / self.counter
            self.z = np.sum(self.buffer[:, 2]) / self.counter
        else:
            [self.x, self.y, self.z] = [0.0, 0.0, 0.0]

        self.buffer = np.zeros([self.buffer_size, 3])
        self.counter = 0

        return [self.x, self.y, self.z]
