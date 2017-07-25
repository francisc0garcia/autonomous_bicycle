import numpy as np
import rospy
from std_msgs.msg import Float32


class FloatHandler(object):
    """
    Handler for ROS topics of type: std_msgs/Float32
    Args:
        topic_name: Name of ROS topic to be subscribed
        buffer_size: Variable buffer, depend on frame rate of topic, default: 500
        queue_size: Subscriber queue_size
    """

    def __init__(self, topic_name, buffer_size=500, queue_size=10):
        self.data_msg = Float32()
        self.data = 0.0

        self.topic_float = topic_name
        self.queue_size = queue_size
        self.buffer_size = buffer_size
        self.counter = 0

        self.buffer = np.zeros([self.buffer_size, 1])

        self.sub = rospy.Subscriber(self.topic_float, Float32, self.callback, queue_size=self.queue_size)

    def callback(self, msg):
        self.data_msg = msg

        if self.counter < self.buffer_size:
            self.buffer[self.counter] = [self.data_msg.data]
        else:
            rospy.loginfo("FloatHandler for: " + self.topic_float + " has reach buffer size.")

        self.counter += 1

    def get_value(self):
        if self.counter > 0:
            self.data = np.sum(self.buffer[:, 0]) / self.counter

        self.buffer = np.zeros([self.buffer_size, 1])
        self.counter = 0

        return self.data
