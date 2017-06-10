import numpy as np
import rospy
from geometry_msgs.msg import TwistStamped


class TwistHandler(object):
    def __init__(self, topic_name, buffer_size=500, queue_size=1):
        self.twist_data = TwistStamped()
        [self.twist_x, self.twist_y, self.twist_z] = [0.0, 0.0, 0.0]

        self.topic_name = topic_name
        self.queue_size = queue_size
        self.buffer_size = buffer_size
        self.counter = 0

        self.buffer = np.zeros([self.buffer_size, 3])

        self.sub = rospy.Subscriber(self.topic_name, TwistStamped, self.callback,
                                    queue_size=self.queue_size)

    def callback(self, msg):
        self.twist_data = msg

        self.buffer[self.counter] = [self.twist_data.twist.linear.x,
                                     self.twist_data.twist.linear.y,
                                     self.twist_data.twist.linear.z]
        self.counter += 1

    def get_value(self):
        if self.counter > 0:
            self.twist_x = np.sum(self.buffer[:, 0]) / self.counter
            self.twist_y = np.sum(self.buffer[:, 1]) / self.counter
            self.twist_z = np.sum(self.buffer[:, 2]) / self.counter

        self.buffer = np.zeros([self.buffer_size, 3])
        self.counter = 0

        return [self.twist_x, self.twist_y, self.twist_z]
