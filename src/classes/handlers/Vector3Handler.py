import numpy as np
import rospy
from geometry_msgs.msg import Vector3Stamped


class Vector3Handler(object):
    """
    Handler for ROS topics of type: geometry_msgs/Vector3Stamped
    Args:
        topic_name: Name of ROS topic to be subscribed
        buffer_size: Variable buffer, depend on frame rate of topic, default: 500
        queue_size: Subscriber queue_size
    """

    def __init__(self, topic_name, buffer_size=500, queue_size=10):
        self.vector_data = Vector3Stamped()
        [self.vector_x, self.vector_y, self.vector_z] = [0.0, 0.0, 0.0]

        self.topic_name = topic_name
        self.queue_size = queue_size
        self.buffer_size = buffer_size
        self.counter = 0

        self.buffer = np.zeros([self.buffer_size, 3])

        self.sub = rospy.Subscriber(self.topic_name, Vector3Stamped, self.callback,
                                    queue_size=self.queue_size)

    def callback(self, msg):
        self.vector_data = msg

        if self.counter < self.buffer_size:
            self.buffer[self.counter] = [self.vector_data.vector.x,
                                         self.vector_data.vector.y,
                                         self.vector_data.vector.z]
        else:
            rospy.loginfo("Vector3Handler for: " + self.topic_name + " has reach buffer size.")

        self.counter += 1

    def get_value(self):
        if self.counter > 0:
            self.vector_x = np.sum(self.buffer[:, 0]) / self.counter
            self.vector_y = np.sum(self.buffer[:, 1]) / self.counter
            self.vector_z = np.sum(self.buffer[:, 2]) / self.counter
        else:
            [self.vector_x, self.vector_y, self.vector_z] = [0.0, 0.0, 0.0]

        self.buffer = np.zeros([self.buffer_size, 3])
        self.counter = 0

        return [self.vector_x, self.vector_y, self.vector_z]
