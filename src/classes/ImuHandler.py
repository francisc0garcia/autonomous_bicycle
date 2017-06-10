import numpy as np
import rospy
import math
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

rad2degrees = 180.0/math.pi
degrees2rad = math.pi / 180.0


class ImuHandler(object):
    def __init__(self, topic_name, buffer_size=500, queue_size=1):
        self.imu_data = Imu()
        [self.roll, self.pitch, self.yaw,
         self.angular_x, self.angular_y, self.angular_z,
         self.lin_acc_x, self.lin_acc_y, self.lin_acc_z] = np.zeros((9, 1))*0.0

        [self._roll, self._pitch, self._yaw,
         self._angular_x, self._angular_y, self._angular_z,
         self._lin_acc_x, self._lin_acc_y, self._lin_acc_z] = np.zeros((9, 1))*0.0

        self.topic_odom = topic_name
        self.queue_size = queue_size
        self.buffer_size = buffer_size
        self.counter = 0

        self.buffer = np.zeros([self.buffer_size, 9])

        self.sub = rospy.Subscriber(self.topic_odom, Imu, self.callback, queue_size=self.queue_size)

    def callback(self, msg):
        self.imu_data = msg
        quaternion = (
            self.imu_data.orientation.x,
            self.imu_data.orientation.y,
            self.imu_data.orientation.z,
            self.imu_data.orientation.w)

        (self._roll, self._pitch, self._yaw) = euler_from_quaternion(quaternion)

        self._angular_x = self.imu_data.angular_velocity.x
        self._angular_y = self.imu_data.angular_velocity.y
        self._angular_z = self.imu_data.angular_velocity.z

        self._lin_acc_x = self.imu_data.linear_acceleration.x
        self._lin_acc_y = self.imu_data.linear_acceleration.y
        self._lin_acc_z = self.imu_data.linear_acceleration.z

        self.buffer[self.counter] = [self._roll, self._pitch, self._yaw,
                                     self._angular_x, self._angular_y, self._angular_z,
                                     self._lin_acc_x, self._lin_acc_y, self._lin_acc_z]
        self.counter += 1

    def get_value(self):
        if self.counter > 0:
            self.roll = np.sum(self.buffer[:, 0]) / self.counter
            self.pitch = np.sum(self.buffer[:, 1]) / self.counter
            self.yaw = np.sum(self.buffer[:, 2]) / self.counter

            self.angular_x = np.sum(self.buffer[:, 3]) / self.counter
            self.angular_y = np.sum(self.buffer[:, 4]) / self.counter
            self.angular_z = np.sum(self.buffer[:, 5]) / self.counter

            self.lin_acc_x = np.sum(self.buffer[:, 6]) / self.counter
            self.lin_acc_y = np.sum(self.buffer[:, 7]) / self.counter
            self.lin_acc_z = np.sum(self.buffer[:, 8]) / self.counter

        self.buffer = np.zeros([self.buffer_size, 9])
        self.counter = 0

        # return instant measurement
        # return [self._roll, self._pitch, self._yaw, self._angular_x, self._angular_y, self._angular_z]
        # return Mean measurement
        return [self.roll, self.pitch, self.yaw,
                self.angular_x, self.angular_y, self.angular_z,
                self.lin_acc_x, self.lin_acc_y, self.lin_acc_z]
