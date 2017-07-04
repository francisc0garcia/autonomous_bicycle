import time
import math
import rospy
import sys
from classes.BNO055 import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class ImuDriver(object):
    def __init__(self, serial_port="/dev/ttyUSB0", calibration_vector=[]):
        self.degrees2rad = math.pi / 180.0

        self.debug = False
        self.string_debug = ''

        self.calibration_vector = calibration_vector
        self.serial_port = serial_port
        self.is_init_imu = False
        self.is_init_device = False
        self.is_init_calibration = False

        self.is_calibrated = False
        self.force_calibration = False
        [self.cal_sys, self.cal_gyro, self.cal_accel, self.cal_mag] = [0., 0., 0., 0.]

        self.bno = BNO055(serial_port=serial_port)

        # IMU info
        self.linear_acceleration_x = 0.0
        self.linear_acceleration_y = 0.0
        self.linear_acceleration_z = 0.0

        self.angular_velocity_x = 0.0
        self.angular_velocity_y = 0.0
        self.angular_velocity_z = 0.0

        self.euler_yaw = 0.0
        self.euler_roll = 0.0
        self.euler_pitch = 0.0

        self.orientation_x = 0.0
        self.orientation_y = 0.0
        self.orientation_z = 0.0
        self.orientation_w = 0.0

        # Offset info
        self.offset_yaw = 0.0
        self.offset_roll = 0.0
        self.offset_pitch = 0.0

        self.temperature = 0

    def init_imu(self):
        rospy.loginfo("initializing IMU, mode: OPERATION_MODE_NDOF")
        while not self.is_init_imu:
            try:
                self.bno.begin(mode=OPERATION_MODE_NDOF)
                # self.bno.begin(mode=OPERATION_MODE_NDOF_FMC_OFF)  # more stable
                # self.bno.begin(mode=OPERATION_MODE_IMUPLUS)
                # self.bno.begin(mode=OPERATION_MODE_M4G)

                self.is_init_imu = True
                self.string_debug = 'Connected to BNO055 at port: ' + str(self.serial_port)
            except:
                self.string_debug = 'Failed to initialize BNO055 at port: ' + str(self.serial_port)
                time.sleep(0.1)
            if self.debug:
                rospy.loginfo(self.string_debug)

        rospy.loginfo("initializing Device")
        while not self.is_init_device:
            status, self_test, error = self.bno.get_system_status(False)
            if error == 0 and status != 0x01:
                self.is_init_device = True
            else:
                self.string_debug = 'Failed to initialize IMU port: ' + str(self.serial_port) + ' error: ' + str(error)
                time.sleep(0.1)

            if self.debug:
                rospy.loginfo(self.string_debug)

        if not self.is_init_calibration:
            rospy.loginfo("Loading calibration vector")
            # Load precomputed calibration values
            self.load_calibration()

        rospy.loginfo("Checking calibration of IMU")
        while not self.is_init_calibration:
            self.get_calibration_status()

            if self.is_calibrated:
                self.is_init_calibration = True
            else:
                self.load_calibration()
                self.string_debug = "Waiting for IMU calibration: [S %f, G %f, A %f, M %f]" % (
                    self.cal_sys, self.cal_gyro, self.cal_accel, self.cal_mag)

            rospy.loginfo(self.string_debug)

    def load_calibration(self):
        # computed using tutorial:
        # https://learn.adafruit.com/bno055-absolute-orientation-sensor-with-raspberry-pi-and-beaglebone-black/webgl-example
        # Bosch video: https://www.youtube.com/watch?v=Bw0WuAyGsnY
        try:
            self.bno.serial_attempt_delay = 0.3
            self.bno.set_calibration(self.calibration_vector)
            self.bno.serial_attempt_delay = 0.0
            time.sleep(1.5)  # wait for stable measurement
            self.update_offset()
            return True
        except:
            return False

    def update_offset(self):
        time.sleep(1)  # wait for stable measurement of IMU
        qx, qy, qz, qw = self.bno.read_quaternion()  # Orientation as a quaternion
        (self.offset_roll, self.offset_pitch, self.offset_yaw) = euler_from_quaternion([qx, qy, qz, qw])

        self.string_debug = "calibration offset: [yaw %f, roll %f, pitch %f]" % (
            self.offset_yaw, self.offset_roll, self.offset_pitch)
        rospy.loginfo(self.string_debug)

    def get_calibration_status(self):
        self.cal_sys, self.cal_gyro, self.cal_accel, self.cal_mag = self.bno.get_calibration_status()

        if self.cal_sys > 0 or self.cal_gyro > 0 or self.cal_accel > 0:
            self.is_calibrated = True
        else:
            self.is_calibrated = False

    def read(self):
        try:
            if self.is_calibrated and not self.force_calibration:
                # mx, my, mz = self.bno.read_magnetometer()     # Magnetometer data (in micro-Teslas)
                # ax, ay, az = self.bno.read_accelerometer()    # Accelerometer data (in meters per second squared)
                # yaw, roll, pitch = self.bno.read_euler()      # Euler angles for heading, roll, pitch (degrees)
                # temp = self.bno.read_temp()                   # Temperature in degrees Celsius
                # Linear acceleration data (i.e. acceleration from movement, not gravity--
                # returned in meters per second squared)
                # lx, ly, lz = self.bno.read_linear_acceleration()

                # Gravity acceleration data (i.e. acceleration just from gravity--returned
                # in meters per second squared):
                ax, ay, az = self.bno.read_gravity()
                qx, qy, qz, qw = self.bno.read_quaternion()  # Orientation as a quaternion
                gx, gy, gz = self.bno.read_gyroscope()  # Gyroscope data (in degrees per second)

                # IMU info
                self.linear_acceleration_x = ax
                self.linear_acceleration_y = ay
                self.linear_acceleration_z = az

                self.angular_velocity_x = gx
                self.angular_velocity_y = gy
                self.angular_velocity_z = gz

                # update (fix) initial offset
                (roll, pitch, yaw) = euler_from_quaternion([qx, qy, qz, qw])

                current_roll = roll - self.offset_roll
                current_pitch = pitch - self.offset_pitch
                current_yaw = yaw - self.offset_yaw

                quat = quaternion_from_euler(current_roll, current_pitch, current_yaw)

                self.orientation_x = quat[0]
                self.orientation_y = quat[1]
                self.orientation_z = quat[2]
                self.orientation_w = quat[3]

                return [True, 'ok']
            else:
                # self.load_calibration()
                self.is_init_device = False
                self.is_init_imu = False
                self.is_init_calibration = False
                self.init_imu()
                self.force_calibration = False
                return [False, 'Calibrating IMU sensor']
        except:
            self.is_init_device = False
            self.is_init_imu = False
            self.is_init_calibration = False
            self.init_imu()
            return [False, 'Error while reading IMU sensor: ' + str(sys.exc_info()[0])]
