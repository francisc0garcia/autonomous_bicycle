#!/usr/bin/env python

from classes.bags.recorder import *

import rospy
import rosbag
import time
import threading

from autonomous_bicycle.srv import record_msg, status_msg


class BagRecorder:
    def __init__(self):
        self._recorder = None
        self.filename = 'test.bag'
        self.all = False
        self.regex = False
        self.limit = 0
        self._bag_lock = threading.RLock()

        self.default_folder = rospy.get_param('~default_folder', '/home/data/')
        self.rate = rospy.get_param('~rate', 100.0)  # the rate of recording

        self.service_control = rospy.Service('change_bag_record', record_msg, self.callback_service)
        self.service_state = rospy.Service('check_bag_record_status', status_msg, self.callback_status)

        self.enable_recording = False
        self.is_recording = False

        # TODO: add yaml config file
        self.topics = [
            '/bicycle/image_raw/compressed', '/bicycle/camera_info', '/bicycle/state_system',
            '/bicycle/altitude', '/bicycle/distance', '/bicycle/velocity', '/tf',
            '/bicycle/imu_1', '/bicycle/imu_2', '/bicycle/imu_steering', '/bicycle/steering_angle',
            '/bicycle/gps_front', '/bicycle/gps_front_velocity', '/bicycle/odom_gps_front',
            '/bicycle/gps_rear', '/bicycle/gps_rear_velocity', '/bicycle/odom_gps_rear',
            '/bicycle/odom_main_frame', '/bicycle/odom_rear_wheel',
            '/imu_lean/pitch', '/imu_lean/roll', '/imu_lean/yaw',
            '/imu_lean_noise/pitch', '/imu_lean_noise/roll', '/imu_lean_noise/yaw',
            '/imu_steering/pitch', '/imu_steering/roll', '/imu_steering/yaw',
            '/imu_steering_noise/pitch', '/imu_steering_noise/roll', '/imu_steering_noise/yaw',
            '/bicycle/imu_steer_calibration', '/bicycle/imu_1_calibration'
        ]

        self.rate_timer = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.rate_timer.sleep()

        rospy.loginfo('Stop recording')

    def callback_status(self, req):
        return self.is_recording

    def callback_service(self, req):
        # print req.topics
        # print req.enable_record
        state = ''
        self.filename = self.default_folder + req.filename
        self.enable_recording = req.enable_record

        if self.enable_recording:
            try:
                if not self.is_recording:
                    self.is_recording = True

                    self._recorder = Recorder(self.filename,
                                              bag_lock=self._bag_lock,
                                              all=self.all,
                                              topics=self.topics,
                                              regex=self.regex,
                                              limit=self.limit)
                    self._recorder.start()
                    state = 'recording'
                else:
                    state = 'Process already running'
            except Exception as ex:
                self.is_recording = False
                rospy.loginfo('Error opening bag ')
                state = 'error'
                rospy.logerr(str(ex))
        else:
            if self.is_recording:
                self._recorder.stop()
            self.is_recording = False
            state = 'stopped'

        rospy.loginfo('state bag recording: ' + state)
        return 'state bag recording: ' + state

# Main function.
if __name__ == '__main__':
    rospy.init_node('BagRecorder', anonymous=True)
    try:
        obj = BagRecorder()
    except Exception as ex:
        rospy.logerr(str(ex))
