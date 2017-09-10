#!/usr/bin/env python

"""
Based on project: https://github.com/AtsushiSakai/rosbag_to_csv
"""
import glob
import os
import sys
from datetime import datetime
import scipy.io as sp_io

import rosbag
import rospy

# Add project into path to get acces to classes located outside of this scope
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(CURRENT_DIR))

from docs.python_notebooks.DatasetImporter import *
from docs.python_notebooks.RealDatasetImporter import *


class Rosbag2csv:
    def __init__(self):
        self.input_filename = rospy.get_param('~input_filename', 'input.bag')
        self.input_path = rospy.get_param('~input_path', '/tmp')
        self.output_path = rospy.get_param('~output_path', '/tmp')
        self.output_processed_filename = rospy.get_param('~output_processed_filename', 'processed.csv')
        self.input_format = rospy.get_param('~input_format', 'gazebo')  # gazebo or real_data
        self.resampling = rospy.get_param('~resampling', '')  # 3S -> 3 seconds or 30L -> 30 milli-seconds

        self.include_header = rospy.get_param('~include_header', True)
        self.filename_merge_csv = ''

        # List of desired topics to be recorded
        self.topic_names = [
            '/bicycle/imu_1', '/bicycle/imu_2', '/bicycle/imu_steering', '/bicycle/steering_angle',
            '/bicycle/gps_front', '/bicycle/gps_front_velocity', '/bicycle/odom_gps_front',
            '/bicycle/gps_rear', '/bicycle/gps_rear_velocity', '/bicycle/odom_gps_rear',
            '/bicycle/odom_main_frame', '/bicycle/odom_rear_wheel',
            '/imu_lean/pitch', '/imu_lean/roll', '/imu_lean/yaw',
            '/imu_lean_noise/pitch', '/imu_lean_noise/roll', '/imu_lean_noise/yaw',
            '/imu_steering/pitch', '/imu_steering/roll', '/imu_steering/yaw',
            '/imu_steering_noise/pitch', '/imu_steering_noise/roll', '/imu_steering_noise/yaw',
            '/bicycle/altitude', '/bicycle/velocity',
            '/bicycle/imu_steer_calibration', '/bicycle/imu_1_calibration'
        ]

        # list of fields to be merge
        self.field_names = [
            'time',
            # IMU Data
            '.orientation.x', '.orientation.y', '.orientation.z', '.orientation.w',
            '.angular_velocity.x', '.angular_velocity.y', '.angular_velocity.z',
            '.linear_acceleration.x', '.linear_acceleration.y', '.linear_acceleration.z',
            # GPS odometry Data
            '.pose.pose.position.x', '.pose.pose.position.y', '.pose.pose.position.z',
            '.latitude', '.longitude',
            # Velocity GPS odometry Data (delivered by GPS)
            '.vector.x', '.vector.y', '.vector.z',
            # odometry velocity (ground truth velocity)
            '.twist.twist.linear.x', '.twist.twist.linear.y',
            '.twist.linear.x', '.twist.linear.y',
            # float data
            '.data',
            # simulated altitude
            '.altitude'
        ]

        self.filename_merge_results = self.input_filename.replace('.bag', '.csv')
        self.directory_csv_results = self.output_path + self.input_filename.replace('.bag', '') + '/'

        rospy.loginfo("Starting conversion of " + self.input_path + self.input_filename)
        self.bag_to_csv()
        rospy.loginfo("successful conversion, results in: " + self.output_path)

        rospy.loginfo("Starting merge of " + self.output_path)
        self.csv_merge()
        rospy.loginfo("successful merge, results in: " + self.filename_merge_results)

    def message_to_csv(self, stream, msg, flatten=False):
        """
        stream: StringIO
        msg: message
        """
        try:
            for s in type(msg).__slots__:
                val = msg.__getattribute__(s)
                self.message_to_csv(stream, val, flatten)
        except:
            msg_str = str(msg)
            if msg_str.find(",") is not -1:
                if flatten:
                    msg_str = msg_str.strip("(")
                    msg_str = msg_str.strip(")")
                    msg_str = msg_str.strip(" ")
                else:
                    msg_str = "\"" + msg_str + "\""
            stream.write("," + msg_str)

    def message_type_to_csv(self, stream, msg, parent_content_name=""):
        """
        stream: StringIO
        msg: message
        """
        try:
            for s in type(msg).__slots__:
                val = msg.__getattribute__(s)
                self.message_type_to_csv(stream, val, ".".join([parent_content_name, s]))
        except:
            stream.write("," + parent_content_name)

    def bag_to_csv(self):
        try:
            bag = rosbag.Bag(self.input_path + self.input_filename)
            streamdict = dict()
        except Exception as e:
            rospy.logfatal('failed to load bag file: %s', e)
            return

        try:
            for topic, msg, time in bag.read_messages(topics=self.topic_names):
                if streamdict.has_key(topic):
                    stream = streamdict[topic]
                else:
                    if not os.path.exists(self.directory_csv_results):
                        os.makedirs(self.directory_csv_results)

                    fileName = self.directory_csv_results + topic.replace('/', '_') + '.csv'

                    rospy.loginfo("output file: " + fileName)

                    stream = open(fileName, 'w')
                    streamdict[topic] = stream

                    if self.include_header:
                        stream.write("time")
                        self.message_type_to_csv(stream, msg)
                        stream.write('\n')

                stream.write(datetime.fromtimestamp(time.to_time()).strftime('%Y/%m/%d/%H:%M:%S.%f'))
                self.message_to_csv(stream, msg, flatten=not self.include_header)
                stream.write('\n')
            [s.close for s in streamdict.values()]
        except Exception as e:
            rospy.logwarn("fail: %s", e)
        finally:
            bag.close()

    def csv_merge(self):
        root = self.directory_csv_results
        pattern = "*.csv"

        # Remove merge file if already exist
        try:
            os.remove(root + self.filename_merge_results)
        except OSError:
            pass

        # get CSV files
        files = glob.glob(root + pattern)

        # define desired columns
        list_columns = self.field_names
        df_desired_columns = pd.DataFrame(list_columns)

        # read all files
        # TODO: Fix problem, drop duplicates eliminate nanosecond information (time key)
        # [time, .header.stamp.secs, .header.stamp.nsecs]
        dfs = [pd.read_csv(fp, parse_dates=['time'])
                   .drop_duplicates(subset=['time'])
                   .set_index(['time'], drop=False, verify_integrity=True)
               for fp in files]

        # replace column name
        for i in range(len(dfs)):
            rospy.loginfo("merging file: " + files[i])
            columns_file = dfs[i].columns.values.tolist()
            df_filtered = pd.merge(pd.DataFrame(columns_file), df_desired_columns, how='inner')
            dfs[i] = dfs[i].filter(items=list(df_filtered.values.flatten()))

            names = dfs[i].columns.tolist()
            name_column_base = files[i].replace(root, '').replace(self.input_filename, '').replace('.csv', '')
            for column in names:
                if column != 'time':
                    names[names.index(column)] = name_column_base + column
                    dfs[i].columns = names

        base_file = dfs[0]

        for i in range(1, len(dfs)):
            # merge csv files using outer join
            base_file = pd.merge(base_file, dfs[i], how='outer', indicator=False, on=['time'])

        # sort values by time
        base_file = base_file.set_index(['time'], drop=False, verify_integrity=True).sort_values(by='time')
        self.filename_merge_csv = root + self.filename_merge_results
        base_file.to_csv(self.filename_merge_csv, index=False)

        self.preprocess_data(self.filename_merge_csv, self.output_processed_filename)

    def preprocess_data(self, input_file_name, output_file_name):
        rospy.loginfo("Preprocessing merged file: " + input_file_name)

        if self.input_format == 'gazebo':
            di = DatasetImporter(input_file_name, fill_na=True)
        else:
            di = RealDatasetImporter(input_file_name, resampling=self.resampling)

        di.data.to_csv(output_file_name)
        rospy.loginfo("Finished: preprocessed file saved to: " + output_file_name)

        # Export to Mat file ----------------------------------------------------------------------

        # rename columns
        di.data.rename(columns=lambda x: 'D_' + x.replace('.', '_').replace('_bicycle_', '').replace('pose', '')[-28:],
                       inplace=True)
        # Create dictionary and export as .mat
        a_dict = {col_name: di.data[col_name].values for col_name in di.data.columns.values}
        sp_io.savemat(output_file_name + '.mat', {'bicycle_data': a_dict})
        rospy.loginfo("Finished: file saved in Matlab format: " + output_file_name + '.mat')
        # ------------------------------------------------------------------------------------------

if __name__ == '__main__':
    rospy.init_node('rosbag_to_csv', anonymous=True)

    Rosbag2csv()
