#!/usr/bin/env python

import rospy
import serial
import time
from binascii import unhexlify


class ConfigGpsSensors:
    # Config constant for UBLOX NEO 7
    BAUD_115200 = 'B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 00 C2 01' \
                  + ' 00 03 00 03 00 00 00 00 00 BC 5E'

    FREQ_5HZ = 'B5 62 06 08 06 00 C8 00 01 00 01 00 DE 6A'

    MODEL_PEDESTRIAN = 'B5 62 06 24 24 00 FF FF 03 03 00 00 00 00 10 27 00 00 05 00 FA' \
                       + ' 00 FA 00 64 00 2C 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 13 76'

    COG_ALWAYS = 'B5 62 06 17 04 00 20 23 00 00 64 D2'

    SAVE_TO_MEMORY = 'B5 62 06 09 0D 00 00 00 00 00 FF FF 00 00 00 00 00 00 03 1D AB'

    def __init__(self):

        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baud = rospy.get_param('~baud', 9600)

        # First change baud rate from 9600 to 115200
        self.change_baud_rate()

        try:
            # open again at 115200
            self.ser = serial.Serial(
                port=self.port,
                baudrate=115200,
                timeout=2
            )

            if self.ser.isOpen():
                # update configuration
                self.write_command(self.FREQ_5HZ)
                self.write_command(self.MODEL_PEDESTRIAN)
                self.write_command(self.COG_ALWAYS)
                self.write_command(self.SAVE_TO_MEMORY)

                self.ser.close()
                rospy.loginfo('configuration successfully updated')
            else:
                rospy.logerr('serial port is closed')

        except rospy.ROSInterruptException:
            rospy.logerr('Error while updating GPS configuration')
            self.ser.close()  # Close GPS serial port

    def change_baud_rate(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=2
            )

            if self.ser.isOpen():
                self.write_command(self.BAUD_115200)
                self.write_command(self.SAVE_TO_MEMORY)
                # self.ser.close()
                rospy.loginfo('successfully changed baud rate')
            else:
                rospy.logerr('serial port is closed')

        except BaseException as e:
            rospy.logerr('Could not open serial ' + self.port + ' at ' + str(self.baud) + ' - ' + str(e))

    def write_command(self, str_command):
        words = str_command.split(" ")
        for w in words:
            # rospy.loginfo("sending: " + w)
            self.ser.write(unhexlify(w))

        time.sleep(0.2)


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('ConfigGpsSensors')

    obj = ConfigGpsSensors()
