#!/usr/bin/env python

# ROS node for the Neato Robot Vacuum
# Copyright (c) 2010 University at Albany. All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the University at Albany nor the names of its 
#       contributors may be used to endorse or promote products derived 
#       from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
ROS node for Neato XV-11 Robot Vacuum.
"""

__author__ = "ferguson@cs.albany.edu (Michael Ferguson)"

import roslib; roslib.load_manifest("neato_node")
import serial
import rospy
import time
from math import sin,cos

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

# from neato_driver.neato_driver import xv11, BASE_WIDTH, MAX_SPEED
# Neato Laser Distance Sensor


class Scan:
    def __init__(self):
        # self.start_time = time.time()
        # self.end_time = self.start_time 
        self.ranges = [None]*360
        # self._index = 0 # higher packet index seen
    
    def add_data(self, index, data):
        ''' Add data to the scan
        index - the index of the packet (in range(0,90))
        data - the 16 bytes of data of the packet,
               as they are received from the LDS
        returns False if the data is for another scan, True otherwise
        '''

        # if index < self._index:
            # the packet is necessarily for the next scan since indexes 
            # in a scan only grows
            # self._finalize()
            # return False
        # self._index=index
        angle_base = index*4
        # Bytes 06 and 05 are 1st  distance measurement in this packet.
        # Bytes 10 and 09 are 2nd  distance measurement in this packet.
        # Bytes 14 and 13 are 3rd  distance measurement in this packet.
        # Bytes 18 and 17 are the 4th distance measurement in this packet.
        if data:
            # for i in range(0,4): ???
            for i in range(0,3):
                self.angle = angle_base + i - 10
                if self.angle < 0 : self.angle += 360
                self.ranges[self.angle] = (data[4*i+0] | (( data[4*i+1] & 0x3f) << 8)) / 1000.0
                self.strength = data[4*i+2] | (data[4*i+3] << 8)
                rospy.loginfo("%d : %d %d", self.angle, self.ranges[self.angle],self.strength);
            
                if data[4*i+1] & 0x80:
                  error = data[4*i+0]
                  distance = 0.0
                  strength_warning = False
                  strength = 0.0
                else:
                  error = 0
                  if data[4*i+1] & 0x40:
                      strength_warning = True
                      rospy.loginfo("strength_warning");
                  else:
                      strength_warning = False
        # Bytes 08:07, 12:11, 16:15, and 20:19 represent information about
        # the surface off of which the laser has bounced to be detected by
        # the LiDAR.
        else:
            # add dummy, data (make same as previous valid data)
            for i in range(0,4):
                self.angle = angle_base + i - 10
                if self.angle < 0 : self.angle += 360
                self.ranges[self.angle] = self.ranges[self.angle-1]

        # if this was the last packet of this scan, we can finalize it now
        if index == 89:
            self._finalize()
        return True

    def _finalize(self):
        # self.end_time = time.time()
        self.header.stamp = rospy.Time.now()

    def complete(self):
        '''Test if the full 360deg have been received'''
        # return self.start_time != self.end_time

class Lds:
    STATE_START = 0
    STATE_INDEX = 1
    STATE_DATA = 2
    
    def __init__(self):
        self.state = 0
        self.buffer = []
        self._scan = Scan()
        

    def parse(self, data):
        data = str(data)
        #restore state of the parser
        state = self.state
        buff = self.buffer
        # The LiDAR spins counterclockwise at 10 revolutions per second.
        # Each revolution yields 90 packets.
        # Each packet contains 22 bytes.
        # Within these 22 bytes are 4 distance measurements and more
        # (4 data points/packet * 90 packets = 360 data points).
        # So there is one data measurement per degree turn.
        for byte in data:
            b = ord(byte)
            if state == Lds.STATE_START:
                # rospy.loginfo("state_start");
               # Byte 01, "FA" is a starting byte which appears between 
               # the ending and beginning of two packets.
                if b == 0xFA : 
                    state = Lds.STATE_INDEX
                    buff.append(b)
                    rospy.loginfo("append {:02X}".format(b));
                else:
                    rospy.loginfo("discard {:02X}".format(b));
                    buff = []
            elif state == Lds.STATE_INDEX:
                # Byte 02 is a hex value from A0-F9, representing the 90 packets
                # outputted per revolution.
                if b >= 0xA0 and b <= 0xF9 :
                    state = Lds.STATE_DATA
                    # rospy.loginfo("index {:02X}".format(b - 0xA0));
                    rospy.loginfo("index %d", (b - 0xA0))
                    # index  = buff[1] - 0xA0
                    buff.append(b)
                else:
                    rospy.loginfo("index discard {:02X}".format(b));
                    state = Lds.STATE_START
                    buff = []
            else:
                rospy.loginfo("buf {:02X}".format(b));
                buff.append(b)
                rospy.loginfo("buflen %d",len(buff));
                if len(buff) == 22:
                    # verify that the received checksum is equal 
                    # to the one computed
                    incoming_checksum = buff[20] + (buff[21] << 8)
                    checksum = Lds.checksum(buff)
                    index  = buff[1] - 0xA0
                    rospy.loginfo("index  {:02X}".format(index));
                    # Bytes 22 and 21 are checksum and are used for determining
                    # the validity of the received packet.
                    if checksum == incoming_checksum:
                        speed_rpm = float( buff[2] + (buff[3] << 8)) / 64.0
                        # should be buff[4:19] instead of 4:20?
                        res = self._scan.add_data(index,  buff[4:19] )
                        if not res:
                            self.on_full_scan(self._scan)
                            self._scan = Scan()
                            self._scan.add_data(index,  buff[4:19] )
                        # self.on_packet_received(index , speed_rpm, buff)
                        #print "index:", index, "speed (Hz):", speed_rpm / 60
                        rospy.loginfo("packet received ! speed {}", speed_rpm)
                    else:
                        # the checksum does not match, something went wrong...
                        # for now, just discard the data... 
                        # TODO see how the error should be handled
                        # rospy.loginfo("LDS chksm error ")
                        # rospy.loginfo("LDS chksm error {:04X}".format(incoming_checksum))
                        # rospy.loginfo("vs  chksm {:04X}".format( checksum))
                        rospy.loginfo("LDS chksm error %d",incoming_checksum)
                        rospy.loginfo("vs  chksm %d", checksum)
                        print "LDS chksm error", buff[1]-0xA0
                        speed_rpm = float( buff[2] + (buff[3] << 8)) / 64.0
                        rospy.loginfo("speed %f", speed_rpm)
                        res = self._scan.add_data(index,  buff[4:20] )
                        if not res:
                            self.on_full_scan(self._scan)
                            self._scan = Scan()
                            self._scan.add_data(index,  buff[4:20] )
                        # self.on_packet_received(index , speed_rpm, buff)
                        #print "index:", index, "speed (Hz):", speed_rpm / 60
                        self._scan.add_data(index, None )
                    state = Lds.STATE_START
                    buff = []
        #save current state
        self.state = state
        self.buffer = buff

    @staticmethod

    # from http://xv11hacking.wikispaces.com/LIDAR+Sensor
    def checksum(data):
        # TODO clean it up so that it works even when data is not of length 20...
        # group the data my word, little endian
        data_list = []
        for t in range(10):
            data_list.append( data[2*t] + (data[2*t+1]<<8) )
            rospy.loginfo("t %d %d", t, data_list[t])

        # data_list = [  data[2*t] + (data[2*t+1]<<8) for t in xrange(10) ]
        # compute the checksum.
        chk32 = 0
        for d in data_list:
            chk32 = (chk32 << 1) + d
        # return a value wrapped around on 15bits, 
        # and truncated to still fit into 15 bits

        # wrap around to fit into 15 bits
        checksum = (chk32 & 0x7FFF) + ( chk32 >> 15 ) 
        checksum = checksum & 0x7FFF # truncate to 15 bits
        return int( checksum )

    #hooks
    def on_packet_received(self, index, speed, data):
        rospy.loginfo("packet received ! speed {}", speed)

    def on_full_scan(self, scan_data):
        rospy.loginfo("full scan !")
        # scan.header.stamp = rospy.Time.now()
        # scan.ranges = scan_data
        # self.scanPub.publish(scan_data)
	rospy.loginfo("Published to ROS")
        
class NeatoNode:

    def __init__(self):
        """ Start up connection to the Neato Robot. """
        rospy.init_node('neato')

        portdev = rospy.get_param('~port', "/dev/ttyACM0")
        # self.port = rospy.get_param('~port', "/dev/prlite_neato")
        # self.port = rospy.get_param('~port', "/dev/ttyS0")
        # self.port = rospy.get_param('~port', "/dev/ttyUSB0")
        rospy.loginfo("Using port: %s"%(portdev))
        self.port = serial.Serial(portdev,115200)
        self.scanPub = rospy.Publisher('base_scan', LaserScan)
        self.lds = Lds()
        # self.lds.on_full_scan = on_full_scan

    def spin(self):        
        encoders = [0,0]

        self.x = 0                  # position in xy plane
        self.y = 0
        self.th = 0
        then = rospy.Time.now()

        # things that don't ever change
        scan_link = rospy.get_param('~frame_id','neato_link')
        scan = LaserScan(header=rospy.Header(frame_id=scan_link)) 
        scan.angle_min = 0
        scan.angle_max = 6.26
        scan.angle_increment = 0.017437326
        scan.range_min = 0.020
        scan.range_max = 5.0

        # The LiDAR spins counterclockwise at 10 revolutions per second.
        # Each revolution yields 90 packets.
        # Each packet contains 22 bytes.
        # Within these 22 bytes are 4 distance measurements and more
        # (4 data points/packet * 90 packets = 360 data points). 
        # So there is one data measurement per degree turn.
        # Byte 01, "FA" is a starting byte which appears between the ending 
        # and beginning of two packets.
        # Byte 02 is a hex value from A0-F9, representing the 90 packets 
        # outputted per revolution.
        # Byte 03 and 04 are a 16bit (combined) value representing the speed 
        # at which the LiDAR is rotating.
        # Bytes 06 and 05 are 1st  distance measurement in this packet.
        # Bytes 10 and 09 are 2nd  distance measurement in this packet.
        # Bytes 14 and 13 are 3rd  distance measurement in this packet.
        # Bytes 18 and 17 are the 4th distance measurement in this packet. 
        # Bytes 08:07, 12:11, 16:15, and 20:19 represent information about 
        # the surface off of which the laser has bounced to be detected by 
        # the LiDAR.
        # Bytes 22 and 21 are checksum and are used for determining the 
        # validity of the received packet.
    
        # main loop of driver
        # r = rospy.Rate(10)
        rospy.loginfo("0")
        # requestScan()
        data = []
        i = 0
        while not rospy.is_shutdown():
            # string = self.port.readline()
            byte = self.port.read()
            b = ord(byte)
            data.append(b)
            i = i +1
            if i > 1000:
              for j in range(0,999):
                rospy.loginfo("%d", j);
                rospy.loginfo(": {:02X}".format(data[j]));
                i = 0
                data = []
            
        # self.lds.parse(string)

if __name__ == "__main__":    
    robot = NeatoNode()
    robot.spin()

