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


class Lds:
    STATE_START      = 0
    STATE_SPEED1     = 1
    STATE_SPEED2     = 2
    STATE_DIST1      = 3
    STATE_DIST2      = 4
    STATE_STRENGTH1  = 5
    STATE_STRENGTH2  = 6
    STATE_END        = 7
    STATE_ERROR      = 8
    
    def __init__(self):
        self.state = 0
        self.prevbyte = 0x00
        self.cached_b2 = 0x00
        self.cached_b3 = 0x00
        self.angle = 0
        self.angle_deg = 0
        self.ranges = list()
        self.scanPub = rospy.Publisher('base_scan', LaserScan)

        # things that don't ever change
        scan_link = rospy.get_param('~frame_id','base_link')
        self.scan = LaserScan(header=rospy.Header(frame_id=scan_link))
        self.scan.angle_min = 0
        self.scan.angle_max = 6.26
        self.scan.angle_increment = 0.017437326
        self.scan.range_min = 0.020
        self.scan.range_max = 5.0
        self.state = Lds.STATE_START

        # the 0 is not aligned with the axis of the LDS
        angle = self.angle_deg - 10
        if angle < 0 : angle += 360


    def parse(self, data):
        # note data is always one byte, therefore should clean up
        data = str(data)
        #restore state of the parser
        state = self.state
        for byte in data:
            b = ord(byte)
            # rospy.loginfo("byte {:02X}".format(b))
            # rospy.loginfo("b {:02X}".format(b))
            # allow reset to start at any time
            if b == 0xC0 and (state == Lds.STATE_START or state == Lds.STATE_ERROR):
                # rospy.loginfo("start " + "{:02X}".format(self.cached_b3) + ", {:02X}".format(self.cached_b2) + ", {:02X}".format(self.prevbyte) + ", {:02X}".format(b));
                self.angle = 0
                state = Lds.STATE_SPEED1
            elif state == Lds.STATE_SPEED1:
                # rospy.loginfo("state_speed1")
                state = Lds.STATE_SPEED2
            elif state == Lds.STATE_SPEED2:
                # goodvalue appears to be 5
                # goodvalue appears to be 5508, 20560 (?), 32776, 2056, 2176
                # goodvalue appears to be 5384
                # expecting 600 RPM?
                speed = self.prevbyte + (b << 8)
                # rospy.loginfo("state_speed2 = " + str(speed))
                # rospy.loginfo("s " + str(speed))
                state = Lds.STATE_DIST1
            elif state == Lds.STATE_DIST1:
                #  data byte 0 : <distance 7:0>`
                # rospy.loginfo("state_dist1")
                self.angle = self.angle + 1
                state = Lds.STATE_DIST2
            elif state == Lds.STATE_DIST2:
                # data byte 1 : 
                # <"invalid data" flg> <"quality warning" flg> <dist 13:8>`
                if b & 0x80:
                    error = self.prevbyte
                    dist = 0.0
                    strength_warning = False
                    strength = 0.0
                    # rospy.loginfo("invalid data(" + str(self.angle) +") = " + str(error))
                    rospy.loginfo("i");
                    state = Lds.STATE_ERROR
                else:
                    self.error = 0
                    if b & 0x40:
                        strength_warning = True
                        # rospy.loginfo("strength warning(" + str(self.angle) +")")
                    else:
                        strength_warning = False
                    dist = self.prevbyte | (( b & 0x3f) << 8)
                    # may want to adjust as axis is off 10
                    self.scan.ranges.append(dist / 1000.0)
                    if (self.angle % 50 == 0):
                        rospy.loginfo("d " + str(self.angle))
                    #   rospy.loginfo("state_dist2(" + str(self.angle) +") = " + str(dist))
                    state = Lds.STATE_STRENGTH1
            elif state == Lds.STATE_STRENGTH1:
                # data byte 2 : <quality 7:0>`
                # rospy.loginfo("state_strength1")
                state = Lds.STATE_STRENGTH2
            elif state == Lds.STATE_STRENGTH2:
                # data byte 3 : <quality 15:8>`
                # This value can get very high when facing a retroreflector. 
                strength = self.prevbyte + (b << 8)
                # rospy.loginfo("strenght(" + str(self.angle) +") = " + str(strength))
                if self.angle == 360:
                  state = Lds.STATE_END
                  # publish to ROS 
                  rospy.loginfo("full"); 
                  self.scan.header.stamp = rospy.Time.now()
                  self.scanPub.publish(self.scan)
	          # rospy.loginfo("Published to ROS")
                else:
                  state = Lds.STATE_DIST1
            elif state == Lds.STATE_END:
                self.angle = self.angle + 1
                endbyte = self.angle - 360
                # rospy.loginfo("state_end" + str(endbyte))
                # 3 end bytes then should be back to C0
                if endbyte > 3:
                  state = Lds.STATE_ERROR
            else:
                state = Lds.STATE_ERROR
                # rospy.loginfo("state_error {:02X}".format(b))
        #save current state
        self.state = state
        self.cached_b3 = self.cached_b2
        self.cached_b2 = self.prevbyte
        self.prevbyte = b

class NeatoNode:

    def __init__(self):
        """ Start up connection to the Neato Robot. """
        rospy.init_node('neato')

        # portdev = rospy.get_param('~port', "/dev/ttyACM0")
        portdev = rospy.get_param('~port', "/dev/prlite-neato")
        rospy.loginfo("Using port: %s"%(portdev))
        self.port = serial.Serial(portdev,115200)
        self.lds = Lds()

    def spin(self):        
        # From http://xv11hacking.wikispaces.com/LIDAR+Sensor
        # The LiDAR spins counterclockwise at 10 revolutions per second.
        #
        # Early XV-11 units may be 3V3 powered! Not 5V!
        # Our XV-11 uses V2.1 firmware, which means that we need to use 3.3V!
        # Maximum current draw is 50 mA from the Arduino pin. 
        # Sensor power consumption (does not include the motor): ~145mA @ 3.3V
        #
        # The motor can be powered at 3.3V continuous ( ~60mA ) in open loop,
        # which will produce a turn rate of around 240rpm on a clean and recent
        # sensor. Hair and dust can however create friction that will lower 
        # the rotation speed.
        #
        # Data format for firmware 2.1 (Sparkfun scans, pre-production units)
        # The periodicity of the data is 1446 bytes.
        # 
        # When valid, format is organized as follow :
        # 
        # `5A A5 00 C0 XX XX <data>``
        #  90 165 0 192
        # XX and XX are speed
        # 
        # `<data>` is composed of 360 group of 4 bytes, organized like this :
        # `byte 0 : <distance 7:0>`
        # `byte 1 : <"invalid data" flg> <"quality warning" flg> <dist 13:8>`
        # `byte 2 : <quality 7:0>`
        # `byte 3 : <quality 15:8>`

        # The bit 7 of byte 1 seems to indicate that the distance could 
        # not be calculated.
        # When this bit is set, the second byte is always `80`, the values 
        # of the first byte may only be `02`, `03`, `21`, `25`, `35` or `50`.
        # When it's `21`, then the whole block is `21 80 XX XX`, 
        # but for all the other values it's the data block is `YY 80 00 00`
        # maybe it's a code to say what type of error ? 
        # (`35` is preponderant, `21` seems to be when the beam is interrupted 
        # by the supports of the cover) .
        # Another thing to have a look to is the temporal repartition of the 
        # data... the first sample after the sync seems to always be 
        # `21 80 XX XX`, and when this pattern appears again, it's immediately 
        # after an other value, without the 0.2ms interval we can see most 
        # of the time between two blocks of 4...

        # The bit 6 of byte 1 is a warning when the reported strength is 
        # greatly inferior to what is expected at this distance. This may 
        # happen when the material has a low reflectance (black material...),
        # or when the dot does not have the expected size or shape (porous 
        # material, transparent fabric, grid, edge of an object...), or maybe 
        # when there are parasitic reflections (glass... ).

        # Byte 2 and 3 are the LSB and MSB of the strength indication. 
        # This value can get very high when facing a retroreflector. 

        # C0 hex = 192

        # main loop of driver
        r = rospy.Rate(5)
        rospy.loginfo("0")
        # requestScan()
        while not rospy.is_shutdown():
            # string = self.port.readline()
            string = self.port.read()
            self.lds.parse(string)

if __name__ == "__main__":    
    robot = NeatoNode()
    robot.spin()

