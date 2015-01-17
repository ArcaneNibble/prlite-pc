#!/usr/bin/env python

import roslib
roslib.load_manifest('pr2lite_moveit_config')
import rospy
from std_msgs.msg import Float64
#from packet_485net_dgram.msg import *
#from packets_485net.msg import *
from pr2lite_moveit_config.msg import *
from sensor_msgs.msg import JointState
import serial
import binascii
import struct
import time
import math
import numpy as np

stream_pub = None
dgram_pub = None
bootloader_pub = None
outgoing_pub = None
prev_time = None
jspr2msg = None
ser = None
pwrser = None
joint_states_msg = None
joint_states_pub = None
r_arm_la_ctr = 0
l_arm_la_ctr = 0
torso_la_ctr= 0
base_la_ctr = 0
bad_usb_tx = 0

class JointStatePR2Message():
    def __init__(self):
        global jspr2msg

        jspr2msg = JointState()
        jspr2msg.name = [ "left_shoulder_tilt_joint", "left_lin_act_cyl_joint", "left_upper_arm_hinge_joint","left_linear_actuator_joint", "right_shoulder_tilt_joint", "right_lin_act_cyl_joint", "right_upper_arm_hinge_joint","right_linear_actuator_joint", "torso_lift_joint", "wheel_linear_actuator_joint", "fl_anchor_rod_joint", "fl_caster_rotation_joint", "fr_anchor_rod_joint", "fr_caster_rotation_joint", "bl_anchor_rod_joint", "bl_caster_rotation_joint", "br_anchor_rod_joint", "br_caster_rotation_joint" ]

        jspr2msg.position = [0.0 for name in jspr2msg.name]
        jspr2msg.velocity = [0.0 for name in jspr2msg.name]
        rospy.loginfo("done init")
        
class JointStateMessages():
    def __init__(self):
      global joint_states_msg
      global joint_states_pub

      joint_states_msg = JointStatePR2Message()
      joint_states_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
      global ser
      global r_arm_la_ctr 
      global l_arm_la_ctr 
      global torso_la_ctr
      global base_la_ctr 
      global bad_usb_tx 

# returns the joint state (e.g. angles) associated with the shoulders 
#   based on linact length per feedback sensor
# must be kept in sync with pr2lite_actuators pkg, which sets the linact length
#   based on the desired angle
def shoulder_handler(packet):
      global  jspr2msg
      global r_arm_la_ctr 
      global l_arm_la_ctr 

      # set some local constants
      ONE_PI = 3.14159265359
      BAR_LEN = 4.3 - .315  # Upper arm bar that LinAct connects to
      # LINACT_MXLN = 4      # length of LinAct extension
      # LINACT_MXLN = 3.7    # length of LinAct extension
      LINACT_MXLN = 4        # length of LinAct extension
      LINACT_DWN = 9.9      # length of LinAct when retracted
      # reducing to 9.5 causes failures of driver
      #LINACT_DWN = 9.5      # length of LinAct when retracted
      LINACT_UP = LINACT_DWN + LINACT_MXLN # length of LinAct when extended
      HYPOTENUSE = 14.968   # Len from bottom LinAct hole to Shoulder Joint
      # HYPOTENUSE = LINACT_DWN + 5.068   # Len from bottom LinAct hole to Shoulder Joint
      LINACT_VEL = .5       # inches per second */
      FIXED_LA_ANGLE = (math.atan(BAR_LEN / HYPOTENUSE))
      INCHES_TO_METERS = 0.0254
      LEFT_SHOULDER_LINACT = 15
      RIGHT_SHOULDER_LINACT = 14
      LEFT_SHOULDER_TILT_JOINT = 0
      LEFT_LIN_ACT_CYL_JOINT = 1
      LEFT_UPPER_ARM_HINGE_JOINT = 2
      LEFT_LINEAR_ACTUATOR_JOINT = 3
      RIGHT_SHOULDER_TILT_JOINT = 4
      RIGHT_LIN_ACT_CYL_JOINT = 5
      RIGHT_UPPER_ARM_HINGE_JOINT = 6
      RIGHT_LINEAR_ACTUATOR_JOINT = 7

      # bytes 4 & 5 are length
      b = struct.unpack('B', packet.data[4])[0]
      b2 = struct.unpack('B', packet.data[5])[0]
      cur_pos = 256.0 * b2 + b
      m_arrived = struct.unpack("B", packet.data[4+2])
      length = (cur_pos) * LINACT_MXLN / 1000.0
      linact_length = length * INCHES_TO_METERS

      # linact_cyl_angle is the upper arm tilt & uppper_arm hinge
      angle=1.44129-math.acos((math.pow((length + LINACT_DWN), 2.0) - 256.0)/(-162.2));
      angle=angle + .1
      linact_cyl_angle = math.asin( (BAR_LEN*INCHES_TO_METERS) * math.sin(ONE_PI-angle) / (linact_length + (LINACT_DWN*INCHES_TO_METERS))) 

      # Bound the angle values to +- PI ?
      upper_arm_angle = angle
      # upper_arm_angle = angle + FIXED_LA_ANGLE 
      # prevent false collisions: unnecessary with fixed la angle?
      # the linact_cyl_angle is mostly for looks.  It's probably off by
      # a tiny fixed bracket angle.  Currently OK as it avoids false conflicts.

      if packet.source == LEFT_SHOULDER_LINACT:
	l_arm_la_ctr = 0
        jspr2msg.position[LEFT_LINEAR_ACTUATOR_JOINT] = linact_length
        jspr2msg.position[LEFT_LIN_ACT_CYL_JOINT] = linact_cyl_angle
        jspr2msg.position[LEFT_SHOULDER_TILT_JOINT] = upper_arm_angle
        jspr2msg.position[LEFT_UPPER_ARM_HINGE_JOINT] = -1 * upper_arm_angle
        # print "linact: Left Shoulder len %f angle %f" %  ( jspr2msg.position[LEFT_LINEAR_ACTUATOR_JOINT], jspr2msg.position[LEFT_SHOULDER_TILT_JOINT])
        if m_arrived:
          jspr2msg.velocity[LEFT_LINEAR_ACTUATOR_JOINT] = 0
          jspr2msg.velocity[LEFT_LIN_ACT_CYL_JOINT] = 0
          jspr2msg.velocity[LEFT_SHOULDER_TILT_JOINT] = 0
        else:
          jspr2msg.velocity[LEFT_LINEAR_ACTUATOR_JOINT] = LINACT_VEL*INCHES_TO_METERS
          jspr2msg.velocity[LEFT_LIN_ACT_CYL_JOINT] = LINACT_VEL*INCHES_TO_METERS
          jspr2msg.velocity[LEFT_SHOULDER_TILT_JOINT] = LINACT_VEL*INCHES_TO_METERS
      elif packet.source == RIGHT_SHOULDER_LINACT:
	  r_arm_la_ctr = 0
          jspr2msg.position[RIGHT_LINEAR_ACTUATOR_JOINT] = linact_length
          jspr2msg.position[RIGHT_LIN_ACT_CYL_JOINT] = linact_cyl_angle
          jspr2msg.position[RIGHT_SHOULDER_TILT_JOINT] = upper_arm_angle
          jspr2msg.position[RIGHT_UPPER_ARM_HINGE_JOINT] = -1 * upper_arm_angle
          # print "linact: Right Shoulder len %f angle %f" %  ( jspr2msg.position[RIGHT_LINEAR_ACTUATOR_JOINT], jspr2msg.position[RIGHT_SHOULDER_TILT_JOINT])
          if m_arrived:
            jspr2msg.velocity[RIGHT_LINEAR_ACTUATOR_JOINT] = 0
            jspr2msg.velocity[RIGHT_LIN_ACT_CYL_JOINT] = 0
            jspr2msg.velocity[RIGHT_SHOULDER_TILT_JOINT] = 0
            # print "linact: Right Shoulder len %f angle %f" %  ( jspr2msg.position[RIGHT_LINEAR_ACTUATOR_JOINT], jspr2msg.position[RIGHT_SHOULDER_TILT_JOINT])
            # rospy.loginfo( "linact: Right Shoulder len %f angle %f" %  ( jspr2msg.position[RIGHT_LINEAR_ACTUATOR_JOINT], jspr2msg.position[RIGHT_SHOULDER_TILT_JOINT]))
          else:
            jspr2msg.velocity[RIGHT_LINEAR_ACTUATOR_JOINT] = LINACT_VEL*INCHES_TO_METERS
            jspr2msg.velocity[RIGHT_LIN_ACT_CYL_JOINT] = LINACT_VEL*INCHES_TO_METERS
            jspr2msg.velocity[RIGHT_SHOULDER_TILT_JOINT] = LINACT_VEL*INCHES_TO_METERS
   
def torso_handler(packet):
      global jspr2msg
      global torso_la_ctr 

      torso_la_ctr = 0
      TORSO_LIFT_JOINT = 8
      INCHES_TO_METERS = 0.0254
      FAST_LINACT_VEL = 2   # inches per second */
      #cur_pos = struct.unpack("B", packet.data[4])
      b = struct.unpack('B', packet.data[4])[0]
      b2 = struct.unpack('B', packet.data[5])[0]
      cur_pos = 256.0 * b2 + b
      # print "Torso_handler!!"
      # print cur_pos
      m_arrived = struct.unpack("B", packet.data[4+2])
      # length = (1000 - cur_pos) * 12.0 / 1000.0 * INCHES_TO_METERS
      length = (cur_pos) * 12.0 / 1000.0 * INCHES_TO_METERS
      # if jspr2msg.position[TORSO_LIFT_JOINT] - length > .006:
        # print "linact: torso cur_pos %d" % cur_pos

      jspr2msg.position[TORSO_LIFT_JOINT] = length
      if m_arrived:
        jspr2msg.velocity[TORSO_LIFT_JOINT] = FAST_LINACT_VEL*INCHES_TO_METERS
      else:
        jspr2msg.velocity[TORSO_LIFT_JOINT] = FAST_LINACT_VEL*INCHES_TO_METERS
        # print "linact: torso moving"

def set_torso_pos(length):
      global jspr2msg
      TORSO_LIFT_JOINT = 8
      jspr2msg.position[TORSO_LIFT_JOINT] = float(length.data)
      # print "linact: torso cur_pos %f" % jspr2msg.position[TORSO_LIFT_JOINT] 

def base_handler(packet):
      global jspr2msg
      global base_la_ctr 

      base_la_ctr = 0
      PUSH_ROD_LEN = 0.1778
      CASTER_LEN = 0.089 / 2.0
      ONE_PI = 3.14159265359
      FAST_LINACT_VEL = 2   # inches per second */
      WHEEL_LINEAR_ACTUATOR_JOINT = 9
      FL_ANCHOR_ROD_JOINT = 10
      #FL_PUSH_ROD_JOINT = 11
      FL_CASTER_ROTATION_JOINT = 11
      FR_ANCHOR_ROD_JOINT = 12
      #FR_PUSH_ROD_JOINT = 14
      FR_CASTER_ROTATION_JOINT = 13
      BL_ANCHOR_ROD_JOINT = 14
      #BL_PUSH_ROD_JOINT = 16
      BL_CASTER_ROTATION_JOINT = 15
      BR_ANCHOR_ROD_JOINT = 16
      #BR_PUSH_ROD_JOINT = 19
      BR_CASTER_ROTATION_JOINT = 17
      INCHES_TO_METERS = 0.0254

      # cur_pos = struct.unpack("B", packet.data[4])
      b = struct.unpack('B', packet.data[4])[0]
      b2 = struct.unpack('B', packet.data[5])[0]
      cur_pos = 256.0 * b2 + b
      # print "base cur_pos ",cur_pos
      m_arrived = struct.unpack("B", packet.data[4+2])
      #length = (1000.0 - cur_pos[0]) * 4.0 / 1000.0 * INCHES_TO_METERS
      length = (cur_pos) * 4.0 / 1000.0 * INCHES_TO_METERS
      retracted_linact_side_angle = ONE_PI   # straight from linact rod to caster
      retracted_caster_angle = ONE_PI / 2.0 

      # a = pushrod ; b = casterlen ; c = side ; C = casterangle
      # cos rule : c = sqrt(a^2 + b^2 - 2ab cos(C))
      retracted_side = math.sqrt( math.pow(PUSH_ROD_LEN,2.0) + math.pow(CASTER_LEN, 2.0) + 2 * PUSH_ROD_LEN * CASTER_LEN * math.cos(retracted_caster_angle))
      retracted_pushrod_angle = 0

      # a = side ; b = pushrod_len ; B = caster_angle
      # sin rule : A = asin((sin(B) * a) / b)
      mid_caster_angle = ONE_PI / 4.0
      mid_side = math.sqrt( math.pow(PUSH_ROD_LEN,2.0) + math.pow(CASTER_LEN, 2.0) + 2 * PUSH_ROD_LEN * CASTER_LEN * math.cos(mid_caster_angle))
      mid_pushrod_angle = math.asin((math.sin(mid_caster_angle) * mid_side) / PUSH_ROD_LEN)
      mid_linact_side_angle = 2 * ONE_PI - mid_pushrod_angle - mid_caster_angle + (ONE_PI / 2.0)

      extended_linact_side_angle = 2*ONE_PI  # straight
      extended_caster_angle = 0
      extended_side = math.sqrt( math.pow(PUSH_ROD_LEN,2.0) + math.pow(CASTER_LEN, 2.0) + 2 * PUSH_ROD_LEN * CASTER_LEN * math.cos(extended_caster_angle))
      extended_pushrod_angle = math.asin((math.sin(extended_caster_angle) * extended_side) / PUSH_ROD_LEN)

      jspr2msg.position[WHEEL_LINEAR_ACTUATOR_JOINT] = length
      if length <= 1 * INCHES_TO_METERS :
        inverse = 1
        jspr2msg.position[FL_ANCHOR_ROD_JOINT] = retracted_linact_side_angle*inverse
        #jspr2msg.position[FL_PUSH_ROD_JOINT] = retracted_pushrod_angle*inverse
        inverse = -1
        jspr2msg.position[FL_CASTER_ROTATION_JOINT] = retracted_caster_angle*inverse
        inverse = 1
        jspr2msg.position[FR_ANCHOR_ROD_JOINT] = retracted_linact_side_angle*inverse
        #jspr2msg.position[FR_PUSH_ROD_JOINT] = retracted_pushrod_angle*inverse
        jspr2msg.position[FR_CASTER_ROTATION_JOINT] = retracted_caster_angle*inverse
        jspr2msg.position[BL_ANCHOR_ROD_JOINT] = retracted_linact_side_angle*inverse
        #jspr2msg.position[BL_PUSH_ROD_JOINT] = retracted_pushrod_angle*inverse
        jspr2msg.position[BL_CASTER_ROTATION_JOINT] = retracted_caster_angle*inverse
        jspr2msg.position[BR_ANCHOR_ROD_JOINT] = retracted_linact_side_angle*inverse
        #jspr2msg.position[BR_PUSH_ROD_JOINT] = retracted_pushrod_angle*inverse
        inverse = -1
        jspr2msg.position[BR_CASTER_ROTATION_JOINT] = retracted_caster_angle*inverse
      elif length <= 3 * INCHES_TO_METERS:
        inverse = 1
        jspr2msg.position[FL_ANCHOR_ROD_JOINT] = mid_linact_side_angle*inverse
        #jspr2msg.position[FL_PUSH_ROD_JOINT] = mid_pushrod_angle*inverse
        inverse = -1
        jspr2msg.position[FL_CASTER_ROTATION_JOINT] = mid_caster_angle*inverse
        inverse = 1
        jspr2msg.position[FR_ANCHOR_ROD_JOINT] = mid_linact_side_angle*inverse
        #jspr2msg.position[FR_PUSH_ROD_JOINT] = mid_pushrod_angle*inverse
        jspr2msg.position[FR_CASTER_ROTATION_JOINT] = mid_caster_angle*inverse
        jspr2msg.position[BL_ANCHOR_ROD_JOINT] = mid_linact_side_angle*inverse
        #jspr2msg.position[BL_PUSH_ROD_JOINT] = mid_pushrod_angle*inverse
        jspr2msg.position[BL_CASTER_ROTATION_JOINT] = mid_caster_angle*inverse
        inverse = -1
        jspr2msg.position[BR_ANCHOR_ROD_JOINT] = mid_linact_side_angle*inverse
        #jspr2msg.position[BR_PUSH_ROD_JOINT] = mid_pushrod_angle*inverse
        jspr2msg.position[BR_CASTER_ROTATION_JOINT] = mid_caster_angle*inverse
      else:
        inverse = 1
        jspr2msg.position[FL_ANCHOR_ROD_JOINT] = extended_linact_side_angle*inverse
        #jspr2msg.position[FL_PUSH_ROD_JOINT] = extended_pushrod_angle*inverse
        inverse = -1
        jspr2msg.position[FL_CASTER_ROTATION_JOINT] = extended_caster_angle*inverse
        inverse = 1
        jspr2msg.position[FR_ANCHOR_ROD_JOINT] = extended_linact_side_angle*inverse
        #jspr2msg.position[FR_PUSH_ROD_JOINT] = extended_pushrod_angle*inverse
        jspr2msg.position[FR_CASTER_ROTATION_JOINT] = extended_caster_angle*inverse
        jspr2msg.position[BL_ANCHOR_ROD_JOINT] = extended_linact_side_angle*inverse
        #jspr2msg.position[BL_PUSH_ROD_JOINT] = extended_pushrod_angle*inverse
        jspr2msg.position[BL_CASTER_ROTATION_JOINT] = extended_caster_angle*inverse
        inverse = -1
        jspr2msg.position[BR_ANCHOR_ROD_JOINT] = extended_linact_side_angle*inverse
        #jspr2msg.position[BR_PUSH_ROD_JOINT] = extended_pushrod_angle*inverse
        jspr2msg.position[BR_CASTER_ROTATION_JOINT] = extended_caster_angle*inverse
      if m_arrived:
        jspr2msg.velocity[FL_ANCHOR_ROD_JOINT] = 0
        #jspr2msg.velocity[FL_PUSH_ROD_JOINT] = 0
        jspr2msg.velocity[FL_CASTER_ROTATION_JOINT] = 0
        jspr2msg.velocity[FR_ANCHOR_ROD_JOINT] = 0
        #jspr2msg.velocity[FR_PUSH_ROD_JOINT] = 0
        jspr2msg.velocity[FR_CASTER_ROTATION_JOINT] = 0
        jspr2msg.velocity[BL_ANCHOR_ROD_JOINT] = 0
        #jspr2msg.velocity[BL_PUSH_ROD_JOINT] = 0
        jspr2msg.velocity[BL_CASTER_ROTATION_JOINT] = 0
        jspr2msg.velocity[BR_ANCHOR_ROD_JOINT] = 0
        #jspr2msg.velocity[BR_PUSH_ROD_JOINT] = 0
        jspr2msg.velocity[BR_CASTER_ROTATION_JOINT] = 0
      else:
        jspr2msg.velocity[FL_ANCHOR_ROD_JOINT] = FAST_LINACT_VEL*INCHES_TO_METERS
        #jspr2msg.velocity[FL_PUSH_ROD_JOINT] = FAST_LINACT_VEL*INCHES_TO_METERS
        jspr2msg.velocity[FL_CASTER_ROTATION_JOINT] = FAST_LINACT_VEL*INCHES_TO_METERS
        jspr2msg.velocity[FL_ANCHOR_ROD_JOINT] = FAST_LINACT_VEL*INCHES_TO_METERS
        #jspr2msg.velocity[FL_PUSH_ROD_JOINT] = FAST_LINACT_VEL*INCHES_TO_METERS
        jspr2msg.velocity[FL_CASTER_ROTATION_JOINT] = FAST_LINACT_VEL*INCHES_TO_METERS
        jspr2msg.velocity[FR_ANCHOR_ROD_JOINT] = FAST_LINACT_VEL*INCHES_TO_METERS
        #jspr2msg.velocity[FR_PUSH_ROD_JOINT] = FAST_LINACT_VEL*INCHES_TO_METERS
        jspr2msg.velocity[FR_CASTER_ROTATION_JOINT] = FAST_LINACT_VEL*INCHES_TO_METERS
        jspr2msg.velocity[BL_ANCHOR_ROD_JOINT] = FAST_LINACT_VEL*INCHES_TO_METERS
        #jspr2msg.velocity[BL_PUSH_ROD_JOINT] = FAST_LINACT_VEL*INCHES_TO_METERS
        jspr2msg.velocity[BL_CASTER_ROTATION_JOINT] = FAST_LINACT_VEL*INCHES_TO_METERS
        jspr2msg.velocity[BR_ANCHOR_ROD_JOINT] = FAST_LINACT_VEL*INCHES_TO_METERS
        #jspr2msg.velocity[BR_PUSH_ROD_JOINT] = FAST_LINACT_VEL*INCHES_TO_METERS
        jspr2msg.velocity[BR_CASTER_ROTATION_JOINT] = FAST_LINACT_VEL*INCHES_TO_METERS

def packet_dgram_handler(packet):
      LEFT_SHOULDER_LINACT = 15
      RIGHT_SHOULDER_LINACT = 14
      TORSO_LINACT = 12
      BASE_LINACT = 13
      # 0,1,2,3 seq num for packet
      # 4, 5 (int) current position
      # 6 (eight bit boolean) arrived
      # Store if it arrived
      # cur_pos = struct.unpack("B", packet.data[4])
      m_arrived = struct.unpack("B", packet.data[4+2])
      # print "packet "
      # print packet
      if packet.source == LEFT_SHOULDER_LINACT or packet.source == RIGHT_SHOULDER_LINACT:
        shoulder_handler(packet)
      elif packet.source == TORSO_LINACT:
        torso_handler(packet)
      elif packet.source == BASE_LINACT:
        base_handler(packet)
   
def publish_LA_states():
        global joint_states_msg
        global joint_states_pub
        # rospy.loginfo("pub LA states")
        jspr2msg.header.stamp = rospy.Time.now()
        joint_states_pub.publish(jspr2msg)
        # print jspr2msg
       
crc8_table = [
    #  0     1       2     3       4     5       6     7       8     9       A     B       C     D       E     F
    0x00, 0xF7, 0xB9, 0x4E, 0x25, 0xD2, 0x9C, 0x6B, 0x4A, 0xBD, 0xF3, 0x04, 0x6F, 0x98, 0xD6, 0x21,
    0x94, 0x63, 0x2D, 0xDA, 0xB1, 0x46, 0x08, 0xFF, 0xDE, 0x29, 0x67, 0x90, 0xFB, 0x0C, 0x42, 0xB5,
    0x7F, 0x88, 0xC6, 0x31, 0x5A, 0xAD, 0xE3, 0x14, 0x35, 0xC2, 0x8C, 0x7B, 0x10, 0xE7, 0xA9, 0x5E,
    0xEB, 0x1C, 0x52, 0xA5, 0xCE, 0x39, 0x77, 0x80, 0xA1, 0x56, 0x18, 0xEF, 0x84, 0x73, 0x3D, 0xCA,
    0xFE, 0x09, 0x47, 0xB0, 0xDB, 0x2C, 0x62, 0x95, 0xB4, 0x43, 0x0D, 0xFA, 0x91, 0x66, 0x28, 0xDF,
    0x6A, 0x9D, 0xD3, 0x24, 0x4F, 0xB8, 0xF6, 0x01, 0x20, 0xD7, 0x99, 0x6E, 0x05, 0xF2, 0xBC, 0x4B,
    0x81, 0x76, 0x38, 0xCF, 0xA4, 0x53, 0x1D, 0xEA, 0xCB, 0x3C, 0x72, 0x85, 0xEE, 0x19, 0x57, 0xA0,
    0x15, 0xE2, 0xAC, 0x5B, 0x30, 0xC7, 0x89, 0x7E, 0x5F, 0xA8, 0xE6, 0x11, 0x7A, 0x8D, 0xC3, 0x34,
    0xAB, 0x5C, 0x12, 0xE5, 0x8E, 0x79, 0x37, 0xC0, 0xE1, 0x16, 0x58, 0xAF, 0xC4, 0x33, 0x7D, 0x8A,
    0x3F, 0xC8, 0x86, 0x71, 0x1A, 0xED, 0xA3, 0x54, 0x75, 0x82, 0xCC, 0x3B, 0x50, 0xA7, 0xE9, 0x1E,
    0xD4, 0x23, 0x6D, 0x9A, 0xF1, 0x06, 0x48, 0xBF, 0x9E, 0x69, 0x27, 0xD0, 0xBB, 0x4C, 0x02, 0xF5,
    0x40, 0xB7, 0xF9, 0x0E, 0x65, 0x92, 0xDC, 0x2B, 0x0A, 0xFD, 0xB3, 0x44, 0x2F, 0xD8, 0x96, 0x61,
    0x55, 0xA2, 0xEC, 0x1B, 0x70, 0x87, 0xC9, 0x3E, 0x1F, 0xE8, 0xA6, 0x51, 0x3A, 0xCD, 0x83, 0x74,
    0xC1, 0x36, 0x78, 0x8F, 0xE4, 0x13, 0x5D, 0xAA, 0x8B, 0x7C, 0x32, 0xC5, 0xAE, 0x59, 0x17, 0xE0,
    0x2A, 0xDD, 0x93, 0x64, 0x0F, 0xF8, 0xB6, 0x41, 0x60, 0x97, 0xD9, 0x2E, 0x45, 0xB2, 0xFC, 0x0B,
    0xBE, 0x49, 0x07, 0xF0, 0x9B, 0x6C, 0x22, 0xD5, 0xF4, 0x03, 0x4D, 0xBA, 0xD1, 0x26, 0x68, 0x9F
]

def valid_dgram_packet(packet):
    if packet.source < 7 or packet.source > 15:
        rospy.loginfo( "bad message source : %d" % packet.source)
        return False
    if packet.destination != 0xF0 and packet.destination != 0x00:
        # rospy.loginfo( "bad message dest : %d" % packet.destination)
        return False
    if packet.dport != 7:
        # rospy.loginfo( "[id %d] invalid dport" % packet.source)
        return False
    # if packet.data.size() != 7:
    if len(packet.data) != 7:
        # probably a wheel rotation
        if packet.source < 7 or packet.source > 11:
           print "[id %d] invalid size" % packet.source
        return False
    return True
   
def do_checksum(data):
    csum = 0xFF;
    for b in data:
        byte = struct.unpack("B", b)[0]
        csum = crc8_table[(csum ^ byte) & 0xFF]
        csum = csum & 0xFF
    csum = csum ^ 0xFF
    return csum


def handler_incoming(packet):
	global stream_pub
	global dgram_pub
	global bootloader_pub
        global prev_time
	global r_arm_la_ctr
	global l_arm_la_ctr
	global torso_la_ctr
	global base_la_ctr
	global bad_usb_tx
	global pwrser

	#packet is a packet_485net_raw
	proto = struct.unpack("B", packet.data[2])[0] & 0b11000000
	
	actual_csum = do_checksum(packet.data[:-1])
	if actual_csum != struct.unpack("B", packet.data[-1])[0]:
		# rospy.loginfo("Packet with bad checksum: %s" % binascii.hexlify(packet.data))
		return
	
	if proto == 0x00:
		#datagram
		if len(packet.data) < 4:
			# rospy.loginfo("Impossibly short (but valid checksum) packet: %s" % binascii.hexlify(packet.data))
			return
		
		pkt = packet_485net_dgram()
		pkt.header		= packet.header
		pkt.source		= struct.unpack("B", packet.data[0])[0]
		pkt.destination	= struct.unpack("B", packet.data[1])[0]
		ports = struct.unpack("B", packet.data[2])[0]
		pkt.sport		= (ports & 0b00111000) >> 3
		pkt.dport		= (ports & 0b00000111)
		pkt.data		= packet.data[3:-1]
		pkt.checksum	= struct.unpack("B", packet.data[-1])[0]
		
		# rospy.loginfo("packet: %s" % binascii.hexlify(packet.source ))
                if valid_dgram_packet(pkt):
                  packet_dgram_handler(pkt)   
                else:
                  # publish latest joint states 5 times a second
		  dgram_pub.publish(pkt)
                cur_time = rospy.Time.now()
                if cur_time.to_sec() > prev_time.to_sec() + .2:
                  r_arm_la_ctr = r_arm_la_ctr + 1
                  l_arm_la_ctr = l_arm_la_ctr + 1
                  torso_la_ctr = torso_la_ctr + 1
                  base_la_ctr = base_la_ctr + 1
                  if r_arm_la_ctr > 5 or l_arm_la_ctr > 5 or torso_la_ctr > 5 or base_la_ctr > 5:
                    bad_usb_tx = 1
                    rospy.loginfo("bad_usb_tx set %d %d %d %d", r_arm_la_ctr, l_arm_la_ctr, torso_la_ctr, base_la_ctr)
                    pwrser.setRTS(False)
                    rospy.sleep(1)
                    pwrser.setRTS(True)
                    r_arm_la_ctr = 0
                    l_arm_la_ctr = 0
                    torso_la_ctr = 0
                    base_la_ctr = 0
                    bad_usb_tx = 0

                  publish_LA_states()
                  # print "pub LA"
                  prev_time = cur_time
	elif proto == 0x40:
		#stream
		if len(packet.data) < 7:
			# rospy.loginfo("Impossibly short (but valid checksum) packet: %s" % binascii.hexlify(packet.data))
			return
		
		pkt = packet_485net_stream()
		pkt.header		= packet.header
		pkt.source		= struct.unpack("B", packet.data[0])[0]
		pkt.destination	= struct.unpack("B", packet.data[1])[0]
		ports = struct.unpack("B", packet.data[2])[0]
		pkt.sport		= (ports & 0b00111000) >> 3
		pkt.dport		= (ports & 0b00000111)
		pkt.type		= struct.unpack("B", packet.data[3])[0]
		pkt.seq			= struct.unpack("<H", packet.data[4:6])[0]
		pkt.data		= packet.data[6:-1]
		pkt.checksum	= struct.unpack("B", packet.data[-1])[0]
		
		stream_pub.publish(pkt)
	elif proto == 0xC0:
		#bootloader
		if len(packet.data) < 4:
			# rospy.loginfo("Impossibly short (but valid checksum) packet: %s" % binascii.hexlify(packet.data))
			return
		
		pkt = packet_485net_bootloader()
		pkt.header		= packet.header
		pkt.source		= struct.unpack("B", packet.data[0])[0]
		pkt.destination	= struct.unpack("B", packet.data[1])[0]
		pkt.protocol	= struct.unpack("B", packet.data[2])[0]
		pkt.data		= packet.data[3:-1]
		pkt.checksum	= struct.unpack("B", packet.data[-1])[0]
		
		bootloader_pub.publish(pkt)
	# else:
		#reserved
		# rospy.loginfo("Packet with reserved protocol: %s" % binascii.hexlify(packet.data))

def handler_stream(packet):
	global outgoing_pub
	#packet is a packet_485net_stream
	pkt = packet_485net_raw()
	pkt.header = packet.header
	pkt.data = struct.pack("BBBB<H", packet.source, packet.destination, 0x40 | (packet.sport << 3) | packet.dport, packet.type, packet.seq) + packet.data
	checksum = do_checksum(pkt.data)
	pkt.data = pkt.data + struct.pack("B", checksum)
	
	outgoing_pub.publish(pkt)

def handler_dgram(packet):
	global outgoing_pub
	#packet is a packet_485net_dgram
	pkt = packet_485net_raw()
	pkt.header = packet.header
	pkt.data = struct.pack("BBB", packet.source, packet.destination, 0x00 | (packet.sport << 3) | packet.dport) + packet.data
	checksum = do_checksum(pkt.data)
	pkt.data = pkt.data + struct.pack("B", checksum)
	
	outgoing_pub.publish(pkt)

def handler_bootloader(packet):
	global outgoing_pub
	#packet is a packet_485net_bootloader
	pkt = packet_485net_raw()
	pkt.header = packet.header
	pkt.data = struct.pack("BBB", packet.source, packet.destination, 0xC0 | packet.protocol) + packet.data
	checksum = do_checksum(pkt.data)
	pkt.data = pkt.data + struct.pack("B", checksum)
	
	outgoing_pub.publish(pkt)

def decode_blob(blob):
    out = []
    lastgoodbyte = 0;
    #this is a really horrible hack
    blob = decode_blob.leftovers + blob
    #print binascii.hexlify(blob)
    i = 0
    while i < len(blob):
    #for i in range(0, len(blob)):
        #print binascii.hexlify(blob)
        #print i
        if blob[i] == "\x00":
            i = i + 1
            continue
            #0 can never be a source address
            #otherwise we always pick up runs of 00 as valid packets
        #1/21/12: hack to look large to small instead
        asdfrange = range(3, min(len(blob)-i, 32))
        asdfrange.reverse()
        for l in asdfrange:
            #the range needs to be at least 3 for normal packets
            #otherwise 10 f0 in the header registers as a valid packet
            #1/21/12: change this to 4 (so every packet has >=1 byte payload)
            #1/28/12: change this back to 3 (0xC0 doesn't have a 1 byte payload. looking large to small may have helped)
            #print "from %d length %d" % (i, l)
            #this assumes all packets do have a checksum
            possiblepkt = blob[i:i+l]
            #print binascii.hexlify(possiblepkt)
            possiblecsum = struct.unpack("B", blob[i+l])[0]
            checksum = do_checksum(possiblepkt)
            #print "csum is %d %d" % (possiblecsum, checksum)
            if checksum == possiblecsum:
                #print binascii.hexlify(possiblepkt)
                pkt = packet_485net_raw()
                pkt.header.stamp = rospy.Time.now()
                pkt.data = possiblepkt + blob[i+l]
                out.append(pkt)
                lastgoodbyte = i+l+1
                #even more hacky
                i = lastgoodbyte
                #print "jumping to %d" % i
                break
        i = i + 1
    decode_blob.leftovers = blob[lastgoodbyte:]
    return out
           
#hack
decode_blob.leftovers = ""

def tx_packet(packet):
   global bad_usb_tx

   if bad_usb_tx == 1:
     return
   #There is no good way to determine if the bus is free
   #We just transmit
   global ser
   delay150us = bytearray.fromhex(u'FF') * 19
   delay312us = bytearray.fromhex(u'FF') * 39
   #ser.setRTS(False)
   #ser.setDTR(False)
   #ser.write(packet.data)
   #ser.flush()
   ser.setRTS(False)
   ser.setDTR(False)
   ser.write(packet.data)
   ser.flush()
   ser.write(delay312us)
   ser.flush()
   ser.write(packet.data)
   ser.flush()
   ser.write(delay150us)
   ser.flush()
   #we really really need a fix for this
   #time.sleep(0.01)
   #change time from 0.2 to 0.01 and appear ok
   #send command twice in a row; the second time, no one should be on the bus
   #time.sleep(0.01)
   ser.setRTS(True)
   ser.setDTR(True)
#    #There is no good way to determine if the bus is free
#    #We just transmit
#    ser.setRTS(False)
#    ser.setDTR(False)
#    ser.write(packet.data)
#    ser.flush()
#    #we really really need a fix for this
#    time.sleep(0.01)
#    #change time from 0.2 to 0.01 and appear ok
#    #send command twice in a row; the second time, no one should be on the bus
#    ser.write(packet.data)
#    ser.flush()
#    time.sleep(0.01)
#    ser.setRTS(True)
#    ser.setDTR(True)


def main():
    global stream_pub
    global dgram_pub
    global bootloader_pub
    global outgoing_pub
    global prev_time
    global ser
    global pwrser
    global bad_usb_tx
    
    print "PC 485net interface (raw serial)"
    print "485net packet sorter"
    rospy.init_node('pc_485net_raw_node')
    # rospy.init_node('net_485net_packet_handler')
    
    jsm = JointStateMessages()
    serport = rospy.get_param("~serport", "/dev/magellan-i2c-serial")
    baud = rospy.get_param("~baud", 1000000)
    print "Using serial port %s at baud rate %d" % (serport, baud)
   
    rospy.loginfo("calling JointStatePR2Message")
    # self.msg = JointStatePR2Message()
    prev_time = rospy.Time.now()

    #ser = serial.Serial(serport, baud, timeout=0)
    ser = serial.Serial(serport, baud, timeout=1)
    ser.setRTS(True)
    ser.setDTR(True)

    pwrser = serial.Serial("/dev/pwrrst", 9600, timeout=1)
    pwrser.setRTS(True)
   
    # rospy.Subscriber("net_485net_incoming_packets", packet_485net_raw, handler_incoming)
    # pub = rospy.Publisher("net_485net_incoming_packets", packet_485net_raw)
    dgram_pub = rospy.Publisher("net_485net_incoming_dgram", packet_485net_dgram,queue_size=10)
    rospy.Subscriber("net_485net_outgoing_packets", packet_485net_raw, tx_packet)
    rospy.Subscriber("net_485net_outgoing_stream", packet_485net_stream, handler_stream)
    rospy.Subscriber("net_485net_outgoing_dgram", packet_485net_dgram, handler_dgram)
    rospy.Subscriber("net_485net_outgoing_bootloader", packet_485net_bootloader, handler_bootloader)
    stream_pub = rospy.Publisher("net_485net_incoming_stream", packet_485net_stream,queue_size=10)
    bootloader_pub = rospy.Publisher("net_485net_incoming_bootloader", packet_485net_bootloader,queue_size=10)
    outgoing_pub = rospy.Publisher("net_485net_outgoing_packets", packet_485net_raw,queue_size=10)
   
    # rospy.Subscriber("net_485net_set_torso_pos", Float64, set_torso_pos)
    # i = 0
    while not rospy.is_shutdown():

          bytes = ser.read(1)
          if bytes:
              bytes += ser.read(ser.inWaiting())
          else:
              raise Exception("timeout")
          # bytes = ser.read(500)
	  if bad_usb_tx == 1:
            # if haven't receive linact packets from any linact for > 1 sec
            # throw away next <50> bytes and continue again
            b_ctr = len(bytes) 
            while b_ctr < 50:
              # bytes = ser.read(500)
              bytes = ser.read(1)
              if bytes:
                  bytes += ser.read(ser.inWaiting())
              else:
                  raise Exception("timeout2")
              b_ctr = b_ctr + len(bytes) 
	    bad_usb_tx = 0
            r_arm_la_ctr = 0
            l_arm_la_ctr = 0
            torso_la_ctr = 0
            base_la_ctr = 0
            rospy.loginfo("bad_usb_tx clear")
            decode_blob.leftovers = ""
            # bytes = ser.read(500)
            bytes = ser.read(1)
            if bytes:
                bytes += ser.read(ser.inWaiting())
            else:
                raise Exception("timeout3")
          if len(bytes) > 0:
            packets = decode_blob(bytes)
            for p in packets:
              # i = i + 1
              # if i == 20:
                # print "packet"
                # i = 0
              handler_incoming(p)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass

