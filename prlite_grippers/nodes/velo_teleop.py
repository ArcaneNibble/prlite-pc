#!/usr/bin/python
#
#  send tele-operation commands to pilot from an HKS racing controller
#
#   Copyright (C) 2011 Austin Robot Technology
#   License: Modified BSD Software License Agreement
#
# $Id$

PKG_NAME = 'prlite_grippers'

# standard Python packages
import sys
import math

import roslib 
roslib.load_manifest(PKG_NAME)
from std_msgs.msg import Float64
import rospy, time
from dynamixel_controllers.srv import TorqueEnable, SetSpeed, SetTorqueLimit

# In ROS Electric, the Joy message gained a header and moved to
# sensor_msgs.  Use that if it's available.
try:
    from sensor_msgs.msg import Joy
except ImportError:
    from joy.msg import Joy

def clamp(minimum, value, maximum):
    "constrain value to the range [minimum .. maximum]"
    return max(minimum, min(value, maximum))

class JoyNode():
    "Pr2Lite joystick tele-operation node."

    def __init__(self):
        "JoyNode constructor"

        # PRLite Joystick controls

#       # Modes -> Front buttons
#       self.right_arm_mode = 5   # front bottom right
#       self.left_arm_mode = 4    # front bottom left
#       self.body = 6             # front top right
#       self.head = 7             # front top left
#
#       # BUTTON mapping
#       self.gripper_open = 1     # "O"
#       self.gripper_close = 3    # "square"
#       self.wrist_rot_left = 0   # triangle
#       self.torso_up = 0         # triangle
#       self.wrist_rot_right = 2  # X
#       self.torso_down = 2       # X
#       self.tuck_arm = 9         # start
#       self.untuck_arm = 8       # select
#   
#       # digital Joystick Axes
#       self.shoulder_tilt = 5    # cross: 1 = up, -1 = down
#       self.shoulder_pan = 4     # cross: 1 = right -1 = left 
#
#       # analog joysticks Axes; mode dependent
#       self.elbow_tilt = 1       # left Joy: 1 = up, -1 = down
#       self.head_tilt = 1        # left Joy: 1 = up, -1 = down
#       self.elbow_pan = 0        # left Joy: 1 = right -1 = left 
#       self.head_pan = 0         # left Joy: 1 = right -1 = left 
#       self.speed = 0            # left Joy: 1 = right -1 = left 
#
#       self.wrist_tilt = 2       # right Joy: 1 = up, -1 = down
#       self.move_fwd_bck = 2     # right Joy: 1 = fwd, -1 = back
#       self.wrist_rot = 3        # right Joy: 1 = right -1 = left 
#       self.move_left_right = 3  # right Joy: 1 = right -1 = left 

        # Velo Testing
        # buttons
        self.speed_up = 1         # "0"
        self.speed_down = 3       # square
        self.torque_up = 0        # triangle
        self.torque_down = 2      # X

        # analog joysticks axes
        self.position = 2         # right joy (up / down)
        self.controller = 'velo_controller'

        # initialize ROS topics 
        rospy.init_node('velo_teleop')

        self.controller = 'velo_controller'

        self.torque = 500
        if self.torque > 1023:
           self.torque = 1023
        if self.torque < 0:
           self.torque = 0
        #torque_service = '/' + self.controller + '/torque_enable'
        torque_service = '/' + self.controller + '/set_torque_limit'
        # torque_service = '/' + self.controller + '/set_compliance_punch' # min torque
        # torque_service = '/' + self.controller + '/set_compliance_limit' # max torque
        # limit, punch 0-1023
        rospy.wait_for_service(torque_service)
        #self.my_torque_service = rospy.ServiceProxy

(torque_service, TorqueEnable)
        self.my_torque_service = rospy.ServiceProxy

(torque_service, SetTorqueLimit)
        self.speed = 0.5
        speed_service = '/' + self.controller + '/set_speed'
        rospy.wait_for_service(speed_service)
        # speed_services.append(rospy.ServiceProxy

(speed_service, SetSpeed))
        self.my_speed_service = rospy.ServiceProxy

(speed_service, SetSpeed)
        # position in radians
        pub_topic = '/' + self.controller + '/command'
        self.pub = rospy.Publisher(pub_topic, Float64)



    def joyCallback(self, joy):
        "invoked every time a joystick message arrives"
        rospy.logdebug('joystick input:\n' + str(joy))

        # handle Velo Test buttons
        if joy.buttons[self.speed_up]:
            self.speed += .01
            rospy.logwarn('speed ' + str(self.speed))
	    self.my_speed_service(self.speed)
        elif joy.buttons[self.speed_down]:
            self.speed -= .01
            rospy.logwarn('speed ' + str(self.speed))
	    self.my_speed_service(self.speed)
        if joy.buttons[self.torque_up]:
            self.torque += 25
            if self.torque > 1023:
              self.torque = 1023
            #print "torque: " + str(self.torque)
            rospy.logwarn('torque ' + str(self.torque))
            #rospy.logwarn('torque ')
            #rospy.logwarn(self.torque)
	    self.my_torque_service(self.torque)
        elif joy.buttons[self.torque_down]:
            self.torque -= 25
            if self.torque < 0:
              self.torque = 0
            #print "torque: " + str(self.torque)
            rospy.logwarn('torque ' + str(self.torque))
            #rospy.logwarn('torque ' + str(self.torque))
	    self.my_torque_service(self.torque)

        # AX12 Position 0 (min) to 300 degrees (max) = 5.236 radians
        # -2.618 radians to 2.618
        pos =  2.6 * (joy.axes[self.position]) 
        pos =  1 * (joy.axes[self.position]) - .9
        rospy.logwarn('joystick pos: ' + str(pos))
        self.pub.publish(pos)

joynode = None

def main():
    global joynode
    joynode = JoyNode()
    rospy.loginfo('joystick vehicle controller starting')
    try:
        rospy.spin()
    except rospy.ROSInterruptException: pass
    rospy.loginfo('joystick vehicle controller finished')

if __name__ == '__main__':
    # run main function and exit
    sys.exit(main())

