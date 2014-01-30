#! /usr/bin/python
import roslib
roslib.load_manifest('pr2lite_arm_navigation')
import rospy
import actionlib
from std_msgs.msg import Float64
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *

rospy.init_node('move_the_head', anonymous=True)
head_pan_pub = rospy.Publisher('/head_pan_controller/command', Float64)
head_tilt_pub = rospy.Publisher('/head_tilt_controller/command', Float64)

rospy.sleep(2)

head_pan_pub.publish(0.0)
head_tilt_pub.publish(1.175)
rospy.sleep(2)
