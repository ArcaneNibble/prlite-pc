#!/usr/bin/env python

PKG = "pr2_mechanism_controllers"

import roslib; roslib.load_manifest(PKG) 

import sys
import os
import string

import rospy
from std_msgs import *

from pr2_msgs.msg import PeriodicCmd
from pr2_msgs.srv import *
from time import sleep

def print_usage(exit_code = 0):
    print '''Usage:
    send_periodic_cmd.py [controller] [profile] [period] [amplitude] [offset]
       - [profile]   - Possible options are linear, linear_blended, sine
       - [period]    - Time for one entire cycle to execute (in seconds)
       - [amplitude] - Distance max value to min value of profile (In radians for laser_tilt controller)
       - [offset]    - Constant cmd to add to profile (offset=0 results in profile centered around 0)
'''
    sys.exit(exit_code)

if __name__ == '__main__':
    argv = rospy.myargv()
    if len(argv) < 6:
        print_usage()

    cmd = PeriodicCmd()
    controller =    argv[1]
    cmd.header =    roslib.msg.Header(None, None, None)
    cmd.profile =   argv[2] 
    cmd.period =    float (argv[3])
    cmd.amplitude = float (argv[4])
    cmd.offset =    float (argv[5])

    print 'Sending Command to %s: ' % controller
    print '  Profile Type: %s' % cmd.profile
    print '  Period:       %f Seconds' % cmd.period
    print '  Amplitude:    %f Radians' % cmd.amplitude
    print '  Offset:       %f Radians' % cmd.offset

    rospy.wait_for_service(controller + '/set_periodic_cmd')                                        
    s = rospy.ServiceProxy(controller + '/set_periodic_cmd', SetPeriodicCmd)
    resp = s.call(SetPeriodicCmdRequest(cmd))        

    #rospy.init_node('periodic_cmd_commander', anonymous=True)
    #sleep(1)
    #command_publisher.publish( cmd )

    #sleep(1)

    print 'Command sent!'
    print '  Resposne: %f' % resp.start_time.to_seconds()
