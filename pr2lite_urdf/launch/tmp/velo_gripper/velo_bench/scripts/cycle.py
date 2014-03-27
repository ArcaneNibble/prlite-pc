#! /usr/bin/python
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# This script cycles the position of the Matei Gripper
#
# Author: Bob Holmberg
#
#
# NOTE: Cycle controller uses simple, standard PR2 transmissions and controllers.
#       Thus, this program can be used to exercise (burn-in) the actuator in
#       a gripper-agnostic way.  We do not need to get a gripper transmission
#       working in order to use this cycle controller.
#
#

CONTROLLER_NAME = "cycle_controller"

import sys,os,signal
import random
import re
import pexpect

import roslib
roslib.load_manifest('robot_mechanism_controllers')
import rospy

from subprocess import Popen,PIPE,STDOUT
from optparse import OptionParser

import std_msgs.msg             as msg_std
import pr2_controllers_msgs.msg as msg_ctrlr

# GLOBAL BECAUSE THIS NEEDS TO EXIST OR roscore IS KILLED.
p_launch = None

def rosInfra():
    global p_launch
    p_ros = Popen(['rostopic','list'],stderr=STDOUT,stdout=PIPE)
    rospy.sleep(2)
    (pout,perr) = p_ros.communicate()
    if re.search('/cycle_controller/command\n',pout):
       return

    # DIDN'T FIND THE RIGHT SERVICES, SO LAUNCH cycle_controller
    p_launch = pexpect.spawn('roslaunch velo_bench cycle.launch')
    msg = 'Started controllers: cycle_controller'
    ans = 2
    while ans == 2:
        ans = p_launch.expect([msg,'\[FATAL\].*','[\r\n]+',
                               pexpect.TIMEOUT], timeout=30)
        if   ans == 0: print(p_launch.before + p_launch.after + '\n')
        elif ans == 2: print(p_launch.before)
        elif ans == 1:
            mo = re.search(' pid: ([0-9]+)',p_launch.after)
            if mo:
                os.kill( int(mo.groups()[0]), signal.SIGKILL )
            print("\nRestarting ROS...\n\n")
            rosInfra()
        else:
            buf = p_launch.before
            if isinstance(p_launch.after,str): buf += p_launch.after
            rospy.sleep(4)
            try:
                buf += p_launch.read_nonblocking(timeout=0)
            except pexpect.ExceptionPexpect:
                pass
            print(buf)
            p_launch.kill(9)
            rospy.sleep(2)
            print("\nTIMEOUT occurred, check your setup and try again.\n")
            sys.exit(0)

G_pv = 0

def cb_pv(msg):
    global G_pv
    if   G_pv == 0: return
    elif G_pv == 1: print("pv = %5.1f mm"%msg.process_value*1000)
    elif G_pv == 2: pass
    elif G_pv == 3:
        print(" pv = %6.1f mm, e = %6.1f mm"%
              (msg.process_value*1000,msg.error*1000))
    G_pv = 0


def main():

    global G_pv

    # SOME BASIC CONSTANTS 
    gearSign  = 1    # Use to set calibration HOME direction
    depth_min = 0.0010
    depth_max = 0.0145
    # Absolute magnitude of stopping point randomization 
    # so we don't stop on exactly the same tooth each time.
    # 0.00325/15 = 0.0002167 = 1 motor rev (with 15 & 3.25mm pitch)
    epsilon_mag = 0.0002167/2

    # PARSE INPUT!
    # THEY MIGHT BE ASKING FOR -h HELP, SO DON'T START ROS YET.
    parser = OptionParser()
    parser.add_option("-n", dest="num_cycles", metavar="num_cycles", 
                      type="int", default=1000,
                      help="Number of cycles" )
    parser.add_option("-a", dest="depth_min", metavar="min_depth",
                      type="float", default=depth_min,
                      help="Min depth for cycles. (default is %4.1f mm)"%(1000*depth_min) )
    parser.add_option("-d", dest="depth_max", metavar="max_depth",
                      type="float", default=depth_max,
                      help="Max depth for cycles. (default is %4.1f mm)"%(1000*depth_max) )
    parser.add_option("-t", dest="cycletime", metavar="cycletime",
                      type="float", default=6.0,
                      help="Time for one cycle" )
    parser.add_option("-v", dest="verbose",
                      action="store_true", default=False,
                      help="Flag for verbose output" )
    (options, args) = parser.parse_args()

    # STANDARDIZE INPUT sign AND NORMALIZE TO SI UNITS
    depth_min = max( abs(options.depth_min), 2*epsilon_mag )
    if depth_min > 0.020:  depth_min /= 1000.0
    depth_max = abs(options.depth_max)
    if depth_max > 0.020:  depth_max /= 1000.0

    # CHECK FOR roscore AND cycle_controller
    rosInfra()

    # NOW SETUP ROS NODE
    joint = "gripper_joint"
    rospy.init_node('cycle', anonymous=True)
    pub = rospy.Publisher("%s/command" % CONTROLLER_NAME, msg_std.Float64)
    sub_pv = rospy.Subscriber('cycle_controller/state',
                              msg_ctrlr.JointControllerState, cb_pv)
    

    # ECHO OPERATING PARAMETERS AFTER STARTING NODE
    print("")
    print "num_cycles = %d" % options.num_cycles
    print "min_depth  = %4.1f mm" % (1000*depth_min)
    print "max_depth  = %4.1f mm" % (1000*depth_max)
    print "cycletime  = %4.1f sec" % options.cycletime

    goal = depth_min
    for n in range(2*options.num_cycles):
        if options.verbose: G_pv = 3
        else:               G_pv = 2
        rospy.sleep(0.010) # ALLOW Callback TO PRINT
        epsilon = epsilon_mag * (2*random.random()-1)
        pubGoal = gearSign*abs(goal)
        pub.publish(msg_std.Float64(pubGoal))
        if goal != depth_min:
            goal = depth_min
            sys.stdout.write("\rCycle # %4d   goal= %7.4lf mm " % 
                             (n/2+1,abs(pubGoal)))
            sys.stdout.flush()
        else:
            goal = depth_max
            sys.stdout.write("\rCycle # %4d   goal= %7.4lf mm " % 
                             (n/2+1,abs(pubGoal)))
            sys.stdout.flush()
        rospy.sleep(options.cycletime/2.0)

        if rospy.is_shutdown():
            break

    pub.publish(msg_std.Float64(gearSign*abs(depth_min)))
    print("\n")

if __name__ == '__main__':
    main()
