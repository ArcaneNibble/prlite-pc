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

# Apparently the PR2 calibration controllers publish an <Empty> message to the
# topic "calibrated", BUT spawner is listening for a <Bool> on the topic
# specified by its --wait=SomeTopic flag.  Thus, spawner cannot listen directly
# to a calibration controller.  Here is some glue that will listen to a
# calibration controller and republish its completion message in a form that
# spawner can understand.
#
# Author: Bob Holmberg


import sys,time
import signal
import roslib
roslib.load_manifest('robot_mechanism_controllers')
import rospy
from std_msgs.msg import *

from optparse import OptionParser

inTopic_result = None

def main():
    print("Starting %s"%__file__)

    inTopic_default = "cal_l_gripper/calibrated"
    outTopic_default= "glueEmpty2Bool/calibratedBool"

    parser=OptionParser()
    parser.add_option("-i", dest="inTopic", metavar="inTopic", 
        type="str", default=inTopic_default,
        help="Topic to listen for <Empty> on. Default is %s."%inTopic_default )
    parser.add_option("-o", dest="outTopic", metavar="outTopic", 
        type="str", default=outTopic_default,
        help="Topic to publish <Bool> to.  Default is %s."%outTopic_default )
    (options, args) = parser.parse_args()


    pub = rospy.Publisher("%s" % options.outTopic, Bool)


    global inTopic_result  # Python scoping sucks

    def wait_for_topic_cb(msg):
        global inTopic_result  # Python scoping really really sucks
        inTopic_result = True
    rospy.Subscriber(options.inTopic, Empty, wait_for_topic_cb)
    stale_time = time.time() + 10.0

    rospy.init_node('glueEmpty2Bool')

    warned_about_not_hearing_anything = False
    while not inTopic_result:
        time.sleep(0.05)
        if rospy.is_shutdown():
            return
        if not warned_about_not_hearing_anything and time.time() > stale_time:
            warned_about_not_hearing_anything = True
            print("%s has't heard anything from its inTopic (%s)" % 
                  (__file__,options.inTopic) )

    print("Heard from %s. Begin publishing to %s"% (options.inTopic,options.outTopic))
    while True:
        pub.publish(Bool(True))
        if rospy.is_shutdown():
            return
        time.sleep(0.5)


if __name__ == '__main__':
    main()
