#! /usr/bin/env python

# Copyright (c) 2010, Arizona Robotics Research Group, University of Arizona
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
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
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
#
# Authors: Antons Rebguns
#

import roslib; roslib.load_manifest('wubble_blocks')
import rospy

from math import sqrt
from math import pow

from wubble_blocks.srv import ClassifyObject
from object_tracking.srv import GetTracksInterval

class ObjectClassifier:
    def __init__(self):
        self.classify_obj_server = rospy.Service('classify_object', ClassifyObject, self.classify)
        self.get_tracks_srv = rospy.ServiceProxy('get_tracks_interval', GetTracksInterval)
        rospy.wait_for_service('get_tracks_interval')
        self.balls = 26
        self.blocks = 12

    def classify(self, req):
        resp = self.get_tracks_srv(req.id, req.swat_time, req.swat_time + rospy.Duration(0.5))
        #self.dump_track_to_file(resp.track)
        return self.categorize_track(resp.track)

    def categorize_track(self, track):
        start_x = track.waypoints[0].pixel.x
        start_y = track.waypoints[0].pixel.y
        end_x = track.waypoints[-1].pixel.x
        end_y = track.waypoints[-1].pixel.y
        dist = sqrt(pow(end_x - start_x, 2.0) + pow(end_y - start_y, 2.0))

        rospy.loginfo("Object [%d] has %d waypoints in its track (%d.%d to %d.%d)",
                      track.id, len(track.waypoints),
                      track.waypoints[0].header.stamp.secs, track.waypoints[0].header.stamp.nsecs,
                      track.waypoints[-1].header.stamp.secs, track.waypoints[-1].header.stamp.nsecs)
        rospy.loginfo('Distance traveled in 0.5 seconds is %f', dist)

        if abs(self.balls - dist) < abs(self.blocks - dist):
            return "Ball"
        else:
            return "Block"

    def dump_track_to_file(self, track):
        # time_string = track.waypoints[0].header.stamp.secs + '.' + track.waypoints[0].header.stamp.nsecs
        fname = 'object_%d.tracks' % track.id
        file = open(fname, 'a+')

        for p in track.waypoints:
            l = '%d,%d.%d,%d,%d\n' % (track.id, p.header.stamp.secs, p.header.stamp.nsecs, p.pixel.x, p.pixel.y)
            file.write(l)

        file.write('\n\n')
        file.flush()
        file.close()

if __name__ == '__main__':
    try:
        rospy.init_node('object_classifier', anonymous=True)
        swat = ObjectClassifier()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
