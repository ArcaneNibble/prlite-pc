#!/usr/bin/env python
#spins off a thread to listen for joint_states messages
#and provides the same information (or subsets of) as a service

import roslib
roslib.load_manifest('pr2lite_arm_navigation')
import rospy
#from pr2lite_arm_navigation.srv import *
from sensor_msgs.msg import JointState
import threading


#holds the latest states obtained from joint_states messages
class LatestJointStates:

    def __init__(self):
        rospy.init_node('joint_states_listener')
        self.lock = threading.Lock()
        # self.name = []
        # self.position = []
        # self.velocity = []
        # self.effort = []
        self.name = list()
        self.position = list()
        self.velocity = list()
        self.effort = list()
        self.thread = threading.Thread(target=self.joint_states_listener)
        self.thread.start()

        s = rospy.Service('return_joint_states', ReturnJointStates, self.return_joint_states)
        

    #thread function: listen for joint_states messages
    def joint_states_listener(self):
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        rospy.spin()


    #callback function: when a joint_states message arrives, save the values
    def joint_states_callback(self, msg):
        self.lock.acquire()
        # self.name = msg.name
        # self.position = msg.position
        # self.velocity = msg.velocity
        # self.effort = msg.effort
        msg_name = list(msg.name)
        for (i,joint_name) in enumerate(msg_name):
          if joint_name in self.name:
            index = self.name.index(joint_name)
            self.position[index] = msg.position[i]
            self.velocity[index] = msg.velocity[i]
            self.effort[index] = 0
          else:
            self.name.append(msg.name[i])
            self.position.append(msg.position[i])
            self.velocity.append(msg.velocity[i])
            self.effort.append(0)
        self.lock.release()


    #returns (found, position, velocity, effort) for the joint joint_name 
    #(found is 1 if found, 0 otherwise)
    def return_joint_state(self, joint_name):

        #rospy.loginfo("joint_name %s" % (joint_name))
        #no messages yet
        if self.name == []:
            rospy.logerr("No robot_state messages received!\n")
            return (0, 0., 0., 0.)

        #return info for this joint
        self.lock.acquire()
        if joint_name in self.name:
            index = self.name.index(joint_name)
            position = self.position[index]
            velocity = self.velocity[index]
            #effort = self.effort[index]
            effort = 0

        #unless it's not found
        else:
            rospy.logerr("Joint %s not found!"% (joint_name))
            self.lock.release()
            return (0, 0., 0., 0.)
        self.lock.release()
        return (1, position, velocity, effort)


    #server callback: returns arrays of position, velocity, and effort
    #for a list of joints specified by name
    def return_joint_states(self, req):
        joints_found = []
        positions = []
        velocities = []
        efforts = []
        for joint_name in req.name:
            (found, position, velocity, effort) = self.return_joint_state(joint_name)
            joints_found.append(found)
            positions.append(position)
            velocities.append(velocity)
            efforts.append(effort)
        return ReturnJointStatesResponse(joints_found, positions, velocities, efforts)


#run the server
if __name__ == "__main__":

    latestjointstates = LatestJointStates()

    print "joints_states_listener server started, waiting for queries"
    rospy.spin()
