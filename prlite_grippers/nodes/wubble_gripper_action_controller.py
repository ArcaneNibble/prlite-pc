#! /usr/bin/env python

# Copyright (c) 2010, Antons Rebguns
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
# Author: Antons Rebguns
#

import math
from threading import Thread

import roslib; roslib.load_manifest('wubble2_robot')

import rospy
import actionlib
import tf

from tf import TransformListener
from tf import LookupException
from tf import ConnectivityException

from std_msgs.msg import Float64
from phidgets_ros.msg import Float64Stamped
from wubble2_robot.msg import WubbleGripperAction
from wubble2_robot.msg import WubbleGripperGoal


def within_tolerance(a, b, tolerance):
    return abs(a - b) < tolerance


class GripperActionController():
    def __init__(self, controller_namespace, controllers):
        self.controller_namespace = controller_namespace
        self.controllers = {}
        
        for c in controllers:
            self.controllers[c.controller_namespace] = c


    def initialize(self):
        l_controller_name = rospy.get_param('~left_controller_name', 'left_finger_controller')
        r_controller_name = rospy.get_param('~right_controller_name', 'right_finger_controller')
        
        if l_controller_name not in self.controllers:
            rospy.logerr('left_controller_name not found, requested: [%s], available: %s' % (l_controller_name, self.controllers))
            return False
            
        if r_controller_name not in self.controllers:
            rospy.logerr('right_controller_name not found, requested: [%s], available: %s' % (r_controller_name, self.controllers))
            return False
            
        self.l_finger_controller = self.controllers[l_controller_name]
        self.r_finger_controller = self.controllers[r_controller_name]
        
        if self.l_finger_controller.port_namespace != self.r_finger_controller.port_namespace:
            rospy.logerr('%s and %s are connected to different serial ports, this is unsupported' % (l_controller_name, r_controller_name))
            return False
            
        # left and right finger are assumed to be connected to the same port
        self.dxl_io = self.l_finger_controller.dxl_io
        
        self.l_finger_state = self.l_finger_controller.joint_state
        self.r_finger_state = self.r_finger_controller.joint_state
        
        self.l_max_speed = self.l_finger_controller.joint_max_speed
        self.r_max_speed = self.r_finger_controller.joint_max_speed
        
        return True


    def start(self):
        self.tf_listener = TransformListener()
        
        # Publishers
        self.gripper_opening_pub = rospy.Publisher('gripper_opening', Float64)
        self.l_finger_ground_distance_pub = rospy.Publisher('left_finger_ground_distance', Float64)
        self.r_finger_ground_distance_pub = rospy.Publisher('right_finger_ground_distance', Float64)
        
        # Pressure sensors
        # 0-3 - left finger
        # 4-7 - right finger
        num_sensors = 8
        self.pressure = [0.0] * num_sensors
        [rospy.Subscriber('/interface_kit/124427/sensor/%d' % i, Float64Stamped, self.process_pressure_sensors, i) for i in range(num_sensors)]
        
        # pressure sensors are at these values when no external pressure is applied
        self.l_zero_pressure = [0.0, 0.0, 0.0, 0.0]
        self.r_zero_pressure = [0.0, 0.0, 130.0, 0.0]
        self.lr_zero_pressure = self.l_zero_pressure + self.r_zero_pressure
        
        self.l_total_pressure_pub = rospy.Publisher('left_finger_pressure', Float64)
        self.r_total_pressure_pub = rospy.Publisher('right_finger_pressure', Float64)
        self.lr_total_pressure_pub = rospy.Publisher('total_pressure', Float64)
        
        self.close_gripper = False
        self.dynamic_torque_control = False
        self.lower_pressure = 800.0
        self.upper_pressure = 1000.0
        
        # IR sensor
        self.gripper_ir_pub = rospy.Publisher('gripper_distance_sensor', Float64)
        self.ir_distance = 0.0
        rospy.Subscriber('/interface_kit/106950/sensor/7', Float64Stamped, self.process_ir_sensor)
        
        # Temperature monitor and torque control thread
        Thread(target=self.gripper_monitor).start()
        
        # Start gripper opening monitoring thread
        Thread(target=self.calculate_gripper_opening).start()
        
        self.action_server = actionlib.SimpleActionServer('wubble_gripper_action',
                                                          WubbleGripperAction,
                                                          execute_cb=self.process_gripper_action,
                                                          auto_start=False)
        self.action_server.start()
        rospy.loginfo('gripper_freespin_controller: ready to accept goals')


    def stop(self):
        pass


    def __send_motor_command(self, l_desired_torque, r_desired_torque):
        l_motor_id = self.l_finger_controller.motor_id
        r_motor_id = self.r_finger_controller.motor_id
        
        l_trq = self.l_finger_controller.spd_rad_to_raw(l_desired_torque)
        r_trq = self.r_finger_controller.spd_rad_to_raw(r_desired_torque)
        
        self.dxl_io.set_multi_speed( ( (l_motor_id,l_trq), (r_motor_id,r_trq) ) )


    def activate_gripper(self, command, torque_limit):
        if command == WubbleGripperGoal.CLOSE_GRIPPER:
            l_desired_torque = -torque_limit * self.l_max_speed
            r_desired_torque =  torque_limit * self.r_max_speed
        else: # assume opening
            l_desired_torque =  torque_limit * self.l_max_speed
            r_desired_torque = -torque_limit * self.r_max_speed
            
        self.__send_motor_command(l_desired_torque, r_desired_torque)
        
        return max(l_desired_torque, r_desired_torque)


    def gripper_monitor(self):
        rospy.loginfo('Gripper temperature monitor and torque control thread started successfully')
        motors_overheating = False
        max_pressure = 8000.0
        r = rospy.Rate(150)
        
        while not rospy.is_shutdown():
            l_total_pressure = max(0.0, sum(self.pressure[:4]) - sum(self.l_zero_pressure))
            r_total_pressure = max(0.0, sum(self.pressure[4:]) - sum(self.r_zero_pressure))
            pressure = l_total_pressure + r_total_pressure
            
            self.l_total_pressure_pub.publish(l_total_pressure)
            self.r_total_pressure_pub.publish(r_total_pressure)
            self.lr_total_pressure_pub.publish(pressure)
            
            #----------------------- TEMPERATURE MONITOR ---------------------------#
            l_temp = max(self.l_finger_state.motor_temps)
            r_temp = max(self.r_finger_state.motor_temps)
            
            if l_temp >= 75 or r_temp >= 75:
                if not motors_overheating:
                    rospy.logwarn('Disabling gripper motors torque [LM: %dC, RM: %dC]' % (l_temp, r_temp))
                    self.__send_motor_command(-0.5, 0.5)
                    motors_overheating = True
            else:
                motors_overheating = False
                
            #########################################################################
            
            # don't do torque control if not requested or
            # when the gripper is open
            # or when motors are too hot
            if not self.dynamic_torque_control or \
               not self.close_gripper or \
               motors_overheating:
                r.sleep()
                continue
                
            #----------------------- TORQUE CONTROL -------------------------------#
            l_current = self.l_finger_state.goal_pos
            r_current = self.r_finger_state.goal_pos
            
            if pressure > self.upper_pressure:   # release
                pressure_change_step = abs(pressure - self.upper_pressure) / max_pressure
                l_goal = min( self.l_max_speed, l_current + pressure_change_step)
                r_goal = max(-self.r_max_speed, r_current - pressure_change_step)
                
                if l_goal < self.l_max_speed or r_goal > -self.r_max_speed:
                    if self.close_gripper:
                        self.__send_motor_command(l_goal, r_goal)
                        rospy.logdebug('>MAX pressure is %.2f, LT: %.2f, RT: %.2f, step is %.2f' % (pressure, l_current, r_current, pressure_change_step))
            elif pressure < self.lower_pressure: # squeeze
                pressure_change_step = abs(pressure - self.lower_pressure) / max_pressure
                l_goal = max(-self.l_max_speed, l_current - pressure_change_step)
                r_goal = min( self.r_max_speed, r_current + pressure_change_step)
                
                if l_goal > -self.l_max_speed or r_goal < self.r_max_speed:
                    if self.close_gripper:
                        self.__send_motor_command(l_goal, r_goal)
                        rospy.logdebug('<MIN pressure is %.2f, LT: %.2f, RT: %.2f, step is %.2f' % (pressure, l_current, r_current, pressure_change_step))
            ########################################################################
            
            r.sleep()


    def calculate_gripper_opening(self):
        timeout = rospy.Duration(5)
        last_reported = rospy.Time(0)
        r = rospy.Rate(50)
        
        while not rospy.is_shutdown():
            try:
                map_frame_id = 'base_footprint'
                palm_frame_id = 'L7_wrist_roll_link'
                l_fingertip_frame_id = 'left_fingertip_link'
                r_fingertip_frame_id = 'right_fingertip_link'
                
                self.tf_listener.waitForTransform(palm_frame_id, l_fingertip_frame_id, rospy.Time(0), rospy.Duration(0.2))
                self.tf_listener.waitForTransform(palm_frame_id, r_fingertip_frame_id, rospy.Time(0), rospy.Duration(0.2))
                
                l_pos, _ = self.tf_listener.lookupTransform(palm_frame_id, l_fingertip_frame_id, rospy.Time(0))
                r_pos, _ = self.tf_listener.lookupTransform(palm_frame_id, r_fingertip_frame_id, rospy.Time(0))
                
                dx = l_pos[0] - r_pos[0]
                dy = l_pos[1] - r_pos[1]
                dz = l_pos[2] - r_pos[2]
                
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                self.gripper_opening_pub.publish(dist)
                
                l_pos, _ = self.tf_listener.lookupTransform(map_frame_id, l_fingertip_frame_id, rospy.Time(0))
                r_pos, _ = self.tf_listener.lookupTransform(map_frame_id, r_fingertip_frame_id, rospy.Time(0))
                
                self.l_finger_ground_distance_pub.publish(l_pos[2])
                self.r_finger_ground_distance_pub.publish(r_pos[2])
            except LookupException as le:
                rospy.logerr(le)
            except ConnectivityException as ce:
                rospy.logerr(ce)
            except tf.Exception as e:
                if rospy.Time.now() - last_reported > timeout:
                    rospy.logerr('TF exception: %s' % str(e))
                    last_reported = rospy.Time.now()
                    
            r.sleep()


    def process_pressure_sensors(self, msg, i):
        self.pressure[i] = msg.data


    def process_ir_sensor(self, msg):
        """
        Given a raw sensor value from Sharp IR sensor (4-30cm model)
        returns an actual distance in meters.
        """
        if msg.data < 80: self.ir_distance = 1.0           # too far
        elif msg.data > 530: self.ir_distance = 0.0        # too close
        else: self.ir_distance =  20.76 / (msg.data - 11)  # just right
        
        self.gripper_ir_pub.publish(self.ir_distance)


    def process_gripper_action(self, req):
        r = rospy.Rate(500)
        timeout = rospy.Duration(2.0)
        
        if req.command == WubbleGripperGoal.CLOSE_GRIPPER:
            self.dynamic_torque_control = req.dynamic_torque_control
            self.lower_pressure = req.pressure_lower
            self.upper_pressure = req.pressure_upper
            
            desired_torque = self.activate_gripper(req.command, req.torque_limit)
            
            if not self.dynamic_torque_control:
                start_time = rospy.Time.now()
                while not rospy.is_shutdown() and \
                      rospy.Time.now() - start_time < timeout and \
                      (not within_tolerance(self.l_finger_state.load, -req.torque_limit, 0.01) or
                       not within_tolerance(self.r_finger_state.load, -req.torque_limit, 0.01)):
                    r.sleep()
            else:
                if desired_torque > 1e-3: rospy.sleep(1.0 / desired_torque)
                
            self.close_gripper = True
            self.action_server.set_succeeded()
        elif req.command == WubbleGripperGoal.OPEN_GRIPPER:
            self.close_gripper = False
            self.activate_gripper(req.command, req.torque_limit)
            
            start_time = rospy.Time.now()
            while not rospy.is_shutdown() and \
                  rospy.Time.now() - start_time < timeout and \
                  (self.l_finger_state.current_pos < 0.8 or
                   self.r_finger_state.current_pos > -0.8):
                r.sleep()
                
            self.__send_motor_command(0.5, -0.5)
            self.action_server.set_succeeded()
        else:
            msg = 'Unrecognized command: %d' % req.command
            rospy.logerr(msg)
            self.action_server.set_aborted(text=msg)


if __name__ == '__main__':
    try:
        gac = GripperActionController()
        rospy.spin()
    except rospy.ROSInterruptException: pass

