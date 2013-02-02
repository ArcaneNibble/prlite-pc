#!/usr/bin/env python

import roslib; roslib.load_manifest("turtlebot_arm_bringup")
import rospy
from sensor_msgs.msg import JointState

rospy.init_node("fake_pub")
p = rospy.Publisher('joint_states', JointState)

msg = JointState()
#msg.name = ["front_castor_joint", "left_wheel_joint", "rear_castor_joint", "right_wheel_joint"]
#msg.name = [ "torso_lift_joint", "wheel_linear_actuator_joint", "fl_anchor_rod_joint", "fl_caster_rotation_joint", "fl_wheel_joint", "base_link_fl_wheel_joint", "fr_base_joint", "fr_anchor_rod_joint", "fr_caster_rotation_joint", "fr_caster_motor_joint", "fr_wheel_joint", "base_link_fr_wheel_joint", "bl_anchor_rod_joint", "bl_caster_rotation_joint", "bl_caster_motor_joint", "bl_wheel_joint", "base_link_bl_wheel_joint", "br_anchor_rod_joint", "br_caster_rotation_joint", "base_link_br_wheel_joint", "left_lin_act_cyl_joint", "left_linear_actuator_joint", "left_shoulder_tilt_joint", "left_upper_arm_hinge_joint", "left_shoulder_tilt_joint", "left_shoulder_pan_joint", "right_lin_act_cyl_joint", "right_linear_actuator_joint", "right_shoulder_tilt_joint", "right_upper_arm_hinge_joint"]
#msg.name = [ "bl_anchor_rod_joint", "bl_caster_rotation_joint", "bl_wheel_joint", "br_anchor_rod_joint", "br_caster_rotation_joint", "br_wheel_joint", "fl_anchor_rod_joint", "fl_caster_rotation_joint", "fl_wheel_joint", "floating_rot_w", "floating_rot_x", "floating_rot_y", "floating_rot_z", "floating_trans_x", "floating_trans_y", "floating_trans_z", "fr_anchor_rod_joint", "fr_caster_rotation_joint", "fr_wheel_joint", "head_pan_joint", "head_tilt_joint", "left_elbow_flex_joint", "left_elbow_joint", "left_elbow_pan_joint", "left_gripper_left_joint", "left_gripper_right_joint", "left_lin_act_cyl_joint", "left_linear_actuator_joint", "left_shoulder_pan_joint", "left_shoulder_tilt_joint", "left_upper_arm_hinge_joint", "left_wrist_flex_joint", "left_wrist_roll_joint", "lidar_tilt_joint", "right_elbow_flex_joint", "right_elbow_joint", "right_elbow_pan_joint", "right_gripper_left_joint", "right_gripper_right_joint", "right_lin_act_cyl_joint", "right_linear_actuator_joint", "right_shoulder_pan_joint", "right_shoulder_tilt_joint", "right_upper_arm_hinge_joint", "right_wrist_flex_joint", "right_wrist_roll_joint", "torso_lift_joint", "wheel_linear_actuator_joint", "bl_anchor_rod_joint", "bl_caster_rotation_joint", "bl_wheel_joint", "br_anchor_rod_joint", "br_caster_rotation_joint", "br_wheel_joint", "fl_anchor_rod_joint", "fl_caster_rotation_joint", "fl_wheel_joint" ]
#msg.name = [ "fl_anchor_rod_joint", "fl_caster_rotation_joint", "fl_wheel_joint", "base_link_fl_wheel_joint", "fr_base_joint", "fr_anchor_rod_joint", "fr_caster_rotation_joint", "fr_caster_motor_joint", "fr_wheel_joint", "base_link_fr_wheel_joint", "bl_anchor_rod_joint", "bl_caster_rotation_joint", "bl_caster_motor_joint", "bl_wheel_joint", "base_link_bl_wheel_joint", "br_anchor_rod_joint", "br_caster_rotation_joint", "base_link_br_wheel_joint", "left_lin_act_cyl_joint", "left_upper_arm_hinge_joint", "right_lin_act_cyl_joint", "right_upper_arm_hinge_joint", "left_linear_actuator_joint", "right_linear_actuator_joint", "right_elbow_pan_joint", "left_elbow_pan_joint", "br_wheel_joint", "floating_rot_w", "floating_rot_x", "floating_rot_y", "floating_rot_z", "floating_trans_x", "floating_trans_y", "floating_trans_z", "torso_lift_joint", "wheel_linear_actuator_joint" ]

#msg.name = [ "fl_anchor_rod_joint", "fl_caster_rotation_joint", "fl_wheel_joint", "base_link_fl_wheel_joint", "fr_base_joint", "fr_anchor_rod_joint", "fr_caster_rotation_joint", "fr_caster_motor_joint", "fr_wheel_joint", "base_link_fr_wheel_joint", "bl_anchor_rod_joint", "bl_caster_rotation_joint", "bl_caster_motor_joint", "bl_wheel_joint", "base_link_bl_wheel_joint", "br_anchor_rod_joint", "br_caster_rotation_joint", "base_link_br_wheel_joint", "left_lin_act_cyl_joint", "left_upper_arm_hinge_joint", "right_lin_act_cyl_joint", "right_upper_arm_hinge_joint", "left_linear_actuator_joint", "right_linear_actuator_joint", "right_elbow_pan_joint", "left_elbow_pan_joint", "br_wheel_joint", "torso_lift_joint", "wheel_linear_actuator_joint" ]

#msg.name = [ "fl_wheel_joint", "fr_wheel_joint", "bl_wheel_joint", "br_wheel_joint" ] 
#msg.name = [ "fl_wheel_joint", "fr_wheel_joint", "bl_wheel_joint", "br_wheel_joint" , "bl_anchor_rod_joint", "bl_caster_rotation_joint", "br_anchor_rod_joint", "br_caster_rotation_joint", "fl_anchor_rod_joint", "fl_caster_rotation_joint", "fr_anchor_rod_joint", "fr_caster_rotation_joint", "torso_lift_joint", "wheel_linear_actuator_joint"] 
msg.name = [ "fl_wheel_joint", "fr_wheel_joint", "bl_wheel_joint", "br_wheel_joint" , "bl_anchor_rod_joint", "bl_caster_rotation_joint", "br_anchor_rod_joint", "br_caster_rotation_joint", "fl_anchor_rod_joint", "fl_caster_rotation_joint", "fr_anchor_rod_joint", "fr_caster_rotation_joint", "wheel_linear_actuator_joint"] 
#msg.name = ["wheel_linear_actuator_joint", "torso_lift_joint", "left_shoulder_tilt_joint", "left_linear_actuator_joint", "left_lin_act_cyl_joint", "left_upper_arm_hinge_joint", "right_shoulder_tilt_joint", "right_linear_actuator_joint", "right_lin_act_cyl_joint", "right_upper_arm_hinge_joint", "fl_caster_rotation_joint", "fl_push_rod_joint", "fl_anchor_rod_joint", "fr_caster_rotation_joint", "fr_push_rod_joint", "fr_anchor_rod_joint", "bl_caster_rotation_joint", "bl_push_rod_joint", "bl_anchor_rod_joint", "br_caster_rotation_joint", "br_push_rod_joint", "br_anchor_rod_joint", "fl_wheel_joint", "fr_wheel_joint", "bl_wheel_joint", "br_wheel_joint" ] 

msg.position = [0.0 for name in msg.name]
msg.velocity = [0.0 for name in msg.name]
# msg.position[-2] = 0.3048
while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    p.publish(msg)
    rospy.sleep(0.1)
