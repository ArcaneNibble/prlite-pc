/*
 * pr2_teleop_booth_commander
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: E. Gil Jones

#include <string>
#include <boost/bind.hpp>

#include <geometry_msgs/Twist.h>

#include "prlite_teleop_general/prlite_teleop_general_commander.h"
#include "prlite_ax12controller.h"

static const std::string LEFT_HAND_LINK_TO_TRACK = "l_gripper_palm_link";
static const std::string RIGHT_HAND_LINK_TO_TRACK = "r_gripper_palm_link";

static const double MAX_HEAD_TRACK_SPEED = 2.0;

static const double GRIPPER_CLOSE_POSITION = 0.1;
static const double GRIPPER_CLOSE_MAX_EFFORT = 10000.0;

static const double GRIPPER_OPEN_POSITION = 1;
static const double GRIPPER_OPEN_MAX_EFFORT = 10000.0;

static const std::string RIGHT_ARM_MANNEQUIN_CONTROLLER = "r_arm_controller_loose";
static const std::string LEFT_ARM_MANNEQUIN_CONTROLLER = "l_arm_controller_loose";

static const std::string HEAD_MANNEQUIN_CONTROLLER = "head_traj_controller_loose";
static const std::string HEAD_POSITION_CONTROLLER = "head_traj_controller";

static const unsigned int WALK_BUFFER = 10;
static prlite_ax12commander *ax12;

GeneralCommander::GeneralCommander(bool control_body,
                                   bool control_head,
                                   bool control_rarm,
                                   bool control_larm,
                                   bool control_prosilica,
                                   std::string arm_controller_name) 
  : n_(),
    control_body_(control_body),
    control_head_(control_head),
    control_rarm_(control_rarm),
    control_larm_(control_larm),
    control_prosilica_(control_prosilica)                                                        
{
  ax12 = new prlite_ax12commander;

  ax12->init();
  r_arm_controller_name_ = "r_"+arm_controller_name;
  l_arm_controller_name_ = "l_"+arm_controller_name;


  std::string urdf_xml,full_urdf_xml;
  n_.param("urdf_xml", urdf_xml, std::string("robot_description"));
  if(!n_.getParam(urdf_xml,full_urdf_xml))
  {
    ROS_ERROR("Could not load the xml from parameter server: %s\n", urdf_xml.c_str());
    robot_model_initialized_ = false;
  }
  else
  {
    robot_model_.initString(full_urdf_xml);
    robot_model_initialized_ = true;
  }

  if(control_head_) { 
  } else {
  }
  if(control_body_) {
    // torso_pub_ = n_.advertise<trajectory_msgs::JointTrajectory>("torso_controller/command", 1);
    base_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  }
  if(control_rarm_) {
  } else {
  }
  if(control_larm_) {
  } else {
  }
  if(control_larm_ || control_rarm_) {
    ax12->tuck();
  } else {
  }

  if(control_rarm_) {
    /* TODO: kinematics */
  }

  if(control_larm_) {
    /* TODO: kinematics */
  }
  

  if(control_prosilica_) {
  }


  //making sure that everything is in the right mode
  head_control_mode_ = HEAD_JOYSTICK;      
  left_arm_control_mode_ = ARM_POSITION_CONTROL;
  right_arm_control_mode_ = ARM_POSITION_CONTROL;
/*
  right_arm_control_mode_ = ARM_MANNEQUIN_MODE;
  left_arm_control_mode_ = ARM_MANNEQUIN_MODE;
  head_control_mode_ = HEAD_MANNEQUIN;
*/
  ax12->arm_head_mode(right_arm_control_mode_, left_arm_control_mode_, head_control_mode_ );

  if(control_head_) {
    laser_control_mode_ = LASER_TILT_OFF;
/*
     HEAD_JOYSTICK);
*/
  }

}

GeneralCommander::~GeneralCommander() {
}


void GeneralCommander::setLaserMode(LaserControlMode mode) {
  if(!control_head_) return;

  if(laser_control_mode_ == mode) return;
 /* TODO
  req.command.profile = "linear";
  if(mode == LASER_TILT_SLOW) {
    ROS_DEBUG("Sending slow");
  } else if(mode == LASER_TILT_FAST) {
    ROS_DEBUG("Sending fast");
  } else {
    ROS_DEBUG("Sending off");
  }
  laser_control_mode_ = mode;
  */
}

void GeneralCommander::setHeadMode(HeadControlMode mode) {
  // if(!control_head_) return;
  if(mode == head_control_mode_) return;
  if(mode == HEAD_TRACK_LEFT_HAND) {
    ROS_DEBUG("Setting head to track left hand");
  } else if(mode == HEAD_TRACK_RIGHT_HAND) {
    ROS_DEBUG("Setting head to track right hand");
  }
  std::vector<std::string> start_controllers;
  std::vector<std::string> stop_controllers;
  if(mode == HEAD_MANNEQUIN) {
	  ax12->arm_head_mode(prlite_ax12commander::UNCHANGED_MODE, prlite_ax12commander::UNCHANGED_MODE, prlite_ax12commander::MANNEQUIN_MODE);
  } else if(head_control_mode_ == HEAD_MANNEQUIN) {

    ax12->arm_head_mode(prlite_ax12commander::UNCHANGED_MODE, prlite_ax12commander::UNCHANGED_MODE, prlite_ax12commander::POSITION_MODE);
  }
  head_control_mode_ = mode;
}

void GeneralCommander::setArmMode(WhichArm arm, ArmControlMode mode) {
  if(!control_rarm_ && !control_larm_) return;
  if(!control_rarm_ && arm == ARMS_RIGHT) return;
  if(!control_larm_ && arm == ARMS_LEFT) return;

  if(arm == ARMS_LEFT) {
    if(mode == left_arm_control_mode_) return;
  } else if(arm == ARMS_RIGHT) {
    if(mode == right_arm_control_mode_) return;
  } else {
    if(mode == left_arm_control_mode_ && mode == right_arm_control_mode_) return;
  }

  std::string left_running_controller;
  std::string right_running_controller;

  if(left_arm_control_mode_ == ARM_MANNEQUIN_MODE) {
    left_running_controller = LEFT_ARM_MANNEQUIN_CONTROLLER;
  } else if(left_arm_control_mode_ == ARM_POSITION_CONTROL) {
    left_running_controller = l_arm_controller_name_;
  }

  if(right_arm_control_mode_ == ARM_MANNEQUIN_MODE) {
    right_running_controller = RIGHT_ARM_MANNEQUIN_CONTROLLER;
  } else if(right_arm_control_mode_ == ARM_POSITION_CONTROL) {
    right_running_controller = r_arm_controller_name_;
  }

  if(mode == ARM_NO_CONTROLLER) {
    if(arm == ARMS_LEFT || arm == ARMS_BOTH) {
      ax12->arm_head_mode(prlite_ax12commander::NO_CONTROLLER_MODE, prlite_ax12commander::UNCHANGED_MODE, prlite_ax12commander::UNCHANGED_MODE);
    }
    if(arm == ARMS_RIGHT || arm == ARMS_BOTH) {
      ax12->arm_head_mode(prlite_ax12commander::UNCHANGED_MODE, prlite_ax12commander::NO_CONTROLLER_MODE, prlite_ax12commander::UNCHANGED_MODE);
    }
  } else if(mode == ARM_MANNEQUIN_MODE) {
    if(arm == ARMS_LEFT || arm == ARMS_BOTH) {
      ax12->arm_head_mode(prlite_ax12commander::MANNEQUIN_MODE, prlite_ax12commander::UNCHANGED_MODE, prlite_ax12commander::UNCHANGED_MODE);
    }
    if(arm == ARMS_RIGHT || arm == ARMS_BOTH) {
      ax12->arm_head_mode(prlite_ax12commander::UNCHANGED_MODE, prlite_ax12commander::MANNEQUIN_MODE, prlite_ax12commander::UNCHANGED_MODE);
    }
  } else if(mode == ARM_POSITION_CONTROL) {
    if(arm == ARMS_LEFT || arm == ARMS_BOTH) {
      ax12->arm_head_mode(prlite_ax12commander::POSITION_MODE, prlite_ax12commander::UNCHANGED_MODE, prlite_ax12commander::UNCHANGED_MODE);
    }
    if(arm == ARMS_RIGHT || arm == ARMS_BOTH) {
      ax12->arm_head_mode(prlite_ax12commander::UNCHANGED_MODE, prlite_ax12commander::POSITION_MODE, prlite_ax12commander::UNCHANGED_MODE);
    }
  }
  if(arm == ARMS_LEFT || arm == ARMS_BOTH) {
    left_arm_control_mode_ = mode;
  }
  if(arm == ARMS_RIGHT || arm == ARMS_BOTH) {
    right_arm_control_mode_ = mode;
  }
}

GeneralCommander::ArmControlMode GeneralCommander::getArmMode(WhichArm arm) {
  if(arm == ARMS_RIGHT || arm == ARMS_BOTH) {
    return right_arm_control_mode_;
  } else {
    return left_arm_control_mode_;
  }
}

GeneralCommander::HeadControlMode GeneralCommander::getHeadMode() {
  return head_control_mode_;
}


void GeneralCommander::sendHeadCommand(double req_pan, double req_tilt) {
  // if(!control_head_) return;
  if(head_control_mode_ != HEAD_JOYSTICK) {
    return;
  }
  ax12->HeadCommand(req_pan, req_tilt);
/*
  ax12->set_desired_pos(prlite_ax12commander::kinectpan, req_pan);
  ax12->set_desired_pos(prlite_ax12commander::kinecttilt, req_tilt);
  ax12->move_to_desired_pos();
*/
}

void GeneralCommander::sendHeadTrackCommand() {
  if(!control_head_) return;
  if(head_control_mode_ != HEAD_TRACK_LEFT_HAND &&
     head_control_mode_ != HEAD_TRACK_RIGHT_HAND) {
    return;
  }
/* TODO
*/
}

void GeneralCommander::sendGripperCommand(WhichArm which, bool close) {
  double position, max_effort;

  if(!control_rarm_ && !control_larm_) return;
  if(!control_rarm_ && which == ARMS_RIGHT) return;
  if(!control_larm_ && which == ARMS_LEFT) return;
  if(which == ARMS_RIGHT || which == ARMS_BOTH) {\
    if(close) {
      position = GRIPPER_CLOSE_POSITION;
      max_effort = GRIPPER_CLOSE_MAX_EFFORT;
    } else {
      position = GRIPPER_OPEN_POSITION;
      max_effort = GRIPPER_OPEN_MAX_EFFORT;
    }
    ax12->set_desired_pos(prlite_ax12commander::rfingerR, position);
    position *= -1;
    ax12->set_desired_pos(prlite_ax12commander::lfingerR, position);
    ax12->move_to_desired_pos();
  }
  if(which == ARMS_LEFT || which == ARMS_BOTH) {
    if(close) {
      position = GRIPPER_CLOSE_POSITION;
      max_effort = GRIPPER_CLOSE_MAX_EFFORT;
    } else {
      position = GRIPPER_OPEN_POSITION;
      max_effort = GRIPPER_OPEN_MAX_EFFORT;
    }
    ax12->set_desired_pos(prlite_ax12commander::lfinger, position);
    position *= -1;
    ax12->set_desired_pos(prlite_ax12commander::rfinger, position);
    ax12->move_to_desired_pos();
  }
  
}

void GeneralCommander::sendTorsoCommand(double pos, double vel) {
  if(!control_body_) return;
  //only do this if we are commanding some velocity and not transitioning
  if(fabs(vel) < .0001 && fabs(last_torso_vel_ < .0001)) {
    //return;
  }

  ax12->setTorsoGoal(vel);

}

void GeneralCommander::sendBaseCommand(double vx, double vy, double vw) {
  if(!control_body_) return;
  geometry_msgs::Twist cmd;
  // ROS_INFO_STREAM("Base: " << vx <<" " << vy << " " << vw);

  cmd.linear.x = vx;
  cmd.linear.y = vy;
  cmd.angular.z = vw;
  base_pub_.publish(cmd);
}

/*
void GeneralCommander::switchControllers(const std::vector<std::string>& start_controllers, const std::vector<std::string>& stop_controllers) {
}
*/

void GeneralCommander::sendWristVelCommands(double right_wrist_vel, double left_wrist_vel, double hz) {
  
  ax12->WristCommand(right_wrist_vel, left_wrist_vel);
}

/*
void GeneralCommander::composeWristRotGoal(const std::string pref, pr2_controllers_msgs::JointTrajectoryGoal& goal, 
                                         std::vector<double>& des_joints, 
                                         double des_vel, double hz) const {

  if (pref == 'L')
    ax12->set_desired_pos(prlite_ax12commander::wristrot, des_joints);
  else if (pref == 'R')
    ax12->set_desired_pos(prlite_ax12commander::wristrotR, des_joints);
  ax12->move_to_desired_pos();
}

void GeneralCommander::updateCurrentWristPositions() {

  if(control_rarm_) {
  }

  if(control_larm_) {
  } 
}

void GeneralCommander::clampDesiredArmPositionsToActual(double max_dist)  {}

void GeneralCommander::unnormalizeTrajectory(trajectory_msgs::JointTrajectory& traj) const {

}
*/

void GeneralCommander::sendArmVelCommands(double r_x_vel, double r_y_vel, double r_z_vel, double r_yaw_vel, 
                                        double l_x_vel, double l_y_vel, double l_z_vel, double l_yaw_vel,
                                        double hz) {

  ax12->ArmCommand(r_x_vel, r_y_vel, r_z_vel,
                     l_x_vel, l_y_vel, l_z_vel);

}

bool GeneralCommander::moveToWalkAlongArmPose() {
  return true;
}

bool GeneralCommander::initWalkAlong() {
  ax12->tuck();
  return true;
}

void GeneralCommander::updateWalkAlongAverages() {

}

void GeneralCommander::sendWalkAlongCommand(double thresh, 
                                          double x_dist_max, double x_speed_scale,
                                          double y_dist_max, double y_speed_scale,
                                          double rot_speed_scale) {
	
}

/*
double GeneralCommander::calcAverage(const std::list<double>& av_list) const {
	double av = 0.0;
  for(std::list<double>::const_iterator it = av_list.begin();
      it != av_list.end();
      it++) {
    av += (*it);
  }
  av /= av_list.size();
  return av;
}
*/

geometry_msgs::Pose GeneralCommander::getPositionFromJointsPose(ros::ServiceClient& service_client,  							   
      std::string fk_link,
      const std::vector<std::string>& joint_names, const std::vector<double>& joint_pos) {
geometry_msgs::Pose ret_pose;

      ROS_DEBUG("GeneralCommander::getPositionFromJointsPose");
  return ret_pose;
}

void GeneralCommander::sendHeadSequence(HeadSequence seq) {

  if(!control_head_) return;

  setHeadMode(HEAD_JOYSTICK);

  if(seq == HEAD_NOD) {

    ax12->set_desired_pos(prlite_ax12commander::kinectpan, HEAD_PAN_MIDDLE);
    ax12->set_desired_pos(prlite_ax12commander::kinectpan, HEAD_TILT_MIDDLE);
    ax12->move_to_desired_pos();

    ax12->set_desired_pos(prlite_ax12commander::kinectpan, HEAD_TILT_DOWN);
    ax12->move_to_desired_pos();


    ax12->set_desired_pos(prlite_ax12commander::kinectpan, HEAD_TILT_UP);
    ax12->move_to_desired_pos();


    ax12->set_desired_pos(prlite_ax12commander::kinectpan, HEAD_TILT_MIDDLE);
    ax12->move_to_desired_pos();

  } else if(seq == HEAD_SHAKE){

    ax12->set_desired_pos(prlite_ax12commander::kinectpan, HEAD_PAN_MIDDLE);
    ax12->set_desired_pos(prlite_ax12commander::kinectpan, HEAD_TILT_MIDDLE);
    ax12->move_to_desired_pos();


    ax12->set_desired_pos(prlite_ax12commander::kinectpan, HEAD_PAN_LEFT);
    ax12->move_to_desired_pos();


    ax12->set_desired_pos(prlite_ax12commander::kinectpan, HEAD_PAN_RIGHT);
    ax12->move_to_desired_pos();


    ax12->set_desired_pos(prlite_ax12commander::kinectpan, HEAD_PAN_MIDDLE);
    ax12->move_to_desired_pos();
  }    
}

/* for emergency stops
void GeneralCommander::powerBoardCallback(const pr2_msgs::PowerBoardStateConstPtr &powerBoardState) {

}

void GeneralCommander::requestProsilicaImage(std::string ns) {

}
*/

/*
void GeneralCommander::tuckArms(WhichArm arm, double shoulder_pan, double shoulder_tilt, 
    double elbow, double wrist_tilt, double wrist_rot, double l_finger, double r_finger) 
*/
void GeneralCommander::tuckArms(WhichArm arm)
{
  
  //can't tuck just one arm for now
  if(!control_rarm_ || !control_larm_) {
    return;
  }

  setArmMode(arm, ARM_POSITION_CONTROL);

  ROS_DEBUG("Sending tuck arms");

  ax12->tuck();
}

/*
void GeneralCommander::untuckArms(WhichArm arm, double shoulder_pan, double shoulder_tilt, 
    double elbow, double wrist_tilt, double wrist_rot, double l_finger, double r_finger) 
*/
void GeneralCommander::untuckArms(WhichArm arm)
{

  //can't tuck just one arm for now
  if(!control_rarm_ || !control_larm_) {
    return;
  }

  setArmMode(arm, ARM_POSITION_CONTROL);
  
  if(arm == ARMS_BOTH) {
  } else {
    ROS_DEBUG("Untucking one arm not supported");
  }

  ROS_DEBUG("Sending untuck arms");

  ax12->untuck();
}

void GeneralCommander::mimicArms(WhichArm arm)
{

  //can't tuck just one arm for now
  if(!control_rarm_ || !control_larm_) {
    return;
  }

  setArmMode(arm, ARM_POSITION_CONTROL);

  if(arm == ARMS_BOTH) {
  } else {
    ROS_DEBUG("Untucking one arm not supported");
  }

  ROS_DEBUG("Sending mimic arms");

  ax12->kinect_callobration_pos();

}

