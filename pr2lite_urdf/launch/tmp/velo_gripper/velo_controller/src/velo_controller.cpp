/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "velo_controller/velo_controller.h"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_DECLARE_CLASS(velo_controller, VeloController, 
                        velo_controller::VeloController, pr2_controller_interface::Controller)

using namespace std;

namespace velo_controller {

VeloController::VeloController()
: joint_state_(NULL),
  loop_count_(0), robot_(NULL), last_time_(0)
{
}

VeloController::~VeloController()
{
  sub_command_.shutdown();
}

bool VeloController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  assert(robot);
  node_ = n;
  robot_ = robot;
  std::string joint_name;
  if (!node_.getParam("joint", joint_name)) {
    ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!(joint_state_ = robot_->getJointState(joint_name)))
  {
    ROS_ERROR("Could not find joint named \"%s\" (namespace: %s)",
              joint_name.c_str(), node_.getNamespace().c_str());
    return false;
  }
  if (joint_state_->joint_->type != urdf::Joint::PRISMATIC)
  {
    ROS_ERROR("The joint \"%s\" was not prismatic (namespace: %s)",
              joint_name.c_str(), node_.getNamespace().c_str());
    return false;
  }

  if (!joint_state_->calibrated_)
  {
    ROS_ERROR("Joint %s is not calibrated (namespace: %s)",
              joint_state_->joint_->name.c_str(), node_.getNamespace().c_str());
    return false;
  }
  if (!pid_.init(ros::NodeHandle(node_, "pid")))
    return false;

  ros::NodeHandle pid_node(node_, "pid");

  /* v_thresh used as a HACK to limit the closing speed of the gripper
     to help limit the kinetic windup so that closing force is more 
     controllable and linear vs. max_effort */
  pid_node.getParam("use_v_thresh", use_v_thresh_); // Binary Boolean
  pid_node.getParam("v_thresh", v_thresh_); // m/s
  v_thresh_ = copysign(v_thresh_,-1.0); // NEGATIVE TO LIMIT CLOSING SPEED

  // Position holding parameters for the control loop
  pid_node.getParam("position_holding/stall_timeout", stall_timeout_); // seconds
  pid_node.getParam("position_holding/stall_threshold", stall_threshold_); // metres
  pid_node.getParam("position_holding/holding_torque", holding_torque_); // torque to be produced

  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<pr2_controllers_msgs::JointControllerState>
    (node_, "state", 1));

  sub_command_ = node_.subscribe<pr2_controllers_msgs::Pr2GripperCommand>(
    "command", 1, &VeloController::commandCB, this);
  return true;
}

void VeloController::update()
{
  if (!joint_state_->calibrated_)
    return;

  assert(robot_ != NULL);
  double error(0);
  ros::Time time = robot_->getTime();
  assert(joint_state_->joint_);
  ros::Duration dt = time - last_time_;

  pr2_controllers_msgs::Pr2GripperCommandConstPtr command;
  command_box_.get(command);
  assert(command);

  // Computes the position error
  error = command->position - joint_state_->position_;

  // TODO: FILTER VELOCITY HERE.
  double error_dot = 0.0 - joint_state_->velocity_;

  double effort = pid_.updatePid(error, error_dot, dt);

  /* v_thresh used as a HACK to limit the closing speed of the gripper to help
     limit the kinetic windup so that closing force is more controllable and
     linear vs. max_effort.  This controller scheme should be replaced using a
     controller that is better-architected for both speed and force control,
     such as a trajectory-following position controller with force limiting.*/
  if ( use_v_thresh_ && 
       joint_state_->velocity_ < v_thresh_ )  // NOTE: Closing speeds are NEGATIVE
  { effort = 0.0;
  }

  // Set the effort (limited)
  effort = std::max(-command->max_effort, std::min(effort, command->max_effort));

  // Check for stall. If the gripper position hasn't moved by less than a threshold for at greater than some timeout, limit the output to a holding torque.
  double delta_position = joint_state_->position_ - stall_start_position_;
  //ROS_INFO("VELOCtrl: delta_pos = %f", delta_position);
  if ( fabs(delta_position) < stall_threshold_ && command->position == last_setpoint_ && command->max_effort == last_max_effort_)
  {
    // Reset the stall check if the position or max effort changes ... ie, a new command was sent.
    ros::Duration stall_length = time - stall_start_time_;
//    ROS_INFO("VELOCtrl: stall_length = %f", stall_length.toSec());
    if (stall_length.toSec() > stall_timeout_ && fabs(effort) > holding_torque_) // Don't increase the torque if the controller is requesting a lower torque.
    {
      // ROS_INFO("Stalled for %f seconds", stall_length.toSec());
      effort = copysign(holding_torque_,effort);
    }
  }
  else // if we're not stalled, update the stored copy of the current position + time parameters.
  {
    stall_start_position_ = joint_state_->position_;
    stall_start_time_ = time;
    last_setpoint_ = command->position;
    last_max_effort_ = command->max_effort;
  }


  joint_state_->commanded_effort_ = effort;

  // if( loop_count_ % 1650 == 0 )
  // {
  //   ROS_INFO("pos=%.4f, effort=%.4f, cmd=%6.2f, maxE=%.1f",
  //            joint_state_->position_,
  //            effort,
  //            command->position,
  //            command->max_effort);
  // }

  // Publish controller state info
  if(loop_count_ % 10 == 0 &&
     controller_state_publisher_ && 
     controller_state_publisher_->trylock() )
  {
    controller_state_publisher_->msg_.header.stamp = time;
    controller_state_publisher_->msg_.set_point = command->position;
    controller_state_publisher_->msg_.process_value = joint_state_->position_;
    controller_state_publisher_->msg_.process_value_dot = joint_state_->velocity_;
    controller_state_publisher_->msg_.error = error;
    controller_state_publisher_->msg_.time_step = dt.toSec();
    controller_state_publisher_->msg_.command = effort;

    double dummy;
    pid_.getGains(controller_state_publisher_->msg_.p,
                  controller_state_publisher_->msg_.i,
                  controller_state_publisher_->msg_.d,
                  controller_state_publisher_->msg_.i_clamp,
                  dummy);
    controller_state_publisher_->unlockAndPublish();
  }
  loop_count_++;

  last_time_ = time;
}

void VeloController::commandCB(const pr2_controllers_msgs::Pr2GripperCommandConstPtr& msg)
{
  command_box_.set(msg);
}

}
