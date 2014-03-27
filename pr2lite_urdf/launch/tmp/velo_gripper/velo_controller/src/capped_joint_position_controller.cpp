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

#include <math.h>
#include "velo_controller/capped_joint_position_controller.h"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_DECLARE_CLASS(velo_controller, CappedJointPositionController,
                             velo_controller::CappedJointPositionController, 
               pr2_controller_interface::Controller);

using namespace std;

namespace velo_controller {

CappedJointPositionController::CappedJointPositionController()
: joint_state_(NULL), command_(0),
  loop_count_(0),  initialized_(false), command_prev_(99.0), robot_(NULL), last_time_(0)
{
}

CappedJointPositionController::~CappedJointPositionController()
{
  sub_command_.shutdown();
}

bool CappedJointPositionController::init(pr2_mechanism_model::RobotState *robot, const std::string &joint_name,
				   const control_toolbox::Pid &pid)
{
  assert(robot);
  robot_ = robot;
  last_time_ = robot->getTime();

  joint_state_ = robot_->getJointState(joint_name);
  if (!joint_state_)
  {
    ROS_ERROR("CappedJointPositionController could not find joint named \"%s\"\n",
              joint_name.c_str());
    return false;
  }
  /************************
  if (!joint_state_->calibrated_)
  {
    ROS_ERROR("Joint %s not calibrated for CappedJointPositionController", joint_name.c_str());
    return false;
  }
  ************************/

  pid_controller_ = pid;

  return true;
}

bool CappedJointPositionController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  assert(robot);
  node_ = n;

  std::string joint_name;

  getNodeParam<std::string>("joint", joint_name);
  getNodeParam<double>("error_max", error_max_);    /**< special-sauce IN THE _CAPPED_ VERSION OF THIS CONTROLLER */
  error_max_ = std::fabs(error_max_);
  getNodeParam<double>("velocity", velocity_);      /**< special-sauce IN THE _CAPPED_ VERSION OF THIS CONTROLLER */

  control_toolbox::Pid pid;
  if (!pid.init(ros::NodeHandle(node_, "pid")))
    return false;

  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<pr2_controllers_msgs::JointControllerState>
    (node_, "state", 1));

  sub_command_ = node_.subscribe<std_msgs::Float64>("command", 1, &CappedJointPositionController::setCommandCB, this);

  return init(robot, joint_name, pid);
}


void CappedJointPositionController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min)
{
  pid_controller_.setGains(p,i,d,i_max,i_min);
}

void CappedJointPositionController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  pid_controller_.getGains(p,i,d,i_max,i_min);
}

std::string CappedJointPositionController::getJointName()
{
  return joint_state_->joint_->name;
}

// Set the joint position command
void CappedJointPositionController::setCommand(double cmd)
{
  command_ = cmd;
}

// Return the current position command
void CappedJointPositionController::getCommand(double & cmd)
{
  cmd = command_;
}

void CappedJointPositionController::update()
{
  /*
    if (!joint_state_->calibrated_)
    return;
  */
  double error(0);
  ros::Time time = robot_->getTime();
  dt_= time - last_time_;

  if (!initialized_)
  {
    initialized_ = true;
    command_ = joint_state_->position_;
  }

  double error_direction = copysign(1.0, command_ - joint_state_->position_);

  if (command_ != command_prev_ )
  {
    //// Nice to have a jolt at the beginning of a motion to unstick us.
    command_via_ = joint_state_->position_ + error_direction*error_max_;
    command_prev_ = command_;

    pid_controller_.reset();
  }
  else
  {
    double via_step = error_direction * velocity_ * dt_.toSec();
    if ( fabs(via_step) > fabs(command_ - command_via_) )
      command_via_ = command_;   // Step would go past command_; stop at command_.
    else
      command_via_ += via_step;  // Take a step toward commad_.
  }


  if(joint_state_->joint_->type == urdf::Joint::REVOLUTE)
  {
    angles::shortest_angular_distance_with_limits(command_, joint_state_->position_, joint_state_->joint_->limits->lower, joint_state_->joint_->limits->upper,error);

  }
  else if(joint_state_->joint_->type == urdf::Joint::CONTINUOUS)
  {
    error = angles::shortest_angular_distance(command_, joint_state_->position_);
  }
  else //prismatic
  {
    error = command_via_ - joint_state_->position_ ;
  }

  double velocity_desired = fabs(error) < fabs(error_max_) ? 0.0 : copysign(velocity_,error);
  double capped_vError = velocity_desired - joint_state_->velocity_;

  double capped_pError = error;
  double commanded_effort = pid_controller_.updatePid(capped_pError, capped_vError, dt_);

  // double commanded_effort = pid_controller_.updatePid(capped_pError, dt_);  // assuming desired velocity is 0

  joint_state_->commanded_effort_ += commanded_effort; // Safety and Joint controllers can also run.

#ifdef DEBUG_VELO_GRIPPER
  if(loop_count_++ % 800 == 0)
  {
    ROS_DEBUG("p= %5.1lf,  via= %5.1lf,  cmd= %5.1lf,  v= %5.1lf,  CE= %7.1lf",
             1000*joint_state_->position_, 
             1000*command_via_, 
             1000*command_, 
             1000*joint_state_->velocity_, 
             joint_state_->commanded_effort_);
  }
#endif

  if(loop_count_++ % 10 == 0)
  {
    if(controller_state_publisher_ && controller_state_publisher_->trylock())
    {
      controller_state_publisher_->msg_.header.stamp = time;
      controller_state_publisher_->msg_.set_point = command_;
      controller_state_publisher_->msg_.process_value = joint_state_->position_;
      controller_state_publisher_->msg_.process_value_dot = joint_state_->velocity_;
      controller_state_publisher_->msg_.error = error;
      controller_state_publisher_->msg_.time_step = dt_.toSec();
      controller_state_publisher_->msg_.command = commanded_effort;

      double dummy;
      getGains(controller_state_publisher_->msg_.p,
               controller_state_publisher_->msg_.i,
               controller_state_publisher_->msg_.d,
               controller_state_publisher_->msg_.i_clamp,
               dummy);
      controller_state_publisher_->unlockAndPublish();
    }
  }

  last_time_ = time;
}

void CappedJointPositionController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
{
  command_ = msg->data;
}

}
