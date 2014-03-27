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

#ifndef VELO_CONTROLLER_H
#define VELO_CONTROLLER_H

#include <ros/node_handle.h>

#include <pr2_controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <control_toolbox/pid_gains_setter.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>

#include <pr2_controllers_msgs/JointControllerState.h>
#include <pr2_controllers_msgs/Pr2GripperCommand.h>

namespace velo_controller
{

class VeloController : public pr2_controller_interface::Controller
{
public:

  VeloController();
  ~VeloController();

  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

  virtual void starting() {
    using namespace pr2_controllers_msgs;
    Pr2GripperCommandPtr c(new Pr2GripperCommand);
    /* Set some 'max_effort' to allow holding position. 
       Useful when there is no gripper installed, so that compensation 
       for (absent) spring force does not cause position drift. */
    c->max_effort = 2.0;
    c->position = joint_state_->position_;
    command_box_.set(c);
  }

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  virtual void update();

  pr2_mechanism_model::JointState *joint_state_;
  realtime_tools::RealtimeBox<pr2_controllers_msgs::Pr2GripperCommandConstPtr> command_box_;

private:
  int loop_count_;
  
  // Velocity filter parameters
  double filtered_velocity_;
  double lambda_;
  int use_v_thresh_;  // Binary Boolean
  double  v_thresh_;  // max closing speed

  // Parameters for the holding torque applied to the motor once stalled
  double stall_threshold_;
  double stall_timeout_;
  double stall_counter_;
  ros::Time stall_start_time_;
  double stall_start_position_;
  double holding_torque_;
  double last_position_;
  double last_setpoint_;
  double last_max_effort_;
  
  pr2_mechanism_model::RobotState *robot_;
  control_toolbox::Pid pid_;
  ros::Time last_time_;

  ros::NodeHandle node_;

  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      pr2_controllers_msgs::JointControllerState> > controller_state_publisher_ ;

  ros::Subscriber sub_command_;
  void commandCB(const pr2_controllers_msgs::Pr2GripperCommandConstPtr& msg);
};

} // namespace

#endif
