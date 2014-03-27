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

#pragma once

#include "pr2_mechanism_model/robot.h"
#include "velo_controller/capped_joint_position_controller.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/Empty.h"
#include "pr2_controllers_msgs/QueryCalibrationState.h"

namespace velo_controller
{

class VeloCalibrationController : public pr2_controller_interface::Controller
{
public:
  VeloCalibrationController();
  ~VeloCalibrationController();

  virtual bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
  virtual void starting();
  virtual void update();

  bool isCalibrated(pr2_controllers_msgs::QueryCalibrationState::Request& req, pr2_controllers_msgs::QueryCalibrationState::Response& resp);


protected:

  std::string joint_name_;
  enum { INITIALIZED, STARTING, CLOSING, BACK_OFF, TOP, HOME, CALIBRATED };
  int state_;
  int close_count_;
  int stop_count_;

  int s0_,c0_;

  ros::NodeHandle node_;
  pr2_mechanism_model::RobotState *robot_;
  ros::Time last_publish_time_;
  ros::ServiceServer is_calibrated_srv_;
  boost::scoped_ptr<realtime_tools::RealtimePublisher<std_msgs::Empty> > pub_calibrated_;

  pr2_mechanism_model::JointState *joint_;
  pr2_hardware_interface::Actuator *actuator_;
  std::vector<pr2_mechanism_model::JointState*> other_joints_;
  double odometer_last_;

  void goalCommand(double goal);
  double zero_offset_;

  double init_time;
  double search_velocity_;
  double stopped_velocity_tolerance_;
  double error_max_;
  int post_cal_count_;

  velo_controller::CappedJointPositionController vc_; /** The joint position controller used to move the joint.*/

private:

  template <typename T> bool getNodeParam(const char *key, T &value)
  {
    if (!node_.getParam(key, value))
    {
      ROS_ERROR("Missing parameter, (namespace: %s) \"%s\"", node_.getNamespace().c_str(), key);
      return false;
    }
    return true;
  }

};


}
