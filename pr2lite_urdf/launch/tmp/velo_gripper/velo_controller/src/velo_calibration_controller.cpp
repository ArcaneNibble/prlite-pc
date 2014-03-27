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

#include "velo_controller/velo_calibration_controller.h"
#include "ros/time.h"
#include "pluginlib/class_list_macros.h"
#include "pr2_mechanism_model/joint.h"

using namespace std;

PLUGINLIB_DECLARE_CLASS(velo_controller, VeloCalibrationController,
                        velo_controller::VeloCalibrationController, pr2_controller_interface::Controller)

namespace velo_controller
{

VeloCalibrationController::VeloCalibrationController()
  : last_publish_time_(0), joint_(NULL), zero_offset_(0.0), post_cal_count_(0)
{
}

VeloCalibrationController::~VeloCalibrationController()
{
}

bool VeloCalibrationController::init(pr2_mechanism_model::RobotState *robot,
                                          ros::NodeHandle &n)
{
  std::string actuator_name;

  assert(robot);
  robot_ = robot;
  node_ = n;

  getNodeParam<std::string>("joint", joint_name_);
  getNodeParam<std::string>("actuator", actuator_name);
  getNodeParam<double>("stopped_velocity_tolerance", stopped_velocity_tolerance_);
//  getNodeParam<double>("error_max", error_max_); // NOT NEEDED HERE. PICKED UP IN vc_ (THE CAPPED CONTROLLER).

  if (!(joint_ = robot->getJointState(joint_name_)))
  {
    ROS_ERROR("Could not find joint \"%s\" (namespace: %s)",
              joint_name_.c_str(), node_.getNamespace().c_str());
    return false;
  }

  if (!(actuator_ = robot->model_->getActuator(actuator_name)))
  {
    ROS_ERROR("Could not find actuator \"%s\" (namespace: %s)",
              actuator_name.c_str(), node_.getNamespace().c_str());
    return false;
  }

  XmlRpc::XmlRpcValue other_joint_names;
  if ( node_.getParam("other_joints", other_joint_names) )
  {
    if (other_joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("\"other_joints\" was not an array (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }
    else
    {
      for (int i = 0; i < other_joint_names.size(); ++i)
      {
        pr2_mechanism_model::JointState *j;
        std::string name = (std::string)other_joint_names[i];
        if ((j = robot->getJointState(name))){
          other_joints_.push_back(j);
        }
        else {
          ROS_ERROR("Could not find joint \"%s\" (namespace: %s)",
                    name.c_str(), node_.getNamespace().c_str());
          return false;
        }
      }
    }
  }

  /*****
  if (actuator_->state_.zero_offset_ != 0){
    ROS_INFO("Joint %s is already calibrated at offset %f", joint_name_.c_str(), actuator_->state_.zero_offset_);
    joint_->calibrated_ = true;
    for (size_t i = 0; i < other_joints_.size(); ++i)
      other_joints_[i]->calibrated_ = true;
    state_ = CALIBRATED;
  }
  */
  if ( false ) {}
  else{
    ROS_INFO("Joint \"%s\" will always (re)calibrate when calibration is run", joint_name_.c_str());
    state_ = INITIALIZED;
    joint_->calibrated_ = false;
  }


  if (!(vc_.init(robot, node_)))
  {
    ROS_ERROR("Could not init PID class from %s",__FILE__);
    return false;
  }

  // advertise service to check calibration
  is_calibrated_srv_ = node_.advertiseService("is_calibrated", &VeloCalibrationController::isCalibrated, this);

  // "Calibrated" topic
  pub_calibrated_.reset(new realtime_tools::RealtimePublisher<std_msgs::Empty>(node_, "calibrated", 1));

  return true;
}


void VeloCalibrationController::starting()
{
  state_ = INITIALIZED;
  actuator_->state_.zero_offset_ = 0.0;
  joint_->calibrated_ = false;
  s0_ = state_;

  // ROS_INFO("STATE MACHINE STATE TABLE:");
  // ROS_INFO("%d : INITIALIZED", INITIALIZED);
  // ROS_INFO("%d : STARTING", STARTING);
  // ROS_INFO("%d : CLOSING", CLOSING);
  // ROS_INFO("%d : BACK_OFF", BACK_OFF);
  // ROS_INFO("%d : TOP", TOP);
  // ROS_INFO("%d : HOME", HOME);
  // ROS_INFO("%d : CALIBRATED", CALIBRATED);
}


bool VeloCalibrationController::isCalibrated(pr2_controllers_msgs::QueryCalibrationState::Request& req,
						  pr2_controllers_msgs::QueryCalibrationState::Response& resp)
{
  resp.is_calibrated = (state_ == CALIBRATED);
  return true;
}

void VeloCalibrationController::goalCommand(double goal)
{
  vc_.setCommand( goal + zero_offset_);
}

void VeloCalibrationController::update()
{
  assert(joint_);
  assert(actuator_);

#ifdef DEBUG_VELO_GRIPPER
  // DEBUG STATE TRANSITIONS
  if ( s0_ != state_ || (stop_count_ && !(stop_count_ % 1000)) )
  { ROS_WARN("STATE: %d --> %d    pos= %7.4lf  (zo= %7.4lf)  odo= %.4lf",
              s0_, state_, joint_->position_, zero_offset_ , joint->joint_statistics_.odometer);
    s0_=state_;
  }
#endif

  // RUN THE CONTROLLER UPDATE.  NEED TO RUN THIS BEFORE STATE ENGINE.
  if (state_ != CALIBRATED)
    vc_.update();

  // Always
  if ( !(joint_->calibrated_) && fabs(joint_->velocity_) < stopped_velocity_tolerance_)
    stop_count_++;
  else
    stop_count_=0;

  int settleCount = 600;

  // BALLSCREW DISTANCE CONSTANTS (in meters)
  const double VELOCC_MTtop     =  0.0165;    // FULL TRAVEL is 0.0162
  const double VELOCC_empty     =  0.0150;    // More travel than this indicates Missing Gripper (or broken tendon)
  const double VELOCC_open      =  0.0113;    // Open gripper
  const double VELOCC_BOinstall =  0.0007;    // "BackOff install" AMOUNT TO RETRACT FROM TOP TO installation point
  const double VELOCC_wrong     =  0.0090;    // Travel must be more than this to be engaged with tendon interlock (0.0075 minimum)
  const double VELOCC_BObottom  =  0.0030;    // "BackOff from Bottom"
  const double VELOCC_MTclosed  = -0.0040;    // "More Than Closed"  (After moving coords to bottom)
  const double VELOCC_MTbottom  = -0.0170;    // "More Than Bottom"  (Starting from anywhere)

  switch (state_)
  {
  case INITIALIZED:
    state_ = STARTING;
    break;

  case STARTING:
    close_count_ = 0;
    stop_count_ = 0;

    actuator_->state_.zero_offset_ = 0.0;
    joint_->calibrated_ = false;
    zero_offset_ = joint_->position_;

    odometer_last_ = joint_->joint_statistics_.odometer_;

    goalCommand( VELOCC_MTbottom ); // More than full travel
    state_ = CLOSING;
    break;

  case CLOSING:
    // Makes sure the gripper is stopped for a while before cal
    if (stop_count_ > settleCount)
    {
      stop_count_ = 0;

      // ALWAYS RESET THE ACTUATOR TO ZERO AT THE BOTTOM
      zero_offset_ = joint_->position_;

      if ( ++close_count_ < 2 )
      {
        // BACK OFF A TIME OR TWO TO MAKE SURE WE ARE AT THE END OF TRAVEL
        // ROS_INFO("FOUND Bottom, now heading to BACKOFF");
        goalCommand( VELOCC_BObottom); // Bump out from our new zero
        state_ = BACK_OFF;
      }
      else
      {
        // FOUND THE BOTTOM OF TRAVEL
        // ROS_INFO("FOUND Bottom, now heading to TOP");

        // Set the actuator zero once.
        actuator_->state_.zero_offset_ = actuator_->state_.position_;
        // Now coords are actually relative to zero.
        zero_offset_ = 0.0;

        goalCommand( VELOCC_MTtop ); // Gripper installation/fully-open position.
        state_ = TOP;
      }
    }
    break;

  case BACK_OFF: // Back off so we can reset from a known good position
    if (stop_count_ > settleCount)
    {
      if ( joint_->joint_statistics_.odometer_ < odometer_last_ + .001)
        ROS_ERROR("Joint \"%s\"is NOT moving.  Breakers turned on?  Joint stuck?",
                  joint_name_.c_str());
      stop_count_ = 0;
      goalCommand( VELOCC_MTclosed );
      state_ = CLOSING;
    }
    break;

  case TOP:
    /* PUSHING THE BALLSCREW ALL THE WAY OUT FROM CLOSED TELLS US THAT WE HAVE A GRIPPER INSTALLED CORRECTLY */
    if (stop_count_ > settleCount)
    {
      stop_count_ = 0;
      if ( joint_->position_ < VELOCC_wrong )
      {
        ROS_ERROR("Gripper \"%s\" NOT installed properly!  Please reinstall and recalibrate.  (pos=%6.4fm)",
                  joint_name_.c_str(),joint_->position_);
        goalCommand( VELOCC_BObottom); // Go to a safe place, not at either end.

      }
      else if ( joint_->position_ > VELOCC_empty )
      {
        ROS_ERROR("Gripper \"%s\" NOT installed!  Please install and recalibrate.  (pos=%6.4fm)",
                  joint_name_.c_str(),joint_->position_);
        goalCommand( joint_->position_ - VELOCC_BOinstall ); // Gripper installation/fully-open position.
      }
      else
      {
        goalCommand( VELOCC_open ); // Gripper open, ready to go.
      }

      // Ballscrew could get jammed at top end of travel.  Watch for it...
      odometer_last_ = joint_->joint_statistics_.odometer_;
      state_ = HOME;
    }
    break;

  case HOME:
    if (stop_count_ > settleCount)
    {
      if ( joint_->joint_statistics_.odometer_ < odometer_last_ + .001)
        ROS_ERROR("Joint \"%s\"is NOT moving. Joint stuck?",joint_name_.c_str());
      stop_count_ = 0;
      joint_->calibrated_ = true;
      for (size_t i = 0; i < other_joints_.size(); ++i)
        other_joints_[i]->calibrated_ = true;

      /* IMPLEMENTED DIFFERENT SOLUTION THAT mutes TRANSMISSION OUTPUT COMPLETELY
       */
      // double p,i,d,i_max,i_min;
      // vc_.getGains(p, i, d, i_max, i_min);
      // p = 5000.0; // Coord change about to happen: tendon --> grip
      // d =  100.0; // So backoff gains to keep things stable.
      // i =    1.0;
      // vc_.setGains(p ,0.0, d ,0.0,0.0);
      // vc_.update();

      post_cal_count_=0;
      state_ = CALIBRATED;
    }
    break;

  case CALIBRATED:

    /* After transmission has run in CALIBRATED state, the joint position will
       be updated to be in "gap" coordinates.  Now we want target in "gap"
       coords to hold position until the cal controller is stopped/unloaded. */
     if ( post_cal_count_++ == 0 )
     {
       vc_.setCommand( joint_->position_ );
       vc_.update();
     }

    if ( pub_calibrated_ && 
         last_publish_time_ + ros::Duration(0.5) < robot_->getTime() &&
         pub_calibrated_->trylock() )
    {
      last_publish_time_ = robot_->getTime();
      pub_calibrated_->unlockAndPublish();
    }
    break;
  }

}
} // namespace
