/*
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

/*
 * Authors: J Hawke & Bob Holmberg
 */

/*
 * propagatePosition (from as to js)
 *   as position and velocity are doubled to get js, since gripper is two sided
 *   as torque is directly converted to gap_effort
 *   as torque to passive joint is /4 since there are 4 links
 * propagateEffort (from js to as)
 *   popluate only as->commanded_.effort_
 *     this is directly transferred as torque and gap effort is 1to1
 *
 * below only for simulation
 *
 * propagatePositionBackwards (from js to as)
 *   as position and velocity transfers 1to1 to joint_angle (1-sided)
 *   as last_measured_effort_ should be 1to1 gap_effort of non-passive js[i]->commanded_effort
 * propagateEffortBackwards
 *   non-passive js->commanded_effort_ is 1to1 with MT
 *   passive js->commanded_effort_ is 1/2?? of MT converted to joint torques
 */

#include "velo_controller/velo_transmission.h"
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <numeric>
#include <angles/angles.h>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <string>

#include <math.h>

using namespace pr2_hardware_interface;
using namespace pr2_mechanism_model;

PLUGINLIB_DECLARE_CLASS(velo_controller, VeloTransmission,
                        velo_controller::VeloTransmission,
                        pr2_mechanism_model::Transmission)

namespace velo_controller {

class VeloTransmission::ParamFetcher
{

private:
  const TiXmlElement *j_;
  const char* joint_name_;

public:

  int error_count_;

  ros::NodeHandle *nh_;


  // CONSTRUCTOR
  ParamFetcher(const TiXmlElement *j, Robot *robot = NULL): nh_(NULL)
  {
    error_count_=0;
    j_ = j;


    // SET joint_name_ FROM XML
    joint_name_ = j_->Attribute("name");
    if (!joint_name_)
    {
      error_count_++;
      ROS_ERROR("VeloTransmission did not specify joint name");
      return;
    }

    // CREATE NODE HANDLE
    nh_ = new ros::NodeHandle(std::string(joint_name_));
    if (!nh_->ok())
    {
      error_count_++;
      ROS_ERROR("VELO Transmission: node handle does not exist/is shutdown");
      return;
    }

    /************************************
    if (robot)
    {
      // SET joint_name_ FROM ROBOT POINTER
      std::string jn_str;
      const boost::shared_ptr<const urdf::Joint> joint = robot->robot_model_.getJoint(jn_str);
      joint_name_ = jn_str.c_str();
      if (!joint)
      {
        error_count_++;
        ROS_ERROR("VeloTransmission could not find joint named \"%s\"", joint_name_);
        return;
      }
    }
    ****************************************/


  }

  // DESTRUCTOR
  ~ParamFetcher();

  // API to retrieve joint_name
  const char * getJointName()
  {
    return joint_name_;
  }

  // ERROR-CHECKING PARAM GETTER WITH CUSTOM MESSAGE.
  bool getParam(const char *key, double &value)
  {
    if ( nh_!=NULL ) // GET INFO FROM PARAMETER SERVER
    {
      if ( nh_->getParam(key,value) )
      {
        return true;
      }
      else
      {
        error_count_++;
        ROS_WARN("VELO Transmission: Couldn't load \"%s\" from parameter server, joint %s.", key, joint_name_);
        return false;
      }
    }
    else            // GET INFO FROM URDF
    {
      const char *attrib = j_->Attribute(key);
      if ( attrib==NULL )
      {
        error_count_++;
        ROS_WARN("VeloTransmission joint \"%s\" has no attribute: %s.", joint_name_, key);
      }
      else
      {
        try
        {
          value = boost::lexical_cast<double>(attrib);
          // RETURN successfully
          return true;
        }
        catch(boost::bad_lexical_cast &e)
        {
          error_count_++;
          ROS_ERROR("%s:(%s) is not a float", key, attrib);
        }
      }
      return false;
    }
  }

  // ERROR-CHECKING PARAM GETTER WITH CUSTOM MESSAGE.
  // Version that sets a default value if param is not defined
  bool getParam(const char *key, double &value, double defaultValue)
  {
    if ( !getParam(key,value) )
    {
      ROS_WARN("VeloTransmission joint \"%s\", attribute \"%s\" using default value: %f.", joint_name_, key, defaultValue);
      value = defaultValue;
      return false;
    }
    else
    {
      return true;
    }
  }

  bool setParam(const char *key, double &value)
  {
    if ( nh_!=NULL ) // GET INFO FROM PARAMETER SERVER
    {
      nh_->setParam(key,value);
    }
    return true;
  }

};


bool VeloTransmission::getItems(ParamFetcher *itemFetcher)
{
  // Load parameters from server that is initialized at instantiation of "itemFetcher" object.
  // Joints

  std::cout << "Init Parameters" << std::endl;

  // Links
  itemFetcher->getParam("links/l0", l0_);
  itemFetcher->getParam("links/l1", l1_);
  itemFetcher->getParam("links/l2", l2_);
  itemFetcher->getParam("links/thickness", thickness_);

  // Radii
  itemFetcher->getParam("radii/r_c0", r_c0_);
  itemFetcher->getParam("radii/r_c1", r_c1_);
  itemFetcher->getParam("radii/r_e0", r_e0_);
  itemFetcher->getParam("radii/r_e1", r_e1_);
  itemFetcher->getParam("radii/r_f1", r_f1_);

  // Spring
  itemFetcher->getParam("spring/k",  spring_k_);
  itemFetcher->getParam("spring/x0", spring_x0_);

  // Limits
  itemFetcher->getParam("limits/theta_open_deg",   theta_open_);
  theta_open_ *= DEG2RAD;    // CONVERT TO SI
  itemFetcher->getParam("limits/theta_closed_deg", theta_closed_);
  theta_closed_ *= DEG2RAD;  // CONVERT TO SI
  itemFetcher->getParam("limits/gap_closed",  gap_closed_);
  itemFetcher->getParam("limits/max_torque",  max_torque_);
  max_torque_ = fabs(max_torque_);

  // Actuator
  itemFetcher->getParam("actuator/screw_lead",    screw_lead_);
  itemFetcher->getParam("actuator/gear_reduction",gear_reduction_);
  itemFetcher->getParam("actuator/efficiency",    gripper_efficiency_);
  // ERROR-CHECK efficiency
  if (gripper_efficiency_ <= 0.0 || gripper_efficiency_ > 1.0)
    gripper_efficiency_ = 1.0;

  // Polynomial coefficients
  // USE tmp BECAUSE single element of vector<double> does not pass by reference.
  double tmp;
  length_to_gap_coeffs_.resize(5);
  itemFetcher->getParam("polynomials/l2g_0", tmp);  length_to_gap_coeffs_[0] = tmp;
  itemFetcher->getParam("polynomials/l2g_1", tmp);  length_to_gap_coeffs_[1] = tmp;
  itemFetcher->getParam("polynomials/l2g_2", tmp);  length_to_gap_coeffs_[2] = tmp;
  itemFetcher->getParam("polynomials/l2g_3", tmp);  length_to_gap_coeffs_[3] = tmp;
  itemFetcher->getParam("polynomials/l2g_4", tmp);  length_to_gap_coeffs_[4] = tmp;

  gap_to_length_coeffs_.resize(5);
  itemFetcher->getParam("polynomials/g2l_0", tmp);  gap_to_length_coeffs_[0] = tmp;
  itemFetcher->getParam("polynomials/g2l_1", tmp);  gap_to_length_coeffs_[1] = tmp;
  itemFetcher->getParam("polynomials/g2l_2", tmp);  gap_to_length_coeffs_[2] = tmp;
  itemFetcher->getParam("polynomials/g2l_3", tmp);  gap_to_length_coeffs_[3] = tmp;
  itemFetcher->getParam("polynomials/g2l_4", tmp);  gap_to_length_coeffs_[4] = tmp;

  // NOTE, FOR HISTORICAL REASONS, ceoffs g2fma are called g2ed 
  // (Flexor Moment Arm aka. Effective Distance)
  gap_to_fma_coeffs_.resize(5);
  itemFetcher->getParam("polynomials/g2ed_0", tmp);  gap_to_fma_coeffs_[0] = tmp;
  itemFetcher->getParam("polynomials/g2ed_1", tmp);  gap_to_fma_coeffs_[1] = tmp;
  itemFetcher->getParam("polynomials/g2ed_2", tmp);  gap_to_fma_coeffs_[2] = tmp;
  itemFetcher->getParam("polynomials/g2ed_3", tmp);  gap_to_fma_coeffs_[3] = tmp;
  itemFetcher->getParam("polynomials/g2ed_4", tmp);  gap_to_fma_coeffs_[4] = tmp;

  // INITIALIZE MAX GAP AND CORRESPONDING TENDON POSITION, CONSISTENT WITH theta_open_
  // gap_open_ USED IN getGapFromTendonLength(), MUST COMPUTE HARD-CODED HERE TO INITIALIZE IT.
  gap_open_    = 2.0 *(l0_ + l1_*cos(theta_open_) - thickness_);
  tendon_open_ = getTendonLengthFromGap( gap_open_ );
  itemFetcher->setParam("gap_open",    gap_open_);
  itemFetcher->setParam("tendon_open", tendon_open_);
  

  if ( itemFetcher->error_count_ > 0 )
  {
    ROS_WARN("itemFetcher error_count = %d",itemFetcher->error_count_);
  }
  return !((bool) itemFetcher->error_count_);
}


bool VeloTransmission::initParametersFromServer(TiXmlElement *j)
{
  itemFetcher_ = new ParamFetcher(j);
  if ( !getItems(itemFetcher_) )
  {
    return false;
  }

  return true;
}

bool VeloTransmission::initParametersFromURDF(TiXmlElement *j, Robot *robot)
{
  itemFetcher_ = new ParamFetcher(j,robot);
  if ( !getItems(itemFetcher_) )
  {
    return false;
  }

  const char *joint_name = itemFetcher_->getJointName();
  gap_joint_ = std::string(joint_name);
  joint_names_.push_back(joint_name);  // The first joint is the gap joint

  int argc = 0;
  char** argv = NULL;

  ros::init(argc, argv, gap_joint_);

  velo_state_publisher_.reset(
      new realtime_tools::RealtimePublisher<velo_controller::VeloTransmissionState>
        (*(itemFetcher_->nh_), "state", 1));

  return true;
}

bool VeloTransmission::initXml(TiXmlElement *config, Robot *robot)
{
  const char *name = config->Attribute("name");
  name_ = name ? name : "";
  //myfile.open("transmission_data.txt");
  TiXmlElement *ael = config->FirstChildElement("actuator");
  const char *actuator_name = ael ? ael->Attribute("name") : NULL;
  if (!actuator_name || !robot->getActuator(actuator_name))
  {
    ROS_ERROR("VeloTransmission could not find actuator named \"%s\"", actuator_name);
    return false;
  }
  robot->getActuator(actuator_name)->command_.enable_ = true;
  actuator_names_.push_back(actuator_name);

  for (TiXmlElement *j = config->FirstChildElement("gap_joint"); j; j = j->NextSiblingElement("gap_joint"))
  {
    if ( !(initParametersFromURDF(j, robot) || initParametersFromServer(j)) )
    {
      return false;
    }
  }


  // Get passive joint informations
  for (TiXmlElement *j = config->FirstChildElement("passive_joint"); j; j = j->NextSiblingElement("passive_joint"))
  {
    const char *joint_name = j->Attribute("name");
    if (!joint_name)
    {
      ROS_ERROR("PR2GripperTransmission did not specify joint name");
      return false;
    }
    const boost::shared_ptr<const urdf::Joint> joint = robot->robot_model_.getJoint(joint_name);

    if (!joint)
    {
      ROS_ERROR("PR2GripperTransmission could not find joint named \"%s\"", joint_name);
      return false;
    }

    // add joint name to list
    joint_names_.push_back(joint_name);  // Adds the passive joints after the gap joint
    passive_joints_.push_back(joint_name);
  }

  // Get screw joint informations
  for (TiXmlElement *j = config->FirstChildElement("simulated_actuated_joint"); j; j = j->NextSiblingElement("simulated_actuated_joint"))
  {
    const char *joint_name = j->Attribute("name");
    if (!joint_name)
    {
      ROS_ERROR("PR2GripperTransmission simulated_actuated_joint did snot specify joint name");
      use_simulated_actuated_joint_=false;
    }
    else
    {
      const boost::shared_ptr<const urdf::Joint> joint = robot->robot_model_.getJoint(joint_name);
      if (!joint)
      {
        ROS_ERROR("PR2GripperTransmission could not find joint named \"%s\"", joint_name);
        use_simulated_actuated_joint_=false;
      }
      else
      {
        use_simulated_actuated_joint_=true;
        joint_names_.push_back(joint_name);  // The first joint is the gap joint

        // get the thread pitch
        const char *simulated_reduction = j->Attribute("simulated_reduction");
        if (!simulated_reduction)
        {
          ROS_ERROR("PR2GripperTransmission's joint \"%s\" has no coefficient: simulated_reduction.", joint_name);
          return false;
        }
        try
        {
          simulated_reduction_ = boost::lexical_cast<double>(simulated_reduction);
        }
        catch (boost::bad_lexical_cast &e)
        {
          ROS_ERROR("simulated_reduction (%s) is not a float",simulated_reduction);
          return false;
        }

        // get any additional joint introduced from this screw joint implementation
        // for the gripper, this is due to the limitation that screw constraint
        // requires axis of rotation to be aligned with line between CG's of the two
        // connected bodies.  For this reason, an additional slider joint was introduced
        // thus, requiring joint state to be published for motion planning packages
        // and that's why we're here.
        const char *passive_actuated_joint_name = j->Attribute("passive_actuated_joint");
        if (passive_actuated_joint_name)
        {
          const boost::shared_ptr<const urdf::Joint> passive_actuated_joint = robot->robot_model_.getJoint(passive_actuated_joint_name);
          if (passive_actuated_joint)
          {
            has_simulated_passive_actuated_joint_ = true;
            joint_names_.push_back(passive_actuated_joint_name);  // The first joint is the gap joint
          }
        }

      }
    }
  }

  // assuming simulated gripper prismatic joint exists, use it

  return true;
}

bool VeloTransmission::initXml(TiXmlElement *config)
{
  const char *name = config->Attribute("name");
  name_ = name ? name : "";

  //myfile.open("transmission_data.txt");
  TiXmlElement *ael = config->FirstChildElement("actuator");
  const char *actuator_name = ael ? ael->Attribute("name") : NULL;

  if (!actuator_name)
  {
    ROS_ERROR("VeloTransmission could not find actuator named \"%s\"", actuator_name);
    return false;
  }
  actuator_names_.push_back(actuator_name);

  for (TiXmlElement *j = config->FirstChildElement("gap_joint"); j; j = j->NextSiblingElement("gap_joint"))
  {
    if ( !(initParametersFromServer(j)) )
    {
      return false;
    }
  }

  // Print all coefficients
  //ROS_DEBUG("Velo transmission parameters for %s: l0=%f, l1=%f, l2=%f, thickness=%f, theta_open=%f, theta_closed=%f, gear_reduction=%f",
  //name_.c_str(), l0_, l1_, l2_, thickness_, theta_open_, theta_closed_, gear_reduction_);

  // Get passive joint informations
  for (TiXmlElement *j = config->FirstChildElement("passive_joint"); j; j = j->NextSiblingElement("passive_joint"))
  {
    const char *joint_name = j->Attribute("name");
    if (!joint_name)
    {
      ROS_ERROR("VeloTransmission did not specify joint name");
      return false;
    }

    // add joint name to list
    joint_names_.push_back(joint_name);  // Adds the passive joints after the gap joint
    passive_joints_.push_back(joint_name);
  }

  // Get screw joint informations
  for (TiXmlElement *j = config->FirstChildElement("simulated_actuated_joint"); j; j = j->NextSiblingElement("simulated_actuated_joint"))
  {
    const char *joint_name = j->Attribute("name");
    if (!joint_name)
    {
      ROS_ERROR("VeloTransmission screw joint did not specify joint name");
      use_simulated_actuated_joint_=false;
    }
    else
    {
      use_simulated_actuated_joint_=true;
      joint_names_.push_back(joint_name);  // The first joint is the gap joint

      // get the thread pitch
      const char *simulated_reduction = j->Attribute("simulated_reduction");
      if (!simulated_reduction)
      {
        ROS_ERROR("VeloTransmission's joint \"%s\" has no coefficient: simulated_reduction.", joint_name);
        return false;
      }
      try
      {
        simulated_reduction_ = boost::lexical_cast<double>(simulated_reduction);
      }
      catch (boost::bad_lexical_cast &e)
      {
        ROS_ERROR("simulated_reduction (%s) is not a float",simulated_reduction);
        return false;
      }

      // get any additional joint introduced from this screw joint implementation
      // for the gripper, this is due to the limitation that screw constraint
      // requires axis of rotation to be aligned with line between CG's of the two
      // connected bodies.  For this reason, an additional slider joint was introduced
      // thus, requiring joint state to be published for motion planning packages
      // and that's why we're here.
      const char *passive_actuated_joint_name = j->Attribute("passive_actuated_joint");
      if (passive_actuated_joint_name)
      {
        has_simulated_passive_actuated_joint_ = true;
        joint_names_.push_back(passive_actuated_joint_name);  // The first joint is the gap joint
      }

    }
  }

  // if simulated gripper prismatic joint exists, use it
  if (config->FirstChildElement("use_simulated_gripper_joint")) use_simulated_gripper_joint = true;

  return true;
}


void VeloTransmission::assertJointConfig( size_t as_size, size_t js_size )
{
  // TODO - CHECK THESE. Suspect that the screw joint should be removed. Leave passive joints + gap joint.

  ROS_ASSERT(as_size == 1); // Only one actuator
  // js has passive joints and 1 gap joint and 1 screw joint
  if (use_simulated_actuated_joint_ && has_simulated_passive_actuated_joint_)
  { ROS_ASSERT(js_size == 1 + passive_joints_.size() + 2); }
  else if (use_simulated_actuated_joint_)
  { ROS_ASSERT(js_size == 1 + passive_joints_.size() + 1); }
  else
  { ROS_ASSERT(js_size == 1 + passive_joints_.size()); }
  //ROS_ASSERT(simulated_reduction_>0.0);
}


///////////////////////////////////////////////////////////
/// assign joint position, velocity, effort from actuator state; ie, Tendon length -> gripper gap.
/// all passive joints are assigned by single actuator state through mimic?
void VeloTransmission::propagatePosition(std::vector<Actuator*>& as, std::vector<JointState*>& js)
{
  assertJointConfig( as.size(),js.size() );

  double tendon_length  = as[0]->state_.position_ * motorGeom2TendonGeom();
  double tendon_vel     = as[0]->state_.velocity_ * motorGeom2TendonGeom();
  double motor_torque   = tqSign_ * as[0]->state_.last_measured_effort_;
  double tendon_force   = motor_torque * motorTorque2TendonForce();

  if ( js[0]->calibrated_ )
  {
    double gap_size   = getGapFromTendonLength(tendon_length);
    double gap_vel    = getGapVelFromTendonLengthVel(tendon_length, tendon_vel);

    // The state of the gap joint.
    js[0]->position_        = gap_size;
    js[0]->velocity_        = gap_vel; // each finger is moving with this velocity.
    js[0]->measured_effort_ = getGripperForceFromTendonForce(tendon_force, gap_size);

    // Determines the states of the passive joints.
    // we need to do this for each finger, in simulation, each finger has it's state filled out

    double joint_angle = getThetaFromGap(gap_size);
    double joint_vel = getThetaVelFromGapVel(gap_vel, gap_size);

    for (size_t i = 1; i < passive_joints_.size()+1; ++i) //
    {
      js[i]->position_           = joint_angle;
      if(i == 3 || i == 4)  // distal links open during closing.
        js[i]->position_ = -joint_angle;
      js[i]->velocity_           = joint_vel;
      js[i]->measured_effort_    = 1.0; // TODO: Old.MT / dtheta_dMR / RAD2REV;
    }
  }
  else
  {  // WHEN CALIBRATING, THE TRANSMISSION IS TO THE TENDON ie BALLSCREW
    js[0]->position_        = tendon_length;
    js[0]->velocity_        = tendon_vel;
    js[0]->measured_effort_ = tendon_force;

    double joint_angle = theta_open_;  // BOBH: just a placeholder for calibration
    double joint_vel   = 0.0;
    for (size_t i = 1; i < passive_joints_.size()+1; ++i) //
    {
      js[i]->position_           = joint_angle;
      if(i == 3 || i == 4)
        js[i]->position_ = -joint_angle;
      js[i]->velocity_           = joint_vel;
      js[i]->measured_effort_    = 1.0;// TODO: Old.MT / dtheta_dMR / RAD2REV;
    }
  }


  if (use_simulated_actuated_joint_)
  {
    // screw joint state is not important to us, fill with zeros
    js[passive_joints_.size()+1]->position_           = 0.0;
    js[passive_joints_.size()+1]->velocity_           = 0.0;
    js[passive_joints_.size()+1]->measured_effort_    = 0.0;
    js[passive_joints_.size()+1]->reference_position_ = 0.0;
    js[passive_joints_.size()+1]->calibrated_         = true; // treat passive simulation joints as "calibrated"
  }
  if (has_simulated_passive_actuated_joint_)
  {
    // screw joint state is not important to us, fill with zeros
    js[passive_joints_.size()+2]->position_           = 0.0;
    js[passive_joints_.size()+2]->velocity_           = 0.0;
    js[passive_joints_.size()+2]->measured_effort_    = 0.0;
    js[passive_joints_.size()+2]->reference_position_ = 0.0;
    js[passive_joints_.size()+2]->calibrated_         = true; // treat passive simulation joints as "calibrated"
  }
}

// this is needed for simulation, so we can recover encoder value given joint angles
// Use joint positions to generate an actuator position.
void VeloTransmission::propagatePositionBackwards(std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
  assertJointConfig( as.size(),js.size() );

  // if(loop_count_ % 1650 == 0)
  // {
  //   ROS_WARN("Read js[0]: %f, js[1]: %f, js[2]: %f, js[3] %f, js[4] %f", 
  //             js[0]->position_, js[1]->position_*RAD2DEG, js[2]->position_*RAD2DEG, 
  //             js[3]->position_*RAD2DEG, js[4]->position_*RAD2DEG);
  // }

  if ( js[0]->calibrated_ )
  {
    double theta       = -js[2]->position_ - theta_closed_; // Proximal joint angle, radians
    double theta_vel   =  js[2]->velocity_;
    //  double torqueJ1     = js[3]->commanded_effort_; // Joints 3/4 are the distal joints.
    double gap_force   = -js[0]->commanded_effort_;  // Negative effort makes gap smaller.

    double gap_size      = getGapFromTheta(theta);
    double tendon_length = getTendonLengthFromGap(gap_size);
    double motor_pos     = tendon_length * tendonGeom2MotorGeom();

    double gap_rate      = theta_vel*cos(theta);
    double tendon_rate   = getTendonLengthVelFromGapVel(gap_rate, gap_size);
    double motor_vel     = tendon_rate * tendonGeom2MotorGeom();

    double tendon_force    = getTendonForceFromGripperForce(gap_force, gap_size);
    double motor_torque    = tendon_force * tendonForce2MotorTorque();

    as[0]->state_.position_             = motor_pos;
    as[0]->state_.velocity_             = motor_vel;
    as[0]->state_.last_measured_effort_ = tqSign_ * motor_torque;
  }
  else
  {  /* WHEN CALIBRATING, THE TRANSMISSION IS TO TENDON ie BALLSCREW */
    as[0]->state_.position_             = js[0]->position_ * tendonGeom2MotorGeom();
    as[0]->state_.velocity_             = js[0]->velocity_ * tendonGeom2MotorGeom();
    as[0]->state_.last_measured_effort_ = tqSign_ * js[0]->commanded_effort_ * tendonForce2MotorTorque();
  }

  // Update the timing (making sure it's initialized).
  if (! simulated_actuator_timestamp_initialized_)
  {
    // Set the time stamp to zero (it is measured relative to the start time).
    as[0]->state_.sample_timestamp_ = ros::Duration(0);

    // Try to set the start time.  Only then do we claim initialized.
    if (ros::isStarted())
    {
      simulated_actuator_start_time_ = ros::Time::now();
      simulated_actuator_timestamp_initialized_ = true;
    }
  }
  else
  {
    // Measure the time stamp relative to the start time.
    as[0]->state_.sample_timestamp_ = ros::Time::now() - simulated_actuator_start_time_;
  }
  // Set the historical (double) timestamp accordingly.
  as[0]->state_.timestamp_ = as[0]->state_.sample_timestamp_.toSec();

  // simulate calibration sensors by filling out actuator states
  this->joint_calibration_simulator_.simulateJointCalibration(js[0],as[0]);
}

void VeloTransmission::propagateEffort(
    std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
  assertJointConfig( as.size(),js.size() );

  double gap_effort, gap_size;
  double tendon_force, motor_torque;
  double tendon_length, motor_pos;

  if ( js[0]->calibrated_ )
  {
   
    if ( mode_ != RUNNING )
    { if ( mode_ == CALIBRATING )
      { /* Mute the actuator output of the transmission when switching from a
           calibration-controller to joint-controller since there can be a 1-2
           second gap between the end of the calibration-controller and
           starting of the joint-controller.
         */
        mute_timeout_ = ros::Time::now() + ros::Duration(8.5);
        mode_ = MUTE;
      }
      else if ( mute_timeout_ < ros::Time::now() )
      { mode_ = RUNNING;
      }
    }
    /* A user wants to specify gap_effort such that positive means "squeezing".
       We apropriately use the "controls" sense here (in the controller);
       such that positive gap_effort increases the gap and negative gap_effort
       makes the gap smaller
    */ 
    gap_effort = -js[0]->commanded_effort_; // user's positive command means reduce gap.
    gap_size   =  js[0]->position_;

    tendon_force  = getTendonForceFromGripperForce(gap_effort, gap_size);

    tendon_length = getTendonLengthFromGap(gap_size);
    motor_pos     = tendon_length * tendonGeom2MotorGeom();
  }
  else
  {
    /* NOTE: When calibrating, the transmission is to tendon ie ballscrew */
    tendon_force  =  js[0]->commanded_effort_; // <-- USED TO SET motor_torque

    /* Fill out the unused, so we can publish/plot during calibration too */
    gap_effort    =  js[0]->commanded_effort_;
    gap_size      =  js[0]->position_;
    tendon_length =  js[0]->position_;
    motor_pos     =  as[0]->state_.position_;
    mode_         =  CALIBRATING;
  }

  /* SET ACTUATOR VALUES */
  if ( mode_ == MUTE )
    motor_torque = 0.0;
  else
    motor_torque = tendon_force * tendonForce2MotorTorque();

  motor_torque = std::max(-max_torque_, std::min(motor_torque, max_torque_));
  as[0]->command_.enable_ = true;
  as[0]->command_.effort_ = tqSign_ * motor_torque;

  if(++loop_count_ % 10 == 0 &&
     velo_state_publisher_ &&
     velo_state_publisher_->trylock() )
  {
    velo_state_publisher_->msg_.header.stamp = ros::Time::now();
    velo_state_publisher_->msg_.gap_size = gap_size;
    velo_state_publisher_->msg_.tendon_position = tendon_length;
    velo_state_publisher_->msg_.motor_position = motor_pos;
    velo_state_publisher_->msg_.gap_force = gap_effort;
    velo_state_publisher_->msg_.tendon_force = tendon_force;
    velo_state_publisher_->msg_.motor_torque = motor_torque;

    velo_state_publisher_->unlockAndPublish();
  }

}


void VeloTransmission::propagateEffortBackwards(
  std::vector<Actuator*>& as, std::vector<JointState*>& js)
{
  assertJointConfig( as.size(),js.size() );

  if ( js[0]->calibrated_ )
  {
    // gap_size is required to compute the effective distance from the tendon to the J0 joint
    double tendon_length = as[0]->state_.position_ * motorGeom2TendonGeom();
    double gap_size      = getGapFromTendonLength(tendon_length);

    double motor_torque  = tqSign_ * as[0]->command_.effort_;
    double tendon_force  = motor_torque * motorTorque2TendonForce();
    double gap_effort    = getGripperForceFromTendonForce(tendon_force, gap_size);

    // propagate fictitious joint effort backwards
    // ROS_ERROR("prop eff back eff=%f",js[0]->commanded_effort_);
    if (use_simulated_actuated_joint_)
    {
      // set screw joint effort if simulated
      js[passive_joints_.size()+1]->commanded_effort_  = gap_effort/simulated_reduction_;
      //js[0]->commanded_effort_                         = gap_effort/2.0;
      //ROS_INFO("propagateEffortBackwards(): js[0]->commanded_effort = %f", gap_effort/simulated_reduction_);
    }
    else
    {
      // an ugly hack to lessen instability due to gripper gains
      double eps=0.01;
      js[0]->commanded_effort_  = (1.0-eps)*js[0]->commanded_effort_ + eps*gap_effort/2.0; // skip slider joint effort
    }
  }
  else
  {
    /* WHEN CALIBRATING, THE TRANSMISSION IS TO TENDON ie BALLSCREW */
    js[0]->commanded_effort_  = tqSign_ * as[0]->command_.effort_ * motorTorque2TendonForce();
  }
}

double VeloTransmission::motorGeom2TendonGeom()
{
  double tendon_qty = RAD2REV / gear_reduction_ * screw_lead_;
  return tendon_qty;
}

double VeloTransmission::tendonGeom2MotorGeom()
{
  return 1.0/motorGeom2TendonGeom();
}

double VeloTransmission::tendonForce2MotorTorque()
{
  return motorGeom2TendonGeom();
}

double VeloTransmission::motorTorque2TendonForce()
{
  return 1.0/motorGeom2TendonGeom();
}



double VeloTransmission::getGapFromTheta(double theta)
{ // The gap spacing is defined by proximal joint angle theta, 
  // the width of the palm (from l0), and thickness of the distal link.
  theta = std::max(theta,theta_open_);
  double gap = 2.0 * (l0_ + l1_*cos(theta) - thickness_);
  return gap;
}

double VeloTransmission::getThetaFromGap(double gap)
{
  static int count = 0;

  // IF gap IS "LARGER", THEN TENDONS ARE SLACK, BUT SET THETA --> theta_open_
  gap = std::min(gap,gap_open_);
  double x   = gap/2.0 + thickness_- l0_;
  double arg = x/l1_;

  if ( fabs(arg) > 1.0 )
  {
    if ( ++count % 1000 == 0 ) {
    ROS_ERROR("GetThetaFromGap invalid - trying to get acos of %.1g", arg);
    ROS_WARN("gap: %.3f \tl0_: %.4f \tgap_open: %.4f \tl1: %.4f \targ: %f", gap, l0_, gap_open_, l1_, arg);
    count=0;
    }
    arg = copysign(0.999999,arg);
  }
  double theta = acos(arg);
  return theta;
}

double VeloTransmission::getTendonLengthFromGap(double gap)
{
  double length = 0.0;
  if ( gap <= gap_open_ )  // USE POLYNOMIAL FIT WHERE VALID
  {
    for (int i = 0; i < (int)gap_to_length_coeffs_.size(); i++)
      length += gap_to_length_coeffs_[i] * pow(gap,i);
  }
  else   // LINEARIZE BEYOND MAX GAP
  {
    length = tendon_open_/gap_open_ * gap;
  } 

  return length;
}

double VeloTransmission::getGapFromTendonLength(double length)
{
  double gap = 0.0;
  if ( length <= tendon_open_ )  // USE POLYNOMIAL FIT WHERE VALID
  {
    for (int i = 0; i < (int)length_to_gap_coeffs_.size(); i++)
      gap += length_to_gap_coeffs_[i] * pow(length,i);
  }
  else   // LINEARIZE BEYOND MAX GAP
  {
    gap = gap_open_/tendon_open_ * length;
  }

  return gap;
}

double VeloTransmission::dGap_dLength(double length)
{
  double dG_dL = 0.0;

  if ( 0 < length && length <= tendon_open_ )  // USE POLYNOMIAL FIT WHERE VALID
  {
    // Calculate dGap/dLength
    for (int i = 1; i < (int)length_to_gap_coeffs_.size(); i++)
      dG_dL += i * (length_to_gap_coeffs_[i] * pow(length, i-1));
  }
  else   // CONSTANT LINEAR SLOPE BEYOND MAX GAP
  {
    dG_dL = gap_open_/tendon_open_;
  }

  return dG_dL;
}

double VeloTransmission::getGapVelFromTendonLengthVel(double length, double length_vel)
{
  // dGap/dt = dGap/dLen * dLen/dt
  double gap_vel = dGap_dLength(length) * length_vel;
  return gap_vel;
}

double VeloTransmission::dLength_dGap(double gap)
{
  double dL_dG = 0.0;

  if ( 0 < gap && gap <= gap_open_ )  // USE POLYNOMIAL FIT WHERE VALID
  {
    // Calculate dLength/dGap
    for (int i = 1; i < (int)gap_to_length_coeffs_.size(); i++)
      dL_dG += i * gap_to_length_coeffs_[i] * pow(gap, i-1);
  }
  else   // CONSTANT LINEAR SLOPE BEYOND MAX GAP
  {
    dL_dG = tendon_open_/gap_open_;
  }

  return dL_dG;
}

double VeloTransmission::getTendonLengthVelFromGapVel(double gap_vel, double gap)
{
  // dLen/dt = dLen/dGap * dGap/dt
  double length_vel = dLength_dGap(gap) * gap_vel;
  return length_vel;
}

double VeloTransmission::getThetaVelFromGapVel(double gap_vel, double gap)
{
  double v = gap_vel/2.0;
  double theta = getThetaFromGap(gap);
  double theta_vel = v * sin(theta) / l1_;

  return theta_vel;
}

double VeloTransmission::getFlexorMomentArm(double gap)
{
  double fma = 0.0;
  if ( gap <= gap_open_ )  // USE POLYNOMIAL FIT WHERE VALID
  {
    for (int i = 0; i < (int)gap_to_fma_coeffs_.size(); i++)
      fma += gap_to_fma_coeffs_[i] * pow(gap,i);
  }
  else // AVOID DISCONTINUITY.  AVOID SIGN CHANGE OF (r_f1_-r_f0_).
  {    // MAKE CONSTANT BEYOND FULLY OPEN.
    fma = getFlexorMomentArm(gap_open_);
    //fma = 0.0;
  } 

  return fma;
}

double VeloTransmission::getGripperForceFromTendonForce(double tendon_force, double gap)
{
  double c = r_c1_/r_c0_; // Ratio of contraint radii.  Usually 1.0
  double theta = getThetaFromGap(gap);
  // ONLY APPLY efficency TO Tendon-->Grip FORCE, NOT EXTENSOR FORCE.
  double Fe = getExtensorTendonForce(theta);
  double Ff = tendon_force*gripper_efficiency_/2.0; // two fingers.

  r_f0_ = getFlexorMomentArm(gap);
  r_g0_ = l2_/2.0 + l1_*sin(theta); // Assume force applied to middle of distal link.
  r_g1_ = l2_/2.0;

  double Fg = ( Fe*(r_e1_-c*r_e0_) + Ff*(c*r_f0_-r_f1_) ) / (r_g1_ - c*r_g0_);
  return Fg;
}

double VeloTransmission::getTendonForceFromGripperForce(double gripper_force, double gap)
{
  double c = r_c1_/r_c0_; // Ratio of contraint radii.  Usually 1.0
  double theta = getThetaFromGap(gap);
  // ONLY APPLY efficency TO Grip-->Tendon FORCE, NOT EXTENSOR FORCE.
  double Fe = getExtensorTendonForce(theta);
  double Fg = gripper_force/gripper_efficiency_;

  r_f0_ = getFlexorMomentArm(gap);
  r_g0_ = l2_/2.0 + l1_*sin(theta); // Assume force applied to the middle of the distal link.
  r_g1_ = l2_/2.0;

  double Ff = 2.0* (Fe*(r_e1_-c*r_e0_) + Fg*(c*r_g0_-r_g1_)) / (r_f1_ - c*r_f0_);
  return Ff; // Double the result as the motor force is split between the two tendons
}

double VeloTransmission::getExtensorTendonForce(double theta)
{
  theta = std::max(theta,theta_open_);  // Don't let current theta go less than open_
  double delta_theta = theta - theta_open_; // change in angle from fully open
  double spring_x = delta_theta*(r_e0_ - r_e1_) + spring_x0_; // spring extension + preload
  double ext_force = spring_k_*spring_x;

  return ext_force;
}

double VeloTransmission::getMotorQtyFromEncoderQty(double encQty)
{
  double motorQty = encQty*REV2RAD;
  return motorQty;
}

double VeloTransmission::getEncoderQtyFromMotorQty(double motorQty)
{
  double encQty = motorQty*RAD2REV;
  return encQty;
}

double VeloTransmission::validateGapSize(double gap)
{
  gap = std::max(gap_closed_,std::min(gap,gap_open_));
  return gap;
}

}
