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
/*
 * Author: Melonee Wise
 */
#include <math.h>
#include "mechanism_model/robot.h"
#include "mechanism_model/wrist_transmission.h"

using namespace mechanism;

ROS_REGISTER_TRANSMISSION(WristTransmission)

bool WristTransmission::initXml(TiXmlElement *elt, Robot *robot)
{
  const char *name = elt->Attribute("name");
  name_ = name ? name : "";


  TiXmlElement *ael = elt->FirstChildElement("rightActuator");
  const char *actuator_name = ael ? ael->Attribute("name") : NULL;
  Actuator *a;
  if (!actuator_name || (a = robot->getActuator(actuator_name)) == NULL )
  {
    fprintf(stderr, "WristTransmission could not find actuator named \"%s\"\n", actuator_name);
    return false;
  }
  a->command_.enable_ = true;
  actuator_names_.push_back(actuator_name);
 
  ael = elt->FirstChildElement("leftActuator");
  actuator_name = ael ? ael->Attribute("name") : NULL;
  if (!actuator_name || (a = robot->getActuator(actuator_name)) == NULL )
  {
    fprintf(stderr, "WristTransmission could not find actuator named \"%s\"\n", actuator_name);
    return false;
  }
  a->command_.enable_ = true;
  actuator_names_.push_back(actuator_name);
   
  TiXmlElement *j = elt->FirstChildElement("flexJoint");
  const char *joint_name = j->Attribute("name");
  if (!joint_name || !robot->getJoint(joint_name))
  {
    fprintf(stderr, "WristTransmission could not find joint named \"%s\"\n", joint_name);
    return false;
  }
  joint_names_.push_back(joint_name);
  const char *joint_red = j->Attribute("mechanicalReduction");
  if (!joint_red)
  {
    fprintf(stderr, "WristTransmission's joint \"%s\" was not given a reduction.\n", joint_name);
    return false;
  }
  mechanical_reduction_.push_back(atof(joint_red));
 
  j = elt->FirstChildElement("rollJoint");
  joint_name = j->Attribute("name");
  if (!joint_name || !robot->getJoint(joint_name))
  {
    fprintf(stderr, "WristTransmission could not find joint named \"%s\"\n", joint_name);
    return false;
  }
  joint_names_.push_back(joint_name);
  joint_red = j->Attribute("mechanicalReduction");
  if (!joint_red)
  {
    fprintf(stderr, "WristTransmission's joint \"%s\" was not given a reduction.\n", joint_name);
    return false;
  }
  mechanical_reduction_.push_back(atof(joint_red));
 

  return true;
}

void WristTransmission::propagatePosition(
  std::vector<Actuator*>& as, std::vector<JointState*>& js)
{
  assert(as.size() == 2);
  assert(js.size() == 2);

  js[0]->position_ = (as[0]->state_.position_ / mechanical_reduction_[0] - as[1]->state_.position_ / mechanical_reduction_[1])/2;
  js[0]->velocity_ = (as[0]->state_.velocity_ / mechanical_reduction_[0] - as[1]->state_.velocity_ / mechanical_reduction_[1])/2;
  js[0]->applied_effort_ = (as[0]->state_.last_measured_effort_ * mechanical_reduction_[0] - as[1]->state_.last_measured_effort_ * mechanical_reduction_[1])/2;
 
  js[1]->position_ = (-as[0]->state_.position_ / mechanical_reduction_[0] - as[1]->state_.position_ / mechanical_reduction_[1])/2;
  js[1]->velocity_ = (-as[0]->state_.velocity_ / mechanical_reduction_[0] - as[1]->state_.velocity_ / mechanical_reduction_[1])/2;
  js[1]->applied_effort_ = (-as[0]->state_.last_measured_effort_ * mechanical_reduction_[0] - as[1]->state_.last_measured_effort_ * mechanical_reduction_[1])/2;
}

void WristTransmission::propagatePositionBackwards(
  std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
  assert(as.size() == 2);
  assert(js.size() == 2);
 
  as[0]->state_.position_ = ((js[0]->position_ - js[1]->position_) * mechanical_reduction_[0]);
  as[0]->state_.velocity_ = ((js[0]->velocity_ - js[1]->velocity_) * mechanical_reduction_[0]);
  as[0]->state_.last_measured_effort_ = (js[0]->applied_effort_ - js[1]->applied_effort_) /(mechanical_reduction_[0]);
 
  as[1]->state_.position_ = ((-js[0]->position_ - js[1]->position_) * mechanical_reduction_[1]);
  as[1]->state_.velocity_ = ((-js[0]->velocity_ - js[1]->velocity_) * mechanical_reduction_[1]);
  as[1]->state_.last_measured_effort_ = (-js[0]->applied_effort_ - js[1]->applied_effort_) /(mechanical_reduction_[1]);
}

void WristTransmission::propagateEffort(
  std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
  assert(as.size() == 2);
  assert(js.size() == 2);
 
  as[0]->command_.effort_ = (js[0]->commanded_effort_ - js[1]->commanded_effort_) /(mechanical_reduction_[0]);
  as[1]->command_.effort_ = (-js[0]->commanded_effort_ - js[1]->commanded_effort_) /(mechanical_reduction_[1]);
}

void WristTransmission::propagateEffortBackwards(
  std::vector<Actuator*>& as, std::vector<JointState*>& js)
{
  assert(as.size() == 2);
  assert(js.size() == 2);
 
  js[0]->commanded_effort_ = (as[0]->command_.effort_ * mechanical_reduction_[0] - as[1]->command_.effort_ * mechanical_reduction_[1])/2;
  js[1]->commanded_effort_ = (-as[0]->command_.effort_ * mechanical_reduction_[0] - as[1]->command_.effort_ * mechanical_reduction_[1])/2;
}

