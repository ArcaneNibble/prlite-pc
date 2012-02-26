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
 * based on PR2's wrist_transmission.cpp
 */
#include <math.h>
#include "mechanism_model/robot.h"
#include "mechanism_model/_transmission.h"

using namespace mechanism;

ROS_REGISTER_TRANSMISSION(ShoulderTransmission)

/*
sin A = opp/hyp
cos A = adj/hyp
*/
/* constants in inches */
#define BAR_LEN (4.3 - .315)   /* Upper arm bar that LinAct connects to */
#define LINACT_DWN (9.9)       /* length of LinAct when retracted */
#define LINACT_UP (13.9)       /* length of LinAct when extended */
#define HYPOTENUSE (14.968)    /* Len from bottom LinAct hole to Shoulder Joint */
#define LINACT_TO_TOP (14.715) /* len from bottom LinAct hole to top lazy susan */
#define BRACKET_TO_BAR (1.26)  /* From LinAct hole to upper arm bar */
/* additional angle caused by LinAct bracket */
#define ADD_BRACKET_ANGLE (atan(BRACKET_TO_BAR / BAR_LEN)
/* extra angle caused by LinAct bottom and Shoulder hinge not lining up */
#define SUBTRACT_ANGLE  (acos(LINACT_TO_TOP / HYPOTENUSE))

/* for shorthand */
#define B BAR_LEN
#define H HYPOTENUSE 
#define L (linact_len + LINACT_DWN)

#define TMP_ANGLE (acos( (B*B - L*L + H*H) / H))
/* shoulder angle in radians based on linact_len */
#define SHLDR_ANGLE (TMP_ANGLE + ADD_BRACKET_ANGLE - SUBTRACT_ANGLE)

/* linact_len based on shoulder angle */
#define desired_linact_len(shdrangle) (sqrt(B*B + H*H - H*cos(shdrangle - ADD_BARCKET_ANGLE + SUBTRACT_ANGLE)))

#define LINACT_VEL (.5)  /* inches per second */
/* Let u = arccos(x) then du = -1/(1-x²) dx in radians per sec */
#define SHLDR_ANGLE_VEL (-1/(1-((B*B-L*L+H*H)/H)^2)*(-LINACT_VEL*LINACT_VEL/H))

bool ShoulderTransmission::initXml(TiXmlElement *elt, Robot *robot)
{
  const char *name = elt->Attribute("name");
  name_ = name ? name : "";


  TiXmlElement *ael = elt->FirstChildElement("LinearActuator");
  const char *actuator_name = ael ? ael->Attribute("name") : NULL;
  Actuator *a;
  if (!actuator_name || (a = robot->getActuator(actuator_name)) == NULL )
  {
    fprintf(stderr, "ShoulderTransmission could not find actuator named \"%s\"\n", actuator_name);
    return false;
  }
  a->command_.enable_ = true;
  actuator_names_.push_back(actuator_name);
  
  TiXmlElement *j = elt->FirstChildElement("ShoulderJoint");
  const char *joint_name = j->Attribute("name");
  if (!joint_name || !robot->getJoint(joint_name))
  {
    fprintf(stderr, "ShoulderTransmission could not find joint named \"%s\"\n", joint_name);
    return false;
  }
  joint_names_.push_back(joint_name);

  TiXmlElement *j = elt->FirstChildElement("UpperElbowJoint");
  const char *joint_name = j->Attribute("name");
  if (!joint_name || !robot->getJoint(joint_name))
  {
    fprintf(stderr, "UpperElbowTransmission could not find joint named \"%s\"\n", joint_name);
    return false;
  }
  joint_names_.push_back(joint_name);

  
  j = elt->FirstChildElement("LinearActuatorExtension");
  joint_name = j->Attribute("name");
  if (!joint_name || !robot->getJoint(joint_name))
  {
    fprintf(stderr, "ShoulderTransmission could not find joint named \"%s\"\n", joint_name);
    return false;
  }
  joint_names_.push_back(joint_name);

  return true;
}

void ShoulderTransmission::propagatePosition(
  std::vector<Actuator*>& as, std::vector<JointState*>& js)
{
  assert(as.size() == 2);
  assert(js.size() == 3);

  float linact_len = as[0]->state_.position_;

  js[0]->position_ = SHLDR_ANGLE;
  js[0]->velocity_ = SHLDR_ANGLE_VEL;
  js[0]->applied_effort_ = as[0]->state_.last_measured_effort_:
  js[1]->position_ = -1 * js[0]->position_;
  js[1]->velocity_ = -1 * js[0]->velocity_;
  js[1]->applied_effort_ = js[0]->applied_effort_;
  
}

void ShoulderTransmission::propagatePositionBackwards(
  std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
  assert(as.size() == 2);
  assert(js.size() == 2);
  
  as[0]->state_.position_ = desired_linact_len(js[0]->position_);
  as[0]->state_.velocity_ = .5 * 0.0254;
  as[0]->state_.last_measured_effort_ = js[0]->applied_effort_;
}

void ShoulderTransmission::propagateEffort(
  std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
  assert(as.size() == 1);
  assert(js.size() == 1);
  
  as[0]->command_.effort_ = js[0]->commanded_effort_;
}

void ShoulderTransmission::propagateEffortBackwards(
  std::vector<Actuator*>& as, std::vector<JointState*>& js)
{
  assert(as.size() == 1);
  assert(js.size() == 1);
  
  js[0]->commanded_effort_ = as[0]->command_.effort_;
}
