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

#include <iostream>
#include "velo_controller/velo_transmission.h"

#define FORCE_TOL		0.1 // N
#define ED_TOL			0.0001 // mm
#define GAP_TOL 		0.0025 // mm
#define THETA_TOL		0.1 // rad
#define LENGTH_TOL		0.0005 // mm

#define ED_CLOSED		0.0133716 // mm
#define ED_MID			0.0108267 // mm
#define ED_OPEN			0.0072319 // mm

#define GAP_CLOSED		0.0021622// m
#define GAP_MID 		0.0830000 // m --> 60 deg theta
#define GAP_OPEN		0.1357631 // m


#define THETA_OPEN		20.0 * M_PI/180.0 // deg
#define THETA_MID		60.0 * M_PI/180.0 //deg
#define THETA_CLOSED	100.0 * M_PI/180.0 // deg

#define LENGTH_OPEN		0.0 	// m
#define LENGTH_MID		-0.0041172 // m
#define LENGTH_CLOSED	-0.0104045 // m

#define GF_OPEN			0.8244836 // N, given a 20N tendon load
#define GF_MID			3.1254007 // N
#define GF_CLOSED		4.3894740 // N, given a 20N tendon load

#define GRIPPER_FORCE	20.0 // N
#define TENDON_FORCE	20.0 // N


using namespace std;
using namespace pr2_mechanism_model;

namespace velo_controller {

class VeloTransmissionTest : public VeloTransmission
{
public:
	int mytest();
};

int VeloTransmissionTest::mytest()
{
	cout << "VeloTransmission tests:" << endl;

	// Check init functions
	TiXmlElement* config =  new TiXmlElement("config");
	config->SetAttribute("name", "test_config");
	
	TiXmlElement* actuator = new TiXmlElement("actuator");
	actuator->SetAttribute("name", "test_actuator");
	
	TiXmlElement* gap_joint = new TiXmlElement("gap_joint");
	gap_joint->SetAttribute("name", "l_gripper_joint");
	
	config->LinkEndChild(actuator);
	config->LinkEndChild(gap_joint);
	
	initXml(config);
	
	// Check kinematic mapping functions
	
	// Gap -> Effective distance mapping
/*	double effective_dist = 0.0;
	cout << "GAP -> EFFECTIVE DISTANCE" << endl;
	effective_dist = getFlexorMomentArm(GAP_CLOSED);
	cout << "Gap size: " << GAP_CLOSED << " ---> effective_dist " << effective_dist << endl;
	assert(effective_dist > (ED_CLOSED - ED_TOL) && effective_dist < (ED_CLOSED + ED_TOL));
	effective_dist = getFlexorMomentArm(GAP_MID);
	cout << "Gap size: " << GAP_MID << " ---> effective_dist " << effective_dist << endl;
	assert(effective_dist > (ED_MID - ED_TOL) && effective_dist < (ED_MID + ED_TOL));
	effective_dist = getFlexorMomentArm(GAP_OPEN);
	cout << "Gap size: " << GAP_OPEN << " ---> effective_dist " << effective_dist << endl;
	assert(effective_dist > (ED_OPEN - ED_TOL) && effective_dist < (ED_OPEN + ED_TOL));		*/
	
	// Gap ---> theta
	double theta_closed = getThetaFromGap(GAP_CLOSED);
	cout << "Gap: " << GAP_CLOSED << " ---> Theta " << theta_closed << endl; 
	assert (theta_closed > (THETA_CLOSED - THETA_TOL) && theta_closed < (THETA_CLOSED + THETA_TOL) );
	double theta_mid = getThetaFromGap(GAP_MID);
	cout << "Gap: " << GAP_MID << " ---> Theta " << theta_mid << endl;
	assert (theta_mid > (THETA_MID - THETA_TOL) && theta_mid < (THETA_MID + THETA_TOL) );
	double theta_open = getThetaFromGap(GAP_OPEN);
	cout << "Gap: " << GAP_OPEN << " ---> Theta " << theta_open << endl;
	assert (theta_open > (THETA_OPEN - THETA_TOL) && theta_open < (THETA_OPEN + THETA_TOL) );
	
	// Theta ---> gap
	double gap_closed = getGapFromTheta(THETA_CLOSED);
	cout << "Theta: " << THETA_CLOSED << " ---> Gap: " << gap_closed << endl;
	assert (gap_closed > (GAP_CLOSED - GAP_TOL) && gap_closed < (GAP_CLOSED + GAP_TOL) );
	double gap_mid = getGapFromTheta(THETA_MID);
	cout << "Theta: " << THETA_MID << " ---> Gap: " << gap_mid << endl;
	assert (gap_mid > (GAP_MID - GAP_TOL) && gap_mid < (GAP_MID + GAP_TOL));
	double gap_open = getGapFromTheta(THETA_OPEN);
	cout << "Theta: " << THETA_OPEN << " ---> Gap: " << gap_open << endl;
	assert (gap_open > (GAP_OPEN - GAP_TOL) && gap_open < (GAP_OPEN + GAP_TOL) );
	
	// Tendon length --> Gap
	gap_closed = getGapFromTendonLength(LENGTH_CLOSED);
	cout << "Length: " << LENGTH_CLOSED << " ---> Gap: " << gap_closed << endl;
	assert (gap_closed > (GAP_CLOSED - GAP_TOL) && gap_closed < (GAP_CLOSED + GAP_TOL) );
	gap_mid = getGapFromTendonLength(LENGTH_MID);
	cout << "Length: " << LENGTH_MID << " ---> Gap: " << gap_mid << endl;
	assert (gap_mid > (GAP_MID - GAP_TOL) && gap_mid < (GAP_MID + GAP_TOL));
	gap_open = getGapFromTendonLength(LENGTH_OPEN);
	cout << "Length: " << LENGTH_OPEN << " ---> Gap: " << gap_open << endl;
	assert (gap_open > (GAP_OPEN - GAP_TOL) && gap_open < (GAP_OPEN + GAP_TOL) );
	
	// Gap --> Tendon length
	double length_closed = getTendonLengthFromGap(GAP_CLOSED);
	cout << "Gap: " << GAP_CLOSED << " ---> Length: " << length_closed << endl;
	assert (length_closed > (LENGTH_CLOSED - LENGTH_TOL) && length_closed < (LENGTH_CLOSED + LENGTH_TOL) );
	double length_mid = getTendonLengthFromGap(GAP_MID);
	cout << "Gap: " << GAP_MID << " ---> Length: " << length_mid << endl;
	assert (length_mid > (LENGTH_MID - LENGTH_TOL) && length_mid < (LENGTH_MID + LENGTH_TOL) );
	double length_open = getTendonLengthFromGap(GAP_OPEN);
	cout << "Gap: " << GAP_OPEN << " ---> Length: " << length_open << endl;
	assert (length_open > (LENGTH_OPEN - LENGTH_TOL) && length_open < (LENGTH_OPEN + LENGTH_TOL) );
	
	// TODO: Motor pos to tendon length
	double motor_open = LENGTH_CLOSED * tendonGeom2MotorGeom();
	
	cout << "Motor_open: " << motor_open << endl;
	// TODO: Tendon length to motor pos
	
	// TODO: Motor torque to tendon force
	double mt_20 = 20.0 * tendonForce2MotorTorque();
	cout << "MT_20: " << mt_20 << endl;
	
	// TODO: Tendon force to motor torque

	double Ff = getTendonForceFromGripperForce(10.0, 0.06);
	cout << "Ff @ 10N: " << Ff << endl;
	double Mt = Ff * tendonForce2MotorTorque();
	cout << "Mt @ 10N: " << Mt << endl;
	
	cout << "Done" << endl;

	return 0;
}
} // END namespace

int main()
{
  velo_controller::VeloTransmissionTest velo_trans;
  velo_trans.mytest();
}
