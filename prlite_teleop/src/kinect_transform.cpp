/*****************************************************************************
*                                                                            *
*  This file is based off of a code file inside OpenNI.  The modification    *
*  reads specific joints from the skeleton source and publishes them to      *
*  Robot Operating System's Transform System for viewing inside of RViz      *
*                                                                            *
*  Author(s): Nathaniel Lewis (2011)                                         *
*                                                                            *
*  OpenNI is free software: you can redistribute it and/or modify            *
*  it under the terms of the GNU Lesser General Public License as published  *
*  by the Free Software Foundation, either version 3 of the License, or      *
*  (at your option) any later version.                                       *
*                                                                            *
*  OpenNI is distributed in the hope that it will be useful,                 *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of            *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              *
*  GNU Lesser General Public License for more details.                       *
*                                                                            *
*  You should have received a copy of the GNU Lesser General Public License  *
*  along with OpenNI. If not, see <http://www.gnu.org/licenses/>.            *
*                                                                            *
*****************************************************************************/

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <ros/ros.h>
#include <cmath>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <prlite_kinematics/SphereCoordinate.h>
#include <prlite_kinematics/some_status_thing.h>
#include "prlite_ax12controller.h"

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------

// #define SAMPLE_XML_PATH "/ros/stacks/ni/openni/lib/SamplesConfig.xml"
#define SAMPLE_XML_PATH "/home/ros/ros/prlite/prlite_teleop/launch/SamplesConfig.xml"

#define CHECK_RC(nRetVal, what)										\
if (nRetVal != XN_STATUS_OK)									\
{																\
    printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
	return nRetVal;												\
}

#define ADJUST_VALUE 1000.0

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
xn::Context g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator g_UserGenerator;

XnBool g_bNeedPose = FALSE;
XnChar g_strPose[20] = "";

ros::Publisher yourmom;	//hack to report status out

int systemret;

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

double calc3DDist( XnPoint3D p1, XnPoint3D p2 ) {
    double dx = p1.X - p2.X;
    double dy = p1.Y - p2.Y;
    double dz = p1.Z - p2.Z;
    return sqrt(dx*dx+dy*dy+dz*dz);
}

static bool arms_up = false;
bool kinect_arms_up() { return arms_up;}

// Callback: New user was detected
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
        arms_up = true;
        // systemret = system("espeak --stdout \"Hello.  Put your arms up like this.\" | aplay &");
	printf("New User %d\n", nId);
	// New user found
	if (g_bNeedPose)
	{
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	}
	else
	{
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}

// Callback: An existing user was lost
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
        // systemret = system("espeak --stdout \"Where did you go?\" | aplay &");
	printf("Lost user %d\n", nId);
		
/*
	prlite_kinematics::some_status_thing status;
	status.lulz = 2;	//lost user
	yourmom.publish(status);
	
	exit(0);	//ugly
*/
}

// Callback: Detected a pose
void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie)
{
        //systemret = system("espeak --stdout \"pose detected.\" | aplay &");
	printf("Pose %s detected for user %d\n", strPose, nId);
	g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
	g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}


// Callback: Started calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie)
{
       //systemret = system("espeak --stdout \"calibration started.  Stay still a little longer.\" | aplay &");
       printf("Calibration started for user %d\n", nId);

}

// Callback: Finished calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie)
{
        arms_up = false;
	if (bSuccess)
	{
		// Calibration succeeded
                // systemret = system("espeak --stdout \"Great.  Now move your arms slowly and I will mimic you.\" | aplay &");
		printf("Calibration complete, start tracking user %d\n", nId);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
		
		prlite_kinematics::some_status_thing status;
		status.lulz = 1;	//calibration ok
		yourmom.publish(status);
	}
	else
	{
		// Calibration failed
                // systemret = system("espeak --stdout \"Calibration failed.\" | aplay &");
		printf("Calibration failed for user %d\n", nId);
		if (g_bNeedPose)
		{
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		}
		else
		{
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
		}
	}
}

// ros::Rate loop_rate(10);
static ros::Publisher leftArmPublisher;
static ros::Publisher rightArmPublisher;
static prlite_ax12commander *ax12;


int kinect_init() {
    ros::NodeHandle nh;
	// ros::init(argc, argv, "PersonTracker");
    ROS_INFO("Initalizing Arm Connection....");
    leftArmPublisher = nh.advertise<prlite_kinematics::SphereCoordinate>("/armik/n0/position", 1000);
    rightArmPublisher = nh.advertise<prlite_kinematics::SphereCoordinate>("/armik/n1/position", 1000);
	yourmom = nh.advertise<prlite_kinematics::some_status_thing>("kinect_hack_status", 1000);
    ros::Duration(2.0).sleep();
    prlite_kinematics::SphereCoordinate coord;
	prlite_kinematics::some_status_thing status;
	status.lulz = 0;	//initialized/reset
	yourmom.publish(status);

	ax12 = new prlite_ax12commander;
        ax12->init();


/* set by joystick
    coord.radius = 35.0;
    coord.phi = 0.0;
    coord.theta = 0.0;
    leftArmPublisher.publish(coord);
    rightArmPublisher.publish(coord);
*/
	ROS_INFO("Initalizing Kinect + NITE....");
	XnStatus nRetVal = XN_STATUS_OK;
	xn::EnumerationErrors errors;
	nRetVal = g_Context.InitFromXmlFile(SAMPLE_XML_PATH, &errors);
	
	if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
	{
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		return (nRetVal);
	} else if (nRetVal != XN_STATUS_OK) {
		printf("Open failed: %s\n", xnGetStatusString(nRetVal));
		return (nRetVal);
	}

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
	CHECK_RC(nRetVal, "Find depth generator");
	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	if (nRetVal != XN_STATUS_OK)
	{
		nRetVal = g_UserGenerator.Create(g_Context);
		CHECK_RC(nRetVal, "Find user generator");
	}

	XnCallbackHandle hUserCallbacks, hCalibrationCallbacks, hPoseCallbacks;
	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
	{
		printf("Supplied user generator doesn't support skeleton\n");
		return 1;
	}
	g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
	g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);
	if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration())
	{
		g_bNeedPose = TRUE;
		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
		{
			printf("Pose required, but not supported\n");
			return 1;
		}
		g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);
		g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");
    ROS_INFO("Ready To Go!\n");
   return 0;
}

void kinect_get_pos(double *center_x, double *center_y, double *center_z)
{

	// while (ros::ok()) {
		// Update OpenNI
		g_Context.WaitAndUpdateAll();
		xn::SceneMetaData sceneMD;
		xn::DepthMetaData depthMD;
		g_DepthGenerator.GetMetaData(depthMD);
		g_UserGenerator.GetUserPixels(0, sceneMD);
		// Print positions
		XnUserID aUsers[15];
		XnUInt16 nUsers = 15;
		g_UserGenerator.GetUsers(aUsers, nUsers);

                // init
                *center_x = 0.0;
                *center_y = 0.0;
                *center_z = 0.0;
		for (int i = 0; i < nUsers; ++i) {
			if(g_UserGenerator.GetSkeletonCap().IsTracking(aUsers[i])) {
				// Read joint positions for person (No WRISTs)
				XnSkeletonJointPosition torsoPosition, lShoulderPosition, rShoulderPosition, neckPosition, headPosition, lElbowPosition, rElbowPosition;
                XnSkeletonJointPosition rWristPosition, lWristPosition, rHipPosition, lHipPosition, lKneePosition, rKneePosition;
                XnSkeletonJointPosition lFootPosition, rFootPosition;
				g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_TORSO, torsoPosition);
                g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_NECK, neckPosition);
                g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_HEAD, headPosition);
				g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_SHOULDER, lShoulderPosition);
				g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_SHOULDER, rShoulderPosition);
				g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_ELBOW, lElbowPosition);
				g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_ELBOW, rElbowPosition);
				g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_HAND, lWristPosition);
				g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_HAND, rWristPosition);
                g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_HIP, lHipPosition);
                g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_HIP, rHipPosition);
                g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_KNEE, lKneePosition);
                g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_KNEE, rKneePosition);
                g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_FOOT, lFootPosition);
                g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_FOOT, rFootPosition);

                // Read Joint Orientations
                XnSkeletonJointOrientation torsoOrientation, lShoulderOrientation, rShoulderOrientation, lHipOrientation, rHipOrientation;
                XnSkeletonJointOrientation lWristOrientation, rWristOrientation, lElbowOrientation, rElbowOrientation;
                XnSkeletonJointOrientation headOrientation, neckOrientation;
                XnSkeletonJointOrientation lKneeOrientation, rKneeOrientation, lFootOrientation, rFootOrientation;
	            g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[i], XN_SKEL_TORSO, torsoOrientation);
	            g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[i], XN_SKEL_HEAD, headOrientation);
	            g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[i], XN_SKEL_NECK, neckOrientation);
	            g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[i], XN_SKEL_LEFT_SHOULDER, lShoulderOrientation);
	            g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[i], XN_SKEL_RIGHT_SHOULDER, rShoulderOrientation);
	            g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[i], XN_SKEL_LEFT_HIP, lHipOrientation);
	            g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[i], XN_SKEL_RIGHT_HIP, rHipOrientation);
	            g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[i], XN_SKEL_LEFT_ELBOW, lElbowOrientation);
	            g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[i], XN_SKEL_RIGHT_ELBOW, rElbowOrientation);
	            g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[i], XN_SKEL_LEFT_HAND, lWristOrientation);
	            g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[i], XN_SKEL_RIGHT_HAND, rWristOrientation);
	            g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[i], XN_SKEL_LEFT_KNEE, lKneeOrientation);
	            g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[i], XN_SKEL_RIGHT_KNEE, rKneeOrientation);
	            g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[i], XN_SKEL_LEFT_FOOT, lFootOrientation);
	            g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(aUsers[i], XN_SKEL_RIGHT_FOOT, rFootOrientation);

	            XnFloat* m = torsoOrientation.orientation.elements;
	            KDL::Rotation torsoO(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
                m = lShoulderOrientation.orientation.elements;
                KDL::Rotation lShouldO(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
                m = rShoulderOrientation.orientation.elements;
                KDL::Rotation rShouldO(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
                m = lHipOrientation.orientation.elements;
                KDL::Rotation lHipO(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
                m = rHipOrientation.orientation.elements;
                KDL::Rotation rHipO(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
                m = headOrientation.orientation.elements;
                KDL::Rotation headO(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
                m = neckOrientation.orientation.elements;
                KDL::Rotation neckO(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
                m = lElbowOrientation.orientation.elements;
                KDL::Rotation lElbowO(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
                m = rElbowOrientation.orientation.elements;
                KDL::Rotation rElbowO(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
                m = lWristOrientation.orientation.elements;
                KDL::Rotation lWristO(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
                m = rWristOrientation.orientation.elements;
                KDL::Rotation rWristO(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
                m = lKneeOrientation.orientation.elements;
                KDL::Rotation lKneeO(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
                m = rKneeOrientation.orientation.elements;
                KDL::Rotation rKneeO(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
                m = lFootOrientation.orientation.elements;
                KDL::Rotation lFootO(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
                m = rFootOrientation.orientation.elements;
                KDL::Rotation rFootO(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);

	        // double qx = 0.0, qy = 0.0, qz = 0.0, qw = 0.0;

		// Get Points in 3D space
		XnPoint3D torso = torsoPosition.position;
		XnPoint3D lShould = lShoulderPosition.position;
		XnPoint3D rShould = rShoulderPosition.position;
                XnPoint3D lElbow = lElbowPosition.position;
                XnPoint3D rElbow = rElbowPosition.position;
                XnPoint3D lWrist = lWristPosition.position;
                XnPoint3D rWrist = rWristPosition.position;
                XnPoint3D neck = neckPosition.position;
                XnPoint3D head = headPosition.position;
                XnPoint3D lHip = lHipPosition.position;
                XnPoint3D rHip = rHipPosition.position;
                XnPoint3D lKnee = lKneePosition.position;
                XnPoint3D rKnee = rKneePosition.position;
                XnPoint3D lFoot = lFootPosition.position;
                XnPoint3D rFoot = rFootPosition.position;

                // for follow me
                *center_x = torsoPosition.position.X / 1000.0;
                *center_y = torsoPosition.position.Y / 1000.0;
                *center_z = torsoPosition.position.Z / 1000.0;

                // ---------- ARM CONTROLLER HACK ------------------
#if 0

                // Calculate arm length of user
                double lenL = calc3DDist( lShould, lElbow ) + calc3DDist( lElbow, lWrist );
                double lenR = calc3DDist( rShould, rElbow ) + calc3DDist( rElbow, rWrist );
                double distL = calc3DDist(lShould, lWrist);
                double distR = calc3DDist(rShould, rWrist);

                // Calculate positions
                prlite_kinematics::SphereCoordinate lCoord;
                prlite_kinematics::SphereCoordinate rCoord;

                lCoord.radius = 35 * (distL / lenL);
                rCoord.radius = 35 * (distR / lenR);

                lCoord.theta = -atan2(lShould.X - lWrist.X, lShould.Z - lWrist.Z);
                rCoord.theta = -atan2(rShould.X - rWrist.X, rShould.Z - rWrist.Z);
/*
*/
                if(lCoord.theta > 1.0) lCoord.theta = 1.0;
                if(lCoord.theta < -1.0) lCoord.theta = -1.0;
                if(rCoord.theta > 1.0) rCoord.theta = 1.0;
                if(rCoord.theta < -1.0) rCoord.theta = -1.0;

                lCoord.phi = -atan2(lShould.Y - lWrist.Y, lShould.X - lWrist.X);
                rCoord.phi = -atan2(rShould.Y - rWrist.Y, rShould.X - rWrist.X);
/*
*/
                if(lCoord.phi > 1.25) lCoord.phi = 1.25;
                if(lCoord.phi < -0.33) lCoord.phi = -0.33;
                if(rCoord.phi > 1.2) rCoord.phi = 1.25;
                if(rCoord.phi < -0.33) rCoord.phi = -0.33;

                ROS_INFO("User %d: Left (%lf,%lf,%lf), Right (%lf,%lf,%lf)", i, lCoord.radius, lCoord.theta, lCoord.phi, rCoord.radius, rCoord.theta, rCoord.phi);

                // Publish to arms
                leftArmPublisher.publish(rCoord);
                rightArmPublisher.publish(lCoord);
#endif

                // ---------- START HACK #2a -----------------

                double right_elbow_pan = atan2(lShould.X - lElbow.X, lShould.Z - lElbow.Z);
                double right_elbow_tilt = atan2(lShould.Y - lElbow.Y, lShould.X - lElbow.X);
                double right_wrist_tilt = -1*atan2(lElbow.X - lWrist.X, lElbow.Z- lWrist.Z);
                
                double left_elbow_pan = atan2(rShould.X - rElbow.X, rShould.Z - rElbow.Z);
                double left_elbow_tilt = atan2(rShould.Y - rElbow.Y, rShould.X - rElbow.X);
                double left_wrist_tilt = atan2(rElbow.Y - rWrist.Y, rElbow.X- rWrist.X);


		// max and min arch checked by Joint Command
ROS_INFO("User %d: Left (%lf,%lf,%lf,%lf, %lf,%lf,%lf,%lf)", i, lShould.X, lElbow.X, lShould.Z,  lElbow.Z, lElbow.X, lWrist.X, lElbow.Z, lWrist.Z );
ROS_INFO("User %d: Left (%lf,%lf,%lf), Right (%lf,%lf,%lf)", i, left_elbow_pan,  left_elbow_tilt, left_wrist_tilt, right_elbow_pan, right_elbow_tilt, right_wrist_tilt);
                ax12->JointCommand( prlite_ax12commander::elbowpanR, right_elbow_pan);
                ax12->JointCommand( prlite_ax12commander::elbowtiltR, right_elbow_tilt);
                ax12->JointCommand( prlite_ax12commander::wristtiltR, right_wrist_tilt);
                ax12->JointCommand( prlite_ax12commander::elbowpan, left_elbow_pan);
                ax12->JointCommand( prlite_ax12commander::elbowtilt, left_elbow_tilt);
                ax12->JointCommand( prlite_ax12commander::wristtilt, left_wrist_tilt);
	
	
                // ---------- START HACK #2b -----------------

#if 0

                double left_shoulder_tilt = atan2(lShould.X - lElbow.X, lShould.Z - lElbow.Z);
                double left_shoulder_pan = atan2(lShould.Y - lElbow.Y, lShould.X - lElbow.X);
                double left_elbow_tilt = atan2(lElbow.X - lWrist.X, lElbow.Z- lWrist.Z);
                double left_elbow_pan = atan2(lElbow.Y - lWrist.Y, lElbow.X- lWrist.X);
                double left_wrist_tilt = left_shoulder_tilt - left_elbow_tilt ;
                double right_shoulder_pan = atan2(rShould.X - rElbow.X, rShould.Z - rElbow.Z);
                double right_shoulder_tilt = atan2(rShould.Y - rElbow.Y, rShould.X - rElbow.X);
                double right_elbow_pan = atan2(rElbow.X - rWrist.X, rElbow.Z- rWrist.Z);
                double right_elbow_tilt = atan2(rElbow.Y - rWrist.Y, rElbow.X- rWrist.X);
                double right_wrist_tilt = right_shoulder_tilt-right_elbow_tilt;


		// max and min arch checked by Joint Command
/*
                JointCommand( prlite_ax12commander::shoulderpanR, right_shoulder_pan);
                // JointCommand( prlite_ax12commander::shouldertiltR, right_shoulder_tilt);
                JointCommand( prlite_ax12commander::elbowtiltR, right_elbowtilt);
                JointCommand( prlite_ax12commander::elbowpanR, right_elbowpan);
*/
ROS_INFO("User %d: Left (%lf,%lf,%lf,%lf, %lf,%lf,%lf,%lf)", i, lShould.X, lElbow.X, lShould.Z,  lElbow.Z, lElbow.X, lWrist.X, lElbow.Z, lWrist.Z );
                ax12->JointCommand( prlite_ax12commander::shoulderpan, left_shoulder_pan);
ROS_INFO("User %d: Left (%lf,%lf,%lf, %lf), Right (%lf,%lf,%lf,%lf)", i, left_shoulder_pan, left_shoulder_tilt, left_elbow_pan,  left_elbow_tilt, 0.0,0.0,0.0,0.0);
                ax12->JointCommand( prlite_ax12commander::shoulderpan, left_shoulder_pan);
                // JointCommand( prlite_ax12commander::shouldertilt, left_shoulder_tilt);
                ax12->JointCommand( prlite_ax12commander::elbowtilt, left_elbow_tilt);
                ax12->JointCommand( prlite_ax12commander::elbowpan, left_elbow_pan);
	
		// figure out desired tilt and compute desired LinAct length to get that tilt
/*
sin A = opp/hyp
cos A = adj/hyp
*/
/* constants in inches */
#define BAR_LEN (4.3 - .315)   /* Upper arm bar that LinAct connects to */
#define LINACT_DWN (9.9)       /* length of LinAct when retracted */
#define LINACT_UP (13.9)       /* length of LinAct when extended */
#define HYPOTENUSE (14.968)    /* Len from bottom LinAct hole to Shoulder Joint 
*/
#define LINACT_TO_TOP (14.715) /* len from bottom LinAct hole to top lazy susan 
*/
#define BRACKET_TO_BAR (1.26)  /* From LinAct hole to upper arm bar */
/* additional angle caused by LinAct bracket */
#define ADD_BRACKET_ANGLE (atan(BRACKET_TO_BAR / BAR_LEN))
/* extra angle caused by LinAct bottom and Shoulder hinge not lining up */
#define FIXED_LA_ANGLE  (acos(LINACT_TO_TOP / HYPOTENUSE))
#define FIXED_LA_ANGLE2  (asin(LINACT_TO_TOP / HYPOTENUSE))
/* for shorthand */
#define B BAR_LEN
#define H HYPOTENUSE 
#define L (linact_len + LINACT_DWN)

#define TMP_ANGLE (acos( (B*B - L*L + H*H) / H))
/* shoulder angle in radians based on linact_len */
#define SHLDR_ANGLE (TMP_ANGLE + ADD_BRACKET_ANGLE - FIXED_LA_ANGLE)

/* linact_len based on shoulder angle */
#define desired_linact_len(shdrangle) sqrt(B*B + H*H - H*cos(shdrangle - ADD_BRACKET_ANGLE + FIXED_LA_ANGLE))

#define TMP2_ANGLE (asin( (B*B - L*L + H*H) / H))
/* linact_angle based on shoulder angle */
#define LINACT_ANGLE (TMP2_ANGLE + FIXED_LA_ANGLE2)

#define LINACT_VEL (.5)  /* inches per second */
/* Let u = arccos(x) then du = -1/(1-x²) dx in radians per sec */
#define SHLDR_ANGLE_VEL (-1/(1-pow(((B*B-L*L+H*H)/H),2))*(-LINACT_VEL*LINACT_VEL/H))

		double left_shoulder_linact = desired_linact_len(left_shoulder_tilt)/4 * 1000;
		double right_shoulder_linact = desired_linact_len(left_shoulder_tilt)/4 * 1000;
                static double last_left_shoulder_pan = left_shoulder_pan;
                static double last_left_elbow_pan = left_elbow_pan;
                static double last_left_wrist_tilt = left_wrist_tilt;
                static double last_right_shoulder_pan = right_shoulder_pan;
                static double last_right_elbow_pan = right_elbow_pan;
                static double last_right_wrist_tilt = right_wrist_tilt;
                static int gripper_toggle_count = 0;

                if ( last_left_shoulder_pan - left_shoulder_pan > .1
                  || last_left_elbow_pan - left_elbow_pan > .1
                  || last_left_wrist_tilt - left_wrist_tilt > .1)
                   gripper_toggle_count = 0;
                else
                   gripper_toggle_count++;
                last_left_shoulder_pan = left_shoulder_pan;
                last_left_elbow_pan = left_elbow_pan;
                last_left_wrist_tilt = left_wrist_tilt;
                if (gripper_toggle_count > 10000) {
                  ROS_INFO("Toggle grippers");
                  ax12->ToggleGrippers();
                }
ROS_INFO("User %d: Left (%lf), Right (%lf)", i, left_shoulder_linact, right_shoulder_linact);
                ax12->setShoulderGoal(right_shoulder_linact, left_shoulder_linact);

#endif
                // ---------- END HACK -----------------

#if 0
                // Publish Transform
                static tf::TransformBroadcaster br;
                tf::Transform transform;
                std::stringstream body_name, neck_name, lshould_name, rshould_name, head_name, relbow_name, lelbow_name, lwrist_name, rwrist_name;
                std::stringstream lhip_name, rhip_name, lknee_name, rknee_name, lfoot_name, rfoot_name;
                body_name << "Torso (" << i << ")" << std::ends;
                neck_name << "Neck (" << i << ")" << std::ends;
                head_name << "Head (" << i << ")" << std::ends;
                lshould_name << "Shoulder L (" << i << ")" << std::ends;
                rshould_name << "Shoulder R (" << i << ")" << std::ends;
                lelbow_name << "Elbow L (" << i << ")" << std::ends;
                relbow_name << "Elbow R (" << i << ")" << std::ends;
                lwrist_name << "Wrist L (" << i << ")" << std::ends;
                rwrist_name << "Wrist R (" << i << ")" << std::ends;
                lhip_name << "Hip L (" << i << ")" << std::ends;
                rhip_name << "Hip R (" << i << ")" << std::ends;
                lknee_name << "Knee L (" << i << ")" << std::ends;
                rknee_name << "Knee R (" << i << ")" << std::ends;
                lfoot_name << "Foot L (" << i << ")" << std::ends;
                rfoot_name << "Foot R (" << i << ")" << std::ends;

                // Publish Torso
                torsoO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(torso.X / ADJUST_VALUE, torso.Y / ADJUST_VALUE, torso.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", body_name.str().c_str()));  
                // Publish Left Shoulder
                lShouldO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(lShould.X / ADJUST_VALUE, lShould.Y / ADJUST_VALUE, lShould.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", lshould_name.str().c_str()));
                // Publish Right Shoulder
                rShouldO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(rShould.X / ADJUST_VALUE, rShould.Y / ADJUST_VALUE, rShould.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", rshould_name.str().c_str())); 
                // Publish Left Elbow
                lElbowO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(lElbow.X / ADJUST_VALUE, lElbow.Y / ADJUST_VALUE, lElbow.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", lelbow_name.str().c_str()));
                // Publish Right Elbow
                rElbowO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(rElbow.X / ADJUST_VALUE, rElbow.Y / ADJUST_VALUE, rElbow.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", relbow_name.str().c_str()));
                // Publish Left Wrist
                lWristO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(lWrist.X / ADJUST_VALUE, lWrist.Y / ADJUST_VALUE, lWrist.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", lwrist_name.str().c_str()));
                // Publish Right Wrist
                rWristO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(rWrist.X / ADJUST_VALUE, rWrist.Y / ADJUST_VALUE, rWrist.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", rwrist_name.str().c_str()));
                // Publish Neck
                neckO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(neck.X / ADJUST_VALUE, neck.Y / ADJUST_VALUE, neck.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", neck_name.str().c_str())); 
                // Publish Head
                headO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(head.X / ADJUST_VALUE, head.Y / ADJUST_VALUE, head.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", head_name.str().c_str()));
                // Publish Left Hip
                lHipO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(lHip.X / ADJUST_VALUE, lHip.Y / ADJUST_VALUE, lHip.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", lhip_name.str().c_str()));
                // Publish Right Hip
                rHipO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(rHip.X / ADJUST_VALUE, rHip.Y / ADJUST_VALUE, rHip.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", rhip_name.str().c_str())); 
                // Publish Left Knee
                lKneeO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(lKnee.X / ADJUST_VALUE, lKnee.Y / ADJUST_VALUE, lKnee.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", lknee_name.str().c_str()));
                // Publish Right Knee
                rKneeO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(rKnee.X / ADJUST_VALUE, rKnee.Y / ADJUST_VALUE, rKnee.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", rknee_name.str().c_str())); 
                // Publish Left Foot
                lFootO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(lFoot.X / ADJUST_VALUE, lFoot.Y / ADJUST_VALUE, lFoot.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", lfoot_name.str().c_str()));
                // Publish Right Foot
                rFootO.GetQuaternion(qx, qy, qz, qw);
                transform.setOrigin(tf::Vector3(rFoot.X / ADJUST_VALUE, rFoot.Y / ADJUST_VALUE, rFoot.Z / ADJUST_VALUE));
                transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", rfoot_name.str().c_str()));
#endif
			}
		}
		// ros::spinOnce();
		//loop_rate.sleep();
	// }
}

void kinect_shutdown()
{
	g_Context.Shutdown();
}


