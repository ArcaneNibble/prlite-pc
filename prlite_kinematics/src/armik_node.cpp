/* Node which serves joint positions to smart arms (By Nathaniel Lewis 2011) */

#define FINGER_CLOSED 1.0
#define FINGER_OPEN   0.2

#include <ros/ros.h>

#include <cstdlib>

#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <prlite_kinematics/SphereCoordinate.h>
#include <prlite_kinematics/GripperState.h>
#include <prlite_kinematics/arm_ik.h>

struct arm_ik positions;

// Arm coordinate publisher
ros::Publisher shoulderpancontroller;
ros::Publisher shouldertiltcontroller;
ros::Publisher elbowtiltcontroller;

// Arm gripper publishers
ros::Publisher fingerleftcontroller;
ros::Publisher fingerrightcontroller;
ros::Publisher wristrotatecontroller;

bool invert1 = false;
bool invert2 = false;
bool invert3 = false;

void armcoord_callback( const prlite_kinematics::SphereCoordinate::ConstPtr& coord ) {
    int result = solveIK(*coord, &positions);
    if(result) ROS_ERROR("(%f,%f,%f) is UNSOLVABLE", (*coord).radius, coord->theta, coord->phi);
    else {
        std_msgs::Float64 jointPosition;

        jointPosition.data = positions.rotation * (invert1 ? -1 : 1);
        shoulderpancontroller.publish(jointPosition);

        jointPosition.data = positions.shoulder * (invert2 ? -1 : 1);
        shouldertiltcontroller.publish(jointPosition);
        
        jointPosition.data = (positions.elbow - positions.shoulder) * (invert3 ? -1 : 1);
        elbowtiltcontroller.publish(jointPosition);
        
        ROS_INFO("(%f,%f,%f); shoulder: %f, elbow: %f, rotation: %f", coord->radius, coord->theta, coord->phi, positions.shoulder, positions.elbow, positions.rotation);
    }
}

void gripperstate_callback( const prlite_kinematics::GripperState::ConstPtr& coord ) {
    // Set gripper positions
    std_msgs::Float64 left_finger;
    std_msgs::Float64 right_finger;
    std_msgs::Float64 wrist_rotation;

    if(coord->state) {
        // Gripper is CLOSED
	left_finger.data = FINGER_CLOSED;
        right_finger.data = FINGER_CLOSED * (-1);
    } else {
        // Gripper is OPN
        left_finger.data = FINGER_OPEN;
        right_finger.data = FINGER_OPEN * (-1);
    }

    wrist_rotation.data = coord->rotation;

    // LOG
    ROS_INFO("wrist: %f, fingerL: %f, fingerR: %f", wrist_rotation.data, left_finger.data, right_finger.data);

    // Publish the finger positions
    fingerleftcontroller.publish(left_finger);
    fingerrightcontroller.publish(right_finger);

    // Set wrist rotation state
    wristrotatecontroller.publish(wrist_rotation);
}

int main (int argc, char **argv) {
    /* Start the IK Node */
    ros::init(argc, argv, "armik");
    ros::NodeHandle nh;

    /* Load ID */
    int nodeid = 0;
    if(argc > 1) nodeid = std::atoi(argv[1]);
    std::stringstream prefix;
    prefix << "/armik/n" << nodeid << "/";

    /* Connect to proper services */
    std::stringstream spc;
    std::string spc_path;
    spc << prefix.str() << "rotation";
    nh.param(spc.str(), spc_path, std::string("/shoulder_pan_controller/command"));
    shoulderpancontroller = nh.advertise<std_msgs::Float64>(spc_path, 1000);

    std::stringstream stc;
    std::string stc_path;
    stc << prefix.str() << "shoulder";
    nh.param(stc.str(), stc_path, std::string("/shoulder_tilt_controller/command"));
    shouldertiltcontroller = nh.advertise<std_msgs::Float64>(stc_path, 1000);

    std::stringstream etc;
    std::string etc_path;
    etc << prefix.str() << "elbow";
    nh.param(etc.str(), etc_path, std::string("/elbow_tilt_controller/command"));
    elbowtiltcontroller = nh.advertise<std_msgs::Float64>(etc_path, 1000);

    std::stringstream wrc;
    std::string wrc_path;
    wrc << prefix.str() << "wrist";
    nh.param(wrc.str(), wrc_path, std::string("/wrist_rotate_controller/command"));
    wristrotatecontroller = nh.advertise<std_msgs::Float64>(wrc_path, 1000);

    std::stringstream flc;
    std::string flc_path;
    flc << prefix.str() << "finger_left";
    nh.param(flc.str(), flc_path, std::string("/finger_left_controller/command"));
    fingerleftcontroller = nh.advertise<std_msgs::Float64>(flc_path, 1000);

    std::stringstream frc;
    std::string frc_path;
    frc << prefix.str() << "finger_right";
    nh.param(frc.str(), frc_path, std::string("/finger_right_controller/command"));
    fingerrightcontroller = nh.advertise<std_msgs::Float64>(frc_path, 1000);

    /* Load Parameters */
    std::stringstream as1;
    double as1v = 10.0;
    as1 << prefix.str() << "arm_segment1_len";
    nh.getParam(as1.str(), as1v);
    positions.arm_segment1 = as1v;   

    std::stringstream as2;
    double as2v = 10.0;
    as2 << prefix.str() << "arm_segment2_len";
    nh.getParam(as2.str(), as2v);
    positions.arm_segment2 = as2v; 

    std::stringstream sinv1;
    sinv1 << prefix.str() << "invert1";
    nh.getParam(sinv1.str(), invert1);

    std::stringstream sinv2;
    sinv2 << prefix.str() << "invert2";
    nh.getParam(sinv2.str(), invert2);

    std::stringstream sinv3;
    sinv3 << prefix.str() << "invert3";
    nh.getParam(sinv3.str(), invert3);

    /* Data Subscriptions */
    std::stringstream coord_listen;
    coord_listen << prefix.str() << "position";
    ros::Subscriber coords = nh.subscribe(coord_listen.str().c_str(), 1000, armcoord_callback);
    ROS_INFO("Arm IK (position) service listening on \"%s\"", coord_listen.str().c_str());

    std::stringstream gripper_listen;
    gripper_listen << prefix.str() << "gripper";
    ros::Subscriber grippers = nh.subscribe(gripper_listen.str().c_str(), 1000, gripperstate_callback);
    ROS_INFO("Arm IK (gripper) service listening on \"%s\"", gripper_listen.str().c_str());

    /* Loop forever */
    ros::spin();
    return 0;
}

