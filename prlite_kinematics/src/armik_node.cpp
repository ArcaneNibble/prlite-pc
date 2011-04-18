/* Node which serves joint positions to smart arms (By Nathaniel Lewis 2011) */

#include <ros/ros.h>

#include <cstdlib>

#include <std_msgs/Float64.h>
#include <prlite_kinematics/SphereCoordinate.h>
#include <prlite_kinematics/arm_ik.h>

struct arm_ik positions;

ros::Publisher shoulderpancontroller;
ros::Publisher shouldertiltcontroller;
ros::Publisher elbowtiltcontroller;

bool isShoulderInverted = false;

void armcoord_callback( const prlite_kinematics::SphereCoordinate::ConstPtr& coord ) {
    int result = solveIK(*coord, &positions);
    if(result) ROS_ERROR("(%f,%f,%f) is UNSOLVABLE", (*coord).radius, coord->theta, coord->phi);
    else {
        std_msgs::Float64 jointPosition;

        jointPosition.data = positions.rotation;
        shoulderpancontroller.publish(jointPosition);

        jointPosition.data = positions.shoulder * (isShoulderInverted ? -1 : 1);
        shouldertiltcontroller.publish(jointPosition);
        
        jointPosition.data = positions.elbow - positions.shoulder; // This factors out the tilt from the first joint
        elbowtiltcontroller.publish(jointPosition);
        
        ROS_INFO("(%f,%f,%f); shoulder: %f, elbow: %f, rotation: %f", coord->radius, coord->theta, coord->phi, positions.shoulder, positions.elbow, positions.rotation);
    }
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

    std::stringstream sinv;
    sinv << prefix.str() << "shoulderi";
    nh.getParam(sinv.str(), isShoulderInverted);

    /* Data Subscriptions */
    std::stringstream coord_listen;
    coord_listen << prefix.str() << "position";
    ros::Subscriber coords = nh.subscribe(coord_listen.str().c_str(), 1000, armcoord_callback);
    ROS_INFO("Arm IK service listening on \"%s\"", coord_listen.str().c_str());

    /* Loop forever */
    ros::spin();
    return 0;
}

