#include "ros/ros.h"
#include "pr2lite_actuators/LinearActuator.hpp"

#include <sstream>
#include <cstdlib>
#include <cmath>

#define BASE_LENGTH 9.9f
#define SWING 4.0f
#define LINACT_TORSO_ID     0x0C 
#define LINACT_RIGHT_ARM_ID 0x0E 
#define LINACT_LEFT_ARM_ID  0x0F 
#define LINACT_WHEELS_ID    0xAA 

int main (int argc, char** argv)
{
    ros::init(argc, argv, "linact_test");
    ros::NodeHandle nh;
    
    pr2lite::actuators::LinearActuator left_arm(nh, LINACT_LEFT_ARM_ID);
    pr2lite::actuators::LinearActuator right_arm(nh, LINACT_RIGHT_ARM_ID);

    double length = sqrt(-162.2 * cos((87.58 - atof(argv[1])) * M_PI / 180.0) + 256.0) - BASE_LENGTH;
    length *= (1000.0f / SWING);

    if(length < 10.0f) length = 10.0f;
    if(length >  1000.0f) length = 1000.0f;

    left_arm.setPosition(length);
    right_arm.setPosition(length);

    ros::spin();

    return 0;
}

