/* Nathaniel Lewis's Joystic to Smart arm bridge for the PR-ARM project */

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "joy/Joy.h"

#include <sstream>
#include <cstdlib>

/* Variables */
ros::Publisher shoulderpancontroller;
int shoulderpanaxis = -1;
double shoulderpanpos = 0.0;
double shoulderpanposmin = -1.75;
double shoulderpanposmax = 1.75;
double shoulderpantuck = 1;
double shoulderpanuntuck = 1;

ros::Publisher wristrotatecontroller;
int wristrotateaxis = -1;
double wristrotatepos = 0.0;
double wristrotateposmax = 1.75;
double wristrotateposmin = -1.75;
double wristrotatetuck = 1;
double wristrotateuntuck = 1;

ros::Publisher shouldertiltcontroller;
int shouldertiltaxis = -1;
double shouldertiltpos = 1.75;
double shouldertiltposmax = 1.75;
double shouldertiltposmin = -1.75;
double shouldertilttuck = 1;
double shouldertiltuntuck = 1;

ros::Publisher elbowtiltcontroller;
int elbowtiltaxis = -1;
double elbowtiltpos = -1.75;
double elbowtiltposmax = 1.75;
double elbowtiltposmin = -1.75;
double elbowtilttuck = 1;
double elbowtiltuntuck = 1;

ros::Publisher fingerleftcontroller;
ros::Publisher fingerrightcontroller;
int openFingersButton = -1;
int closeFingersButton = -1;
double leftFingerPos = 0;
double rightFingerPos = 0;
double leftFingertuck = 0;
double rightFingertuck = 0;
double leftFingeruntuck = 0;
double rightFingeruntuck = 0;
double closePosLeft = 1.1;
double closePosRight = -1.1;

int resetbutton = -1;

joy::Joy joystate;
bool     jshasinited = false;

void joyCallBack( const joy::Joy::ConstPtr& _joystate ) {
	joystate = *_joystate;
    if(!jshasinited) {
        ROS_INFO("Joystick Alive");
        jshasinited = true;
    }
}	

void timerCallback(const ros::TimerEvent& e) {
    if(shoulderpanaxis > -1) {
        shoulderpanpos += joystate.axes[shoulderpanaxis] / 25.0;
        if(shoulderpanpos > shoulderpanposmax) shoulderpanpos = shoulderpanposmax;
        if(shoulderpanpos < shoulderpanposmin) shoulderpanpos = shoulderpanposmin;
    }
    if(wristrotateaxis > -1) {
        wristrotatepos += joystate.axes[wristrotateaxis] / 45.0;
        if(wristrotatepos > wristrotateposmax) wristrotatepos = wristrotateposmax;
        if(wristrotatepos < wristrotateposmin) wristrotatepos = wristrotateposmin;
    }
    if(shouldertiltaxis > -1) {
        shouldertiltpos += joystate.axes[shouldertiltaxis] / 45.0;
        if(shouldertiltpos > shouldertiltposmax) shouldertiltpos = shouldertiltposmax;
        if(shouldertiltpos < shouldertiltposmin) shouldertiltpos = shouldertiltposmin;
    }
    if(elbowtiltaxis > -1) {
        elbowtiltpos += joystate.axes[elbowtiltaxis] / 45.0;
        if(elbowtiltpos > elbowtiltposmax) elbowtiltpos = elbowtiltposmax;
        if(elbowtiltpos < elbowtiltposmin) elbowtiltpos = elbowtiltposmin;
    }
    if((openFingersButton > -1) && (closeFingersButton > -1)) {
        if(joystate.buttons[openFingersButton]) leftFingerPos = rightFingerPos = 0.0;
        if(joystate.buttons[closeFingersButton]) {
            leftFingerPos = closePosLeft;
            rightFingerPos = closePosRight;
        }
    }
    if(resetbutton > -1) {
        if(joystate.buttons[resetbutton]) {
            wristrotatepos = shoulderpanpos = 0.0;
            leftFingerPos = rightFingerPos = 0.0;
            shouldertiltpos = 1.75;
            elbowtiltpos = -1.75;
        }
    }
    std_msgs::Float64 command;
    command.data = shoulderpanpos;
    shoulderpancontroller.publish(command);
    command.data = wristrotatepos;
    wristrotatecontroller.publish(command);
    command.data = shouldertiltpos;
    shouldertiltcontroller.publish(command);
    command.data = elbowtiltpos;
    elbowtiltcontroller.publish(command);
    command.data = leftFingerPos;
    fingerleftcontroller.publish(command);
    command.data = rightFingerPos;
    fingerrightcontroller.publish(command);
    //ROS_INFO("Shoulder Pan: %lf  Elbow Tilt: %lf", shoulderpanpos, elbowtiltpos);
    //ROS_INFO("Subs: %d", shoulderpancontroller.getNumSubscribers());
}

int main(int argc, char **argv) {
    /* Init */
    ros::init(argc, argv, "joyarmbridge");
    ros::NodeHandle n;
    shoulderpancontroller = n.advertise<std_msgs::Float64>("/shoulder_pan_controller/command", 1000);
    wristrotatecontroller = n.advertise<std_msgs::Float64>("/wrist_rotate_controller/command", 1000);
    shouldertiltcontroller = n.advertise<std_msgs::Float64>("/shoulder_tilt_controller/command", 1000);
    elbowtiltcontroller = n.advertise<std_msgs::Float64>("/elbow_tilt_controller/command", 1000);
    fingerleftcontroller = n.advertise<std_msgs::Float64>("/finger_left_controller/command", 1000);
    fingerrightcontroller = n.advertise<std_msgs::Float64>("/finger_right_controller/command", 1000);
	ros::Subscriber joy_input = n.subscribe("joy", 1000, joyCallBack);
    /* Parameters */
    ROS_INFO("1");
    while(!jshasinited) ros::spinOnce();
    ROS_INFO("2");
	if(n.getParam("/ax12_arm_joycontroller/shoulder_pan_axis", shoulderpanaxis)) {
        std::stringstream ss;
        ss << "Using shoulder pan axis: " << shoulderpanaxis;
        ROS_INFO("%s", ss.str().c_str());
    }
    ROS_INFO("3");
	if(n.getParam("/ax12_arm_joycontroller/wrist_rotate_axis", wristrotateaxis)) {
        std::stringstream ss;
        ss << "Using wrist rotate axis: " << wristrotateaxis;
        ROS_INFO("%s", ss.str().c_str());
    }
    ROS_INFO("4");
	if(n.getParam("/ax12_arm_joycontroller/shoulder_tilt_axis", shouldertiltaxis)) {
        std::stringstream ss;
        ss << "Using shoulder tilt axis: " << shouldertiltaxis;
        ROS_INFO("%s", ss.str().c_str());
    }
    ROS_INFO("5");
	if(n.getParam("/ax12_arm_joycontroller/elbow_tilt_axis", elbowtiltaxis)) {
        std::stringstream ss;
        ss << "Using elbow tilt axis: " << elbowtiltaxis;
        ROS_INFO("%s", ss.str().c_str());
    }
    ROS_INFO("6");
	if(n.getParam("/ax12_arm_joycontroller/reset_button", resetbutton)) {
        std::stringstream ss;
        ss << "Using reset button: " << resetbutton;
        ROS_INFO("%s", ss.str().c_str());
    }
    ROS_INFO("7");
	if(n.getParam("/ax12_arm_joycontroller/open_fingers_button", openFingersButton)) {
        std::stringstream ss;
        ss << "Using open fingers button: " << openFingersButton;
        ROS_INFO("%s", ss.str().c_str());
    }
    ROS_INFO("8");
	if(n.getParam("/ax12_arm_joycontroller/close_fingers_button", closeFingersButton)) {
        std::stringstream ss;
        ss << "Using close fingers button: " << closeFingersButton;
        ROS_INFO("%s", ss.str().c_str());
    }
    ros::Duration(5.0).sleep();
    ROS_INFO("READY!!!");
    ros::Timer timer = n.createTimer(ros::Duration(0.01), timerCallback);
    ros::spin();
	return 0;
}

