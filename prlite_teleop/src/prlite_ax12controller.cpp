#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "joy/Joy.h"

#include <sstream>
#include <cstdlib>
#include "prlite_ax12controller.h"

#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "packets_485net/packet_485net_dgram.h"

#include <prlite_kinematics/SphereCoordinate.h>
/*
#include <prlite_kinematics/some_status_thing.h>
*/

const double X_MULT = 7.51913116; // speed is ticks per interval and interval is 1/10 sec, so should be WHEEL_TICKS_PER_METER / 10
const double TH_MULT = 5;
const uint16_t TORSO_DOWN = 990;
const uint16_t TORSO_MID= 500;
const uint16_t TORSO_UP = 100;
const uint16_t LINACT_PRECISION = 15;

ros::Publisher linact_pub;

/*
double tuck_shoulder_pan_;
double tuck_shoulder_tilt_;
double tuck_elbow_tilt_;
double tuck_wrist_rotate_;
double tuck_finger_left_;
double tuck_finger_right_;

double untuck_shoulder_pan_;
double untuck_shoulder_tilt_;
double untuck_elbow_tilt_;
double untuck_wrist_rotate_;
double untuck_finger_left_;
double untuck_finger_right_;
*/


bool linact_arrived = false;
bool linact_new_goal = false;
int linact_goal = TORSO_DOWN;
int linact_goal_last = 10000;
ros::Subscriber linact_sub;
ros::Publisher leftArmPublisher;
ros::Publisher rightArmPublisher;

void TorsoPublish(void)
{
  if (linact_arrived && linact_goal == linact_goal_last)
  {
    // publish wheel commands
  }
  else 
  {
    if (linact_goal != linact_goal_last)
    {
		uint16_t itmp;
      // publish linear actuator command
      packets_485net::packet_485net_dgram linact_cmd;
      linact_cmd.destination = 0x0D;
	  linact_cmd.source = 0xF0;
	  linact_cmd.sport = 7;
	  linact_cmd.dport = 1;
	  
	  itmp = linact_goal - LINACT_PRECISION;
	  linact_cmd.data[0] = itmp & 0xFF;
	  linact_cmd.data[1] = (itmp >> 16) & 0xFF;
	  itmp = linact_goal + LINACT_PRECISION;
	  linact_cmd.data[2] = itmp & 0xFF;
	  linact_cmd.data[3] = (itmp >> 16) & 0xFF;
	  
      linact_pub.publish(linact_cmd);
      linact_goal_last = linact_goal;
      linact_new_goal = true;
      linact_arrived = false;
      ROS_INFO("linact %d", linact_goal);
    }
  }
}

void TorsoCallback(const packets_485net::packet_485net_dgram& linear_actuator_status)
{
	if(linear_actuator_status.source != 0x0D)
		return;
	if(!(linear_actuator_status.destination == 0xF0 || linear_actuator_status.destination == 0x00))
		return;
	if(linear_actuator_status.dport != 7)
		return;
  linact_arrived = linear_actuator_status.data[4+2];
  if (linact_new_goal) // if new linear actuator goal then linact_arrived may take a few frames to update
  {
    if (!linact_arrived)
      linact_new_goal = false;
    else
      linact_arrived = false;
  }
  //linact_arrived = true; // hack for if linear actuator isn't working
  TorsoPublish();
}


void TorsoInit(void)
{
  ros::NodeHandle n;

  linact_sub = n.subscribe("net_485net_incoming_dgram", 1000, TorsoCallback);
  linact_pub = n.advertise<i2c_net_packets::linact_target>("net_485net_outgoing_dgram", 1000);

}

void prlite_ax12commander::setTorsoGoal(int goal)
{
  if (0 == goal) return;
  linact_goal -= goal*LINACT_PRECISION;
  if (linact_goal < TORSO_UP)
    linact_goal = TORSO_UP;
  else if (linact_goal > TORSO_DOWN)
    linact_goal = TORSO_DOWN;

  ROS_INFO_STREAM("Torso: " << linact_goal);
  TorsoPublish();
}

static std::string prlite_ax12controllername[] = {
   "shoulder_pan_controllerL/",
   "shoulder_tilt_controllerL/", 
   "elbow_tilt_controllerL/",
   "wrist_rotate_controllerL/", 
   "finger_left_controllerL/",
   "finger_right_controllerL/",
   "shoulder_pan_controllerR/",
   "shoulder_tilt_controllerR/", 
   "elbow_tilt_controllerR/",
   "wrist_rotate_controllerR/", 
   "finger_left_controllerR/",
   "finger_right_controllerR/",
   "kinect_pan_controller/",
   "kinect_tilt_controller/",
   "laser_tilt_controller/"
};

static std::string prlite_ax12param[] = {
   "/prlite_ax12/shoulder_pan_tuck", 
   "/prlite_ax12/shoulder_pan_tuck",
   "/prlite_ax12/shoulder_tilt_tuck", 
   "/prlite_ax12/elbow_tilt_tuck",
   "/prlite_ax12/wrist_rotate_tuck", 
   "/prlite_ax12/finger_left_tuck",
   "/prlite_ax12/finger_right_tuck",
   "/prlite_ax12/kinect_pan_tuck",
   "/prlite_ax12/kinect_tilt_tuck",
   "/prlite_ax12/laser_tilt_tuck",
   "/prlite_ax12/shoulder_pan_untuck", 
   "/prlite_ax12/shoulder_pan_untuck",
   "/prlite_ax12/shoulder_tilt_untuck", 
   "/prlite_ax12/elbow_tilt_untuck",
   "/prlite_ax12/wrist_rotate_untuck", 
   "/prlite_ax12/finger_left_untuck",
   "/prlite_ax12/finger_right_untuck",
   "/prlite_ax12/kinect_pan_untuck",
   "/prlite_ax12/kinect_tilt_untuck",
   "/prlite_ax12/laser_tilt_untuck"
};

typedef struct ax12_joint_s {
  ros::Publisher controller;
  ros::Subscriber subscriber;
  ros::Publisher torque;
  dynamixel_msgs::JointState joint_state;
  double directpos;
  double desiredpos;
  dynamixel_msgs::MotorState motor_state[2];  
  double minpos;
  double maxpos;
  double nxtpos;
  double incrpos;
  double tuck;
  double untuck;
  double kinect_callobrate;
  double maxvolt;
} ax12_joint_t;

static ax12_joint_t prlite_ax12joint[prlite_ax12commander::numjoints];
static ros::Subscriber motor_state_sub;

void prlite_ax12commander::get_params()
{
   int i;
    ros::NodeHandle n;
/*
    ros::NodeHandle n_local("~");

    // Head pan/tilt parameters
    n_local.param("max_pan", max_pan_, 2.7);
    n_local.param("max_tilt", max_tilt_, 1.4);
    n_local.param("min_tilt", min_tilt_, -0.4);

    n_local.param("tuck_shoulder_pan", tuck_shoulder_pan_, .0);
    n_local.param("tuck_shoulder_tilt", tuck_shoulder_tilt_, 1.75);
    n_local.param("tuck_elbow_tilt", tuck_elbow_tilt_, -1.75);
    n_local.param("tuck_wrist_rotate", tuck_wrist_rotate_, .0);
    n_local.param("tuck_finger_left", tuck_finger_left_, .1);
    n_local.param("tuck_finger_right", tuck_finger_right_, .1);

    n_local.param("untuck_shoulder_pan", untuck_shoulder_pan_, .0);
    n_local.param("untuck_shoulder_tilt", untuck_shoulder_tilt_, 1.75);
    n_local.param("untuck_elbow_tilt", untuck_elbow_tilt_, -.01);
    n_local.param("untuck_wrist_rotate", untuck_wrist_rotate_, .0);
    n_local.param("untuck_finger_left", untuck_finger_left_, .1);
    n_local.param("untuck_finger_right", untuck_finger_right_, .1);
*/

   for (i = 0; i <= prlite_ax12commander::rfinger; i ++) {
       if(n.getParam(prlite_ax12param[i], prlite_ax12joint[i].tuck)) {
           std::stringstream ss;
           ss << prlite_ax12param[i] << ": " 
              << prlite_ax12joint[i].tuck;
           ROS_INFO("%s", ss.str().c_str());
       }

      if(n.getParam(prlite_ax12param[i+prlite_ax12commander::rfinger], 
           prlite_ax12joint[i].untuck)) {
        std::stringstream ss;
        ss << prlite_ax12param[i+prlite_ax12commander::rfinger] << ": " 
           << prlite_ax12joint[i].untuck;
        ROS_INFO("%s", ss.str().c_str());
      }
    }
}

// void prlite_ax12commander::motor_state_callback(const dynamixel_msgs::MotorStateList& motor_state_list)
void motor_state_callback(const dynamixel_msgs::MotorStateList& motor_state_list)
{
  int i, j;

  for (i = 0; i <= prlite_ax12commander::numjoints; i ++) {
    for (j = 0; j <= prlite_ax12commander::numjoints; j ++) {
      if (prlite_ax12joint[j].joint_state.motor_ids[0] == motor_state_list.motor_states[i].id) {
        prlite_ax12joint[j].motor_state[0] = motor_state_list.motor_states[i];
       } else if ((j == prlite_ax12commander::shouldertilt
          || j == prlite_ax12commander::shouldertiltR
          || j == prlite_ax12commander::elbowtilt
          || j == prlite_ax12commander::elbowtiltR)
          && (prlite_ax12joint[j].joint_state.motor_ids[0] == motor_state_list.motor_states[i].id)) {
            prlite_ax12joint[j].motor_state[1] = motor_state_list.motor_states[i];
       }
    }
  }
}

int prlite_ax12commander::get_joint_by_name(std::string name)
{
  int joint;

    if (name == "shoulder_pan_joint") { 
        joint = 0;
    } else if (name == "shoulder_tilt_joint") {
        joint = 1;
    } else if (name == "elbow_tilt_joint") {
        joint = 2;
    } else if (name == "wrist_rotate_joint") {
        joint = 3;
    } else if (name == "finger_left_joint") {
        joint = 4;
    } else if (name == "finger_right_joint") {
        joint = 5;
    } else if (name == "shoulder_pan_jointR") { 
        joint = 6;
    } else if (name == "shoulder_tilt_jointR") {
        joint = 7;
    } else if (name == "elbow_tilt_jointR") {
        joint = 8;
    } else if (name == "wrist_rotate_jointR") {
        joint = 9;
    } else if (name == "finger_left_jointR") {
        joint = 10;
    } else if (name == "finger_right_jointR") {
        joint = 11;
    } else if (name == "kinect_pan_joint") {
        joint = 12;
    } else if (name == "kinect_tilt_joint") {
        joint = 13;
    } else if (name == "laser_tilt_joint") {
        joint = 14;
    } else {
       joint = -1;
    }
   return(joint);
}

// void prlite_ax12commander::joint_state_callback(const dynamixel_msgs::JointState& joint_msg_ptr)
void joint_state_callback(const dynamixel_msgs::JointState& joint_msg_ptr)
{
    int joint;

    if (joint_msg_ptr.name == "shoulder_pan_joint") {
        joint = 0;
    } else if (joint_msg_ptr.name == "shoulder_tilt_joint") {
        joint = 1;
    } else if (joint_msg_ptr.name == "elbow_tilt_joint") {
        joint = 2;
    } else if (joint_msg_ptr.name == "wrist_rotate_joint") {
        joint = 3;
    } else if (joint_msg_ptr.name == "finger_left_joint") {
        joint = 4;
    } else if (joint_msg_ptr.name == "finger_right_joint") {
        joint = 5;
    } else if (joint_msg_ptr.name == "shoulder_pan_jointR") {
        joint = 6;
    } else if (joint_msg_ptr.name == "shoulder_tilt_jointR") {
        joint = 7;
    } else if (joint_msg_ptr.name == "elbow_tilt_jointR") {
        joint = 8;
    } else if (joint_msg_ptr.name == "wrist_rotate_jointR") {
        joint = 9;
    } else if (joint_msg_ptr.name == "finger_left_jointR") {
        joint = 10;
    } else if (joint_msg_ptr.name == "finger_right_jointR") {
        joint = 11;
    } else if (joint_msg_ptr.name == "kinect_pan_joint") {
        joint = 12;
    } else if (joint_msg_ptr.name == "kinect_tilt_joint") {
        joint = 13;
    } else if (joint_msg_ptr.name == "laser_tilt_joint") {
        joint = 14;
    } else {
       joint = -1;
    }

    prlite_ax12joint[joint].joint_state = joint_msg_ptr;
}

void prlite_ax12commander::init()
{
   int i;
   ros::NodeHandle n;
   std::string controller;
   

   for (i = 0; i < prlite_ax12commander::numjoints; i ++) {
      controller = prlite_ax12controllername[i] + "command";
      prlite_ax12joint[i].controller = n.advertise<std_msgs::Float64>(controller, 1000);

      controller = prlite_ax12controllername[i] + "torque_enable";
      prlite_ax12joint[i].torque = n.advertise<std_msgs::Bool>(controller, 1000);

      leftArmPublisher = n.advertise<prlite_kinematics::SphereCoordinate>("/armik/n0/position", 1000);
      rightArmPublisher = n.advertise<prlite_kinematics::SphereCoordinate>("/armik/n1/position", 1000);

    }

   prlite_ax12commander::get_params();


   prlite_ax12commander::arm_head_mode(POSITION_MODE, POSITION_MODE, POSITION_MODE);
   for (i = 0; i < prlite_ax12commander::numjoints; i ++) {
      prlite_ax12joint[i].minpos = -1.75;
      prlite_ax12joint[i].maxpos =  1.75;
      prlite_ax12joint[i].incrpos =  0.1;
      if (i == prlite_ax12commander::shoulderpanR
         || i == prlite_ax12commander::shoulderpan) {
        prlite_ax12joint[i].minpos = -1.50;
        prlite_ax12joint[i].maxpos =  1.50;
      }
      if (i == prlite_ax12commander::lfinger ) {
        prlite_ax12joint[i].minpos = 0;
        prlite_ax12joint[i].maxpos =  1.1;
      } else if (i == prlite_ax12commander::rfinger) {
        prlite_ax12joint[i].minpos = -1.1;
        prlite_ax12joint[i].maxpos =  0;
      } else if (i == prlite_ax12commander::lfingerR) {
        prlite_ax12joint[i].minpos = 0;
        prlite_ax12joint[i].maxpos =  -1.1;
      } else if (i == prlite_ax12commander::rfingerR) {
        prlite_ax12joint[i].minpos = 1.1;
        prlite_ax12joint[i].maxpos =  0;
      }
      prlite_ax12joint[i].nxtpos = prlite_ax12joint[i].joint_state.current_pos;
      prlite_ax12joint[i].desiredpos = prlite_ax12joint[i].joint_state.current_pos;
    }
    {
      // arms safley put back position
      prlite_ax12joint[shoulderpan].tuck =  0.0;
      prlite_ax12joint[shouldertilt].tuck =  1.75;
      prlite_ax12joint[elbowtilt].tuck = .01;
      prlite_ax12joint[wristrot].tuck =  0.0;
      prlite_ax12joint[lfinger].tuck =  0.9;
      prlite_ax12joint[rfinger].tuck =  -0.9;

      prlite_ax12joint[shoulderpanR].tuck =  0.0;
      prlite_ax12joint[shouldertiltR].tuck =  1.75;
      prlite_ax12joint[elbowtiltR].tuck =  -.01;
      prlite_ax12joint[wristrotR].tuck =  0.0;
      prlite_ax12joint[lfingerR].tuck =  -0.9;
      prlite_ax12joint[rfingerR].tuck =  0.9;

      // cobra position
      prlite_ax12joint[shoulderpan].untuck =  0.0;
      prlite_ax12joint[shouldertilt].untuck =  1.75;
      prlite_ax12joint[elbowtilt].untuck =  -1.75;
      prlite_ax12joint[wristrot].untuck =  0.0;
      prlite_ax12joint[lfinger].untuck =  0.1;
      prlite_ax12joint[rfinger].untuck =  -0.1;

      prlite_ax12joint[shoulderpanR].untuck =  0.0;
      prlite_ax12joint[shouldertiltR].untuck =  1.75;
      prlite_ax12joint[elbowtiltR].untuck =  -1.75;
      prlite_ax12joint[wristrotR].untuck =  0.0;
      prlite_ax12joint[lfingerR].untuck =  -0.1;
      prlite_ax12joint[rfingerR].untuck =  0.1;

      // hands up position
/* OLD : wrong
      prlite_ax12joint[shoulderpan].kinect_callobrate =  1.50;
      prlite_ax12joint[shouldertilt].kinect_callobrate =  1.15;
      prlite_ax12joint[elbowtilt].kinect_callobrate =  .65;
      prlite_ax12joint[wristrot].kinect_callobrate =  0.0;
      prlite_ax12joint[lfinger].kinect_callobrate =  0.9;
      prlite_ax12joint[rfinger].kinect_callobrate =  -0.9;

      prlite_ax12joint[shoulderpanR].kinect_callobrate =  -1.50;
      prlite_ax12joint[shouldertiltR].kinect_callobrate =  1.15;
      prlite_ax12joint[elbowtiltR].kinect_callobrate =  .65;
      prlite_ax12joint[wristrotR].kinect_callobrate =  0.0;
      prlite_ax12joint[lfingerR].kinect_callobrate =  -0.9;
      prlite_ax12joint[rfingerR].kinect_callobrate =  0.9;
*/
      prlite_ax12joint[shoulderpan].kinect_callobrate =  1.50;
      prlite_ax12joint[shouldertilt].kinect_callobrate =  -0.275868;
      prlite_ax12joint[elbowtilt].kinect_callobrate =  1.75;
      prlite_ax12joint[wristrot].kinect_callobrate =  -1.1;
      prlite_ax12joint[lfinger].kinect_callobrate =  0.9;
      prlite_ax12joint[rfinger].kinect_callobrate =  -0.9;

      prlite_ax12joint[shoulderpanR].kinect_callobrate =  -1.50;
      prlite_ax12joint[shouldertiltR].kinect_callobrate =  -0.275868;
      prlite_ax12joint[elbowtiltR].kinect_callobrate =  1.75;
      prlite_ax12joint[wristrotR].kinect_callobrate =  0.9;
      prlite_ax12joint[lfingerR].kinect_callobrate =  -0.9;
      prlite_ax12joint[rfingerR].kinect_callobrate =  0.9;


    }
    TorsoInit();
}


void prlite_ax12commander::go_directly_to_pos()
{
}
 

int prlite_ax12commander::prlite_ax12commander::move_to_next_pos(int joint, double desired_pos, int dir)
{
     /* turn off, reset, no retry? */
     return(1);
}


void prlite_ax12commander::move_to_desired_pos()
{
     ros::Duration(0.1).sleep();
}

void prlite_ax12commander::set_desired_pos(int joint, double desired_pos)
{
  std_msgs::Float64 desired_pos_pub;
  if (joint == shouldertiltR || joint == lfingerR || joint == rfingerR) 
          desired_pos *= -1;
  if (prlite_ax12joint[joint].desiredpos == desired_pos) return;
  ROS_INFO_STREAM("set_desired_pos " << joint << " " << desired_pos);
  desired_pos_pub.data = desired_pos;
  if (/* joint != lfingerR && joint != rfingerR */ true)  // bad motors
    prlite_ax12joint[joint].controller.publish(desired_pos_pub);
  prlite_ax12joint[joint].desiredpos = desired_pos;
}

void prlite_ax12commander::tuck()
{
   int i;

   for (i = 0; i <= prlite_ax12commander::rfingerR; i ++) {
     set_desired_pos(i, prlite_ax12joint[i].tuck);
     // ros::Duration(0.1).sleep();
  }
}

void prlite_ax12commander::untuck()
{
   int i;

   for (i = 0; i <= prlite_ax12commander::rfingerR; i ++) {
     set_desired_pos(i, prlite_ax12joint[i].untuck);
     // ros::Duration(0.1).sleep();
  }
}

void prlite_ax12commander::kinect_callobration_pos()
{
   int i;

   for (i = 0; i <= prlite_ax12commander::rfingerR; i ++) {
     set_desired_pos(i, prlite_ax12joint[i].kinect_callobrate);
     // ros::Duration(0.1).sleep();
  }
}


void prlite_ax12commander::arm_head_mode(int left_arm_control_mode, int right_arm_control_mode, int head_control_mode)
{
   int i;
   std_msgs::Int32 torque_pub;

   if (left_arm_control_mode == UNCHANGED_MODE) {
   } else if (left_arm_control_mode == MANNEQUIN_MODE || left_arm_control_mode == NO_CONTROLLER_MODE) {
     /* turn off torque */
     torque_pub.data = 0;
     for (i = 0; i <= prlite_ax12commander::rfinger; i ++) {
       prlite_ax12joint[i].torque.publish(torque_pub);
     }
   } else if (left_arm_control_mode == POSITION_MODE) {
     torque_pub.data = 1;
     for (i = 0; i <= prlite_ax12commander::rfinger; i ++) {
       prlite_ax12joint[i].torque.publish(torque_pub);
     }
     ros::Duration(0.1).sleep();
     for (i = 0; i <= prlite_ax12commander::rfinger; i ++) {
       ROS_INFO("joint[%d] set to %f", i, prlite_ax12joint[i].joint_state.current_pos);
     }
   }
   if (right_arm_control_mode == UNCHANGED_MODE) {
   } else if (right_arm_control_mode == MANNEQUIN_MODE || right_arm_control_mode == NO_CONTROLLER_MODE) {
     /* turn off torque */
     torque_pub.data = 0;
     for (i = prlite_ax12commander::shoulderpanR; i <= prlite_ax12commander::rfingerR; i ++) {
       prlite_ax12joint[i].torque.publish(torque_pub);
     }
   } else if (right_arm_control_mode == POSITION_MODE) {
     torque_pub.data = 1;
     for (i = prlite_ax12commander::shoulderpanR; i <= prlite_ax12commander::rfingerR; i ++) {
       prlite_ax12joint[i].torque.publish(torque_pub);
     }
     ros::Duration(0.1).sleep();
     for (i = prlite_ax12commander::shoulderpanR; i <= prlite_ax12commander::rfingerR; i ++) {
       ROS_INFO("joint[%d] set to %f", i, prlite_ax12joint[i].joint_state.current_pos);
     }
   }
    
   if (head_control_mode == UNCHANGED_MODE) {
   } else if (head_control_mode == MANNEQUIN_MODE || head_control_mode == NO_CONTROLLER_MODE) {
     /* turn off torque */
     torque_pub.data = 0;
     for (i = prlite_ax12commander::kinectpan; i < prlite_ax12commander::numjoints; i ++) {
       prlite_ax12joint[i].torque.publish(torque_pub);
     }
   } else if (head_control_mode == POSITION_MODE) {
     torque_pub.data = 1;
     for (i = prlite_ax12commander::kinectpan; i < prlite_ax12commander::numjoints; i ++) {
       prlite_ax12joint[i].torque.publish(torque_pub);
     }
     ros::Duration(0.1).sleep();
     for (i = prlite_ax12commander::kinectpan; i < prlite_ax12commander::numjoints; i ++) {
	     prlite_ax12joint[i].nxtpos = prlite_ax12joint[i].joint_state.current_pos;
       prlite_ax12joint[i].desiredpos = prlite_ax12joint[i].joint_state.current_pos;
       ROS_INFO("joint[%d] set to %f", i, prlite_ax12joint[i].joint_state.current_pos);
     }
   }   
}


void prlite_ax12commander::set_arm_goal
  (double rx, double ry, double rz, double lx, double ly, double lz)
{
	// Calculate positions
	prlite_kinematics::SphereCoordinate lCoord;
	prlite_kinematics::SphereCoordinate rCoord;

	rCoord.radius = sqrt(rx*rx + ry*ry + rz*rz);
	lCoord.radius = sqrt(lx*lx + ly*ly + lz*lz);

	lCoord.theta = -atan2(lx, lz );
	rCoord.theta = -atan2(rx, rz);
	if(lCoord.theta > 1.0) lCoord.theta = 1.0;
	if(lCoord.theta < -1.0) lCoord.theta = -1.0;
	if(rCoord.theta > 1.0) rCoord.theta = 1.0;
	if(rCoord.theta < -1.0) rCoord.theta = -1.0;

	lCoord.phi = -atan2(ly, lx);
	rCoord.phi = -atan2(ry, rx);
	if(lCoord.phi > 1.25) lCoord.phi = 1.25;
	if(lCoord.phi < -0.33) lCoord.phi = -0.33;
	if(rCoord.phi > 1.2) rCoord.phi = 1.25;
	if(rCoord.phi < -0.33) rCoord.phi = -0.33;

	ROS_INFO("set_arm_goal: Left (%lf,%lf,%lf), Right (%lf,%lf,%lf)", lCoord.radius, lCoord.theta, lCoord.phi, rCoord.radius, rCoord.theta, rCoord.phi);

	// Publish to arms
	leftArmPublisher.publish(rCoord);
	rightArmPublisher.publish(lCoord);
}


void prlite_ax12commander::JointCommand(int joint, double vel)
{
    double pos;

      if (joint == shouldertiltR) 
         pos = vel * prlite_ax12joint[joint].incrpos - 
            prlite_ax12joint[joint].desiredpos;
      else
         pos = vel * prlite_ax12joint[joint].incrpos + 
            prlite_ax12joint[joint].desiredpos;
      if (pos < prlite_ax12joint[joint].minpos)
        pos = prlite_ax12joint[joint].minpos;
      if (pos > prlite_ax12joint[joint].maxpos)
        pos = prlite_ax12joint[joint].maxpos;
      set_desired_pos(joint, pos);
} 

void prlite_ax12commander::WristCommand(double right_wrist_vel, double left_wrist_vel)
{
   JointCommand( prlite_ax12commander::wristrotR, right_wrist_vel);
   JointCommand( prlite_ax12commander::wristrot, left_wrist_vel);
}

void prlite_ax12commander::ArmCommand(
       double r_x_vel, double r_y_vel, double r_z_vel, 
       double l_x_vel, double l_y_vel, double l_z_vel)
{
   JointCommand( prlite_ax12commander::shoulderpanR, r_x_vel);
   JointCommand( prlite_ax12commander::shouldertiltR, r_y_vel);
   JointCommand( prlite_ax12commander::elbowtiltR, r_z_vel);
   JointCommand( prlite_ax12commander::shoulderpan, l_x_vel);
   JointCommand( prlite_ax12commander::shouldertilt, l_y_vel);
   JointCommand( prlite_ax12commander::elbowtilt, l_z_vel);
}

void prlite_ax12commander::HeadCommand(double pan_vel, double tilt_vel)
{
   JointCommand( prlite_ax12commander::kinectpan, pan_vel);
   JointCommand( prlite_ax12commander::kinecttilt, tilt_vel);
}

