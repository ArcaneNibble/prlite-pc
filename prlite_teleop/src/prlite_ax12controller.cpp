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

const double X_MULT = 7.51913116; // speed is ticks per interval and interval is 1/10 sec, so should be WHEEL_TICKS_PER_METER / 10
const double TH_MULT = 5;
const uint16_t TORSO_UP = 1000;
const uint16_t TORSO_MID= 500;
const uint16_t TORSO_DOWN = 0;
const uint16_t ARM_DOWN = 1000;
const uint16_t ARM_UP = 0;
const uint16_t LINACT_PRECISION = 15;

ros::Publisher linact_pub;
ros::Subscriber linact_sub;
ros::Publisher leftArmPublisher;
ros::Publisher rightArmPublisher;
#define LINACT_TORSO        0
#define LINACT_WHEELS       1
#define LINACT_RIGHT_ARM    2
#define LINACT_LEFT_ARM     3
#define NUM_LINACT          4
#define LINACT_TORSO_ID     0x0C 
#define LINACT_WHEELS_ID    0x0D 
#define LINACT_RIGHT_ARM_ID 0x0E 
#define LINACT_LEFT_ARM_ID  0x0F 
int linact_id[NUM_LINACT] = 
{LINACT_TORSO_ID, LINACT_WHEELS_ID, LINACT_RIGHT_ARM_ID, LINACT_LEFT_ARM_ID};
bool linact_arrived[NUM_LINACT];
bool linact_new_goal[NUM_LINACT];
bool linact_goal_last[NUM_LINACT];
int linact_goal[NUM_LINACT] = {TORSO_DOWN, 1000, ARM_DOWN, ARM_DOWN};

void LinactPublish(int this_linact)
{
	uint16_t itmp;

  if (linact_arrived[this_linact] 
     && linact_goal[this_linact] == linact_goal_last[this_linact])
  {
    // publish wheel commands
    ROS_INFO("same goal: linact %d at %d", this_linact, linact_goal[this_linact]);
  }
  else 
  {
    if (linact_goal[this_linact] != linact_goal_last[this_linact])
    {
      // publish linear actuator command
      packets_485net::packet_485net_dgram linact_cmd;
      linact_cmd.destination = linact_id[this_linact];
          linact_cmd.source = 0xF0;
          linact_cmd.sport = 7;
          linact_cmd.dport = 1;

          itmp = linact_goal[this_linact] > LINACT_PRECISION ? linact_goal[this_linact] - LINACT_PRECISION : 0;
          linact_cmd.data.push_back(itmp & 0xFF);
          linact_cmd.data.push_back((itmp >> 8) & 0xFF);
          itmp = linact_goal[this_linact] < (1023-LINACT_PRECISION) ? linact_goal[this_linact] + LINACT_PRECISION : 1023;
          linact_cmd.data.push_back(itmp & 0xFF);
          linact_cmd.data.push_back((itmp >> 8) & 0xFF);

      linact_pub.publish(linact_cmd);
      linact_goal_last[this_linact] = linact_goal[this_linact];
      linact_new_goal[this_linact] = true;
      linact_arrived[this_linact] = false;
      ROS_INFO("linact %d at %d", this_linact, linact_goal[this_linact]);
    }
  }
}

void LinactCallback(const packets_485net::packet_485net_dgram& linear_actuator_status)
{
    int this_linact = NUM_LINACT;
    int i;

    for (i = 0; i < NUM_LINACT; i++) { 
      if(linear_actuator_status.source == linact_id[i]) {
        this_linact = i;
        break;
      }
    }
    if (this_linact == NUM_LINACT) {
      // ROS_INFO("invalid source %d", linear_actuator_status.source);
      return;
    }
    if(!(linear_actuator_status.destination == 0xF0
       || linear_actuator_status.destination == 0x00))
	return;
    if(linear_actuator_status.dport != 7) {
      ROS_INFO("invalid dport");
      return;
    }
    if(linear_actuator_status.data.size() != 7) {
      ROS_INFO("invalid size");
      return;
    }
    linact_arrived[this_linact] = linear_actuator_status.data[4+2];
    if (linact_new_goal[this_linact]) 
      // if new linear actuator goal then linact_arrived may take a 
      // few frames to update
    {
      if (!linact_arrived[this_linact])
        linact_new_goal[this_linact] = false;
      else
        linact_arrived[this_linact] = false;
    }
    // hack for if linear actuator isn't working
    //linact_arrived[this_linact] = true; 
    LinactPublish(this_linact);
}


void LinactInit(void)
{
  ros::NodeHandle n;
  int i;

  linact_sub = n.subscribe("net_485net_incoming_dgram", 1000, LinactCallback);
  linact_pub = n.advertise<packets_485net::packet_485net_dgram>("net_485net_outgoing_dgram", 1000);
  for (i = 0 ; i < NUM_LINACT; i++) {
    linact_goal_last[i] = 10000;
    linact_arrived[i] = false; 
    linact_new_goal[i] = false; 
  }
}

void prlite_ax12commander::setTorsoGoal(int goal)
{
  static int direction = 0;

  if (0 == goal) return;
  linact_goal[LINACT_TORSO] -= goal*LINACT_PRECISION;
  if (linact_goal[LINACT_TORSO] > TORSO_UP)
    linact_goal[LINACT_TORSO] = TORSO_UP;
  else if (linact_goal[LINACT_TORSO] < TORSO_DOWN)
    linact_goal[LINACT_TORSO] = TORSO_DOWN;
  if (
     !(direction == goal 
     && !linact_arrived[LINACT_TORSO]
     && linact_new_goal[LINACT_TORSO]
     )) {
    LinactPublish(LINACT_TORSO);
    direction = goal;
  } else {
    ROS_INFO("skip torso ");
  }
  ROS_INFO_STREAM("Torso: " << linact_goal[LINACT_TORSO]);
}

void prlite_ax12commander::setShoulderGoal(int right_goal, int left_goal)
{
  static int direction[2] = {0, 0};

  if(0 == left_goal && 0 == right_goal) return;
  linact_goal[LINACT_RIGHT_ARM] -= right_goal*LINACT_PRECISION;
  linact_goal[LINACT_LEFT_ARM] -= left_goal*LINACT_PRECISION;
  if (linact_goal[LINACT_RIGHT_ARM] < ARM_UP)
    linact_goal[LINACT_RIGHT_ARM] = ARM_UP;
  else if (linact_goal[LINACT_RIGHT_ARM] > ARM_DOWN)
    linact_goal[LINACT_RIGHT_ARM] = ARM_DOWN;
  if (linact_goal[LINACT_LEFT_ARM] < ARM_UP)
    linact_goal[LINACT_LEFT_ARM] = ARM_UP;
  else if (linact_goal[LINACT_LEFT_ARM] > ARM_DOWN)
    linact_goal[LINACT_LEFT_ARM] = ARM_DOWN;
  if (
     !(direction[0] == right_goal && !linact_arrived[LINACT_RIGHT_ARM]
     && linact_new_goal[LINACT_RIGHT_ARM]
     )) {
    LinactPublish(LINACT_RIGHT_ARM);
    direction[0] = right_goal;
  } else {
    // ROS_INFO("skip right Shoulder ");
  }
  if (
     !(direction[1] == left_goal && !linact_arrived[LINACT_LEFT_ARM]
     && linact_new_goal[LINACT_LEFT_ARM]
     )) {
    LinactPublish(LINACT_LEFT_ARM);
    direction[1] = left_goal;
  } else {
    // ROS_INFO("skip Left Shoulder ");
  }
  // ROS_INFO_STREAM("Left Shoulder: " << linact_goal[LINACT_LEFT_ARM] 
  //    << "  Right Shoulder: " << linact_goal[LINACT_RIGHT_ARM]);
}

static std::string prlite_ax12controllername[] = {
   "shoulder_pan_controllerL/",
   /* "shoulder_tilt_controllerL/", */
   "elbow_pan_controllerL/",
   "elbow_tilt_controllerL/",
   "wrist_rotate_controllerL/", 
   "wrist_tilt_controllerL/", 
   "finger_left_controllerL/",
   "finger_right_controllerL/",
   "shoulder_pan_controllerR/",
   /* "shoulder_tilt_controllerR/", */
   "elbow_pan_controllerR/",
   "elbow_tilt_controllerR/",
   "wrist_rotate_controllerR/", 
   "wrist_tilt_controllerR/", 
   "finger_left_controllerR/",
   "finger_right_controllerR/",
   "kinect_pan_controller/",
   "kinect_tilt_controller/",
   "laser_tilt_controller/"
};

static std::string prlite_ax12param[] = {
   "/prlite_ax12/shoulder_pan_tuck", 
   /* "/prlite_ax12/shoulder_tilt_tuck",  */
   "/prlite_ax12/elbow_pan_tuck", 
   "/prlite_ax12/elbow_tilt_tuck",
   "/prlite_ax12/wrist_rotate_tuck", 
   "/prlite_ax12/wrist_tilt_tuck", 
   "/prlite_ax12/finger_left_tuck",
   "/prlite_ax12/finger_right_tuck",
   "/prlite_ax12/kinect_pan_tuck",
   "/prlite_ax12/kinect_tilt_tuck",
   "/prlite_ax12/laser_tilt_tuck",
   "/prlite_ax12/shoulder_pan_untuck", 
   /* "/prlite_ax12/shoulder_tilt_untuck", */
   "/prlite_ax12/elbow_pan_tuck", 
   "/prlite_ax12/elbow_tilt_untuck",
   "/prlite_ax12/wrist_rotate_untuck", 
   "/prlite_ax12/wrist_tilt_untuck", 
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
    n_local.param("tuck_shoulder_tilt", tuck_shoulder_tilt_, 2.61);
    n_local.param("tuck_elbow_tilt", tuck_elbow_tilt_, -3.61);
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
       } else if ((j == prlite_ax12commander::elbowtilt
          || j == prlite_ax12commander::elbowtiltR
          || j == prlite_ax12commander::wristtilt
          || j == prlite_ax12commander::wristtiltR)
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
    } else if (name == "elbow_pan_joint") {
        joint = 1;
    } else if (name == "elbow_tilt_joint") {
        joint = 2;
    } else if (name == "wrist_rotate_joint") {
        joint = 3;
    } else if (name == "wrist_tilt_joint") {
        joint = 4;
    } else if (name == "finger_left_joint") {
        joint = 5;
    } else if (name == "finger_right_joint") {
        joint = 6;
    } else if (name == "shoulder_pan_jointR") { 
        joint = 7;
    } else if (name == "elbow_pan_jointR") {
        joint = 8;
    } else if (name == "elbow_tilt_jointR") {
        joint = 9;	
    } else if (name == "wrist_rotate_jointR") {
        joint = 10;
    } else if (name == "wrist_tilt_jointR") {
        joint = 11;
    } else if (name == "finger_left_jointR") {
        joint = 12;
    } else if (name == "finger_right_jointR") {
        joint = 13;
    } else if (name == "kinect_pan_joint") {
        joint = 14;
    } else if (name == "kinect_tilt_joint") {
        joint = 15;
    } else if (name == "laser_tilt_joint") {
        joint = 16;
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
    } else if (joint_msg_ptr.name == "elbow_pan_joint") {
        joint = 1;
    } else if (joint_msg_ptr.name == "elbow_tilt_joint") {
        joint = 2;
    } else if (joint_msg_ptr.name == "wrist_rotate_joint") {
        joint = 3;
    } else if (joint_msg_ptr.name == "wrist_tilt_joint") {
        joint = 4;
    } else if (joint_msg_ptr.name == "finger_left_joint") {
        joint = 5;
    } else if (joint_msg_ptr.name == "finger_right_joint") {
        joint = 6;
    } else if (joint_msg_ptr.name == "shoulder_pan_jointR") {
        joint = 7;
    } else if (joint_msg_ptr.name == "elbow_pan_jointR") {
        joint = 8;
    } else if (joint_msg_ptr.name == "elbow_tilt_jointR") {
        joint = 9;
    } else if (joint_msg_ptr.name == "wrist_rotate_jointR") {
        joint = 10;
    } else if (joint_msg_ptr.name == "wrist_tilt_jointR") {
        joint = 11;
    } else if (joint_msg_ptr.name == "finger_left_jointR") {
        joint = 12;
    } else if (joint_msg_ptr.name == "finger_right_jointR") {
	joint = 13;
    } else if (joint_msg_ptr.name == "kinect_pan_joint") {
        joint = 14;
    } else if (joint_msg_ptr.name == "kinect_tilt_joint") {
        joint = 15;
    } else if (joint_msg_ptr.name == "laser_tilt_joint") {
        joint = 16;
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
      prlite_ax12joint[i].minpos = -3.14;
      prlite_ax12joint[i].maxpos =  3.14;
      prlite_ax12joint[i].incrpos =  0.1;
      if (i == prlite_ax12commander::wristtiltR ||
         i == prlite_ax12commander::wristtilt) {
        prlite_ax12joint[i].minpos = -2;
        prlite_ax12joint[i].maxpos =  2;
      }
      if (i == prlite_ax12commander::elbowtiltR ||
         i == prlite_ax12commander::elbowtilt) {
        prlite_ax12joint[i].minpos = -.7;
        prlite_ax12joint[i].maxpos =  3.14;
      }

      if (i == prlite_ax12commander::shoulderpanR ||
         i == prlite_ax12commander::shoulderpan) {
        prlite_ax12joint[i].minpos = -1.50;
        prlite_ax12joint[i].maxpos =  1.50;
        prlite_ax12joint[i].incrpos =  0.02;
      }
      if (i == prlite_ax12commander::kinectpan) {
        prlite_ax12joint[i].incrpos =  0.02;
      }
      if (i == prlite_ax12commander::kinecttilt) {
        prlite_ax12joint[i].minpos = -1.50;
        prlite_ax12joint[i].maxpos =  1.50;
      }
      if (i == prlite_ax12commander::lfinger ) {
        prlite_ax12joint[i].minpos = 0;
        prlite_ax12joint[i].maxpos =  1.1;
      } else if (i == prlite_ax12commander::rfinger) {
        prlite_ax12joint[i].minpos = 0;
        prlite_ax12joint[i].maxpos =  1.1;
      } else if (i == prlite_ax12commander::lfingerR) {
        prlite_ax12joint[i].minpos = 0;
        prlite_ax12joint[i].maxpos =  1.1;
      } else if (i == prlite_ax12commander::rfingerR) {
        prlite_ax12joint[i].minpos =  0;
        prlite_ax12joint[i].maxpos = 1.1;
      }
      prlite_ax12joint[i].nxtpos = prlite_ax12joint[i].joint_state.current_pos;
      prlite_ax12joint[i].desiredpos = prlite_ax12joint[i].joint_state.current_pos;
    }
    {
      // arms safley put back position
      prlite_ax12joint[prlite_ax12commander::shoulderpan].tuck =  0.0;
      // prlite_ax12joint[prlite_ax12commander::shouldertilt].tuck =  1.75;
      prlite_ax12joint[prlite_ax12commander::elbowpan].tuck = .00;
      prlite_ax12joint[prlite_ax12commander::elbowtilt].tuck = 1.4;
      prlite_ax12joint[prlite_ax12commander::wristrot].tuck =  0.0;
      prlite_ax12joint[prlite_ax12commander::wristtilt].tuck =  -1.7;
      prlite_ax12joint[prlite_ax12commander::lfinger].tuck =  0.1;
      prlite_ax12joint[prlite_ax12commander::rfinger].tuck =  -0.1;

      prlite_ax12joint[prlite_ax12commander::shoulderpan].tuck =  0.0;
      // prlite_ax12joint[prlite_ax12commander::shouldertiltR].tuck =  1.75;
      prlite_ax12joint[prlite_ax12commander::elbowpanR].tuck =  0.0;
      prlite_ax12joint[prlite_ax12commander::elbowtiltR].tuck =  1.4;
      prlite_ax12joint[prlite_ax12commander::wristrotR].tuck =  0.0;
      prlite_ax12joint[prlite_ax12commander::wristtiltR].tuck =  -1.7;
      prlite_ax12joint[prlite_ax12commander::lfingerR].tuck =  -0.1;
      prlite_ax12joint[prlite_ax12commander::rfingerR].tuck =  0.1;

      // cobra position or keep same as tuck
      prlite_ax12joint[prlite_ax12commander::shoulderpan].untuck =  0.0;
      // prlite_ax12joint[prlite_ax12commander::shouldertilt].untuck =  1.75;
      prlite_ax12joint[prlite_ax12commander::elbowpan].untuck =  0.0;
      prlite_ax12joint[prlite_ax12commander::elbowtilt].untuck =  1.4;
      prlite_ax12joint[prlite_ax12commander::wristrot].untuck =  0.0;
      prlite_ax12joint[prlite_ax12commander::wristtilt].untuck =  -1.7;
      prlite_ax12joint[prlite_ax12commander::lfinger].untuck =  0.1;
      prlite_ax12joint[prlite_ax12commander::rfinger].untuck =  -0.1;

      prlite_ax12joint[prlite_ax12commander::shoulderpanR].untuck =  0.0;
      // prlite_ax12joint[prlite_ax12commander::shouldertiltR].untuck =  1.75;
      prlite_ax12joint[prlite_ax12commander::elbowpanR].untuck =  0.0;
      prlite_ax12joint[prlite_ax12commander::elbowtiltR].untuck =  1.4;
      prlite_ax12joint[prlite_ax12commander::wristrotR].untuck =  0.0;
      prlite_ax12joint[prlite_ax12commander::wristtiltR].untuck =  -1.7;
      prlite_ax12joint[prlite_ax12commander::lfingerR].untuck =  -0.1;
      prlite_ax12joint[prlite_ax12commander::rfingerR].untuck =  0.1;

      // hands up position
      prlite_ax12joint[shoulderpan].kinect_callobrate =  -1.50;
      // prlite_ax12joint[shouldertilt].kinect_callobrate =  -0.275868;
      prlite_ax12joint[elbowpan].kinect_callobrate =  0;
      prlite_ax12joint[elbowtilt].kinect_callobrate =  0.15;
      prlite_ax12joint[wristrot].kinect_callobrate =  0;
      prlite_ax12joint[wristrot].kinect_callobrate =  0;
      prlite_ax12joint[lfinger].kinect_callobrate =  0.9;
      prlite_ax12joint[rfinger].kinect_callobrate =  -0.9;

      prlite_ax12joint[shoulderpanR].kinect_callobrate =  1.50;
      // prlite_ax12joint[shouldertiltR].kinect_callobrate =  -0.275868;
      prlite_ax12joint[elbowpanR].kinect_callobrate =  0;
      prlite_ax12joint[elbowtiltR].kinect_callobrate =  0.15;
      prlite_ax12joint[wristrotR].kinect_callobrate =  0;
      prlite_ax12joint[wristtiltR].kinect_callobrate =  0.9;
      prlite_ax12joint[lfingerR].kinect_callobrate =  -0.9;
      prlite_ax12joint[rfingerR].kinect_callobrate =  0.9;


    }
    LinactInit();
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
  if (joint == lfingerR  || joint == rfingerR)
          desired_pos *= -1;
  if (prlite_ax12joint[joint].desiredpos == desired_pos) return;
/*
  if (joint == 8 && desired_pos >= -.1 && desired_pos <= .1)
    return;  // joystick misbehaves 
*/
  ROS_INFO_STREAM("set_desired_pos " << joint << " " << desired_pos);
  desired_pos_pub.data = desired_pos;
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
   setShoulderGoal(0.0, 0.0);
}

void prlite_ax12commander::untuck()
{
   int i;

   for (i = 0; i <= prlite_ax12commander::rfingerR; i ++) {
     set_desired_pos(i, prlite_ax12joint[i].untuck);
     // ros::Duration(0.1).sleep();
   }
   setShoulderGoal(0.0, 0.0);
}

void prlite_ax12commander::kinect_callobration_pos()
{
   int i;

   for (i = 0; i <= prlite_ax12commander::rfingerR; i ++) {
     set_desired_pos(i, prlite_ax12joint[i].kinect_callobrate);
     // ros::Duration(0.1).sleep();
   }
   setShoulderGoal(1000, 1000);
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
     for (i = prlite_ax12commander::shoulderpanR; i <= prlite_ax12commander::rfingerR; i ++) 
     {
       prlite_ax12joint[i].torque.publish(torque_pub);
     }
   } else if (right_arm_control_mode == POSITION_MODE) {
     torque_pub.data = 1;
     for (i = prlite_ax12commander::shoulderpanR; i <= prlite_ax12commander::rfingerR; i ++) 
     {
       prlite_ax12joint[i].torque.publish(torque_pub);
     }
     ros::Duration(0.1).sleep();
     for (i = prlite_ax12commander::shoulderpanR; i <= prlite_ax12commander::rfingerR; i ++) 
     {
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

void prlite_ax12commander::ShoulderCommand(double right_shoulderpan_vel, double left_shoulderpan_vel)
{
   // JointCommand( prlite_ax12commander::shoulderpanR, right_shoulderpan_vel);
   // JointCommand( prlite_ax12commander::shoulderpan, left_shoulderpan_vel);
}

void prlite_ax12commander::ArmCommand(
       double r_x_vel, double r_y_vel, double r_z_vel, 
       double l_x_vel, double l_y_vel, double l_z_vel)
{
   JointCommand( prlite_ax12commander::elbowpanR, r_x_vel);
   JointCommand( prlite_ax12commander::elbowtiltR, r_y_vel);
   JointCommand( prlite_ax12commander::wristtiltR, r_z_vel);
   JointCommand( prlite_ax12commander::elbowpan, l_x_vel);
   JointCommand( prlite_ax12commander::elbowtilt, l_y_vel);
   JointCommand( prlite_ax12commander::wristtilt, l_z_vel);
}

void prlite_ax12commander::HeadCommand(double pan_vel, double tilt_vel)
{
   JointCommand( prlite_ax12commander::kinectpan, pan_vel);
   JointCommand( prlite_ax12commander::kinecttilt, tilt_vel);
}

void prlite_ax12commander::ToggleGrippers() 
{
  static const double GRIPPER_OPEN_POSITION = 0.9;
  static const double GRIPPER_CLOSE_POSITION = 0.1;
  static int open = false;

  if (open) {
    open = false;
    set_desired_pos(prlite_ax12commander::rfingerR,GRIPPER_OPEN_POSITION);
    set_desired_pos(prlite_ax12commander::lfingerR,-1*GRIPPER_OPEN_POSITION);
    move_to_desired_pos();
  } else {
    open = true;
    set_desired_pos(prlite_ax12commander::rfingerR,GRIPPER_CLOSE_POSITION);
    set_desired_pos(prlite_ax12commander::lfingerR,-1*GRIPPER_CLOSE_POSITION);
    move_to_desired_pos();
  }
}
