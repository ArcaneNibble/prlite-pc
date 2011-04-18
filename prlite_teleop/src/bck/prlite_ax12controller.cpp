#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "joy/Joy.h"

#include <sstream>
#include <cstdlib>
#include "prlite_ax12controller.h"

#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "i2c_net_packets/linact_position.h"
#include "i2c_net_packets/linact_target.h"

const double X_MULT = 7.51913116; // speed is ticks per interval and interval is 1/10 sec, so should be WHEEL_TICKS_PER_METER / 10
const double TH_MULT = 5;
const uint16_t TORSO_DOWN = 1000;
const uint16_t TORSO_MID= 500;
const uint16_t TORSO_UP = 0;
const uint16_t LINACT_PRECISION = 15;

ros::Publisher linact_pub;
bool stopped = false;

bool linact_arrived = false;
bool linact_new_goal = false;
uint16_t linact_goal = TORSO_DOWN;
uint16_t linact_goal_last = 10000;
ros::Subscriber linact_sub;

void TorsoPublish(void)
{
  if (linact_arrived && linact_goal == linact_goal_last)
  {
    // publish wheel commands
  }
  else if (stopped)
  {
    if (linact_goal != linact_goal_last)
    {
      // publish linear actuator command
      i2c_net_packets::linact_target linact_cmd;
      linact_cmd.dstaddr = 6;
      linact_cmd.which = 1;
      linact_cmd.min = linact_goal - LINACT_PRECISION;
      linact_cmd.max = linact_goal + LINACT_PRECISION;
      linact_pub.publish(linact_cmd);
      linact_goal_last = linact_goal;
      linact_new_goal = true;
      linact_arrived = false;
      ROS_INFO("linact %d", linact_goal);
    }
  }
}

void TorsoCallback(const i2c_net_packets::linact_position& linear_actuator_status)
{
  linact_arrived = linear_actuator_status.arr0;
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

  linact_sub = n.subscribe("linear_actuator_status", 1000, TorsoCallback);
  linact_pub = n.advertise<i2c_net_packets::linact_target>("linear_actuator_target", 1000);

}

void prlite_ax12commander::setTorsoGoal(int goal)
{
  linact_goal = goal;
  TorsoPublish();
}

static std::string prlite_ax12controllername[] = {
   "/shoulder_pan_controller/",
   "/shoulder_tilt_controller/", 
   "/elbow_tilt_controller/",
   "/wrist_rotate_controller/", 
   "/finger_left_controller/",
   "/finger_right_controller/",
   "/shoulder_pan_controllerR/",
   "/shoulder_tilt_controllerR/", 
   "/elbow_tilt_controllerR/",
   "/wrist_rotate_controllerR/", 
   "/finger_left_controllerR/",
   "/finger_right_controllerR/",
   "/kinect_pan_controller/",
   "/kinect_tilt_controller/",
   "/laser_tilt_controller/"
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
  ua_controller_msgs::JointState joint_state;
  double directpos;
  double desiredpos;
  ax12_driver_core::MotorState motor_state[2];  
  double minpos;
  double maxpos;
  double nxtpos;
  double incrpos;
  double tuck;
  double untuck;
  double maxvolt;
} ax12_joint_t;

static ax12_joint_t prlite_ax12joint[prlite_ax12commander::numjoints];
static ros::Subscriber motor_state_sub;

void prlite_ax12commander::get_params()
{
   int i;
    ros::NodeHandle n;

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

// void prlite_ax12commander::motor_state_callback(const ax12_driver_core::MotorStateList& motor_state_list)
void motor_state_callback(const ax12_driver_core::MotorStateList& motor_state_list)
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

// void prlite_ax12commander::joint_state_callback(const ua_controller_msgs::JointState& joint_msg_ptr)
void joint_state_callback(const ua_controller_msgs::JointState& joint_msg_ptr)
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

      controller = prlite_ax12controllername[i] + "state";
      prlite_ax12joint[i].subscriber = n.subscribe(controller, 1000, 
              joint_state_callback);
              // prlite_ax12commander::joint_state_callback,);
    }

   controller = "motor_states";
   motor_state_sub = n.subscribe(controller, 1000, 
              motor_state_callback);
              // prlite_ax12commander::motor_state_callback, this);

   prlite_ax12commander::get_params();


   prlite_ax12commander::arm_head_mode(POSITION_MODE, POSITION_MODE, POSITION_MODE);
   for (i = 0; i < prlite_ax12commander::numjoints; i ++) {
      prlite_ax12joint[i].nxtpos = prlite_ax12joint[i].joint_state.current_pos;
      prlite_ax12joint[i].desiredpos = prlite_ax12joint[i].joint_state.current_pos;
    }
    TorsoInit();
/* main needs to run  
for publishers:
 103     ros::spinOnce();
 104 
 105     loop_rate.sleep();
For subscriber:
ros::init(argc, argv, "listener");
ros::spin(); */
}


void prlite_ax12commander::go_directly_to_pos()
{
   int i;

   for (i = 0; i < prlite_ax12commander::numjoints; i ++) {
      prlite_ax12joint[i].controller.publish(prlite_ax12joint[i].nxtpos);
   }
}

int prlite_ax12commander::prlite_ax12commander::move_to_next_pos(int joint, double desired_pos, int dir)
{
     std::stringstream ss;
     int reset_attempts;

     if (dir > 0) { 
       prlite_ax12joint[joint].nxtpos = prlite_ax12joint[joint].joint_state.current_pos + prlite_ax12joint[joint].incrpos;
       if (desired_pos > 0 && prlite_ax12joint[joint].nxtpos > desired_pos)
         prlite_ax12joint[joint].nxtpos = desired_pos;
       prlite_ax12joint[joint].desiredpos = desired_pos;
     } else if(dir <= 0){
       prlite_ax12joint[joint].nxtpos = prlite_ax12joint[joint].joint_state.current_pos - prlite_ax12joint[joint].incrpos;
       if (desired_pos > 0 && prlite_ax12joint[joint].nxtpos < desired_pos)
         prlite_ax12joint[joint].nxtpos = desired_pos;
       prlite_ax12joint[joint].desiredpos = desired_pos;
     } 
     if (desired_pos > prlite_ax12joint[joint].maxpos){
       prlite_ax12joint[joint].desiredpos = prlite_ax12joint[joint].maxpos;
     } else if (desired_pos < prlite_ax12joint[joint].minpos){
       prlite_ax12joint[joint].desiredpos = prlite_ax12joint[joint].minpos;    
     }

     prlite_ax12commander::go_directly_to_pos();


     ros::Duration(0.1).sleep();
     if (prlite_ax12joint[joint].joint_state.is_moving)
       ros::Duration(0.1).sleep();
     for (reset_attempts = 0; reset_attempts < 1; reset_attempts++){
       if (prlite_ax12joint[joint].joint_state.error) {
          ss << "joint " << joint << " error";
          ROS_INFO("%s", ss.str().c_str());
          prlite_ax12joint[joint].torque.publish(0);
          prlite_ax12joint[joint].torque.publish(1);
          return(1);
       } else if (prlite_ax12joint[joint].joint_state.load > MAXLOAD
        || prlite_ax12joint[joint].motor_state[0].voltage > MAXVOLT
        || prlite_ax12joint[joint].motor_state[0].temperature > MAXTEMP)
        {
          /* set error state, turn off torque */
          ss << "joint " << joint << " load " 
             << prlite_ax12joint[joint].motor_state[0].load 
             << " voltage " << prlite_ax12joint[joint].motor_state[0].voltage 
             << " temp " << prlite_ax12joint[joint].motor_state[0].temperature;
          ROS_INFO("%s", ss.str().c_str());
          /*
          prlite_ax12joint[joint].torque.publish(0);
          prlite_ax12joint[joint].torque.publish(1);
          */
          return(0);
        } else
          return(0);
     }
     /* turn off, reset, no retry? */
     return(1);
}


void prlite_ax12commander::move_to_desired_pos()
{
  int more_to_move = 1;
  int i;
  int ret;

  while (more_to_move) {
    more_to_move = 0;
    for (i = 0; i < prlite_ax12commander::numjoints; i ++) {
      ret = prlite_ax12commander::move_to_next_pos(i, prlite_ax12joint[i].desiredpos,
        ((prlite_ax12joint[i].desiredpos > prlite_ax12joint[i].joint_state.current_pos)? 1:0));  
      if (ret == 0 && prlite_ax12joint[i].desiredpos != prlite_ax12joint[i].joint_state.current_pos) 
        more_to_move = 1;
    }
  }
}

void prlite_ax12commander::set_desired_pos(int joint, double desired_pos)
{
  prlite_ax12joint[joint].desiredpos = desired_pos;
}

void prlite_ax12commander::tuck()
{
   int i;

   for (i = 0; i < prlite_ax12commander::rfingerR; i ++) {
     move_to_next_pos(i, prlite_ax12joint[i].tuck, 
       ((prlite_ax12joint[i].desiredpos > prlite_ax12joint[i].joint_state.current_pos)? 1:0));    
  }
}

void prlite_ax12commander::untuck()
{
   int i;

   for (i = 0; i < prlite_ax12commander::rfingerR; i ++) {
     prlite_ax12commander::move_to_next_pos(i, prlite_ax12joint[i].untuck,
       ((prlite_ax12joint[i].desiredpos > prlite_ax12joint[i].joint_state.current_pos)? 1:0));    
  }
}


void prlite_ax12commander::arm_head_mode(int left_arm_control_mode, int right_arm_control_mode, int head_control_mode)
{
   int i; 

   if (left_arm_control_mode == UNCHANGED_MODE) {
   } else if (left_arm_control_mode == MANNEQUIN_MODE || left_arm_control_mode == NO_CONTROLLER_MODE) {
     /* turn off torque */
     for (i = 0; i <= prlite_ax12commander::rfinger; i ++) {
       prlite_ax12joint[i].torque.publish(0);
     }
   } else if (left_arm_control_mode == POSITION_MODE) {
     for (i = 0; i <= prlite_ax12commander::rfinger; i ++) {
       prlite_ax12joint[i].torque.publish(1);
     }
     ros::Duration(0.1).sleep();
     for (i = 0; i <= prlite_ax12commander::rfinger; i ++) {
       ROS_INFO("joint[%d] set to %f", i, prlite_ax12joint[i].joint_state.current_pos);
     }
   }
   if (right_arm_control_mode == UNCHANGED_MODE) {
   } else if (right_arm_control_mode == MANNEQUIN_MODE || right_arm_control_mode == NO_CONTROLLER_MODE) {
     /* turn off torque */
     for (i = prlite_ax12commander::shoulderpanR; i <= prlite_ax12commander::rfingerR; i ++) {
       prlite_ax12joint[i].torque.publish(0);
     }
   } else if (right_arm_control_mode == POSITION_MODE) {
     for (i = prlite_ax12commander::shoulderpanR; i <= prlite_ax12commander::rfingerR; i ++) {
       prlite_ax12joint[i].torque.publish(1);
     }
     ros::Duration(0.1).sleep();
     for (i = prlite_ax12commander::shoulderpanR; i <= prlite_ax12commander::rfingerR; i ++) {
       ROS_INFO("joint[%d] set to %f", i, prlite_ax12joint[i].joint_state.current_pos);
     }
   }
    
   if (head_control_mode == UNCHANGED_MODE) {
   } else if (head_control_mode == MANNEQUIN_MODE || head_control_mode == NO_CONTROLLER_MODE) {
     /* turn off torque */
     for (i = prlite_ax12commander::kinectpan; i < prlite_ax12commander::numjoints; i ++) {
       prlite_ax12joint[i].torque.publish(0);
     }
   } else if (head_control_mode == POSITION_MODE) {
     for (i = prlite_ax12commander::kinectpan; i < prlite_ax12commander::numjoints; i ++) {
       prlite_ax12joint[i].torque.publish(1);
     }
     ros::Duration(0.1).sleep();
     for (i = prlite_ax12commander::kinectpan; i < prlite_ax12commander::numjoints; i ++) {
	     prlite_ax12joint[i].nxtpos = prlite_ax12joint[i].joint_state.current_pos;
       prlite_ax12joint[i].desiredpos = prlite_ax12joint[i].joint_state.current_pos;
       ROS_INFO("joint[%d] set to %f", i, prlite_ax12joint[i].joint_state.current_pos);
     }
   }   
}

