/*
 * prlite_teleop_general_joystick
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: E. Gil Jones

#include <termios.h>
#include <signal.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <math.h>

#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include "prlite_teleop_general/prlite_teleop_general_commander.h"
#include "kinect_transform.h"

enum JoystickLayoutMode {
  LAYOUT_NONE,
  LAYOUT_BODY,
  LAYOUT_HEAD,
  LAYOUT_KINECT,
  LAYOUT_RIGHT_ARM,
  LAYOUT_LEFT_ARM,
  LAYOUT_BOTH_ARMS
};

/*
Joystick layout

MODES
Button 5 -> front bot right -> RIGHT ARM
Button 4 -> front bot left  -> LEFT ARM
Button 6 -> front top left  -> BODY
Button 7 -> front top right -> HEAD

Axes[5] = 1 -> cross up (digital) -> SHOULDER TILT UP
Axes[5] = -1 -> cross down -> SHOULDER TILT DOWN
Axes[4] = -1 -> cross right -> shoulder pan left
Axes[4] = 1 -> cross left -> shoulder pan right

Axes[1] = 1 -> left joy up (analog) -> elbow tilt down, head tilt up, none
Axes[1] = -1 -> left joy down -> elbow tilt up, head tilt down, none
Axes[0] = 1 -> left joy left (analog) -> elbow pan left, head pan left, speed
Axes[0] = -1 -> left joy right -> elbow pan right, head pan right, speed

Axes[2] = 1 -> right joy up (analog) -> wrist tilt up, none, move fwd
Axes[2] = -1 -> right joy down -> wrist tilt down, none, move back
Axes[3] = 1 -> right joy left (analog) -> none, none, move right
Axes[3] = -1 -> right joy right -> none, none, move left

Button 1 -> O -> gripper open
Button 3 -> Square  -> gripper close

Button 0 -> triangle -> wrist rot left, torso up
Button 2 -> X -> wrist rot right, torso down

Button 9 -> start -> Arm tuck
Button 8 -> select -> Arm toggle

*/

static const unsigned int BODY_LAYOUT_BUTTON = 6; // 10;
static const unsigned int RIGHT_ARM_LAYOUT_BUTTON = 5; // 9;
static const unsigned int LEFT_ARM_LAYOUT_BUTTON = 4; // 8;
static const unsigned int HEAD_LAYOUT_BUTTON = 7; // 11;

static const unsigned int HEAD_MODE_TOGGLE_BUTTON = 9; // 4;
static const unsigned int PROJECTOR_TOGGLE_BUTTON = 20; // (NA) 5;
static const unsigned int LASER_TOGGLE_BUTTON =  20; // (NA) 7;
static const unsigned int PROSILICA_POLL_BUTTON =  20; // (NA) 6;
static const unsigned int OPEN_GRIPPER_BUTTON = 1; // 15;
static const unsigned int CLOSE_GRIPPER_BUTTON = 3; // 13;
static const unsigned int ARM_MODE_TOGGLE_BUTTON = 8; // 4;

static const unsigned int LEFT_AXIS_NUMBER = 1;
static const unsigned int RIGHT_AXIS_NUMBER = 1;

static const unsigned int TORSO_UP_BUTTON = 0; // 12;
static const unsigned int TORSO_DOWN_BUTTON = 2; // 14;

static const unsigned int WRIST_CLOCKWISE_BUTTON = 0; // 12;
static const unsigned int WRIST_COUNTER_BUTTON = 2; // 14;

// direction to moving wheels
static const unsigned int VX_AXIS = 1;
static const unsigned int VY_AXIS = 0;
static const unsigned int VW_AXIS = 2;

static const unsigned int HEAD_PAN_AXIS = 0;
static const unsigned int HEAD_TILT_AXIS = 1;

static const unsigned int ARM_X_AXIS = 0;
static const unsigned int ARM_Y_AXIS = 1;
static const unsigned int ARM_Z_AXIS = 2;
static const unsigned int ARM_YAW_AXIS = 3;


#define SHOULDER_PAN_LEFT   (joy_msg->axes[4] == 1)
#define SHOULDER_PAN_RIGHT  (joy_msg->axes[4] == -1)
#define SHOULDER_TILT_UP    (joy_msg->axes[5] == 1)
#define SHOULDER_TILT_DOWN  (joy_msg->axes[5] == -1)

// toggle
static const unsigned int ARM_UNTUCK_BUTTON = 9;
static const unsigned int ARM_TUCK_BUTTON = 9;
#define TUCK_MODE 0
#define UNTUCK_MODE 1
#define NO_TUCK_MODE 2
#define MIMIC_MODE 3
static int tuck_mode = NO_TUCK_MODE;

static const unsigned int MOVE_TO_WALK_ALONG_BUTTON = 0;
static const unsigned int SET_WALK_ALONG_BUTTON = 3;

static JoystickLayoutMode layout = LAYOUT_NONE;
static const ros::Duration DOUBLE_TAP_TIMEOUT(.25);
static int kinect_follow_ = 0;
  double max_pan_, max_tilt_, min_tilt_;
  int axis_pan_, axis_tilt_;
  double pan_scale_, tilt_scale_;

  double des_pan_pos_;
  double des_tilt_pos_;

  double vel_val_pan_;
  double vel_val_tilt_;

  double des_vx_;
  double des_vy_;
  double des_vw_;
  double vx_scale_;
  double vy_scale_;
  double vw_scale_;

  double arm_x_scale_;
  double arm_y_scale_;
  double arm_z_scale_;

  double right_arm_vx_;
  double right_arm_vy_;
  double right_arm_vz_;

  double left_arm_vx_;
  double left_arm_vy_;
  double left_arm_vz_;

  double tuck_shoulder_pan_;

  double tuck_elbow_tilt_;
  double tuck_wrist_rotate_;
  double tuck_finger_left_;
  double tuck_finger_right_;

  double untuck_shoulder_pan_;
  double untuck_shoulder_tilt_; 
  double untuck_elbow_pan_; 
  double untuck_elbow_tilt_; 
  double untuck_wrist_rotate_; 
  double untuck_wrist_tilt_; 
  double untuck_finger_left_; 
  double untuck_finger_right_; 

  bool head_init_;
  bool torso_init_;

  double req_torso_vel_;
  double req_torso_pos_;

  double des_torso_pos_;
  double des_torso_vel_;
  double torso_step_;
  double min_torso_;
  double max_torso_;

  double wrist_velocity_;
  double shoulder_pan_velocity_;
  double shoulder_tilt_velocity_;
  double des_right_wrist_vel_;  
  double des_left_wrist_vel_;
  double des_left_shoulder_pan_vel_;
  double des_right_shoulder_pan_vel_;
  double des_left_shoulder_tilt_vel_;
  double des_right_shoulder_tilt_vel_;

  double walk_along_x_speed_scale_;
  double walk_along_y_speed_scale_;
  double walk_along_w_speed_scale_;
  double walk_along_thresh_;
  double walk_along_x_dist_max_;
  double walk_along_y_dist_max_;

  bool walk_along_init_waiting_;
  bool set_walk_along_mode_;

  std::string prosilica_namespace_;

  bool proj_toggle_com_; 
 
  int projector_toggle_button_;
  int tilt_toggle_button_;
  int switch_head_control_mode_button_;

  GeneralCommander* gc;

  ros::Time joy_deadman_;

  bool first_callback_;

  sensor_msgs::JoyConstPtr last_joy_;

class Pr2TeleopGeneralJoystick
{
public:
  Pr2TeleopGeneralJoystick(bool deadman_no_publish = false)
  { 
    gc = NULL;
  }

  void init()
  {
    des_pan_pos_ = des_tilt_pos_ = 0;
    vel_val_pan_ = vel_val_tilt_ = 0;
    
    des_torso_pos_ = 0;

    des_torso_vel_ = 0.0;
    
    ros::NodeHandle n_local("~");

    // Head pan/tilt parameters
    n_local.param("max_pan", max_pan_, 2.7);
    n_local.param("max_tilt", max_tilt_, 1.4);
    n_local.param("min_tilt", min_tilt_, -0.4);
    
    n_local.param("tilt_scale", tilt_scale_, .1);
    n_local.param("pan_scale", pan_scale_, .1);
    
    n_local.param("torso_step", torso_step_, 0.05);
    n_local.param("min_torso", min_torso_, 0.0);
    n_local.param("max_torso", max_torso_, 0.3);
    // n_local.param("kinect_follow", kinect_follow_, 0);

    n_local.param("vx_scale", vx_scale_, 1.0);
    n_local.param("vy_scale", vy_scale_, 1.0);
    n_local.param("vw_scale", vw_scale_, 1.0);
    
    n_local.param("arm_x_scale", arm_x_scale_, 1.0);
    n_local.param("arm_y_scale", arm_y_scale_, 1.0);
    n_local.param("arm_z_scale", arm_z_scale_, 1.0);

/*
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
    n_local.param("wrist_velocity",wrist_velocity_, 1.0);
    n_local.param("shoulder_pan_velocity",shoulder_pan_velocity_, 1.0);
    n_local.param("shoulder_tilt_velocity",shoulder_tilt_velocity_, 1.0);
    n_local.param("walk_along_w_speed_scale", walk_along_w_speed_scale_, 9.0);
    n_local.param("walk_along_thresh", walk_along_thresh_, .015);
    n_local.param("walk_along_x_dist_max", walk_along_x_dist_max_, .5);
    n_local.param("walk_along_y_dist_max", walk_along_y_dist_max_, .5);

    n_local.param("prosilica_namespace", prosilica_namespace_, std::string("prosilica_polled"));

    bool control_prosilica;
    n_local.param("control_prosilica", control_prosilica, true);
    
    bool control_body;
    n_local.param("control_body", control_body, true);

    bool control_larm;
    n_local.param("control_larm", control_larm, true);

    bool control_rarm;
    n_local.param("control_rarm", control_rarm, true);

    bool control_head;
    n_local.param("control_head", control_head, true);

    std::string arm_controller_name;
    n_local.param("arm_controller_name", arm_controller_name,std::string(""));

    ROS_DEBUG("tilt scale: %.3f rad\n", tilt_scale_);
    ROS_DEBUG("pan scale: %.3f rad\n", pan_scale_);
    
    ROS_INFO("Initing general commander");

    if(arm_controller_name.empty()) {
      gc = new GeneralCommander(control_body, 
                                control_head, 
                                control_rarm,
                                control_larm,
                                control_prosilica);
    } else {
      gc = new GeneralCommander(control_body, 
                                control_head, 
                                control_rarm,
                                control_larm,
                                control_prosilica,
                                arm_controller_name);
    }
    first_callback_ = true;
    
    head_init_ = false;
    torso_init_ = false;
    
    proj_toggle_com_ = false;
    
    walk_along_init_waiting_ = false;
    set_walk_along_mode_ = false;

    joy_sub_ = n_.subscribe("joy", 10, &Pr2TeleopGeneralJoystick::joy_cb, this);
    // kinect_init();
  }


  ~Pr2TeleopGeneralJoystick() {  
    if(gc != NULL) {
      delete gc;
    }
  }

  bool buttonOkAndOn(unsigned int buttonNum, const sensor_msgs::Joy::ConstPtr& joy_msg) const {
    if(buttonNum >= joy_msg->buttons.size()) return false;
    return(joy_msg->buttons[buttonNum]);
  }

  bool axisOk(unsigned int axisNum, const sensor_msgs::Joy::ConstPtr& joy_msg) const {
    return (axisNum < joy_msg->axes.size());
  }

  bool sameValueAsLast(unsigned int button, 
                       const sensor_msgs::Joy::ConstPtr& new_msg,
                       const sensor_msgs::Joy::ConstPtr& old_msg) {
    return (buttonOkAndOn(button, new_msg) == buttonOkAndOn(button, old_msg));
  }
                       

  void joy_cb(const sensor_msgs::Joy::ConstPtr& joy_msg)
  {
    if(first_callback_) {
      last_joy_ = joy_msg;
      first_callback_ = false;
    }

    
    for ( int i = 0 ; i < 10; i++)
      if(buttonOkAndOn(i, joy_msg)) {
        ROS_INFO_STREAM("Button " << i);
      }

    for ( int i = 0 ; i < 10; i++)
      if(axisOk(i, joy_msg) && joy_msg->axes[i] != 0)
      {
        ROS_INFO_STREAM("AXES[" << i << "] " << joy_msg->axes[i]);
      }
    if (layout == LAYOUT_KINECT) {
       if (buttonOkAndOn(BODY_LAYOUT_BUTTON, joy_msg)
          && buttonOkAndOn(RIGHT_ARM_LAYOUT_BUTTON, joy_msg) 
          && buttonOkAndOn(LEFT_ARM_LAYOUT_BUTTON, joy_msg)
          && buttonOkAndOn(HEAD_LAYOUT_BUTTON, joy_msg)) {
         layout = LAYOUT_NONE;
         if (system("espeak --stdout \"joystick mode\" | aplay &"))
           ROS_INFO("espeak");
       }
    } else if (buttonOkAndOn(BODY_LAYOUT_BUTTON, joy_msg)
       && buttonOkAndOn(RIGHT_ARM_LAYOUT_BUTTON, joy_msg) 
       && buttonOkAndOn(LEFT_ARM_LAYOUT_BUTTON, joy_msg)
       && buttonOkAndOn(HEAD_LAYOUT_BUTTON, joy_msg)) {
/*
      layout = LAYOUT_KINECT;
      if (system("espeak --stdout \"mimic mode\" | aplay &"))
          ROS_INFO("espeak");
      ROS_INFO("Layout kinect");
*/
      ROS_INFO("Layout NA");
      // put torso up?  Currently optionally do by joystick
      // generaljoy.gc->sendTorsoCommand(0, 100);
    } else if(buttonOkAndOn(BODY_LAYOUT_BUTTON, joy_msg)) {
      layout = LAYOUT_BODY;
      ROS_INFO("Layout body");
    } else if (buttonOkAndOn(RIGHT_ARM_LAYOUT_BUTTON, joy_msg) && buttonOkAndOn(LEFT_ARM_LAYOUT_BUTTON, joy_msg)) {
      layout = LAYOUT_BOTH_ARMS;
      ROS_INFO("Layout both arms");
    } else if (buttonOkAndOn(RIGHT_ARM_LAYOUT_BUTTON,joy_msg)) {
      layout = LAYOUT_RIGHT_ARM;
      ROS_INFO("Layout right arm");
    } else if (buttonOkAndOn(LEFT_ARM_LAYOUT_BUTTON, joy_msg)) {
      layout = LAYOUT_LEFT_ARM;
      ROS_INFO("Layout left arm");
    } else if (buttonOkAndOn(HEAD_LAYOUT_BUTTON, joy_msg)) {
      layout = LAYOUT_HEAD;
       ROS_INFO("Layout head");
    } else if (layout != LAYOUT_KINECT) {
	    layout = LAYOUT_NONE;
       ROS_INFO("Layout none");
    }
    bool setting_walk_along_this_cycle_ = false;
    if (layout == LAYOUT_BOTH_ARMS || layout == LAYOUT_RIGHT_ARM
       || layout == LAYOUT_LEFT_ARM) {
      if(tuck_mode == UNTUCK_MODE) {
        tuck_mode = NO_TUCK_MODE; // untuck allows joystick override
        ROS_INFO("NO TUCK");
      } else if(buttonOkAndOn(ARM_TUCK_BUTTON, joy_msg) && tuck_mode == NO_TUCK_MODE) {
        tuck_mode = TUCK_MODE;
        ROS_INFO("TUCK");
      } else if(buttonOkAndOn(ARM_TUCK_BUTTON, joy_msg) && tuck_mode == TUCK_MODE) {
        tuck_mode = UNTUCK_MODE;
        ROS_INFO("UNTUCK");
      } else if(buttonOkAndOn(ARM_TUCK_BUTTON, joy_msg) && tuck_mode == UNTUCK_MODE) {
        tuck_mode = NO_TUCK_MODE;
        ROS_INFO("NO TUCK");
      }
    }


/*
    if(layout == LAYOUT_KINECT) {
        if (system("espeak --stdout \"put your arms like this\" | aplay"))
          ROS_INFO("espeak");
        ROS_INFO("KINECT");
        gc->mimicArms(GeneralCommander::ARMS_RIGHT);
    }
*/

    if(layout == LAYOUT_HEAD) {
	    if(buttonOkAndOn(MOVE_TO_WALK_ALONG_BUTTON, joy_msg) && !sameValueAsLast(MOVE_TO_WALK_ALONG_BUTTON, joy_msg, last_joy_)) {
        ros::Time now = ros::Time::now();
        if(now-last_walk_along_time_ < DOUBLE_TAP_TIMEOUT) {
          //only matters if this is off
          if(!gc->isWalkAlongOk()) {
            set_walk_along_mode_ = true;
            setting_walk_along_this_cycle_ = true;
          }
        }
        if(gc->isWalkAlongOk()) {
          gc->turnOffWalkAlong();
          ROS_INFO("Turning off walk along");
        } else {
          last_walk_along_time_ = now;
        }
      }
    }

    bool in_walk_along = false;
    if(gc->isWalkAlongOk()) {
      if(buttonOkAndOn(MOVE_TO_WALK_ALONG_BUTTON, joy_msg) && !sameValueAsLast(MOVE_TO_WALK_ALONG_BUTTON, joy_msg, last_joy_)) {
	gc->turnOffWalkAlong();
	ROS_INFO("Turning off walk along");
      } else {
	vel_val_pan_ = 0.0;
	vel_val_tilt_ = 0.0;
	des_torso_vel_ = 0.0;
	des_vx_ = 0.0;
	des_vy_ = 0.0;
	des_vw_ = 0.0;
	des_right_wrist_vel_ = 0.0;
	des_right_shoulder_pan_vel_ = 0.0;
	des_right_shoulder_tilt_vel_ = 0.0;
	right_arm_vx_ = 0.0;
	right_arm_vy_ = 0.0;
	right_arm_vz_ = 0.0;
	des_left_wrist_vel_ = 0.0;
	des_left_shoulder_pan_vel_ = 0.0;
	des_left_shoulder_tilt_vel_ = 0.0;
	left_arm_vx_ = 0.0;
	left_arm_vy_ = 0.0;
	left_arm_vz_ = 0.0;
	in_walk_along = true;
      }
    }

/*
    //we must be moving the arms into the mode
    if(!in_walk_along && layout == LAYOUT_HEAD && set_walk_along_mode_) {
      if(buttonOkAndOn(SET_WALK_ALONG_BUTTON, joy_msg) 
	 && !sameValueAsLast(SET_WALK_ALONG_BUTTON, joy_msg, last_joy_)) {
        ROS_INFO("HeadSeq");
        gc->sendHeadSequence(GeneralCommander::HEAD_SHAKE);
      }
      if(!setting_walk_along_this_cycle_ && buttonOkAndOn(MOVE_TO_WALK_ALONG_BUTTON, joy_msg) && !sameValueAsLast(MOVE_TO_WALK_ALONG_BUTTON, joy_msg, last_joy_)) {
        set_walk_along_mode_ = false;
        ROS_INFO("No longer waiting for set");
      }
    }

    if(!in_walk_along && layout == LAYOUT_HEAD && walk_along_init_waiting_) {
      if(buttonOkAndOn(SET_WALK_ALONG_BUTTON, joy_msg) 
         && !sameValueAsLast(SET_WALK_ALONG_BUTTON, joy_msg, last_joy_)) {
        bool ok = gc->initWalkAlong();
        if(!ok) {
          ROS_INFO("HeadShake");
          gc->sendHeadSequence(GeneralCommander::HEAD_SHAKE);
        } else {
          ROS_INFO("Headnod");
          gc->sendHeadSequence(GeneralCommander::HEAD_NOD);
          ROS_INFO("Should be in walk along");
          walk_along_init_waiting_ = false;
        }
      }
      if(buttonOkAndOn(MOVE_TO_WALK_ALONG_BUTTON, joy_msg) && !sameValueAsLast(MOVE_TO_WALK_ALONG_BUTTON, joy_msg, last_joy_)) {
        walk_along_init_waiting_ = false;
        ROS_INFO("No longer waiting for init");
      }
    }
*/
    if(layout == LAYOUT_RIGHT_ARM || layout == LAYOUT_LEFT_ARM || layout == LAYOUT_BOTH_ARMS) {
      if(buttonOkAndOn(CLOSE_GRIPPER_BUTTON, joy_msg) 
	 && !sameValueAsLast(CLOSE_GRIPPER_BUTTON, joy_msg, last_joy_)) {
        ROS_INFO("Close Gripper");
        if(layout == LAYOUT_RIGHT_ARM) {
          gc->sendGripperCommand(GeneralCommander::ARMS_RIGHT, true);
        } else if(layout == LAYOUT_LEFT_ARM) {
          gc->sendGripperCommand(GeneralCommander::ARMS_LEFT, true);
        } else {
          gc->sendGripperCommand(GeneralCommander::ARMS_BOTH, true);
        }
      }
      if(buttonOkAndOn(OPEN_GRIPPER_BUTTON, joy_msg) 
	 && !sameValueAsLast(OPEN_GRIPPER_BUTTON, joy_msg, last_joy_)) {
        ROS_INFO("Open Gripper");
        if(layout == LAYOUT_RIGHT_ARM) {
          gc->sendGripperCommand(GeneralCommander::ARMS_RIGHT, false);
        } else if(layout == LAYOUT_LEFT_ARM) {
          gc->sendGripperCommand(GeneralCommander::ARMS_LEFT, false);
        } else {
          gc->sendGripperCommand(GeneralCommander::ARMS_BOTH, false);
        }
      }
    }

    if(/* !in_walk_along && */ layout == LAYOUT_HEAD) {

/*
      if(buttonOkAndOn(PROJECTOR_TOGGLE_BUTTON, joy_msg) 
	 && !sameValueAsLast(PROJECTOR_TOGGLE_BUTTON,joy_msg, last_joy_)) {
        proj_toggle_com_ = !proj_toggle_com_;
        gc->sendProjectorStartStop(proj_toggle_com_);
      }
      
      if(buttonOkAndOn(PROSILICA_POLL_BUTTON, joy_msg) 
	 && !sameValueAsLast(PROSILICA_POLL_BUTTON,joy_msg, last_joy_)) {
        gc->requestProsilicaImage(prosilica_namespace_);
      }
*/

      if(buttonOkAndOn(HEAD_MODE_TOGGLE_BUTTON, joy_msg) 
	 // && !sameValueAsLast(HEAD_MODE_TOGGLE_BUTTON, joy_msg, last_joy_)
      ) {
        
/*
        if(gc->getHeadMode() == GeneralCommander::HEAD_JOYSTICK) {
          ROS_DEBUG("Head mode to left");
          gc->setHeadMode(GeneralCommander::HEAD_TRACK_LEFT_HAND);
        } else if(gc->getHeadMode() == GeneralCommander::HEAD_TRACK_LEFT_HAND) {
          gc->setHeadMode(GeneralCommander::HEAD_TRACK_RIGHT_HAND);
          ROS_DEBUG("Head mode to right");
        } else if(gc->getHeadMode() == GeneralCommander::HEAD_TRACK_RIGHT_HAND)
*/

        if(gc->getHeadMode() == GeneralCommander::HEAD_JOYSTICK){
          gc->setHeadMode(GeneralCommander::HEAD_MANNEQUIN);
          ROS_INFO("Head mode to mannequin");
        } else {
          ROS_INFO("Head mode to joystick");
          head_init_ = false;
          gc->setHeadMode(GeneralCommander::HEAD_JOYSTICK);
        }
      }

/*
      if(buttonOkAndOn(LASER_TOGGLE_BUTTON, joy_msg) 
	 && !sameValueAsLast(LASER_TOGGLE_BUTTON, joy_msg, last_joy_)) {
        if(gc->getLaserMode() == GeneralCommander::LASER_TILT_OFF) {
          gc->setLaserMode(GeneralCommander::LASER_TILT_SLOW);
        } else if(gc->getLaserMode() == GeneralCommander::LASER_TILT_SLOW) {
          gc->setLaserMode(GeneralCommander::LASER_TILT_FAST);
        } else {
          gc->setLaserMode(GeneralCommander::LASER_TILT_OFF);
        }
      }
*/

      if(axisOk(HEAD_PAN_AXIS, joy_msg))
      {
        vel_val_pan_ = -1 * joy_msg->axes[HEAD_PAN_AXIS] * pan_scale_;
        ROS_INFO_STREAM("PAN AXIS "<< joy_msg->axes[HEAD_PAN_AXIS]);
      }
      
      if(axisOk(HEAD_TILT_AXIS, joy_msg))
      {
        // ROS_INFO("TILT AXIS");
        vel_val_tilt_ = -1 * joy_msg->axes[HEAD_TILT_AXIS] * tilt_scale_;
      }   
    } else {
      vel_val_pan_ = 0.0;
      vel_val_tilt_ = 0.0;
    }

    if(/* !in_walk_along &&  */
      layout == LAYOUT_BODY) {
      bool down = buttonOkAndOn(TORSO_DOWN_BUTTON, joy_msg);
      bool up = buttonOkAndOn(TORSO_UP_BUTTON, joy_msg);
      
      ROS_INFO_STREAM("Down is " << down);
      ROS_INFO_STREAM("Up is " << up);
      
      if(down && !up) {
        des_torso_vel_ = 1;
      } else if(!down && up) {
        des_torso_vel_ = -1;
      } else {
	//ROS_INFO_STREAM("Setting des vel to 0.0");
        des_torso_vel_ = 0.0;
      }
      if(axisOk(VX_AXIS, joy_msg)) {
        des_vx_ = joy_msg->axes[VX_AXIS]*vx_scale_;
      } else {
        des_vx_ = 0.0;
      }
      if(axisOk(VY_AXIS, joy_msg)) {
        des_vy_ = joy_msg->axes[VY_AXIS]*vy_scale_;
      } else {
        des_vy_ = 0.0;
      }
      if(axisOk(VW_AXIS, joy_msg)) {
        des_vw_ = joy_msg->axes[VW_AXIS]*vw_scale_;
      } else {
        des_vw_ = 0.0;
      }
    } else {
      des_torso_vel_ = 0.0;
      des_vx_ = 0.0;
      des_vy_ = 0.0;
      des_vw_ = 0.0;
    }

    if(layout == LAYOUT_RIGHT_ARM) {
      if(buttonOkAndOn(ARM_MODE_TOGGLE_BUTTON, joy_msg) && !sameValueAsLast(ARM_MODE_TOGGLE_BUTTON, joy_msg, last_joy_)) {
	if(in_walk_along) {
	  gc->turnOffWalkAlong();
          ROS_INFO("Turning off walk along");
	}
        if(gc->getArmMode(GeneralCommander::ARMS_RIGHT) == GeneralCommander::ARM_POSITION_CONTROL) {
          ROS_INFO("ARM MANNIQUIN");
          gc->setArmMode(GeneralCommander::ARMS_RIGHT,GeneralCommander::ARM_MANNEQUIN_MODE);
        } else if (gc->getArmMode(GeneralCommander::ARMS_RIGHT) == GeneralCommander::ARM_MANNEQUIN_MODE) {
          gc->setArmMode(GeneralCommander::ARMS_RIGHT,GeneralCommander::ARM_NO_CONTROLLER);
          ROS_INFO("ARM NO CONTROLLER");
        } else {
          gc->setArmMode(GeneralCommander::ARMS_RIGHT,GeneralCommander::ARM_POSITION_CONTROL);
          ROS_INFO("ARM POS CONTROLLER");
        }
      }

      if (tuck_mode == TUCK_MODE) {
        if(in_walk_along) {
	  gc->turnOffWalkAlong();
          ROS_INFO("Turning off walk along");
	}
        ROS_INFO("TUCK");
        gc->tuckArms(GeneralCommander::ARMS_RIGHT);
      } else if (tuck_mode == MIMIC_MODE) {
        if(in_walk_along) {
          gc->turnOffWalkAlong();
          ROS_INFO("Turning off walk along");
        }
        ROS_INFO("MIMIC");
        gc->mimicArms(GeneralCommander::ARMS_RIGHT);
      } else if (tuck_mode == UNTUCK_MODE) {
        if(in_walk_along) {
	  gc->turnOffWalkAlong();
          ROS_INFO("Turning off walk along");
	}
        ROS_INFO("UNTUCK");
        gc->untuckArms(GeneralCommander::ARMS_RIGHT);
      } 

      // if(!in_walk_along) 
      {

        bool lookAnalog = false;
        bool rotClock = buttonOkAndOn(WRIST_CLOCKWISE_BUTTON, joy_msg);
        bool rotCounter = buttonOkAndOn(WRIST_COUNTER_BUTTON, joy_msg);
        if(rotClock && !rotCounter) {
          des_right_wrist_vel_ = wrist_velocity_;
        } else if(!rotClock && rotCounter) {
          des_right_wrist_vel_ = -wrist_velocity_;
        } else {
          des_right_wrist_vel_ = 0.0;
          lookAnalog = true;
        }
        bool shoulder_pan_left = SHOULDER_PAN_LEFT;
        bool shoulder_pan_right = SHOULDER_PAN_RIGHT;
        if(shoulder_pan_left && !shoulder_pan_right) {
          des_right_shoulder_pan_vel_ = shoulder_pan_velocity_;
        } else if(!shoulder_pan_left && shoulder_pan_right) {
          des_right_shoulder_pan_vel_ = -shoulder_pan_velocity_; 
        } else {
          des_right_shoulder_pan_vel_ = 0.0;
          lookAnalog = true;
        }

        bool shoulder_tilt_up = SHOULDER_TILT_UP;
        bool shoulder_tilt_down = SHOULDER_TILT_DOWN;
        if(shoulder_tilt_up && !shoulder_tilt_down) {
	  des_right_shoulder_tilt_vel_ = -shoulder_tilt_velocity_;        
	} else if(!shoulder_tilt_up && shoulder_tilt_down) {
          des_right_shoulder_tilt_vel_ = shoulder_tilt_velocity_; 
        } else {
          des_right_shoulder_tilt_vel_ = 0.0;
          lookAnalog = true;
        }
        if(lookAnalog) {
          //look at analog sticks if we aren't supposed to wrist rotate
          if(axisOk(ARM_X_AXIS, joy_msg)) {
            right_arm_vx_ = joy_msg->axes[ARM_X_AXIS]*arm_x_scale_;
          } else {
            right_arm_vx_ = 0.0;
          }
          if(axisOk(ARM_Y_AXIS, joy_msg)) {
            right_arm_vy_ = -1 * joy_msg->axes[ARM_Y_AXIS]*arm_y_scale_;
          } else {
            right_arm_vy_ = 0.0;
          }
          if(axisOk(ARM_Z_AXIS, joy_msg)) {
            right_arm_vz_ = -joy_msg->axes[ARM_Z_AXIS]*arm_z_scale_;
          } else {
            right_arm_vz_ = 0.0;
          }
          // ROS_INFO_STREAM("Setting vx " << right_arm_vx_ << " " << right_arm_vy_ << " " << right_arm_vz_);
        } else {
          right_arm_vx_ = 0.0;
          right_arm_vy_ = 0.0;
          right_arm_vz_ = 0.0;
        }
      }
  } else if (layout != LAYOUT_BOTH_ARMS) {
      des_right_wrist_vel_ = 0.0;
      des_right_shoulder_pan_vel_ = 0.0;
      des_right_shoulder_tilt_vel_ = 0.0;
      right_arm_vx_ = 0.0;
      right_arm_vy_ = 0.0;
      right_arm_vz_ = 0.0;
    }
    if(layout == LAYOUT_LEFT_ARM) {
      if(buttonOkAndOn(ARM_MODE_TOGGLE_BUTTON, joy_msg) && !sameValueAsLast(ARM_MODE_TOGGLE_BUTTON, joy_msg, last_joy_)) {
	if(in_walk_along) {
          gc->turnOffWalkAlong();
          ROS_INFO("Turning off walk along");
        }
        if(gc->getArmMode(GeneralCommander::ARMS_LEFT) == GeneralCommander::ARM_POSITION_CONTROL) {
          gc->setArmMode(GeneralCommander::ARMS_LEFT,GeneralCommander::ARM_MANNEQUIN_MODE);
          ROS_INFO("ARM MANNIQUIN");
        } else if (gc->getArmMode(GeneralCommander::ARMS_LEFT) == GeneralCommander::ARM_MANNEQUIN_MODE) {
          gc->setArmMode(GeneralCommander::ARMS_LEFT,GeneralCommander::ARM_NO_CONTROLLER);
          ROS_INFO("ARM NO CONTROLLER");
        } else {
          gc->setArmMode(GeneralCommander::ARMS_LEFT,GeneralCommander::ARM_POSITION_CONTROL);
          ROS_INFO("ARM POS CONTROLLER");
        }
      }

      // if(buttonOkAndOn(ARM_TUCK_BUTTON, joy_msg) && !sameValueAsLast(ARM_TUCK_BUTTON, joy_msg, last_joy_)) 
      if (tuck_mode == TUCK_MODE) {
        if(in_walk_along) {
	  gc->turnOffWalkAlong();
          ROS_INFO("Turning off walk along");
	}
        ROS_INFO("Tuck");
        gc->tuckArms(GeneralCommander::ARMS_LEFT);        
      } else if (tuck_mode == MIMIC_MODE) {
        if(in_walk_along) {
          gc->turnOffWalkAlong();
          ROS_INFO("Turning off walk along");
        }
        gc->mimicArms(GeneralCommander::ARMS_LEFT);
        ROS_INFO("Mimic");
      } else if (tuck_mode == UNTUCK_MODE) {
      // else if(buttonOkAndOn(ARM_UNTUCK_BUTTON, joy_msg) && !sameValueAsLast(ARM_UNTUCK_BUTTON, joy_msg, last_joy_)) 
        if(in_walk_along) {
	  gc->turnOffWalkAlong();
          ROS_INFO("Turning off walk along");
	}
        gc->untuckArms(GeneralCommander::ARMS_LEFT);
        ROS_INFO("Untuck");
      } 

      // if(!in_walk_along) 
      {
        bool lookAnalog = false;
        bool rotClock = buttonOkAndOn(WRIST_CLOCKWISE_BUTTON, joy_msg);
        bool rotCounter = buttonOkAndOn(WRIST_COUNTER_BUTTON, joy_msg);
        if(rotClock && !rotCounter) {
          des_left_wrist_vel_ = wrist_velocity_;
        } else if(!rotClock && rotCounter) {
          des_left_wrist_vel_ = -wrist_velocity_;
        } else {
          des_left_wrist_vel_ = 0.0;
          lookAnalog = true;
        }
        
        bool shoulder_pan_left = SHOULDER_PAN_LEFT;
        bool shoulder_pan_right = SHOULDER_PAN_RIGHT;
        if(shoulder_pan_left && !shoulder_pan_right) {
          des_left_shoulder_pan_vel_ = shoulder_pan_velocity_;
	} else if(!shoulder_pan_left && shoulder_pan_right) {
          des_left_shoulder_pan_vel_ = -shoulder_pan_velocity_;
        } else {
          des_left_shoulder_pan_vel_ = 0.0;
          lookAnalog = true;
        }

        bool shoulder_tilt_up = SHOULDER_TILT_UP;
        bool shoulder_tilt_down = SHOULDER_TILT_DOWN;
        if(shoulder_tilt_up && !shoulder_tilt_down) {
          des_left_shoulder_tilt_vel_ = -shoulder_tilt_velocity_;
	} else if(!shoulder_tilt_up && shoulder_tilt_down) {
          des_left_shoulder_tilt_vel_ = shoulder_tilt_velocity_;
        } else {
          des_left_shoulder_tilt_vel_ = 0.0;
          lookAnalog = true;
        }

        if(lookAnalog) {
          //look at analog sticks if we aren't supposed to wrist rotate
          if(axisOk(ARM_X_AXIS, joy_msg)) {
            left_arm_vx_ = joy_msg->axes[ARM_X_AXIS]*arm_x_scale_;
          } else {
            left_arm_vx_ = 0.0;
          }
          if(axisOk(ARM_Y_AXIS, joy_msg)) {
            left_arm_vy_ = -1 * joy_msg->axes[ARM_Y_AXIS]*arm_y_scale_;
          } else {
            left_arm_vy_ = 0.0;
          }
          if(axisOk(ARM_Z_AXIS, joy_msg)) {
            left_arm_vz_ = -joy_msg->axes[ARM_Z_AXIS]*arm_z_scale_;
          } else {
            left_arm_vz_ = 0.0;
          }
          // ROS_INFO_STREAM("Setting vx " << left_arm_vx_ << " " << left_arm_vy_ << " " << left_arm_vz_);
        } else {
          left_arm_vx_ = 0.0;
          left_arm_vy_ = 0.0;
          left_arm_vz_ = 0.0;
        }
      }
    } else if (layout != LAYOUT_BOTH_ARMS) {
      des_left_wrist_vel_ = 0.0;
      des_left_shoulder_pan_vel_ = 0.0;
      des_left_shoulder_tilt_vel_ = 0.0;
      left_arm_vx_ = 0.0;
      left_arm_vy_ = 0.0;
      left_arm_vz_ = 0.0;
    }
    if(layout == LAYOUT_BOTH_ARMS) {
      if(buttonOkAndOn(ARM_MODE_TOGGLE_BUTTON, joy_msg) && !sameValueAsLast(ARM_MODE_TOGGLE_BUTTON, joy_msg, last_joy_)) {
        GeneralCommander::ArmControlMode toSend;
	if(in_walk_along) {
          gc->turnOffWalkAlong();
          ROS_INFO("Turning off walk along");
        }
        if(gc->getArmMode(GeneralCommander::ARMS_RIGHT) == GeneralCommander::ARM_POSITION_CONTROL) {
          toSend = GeneralCommander::ARM_MANNEQUIN_MODE;
          ROS_INFO("BOTH ARM MANNEQUIN");
        } else if (gc->getArmMode(GeneralCommander::ARMS_RIGHT) == GeneralCommander::ARM_MANNEQUIN_MODE) {
          toSend = GeneralCommander::ARM_NO_CONTROLLER;
          ROS_INFO("BOTH ARM NO CONTROLLER");
        } else {
          toSend = GeneralCommander::ARM_POSITION_CONTROL;
          ROS_INFO("BOTH ARM POS CONTROLLER");
        }
        gc->setArmMode(GeneralCommander::ARMS_BOTH, toSend);
      }

      // if(buttonOkAndOn(ARM_TUCK_BUTTON, joy_msg) && !sameValueAsLast(ARM_TUCK_BUTTON, joy_msg, last_joy_)) 
      if (tuck_mode == TUCK_MODE) {
        if(in_walk_along) {
          gc->turnOffWalkAlong();
          ROS_INFO("Turning off walk along");
        }
        ROS_INFO("Tuck");
        gc->tuckArms(GeneralCommander::ARMS_BOTH);        
      } else if (tuck_mode == MIMIC_MODE) {
        if(in_walk_along) {
          gc->turnOffWalkAlong();
          ROS_INFO("Turning off walk along");
        }
        ROS_INFO("Mimic");
        gc->mimicArms(GeneralCommander::ARMS_BOTH);
      } else if (tuck_mode == UNTUCK_MODE) {
      // else if(buttonOkAndOn(ARM_UNTUCK_BUTTON, joy_msg) && !sameValueAsLast(ARM_UNTUCK_BUTTON, joy_msg, last_joy_)) 
        if(in_walk_along) {
          gc->turnOffWalkAlong();
          ROS_INFO("Turning off walk along");
        }
        ROS_INFO("UnTuck");
        gc->untuckArms(GeneralCommander::ARMS_BOTH);
      } 

      // if(!in_walk_along) 
      {
        bool lookAnalog = false;
        bool rotClock = buttonOkAndOn(WRIST_CLOCKWISE_BUTTON, joy_msg);
        bool rotCounter = buttonOkAndOn(WRIST_COUNTER_BUTTON, joy_msg);
        if(rotClock && !rotCounter) {
          des_left_wrist_vel_ = wrist_velocity_;
          des_right_wrist_vel_ = wrist_velocity_;
        } else if(!rotClock && rotCounter) {
          des_left_wrist_vel_ = -wrist_velocity_;
          des_right_wrist_vel_ = -wrist_velocity_;
        } else {
          des_left_wrist_vel_ = 0.0;
          des_right_wrist_vel_ = 0.0;
          lookAnalog = true;
        }

        bool shoulder_pan_left = SHOULDER_PAN_LEFT;
        bool shoulder_pan_right = SHOULDER_PAN_RIGHT;
        if(shoulder_pan_left && !shoulder_pan_right) {
          des_left_shoulder_pan_vel_ = shoulder_pan_velocity_;
          des_right_shoulder_pan_vel_ = shoulder_pan_velocity_;
        } else if(!shoulder_pan_left && shoulder_pan_right) {
          des_left_shoulder_pan_vel_ = shoulder_pan_velocity_;
          des_right_shoulder_pan_vel_ = shoulder_pan_velocity_; 
        } else {
          des_left_shoulder_pan_vel_ = 0.0;
          des_right_shoulder_pan_vel_ = 0.0;
          lookAnalog = true;
        }

        bool shoulder_tilt_up = SHOULDER_TILT_UP;
        bool shoulder_tilt_down = SHOULDER_TILT_DOWN;
        if(shoulder_tilt_up && !shoulder_tilt_down) {
          des_left_shoulder_tilt_vel_ = -shoulder_tilt_velocity_;
          des_right_shoulder_tilt_vel_ = -shoulder_tilt_velocity_;
        } else if(!shoulder_tilt_up && shoulder_tilt_down) {
          des_left_shoulder_tilt_vel_ = shoulder_tilt_velocity_;
          des_right_shoulder_tilt_vel_ = shoulder_tilt_velocity_; 
        } else {
          des_left_shoulder_tilt_vel_ = 0.0;
          des_right_shoulder_tilt_vel_ = 0.0;
          lookAnalog = true;
        }
        if(lookAnalog) {
          //look at analog sticks if we aren't supposed to wrist rotate
          if(axisOk(ARM_X_AXIS, joy_msg)) {
            left_arm_vx_ = joy_msg->axes[ARM_X_AXIS]*arm_x_scale_;
            right_arm_vx_ = joy_msg->axes[ARM_X_AXIS]*arm_x_scale_;
          } else {
            left_arm_vx_ = 0.0;
            right_arm_vz_ = 0.0;
          }
          if(axisOk(ARM_Y_AXIS, joy_msg)) {
            left_arm_vy_ = -1 * joy_msg->axes[ARM_Y_AXIS]*arm_y_scale_;
            right_arm_vy_ = -1 * joy_msg->axes[ARM_Y_AXIS]*arm_y_scale_;
          } else {
            left_arm_vy_ = 0.0;
            right_arm_vz_ = 0.0;
          }
          if(axisOk(ARM_Z_AXIS, joy_msg)) {
            left_arm_vz_ = -joy_msg->axes[ARM_Z_AXIS]*arm_z_scale_;
            right_arm_vz_ = -joy_msg->axes[ARM_Z_AXIS]*arm_z_scale_;
          } else {
            left_arm_vz_ = 0.0;
            right_arm_vz_ = 0.0;
          }
        // ROS_INFO_STREAM("Setting vx " << left_arm_vx_ << " " << left_arm_vy_ << " " << left_arm_vz_);
        } else {
          left_arm_vx_ = 0.0;
          left_arm_vy_ = 0.0;
          left_arm_vz_ = 0.0;
          right_arm_vx_ = 0.0;
          right_arm_vy_ = 0.0;
          right_arm_vz_ = 0.0;
        }
      }
    } else if (layout != LAYOUT_RIGHT_ARM && layout != LAYOUT_LEFT_ARM) {
      des_right_wrist_vel_ = 0.0;
      des_left_wrist_vel_ = 0.0;
      des_left_shoulder_pan_vel_ = 0.0;
      des_right_shoulder_pan_vel_ = 0.0;
      des_left_shoulder_tilt_vel_ = 0.0;
      des_right_shoulder_tilt_vel_ = 0.0;
      left_arm_vx_ = 0.0;
      left_arm_vy_ = 0.0;
      left_arm_vz_ = 0.0;
      right_arm_vx_ = 0.0;
      right_arm_vy_ = 0.0;
      right_arm_vz_ = 0.0;
    }

    joy_deadman_ = ros::Time::now();
    last_joy_ = joy_msg;
  }

/*
  bool convertCurrentVelToDesiredTorsoPos(double hz) {
    if(!torso_init_)  {
      double cur_torso_pos = 0.0;
      bool ok = gc->getJointPosition("torso_lift_joint", cur_torso_pos);
      if(!ok) return false;
      des_torso_pos_ = cur_torso_pos;
      torso_init_ = true;
    }
    double dt = 1.0/double(hz);
    double horizon = dt*5.0;
    double cur_torso_pos = 0.0;
    gc->getJointPosition("torso_lift_joint", cur_torso_pos);
    des_torso_pos_ = cur_torso_pos+des_torso_vel_ * horizon;
    des_torso_pos_ = std::min(des_torso_pos_, max_torso_);
    des_torso_pos_ = std::max(des_torso_pos_, min_torso_);
    return true;
  }

  bool convertCurrentVelToDesiredHeadPos(double hz) {
   
    if(!head_init_)  {
      double cur_pan_pos = 0.0;
      double cur_tilt_pos = 0.0;
      bool ok = gc->getJointPosition("head_pan_joint", cur_pan_pos);
      if(!ok) return false;
      ok = gc->getJointPosition("head_tilt_joint", cur_tilt_pos);
      if(!ok) return false;
      des_pan_pos_ = cur_pan_pos;
      des_tilt_pos_ = cur_tilt_pos;
      head_init_ = true;
    }
    if(fabs(vel_val_pan_) > .0001) {
      des_pan_pos_ = des_pan_pos_ + vel_val_pan_*(1.0/hz);
      des_pan_pos_ = std::min(des_pan_pos_, max_pan_);
      des_pan_pos_ = std::max(des_pan_pos_, -max_pan_);
    }
    if(fabs(vel_val_tilt_) > .0001) {
      des_tilt_pos_ = des_tilt_pos_ - vel_val_tilt_*(1.0/hz);
      des_tilt_pos_ = std::min(des_tilt_pos_, max_tilt_);
      des_tilt_pos_ = std::max(des_tilt_pos_, min_tilt_);
    }
    //ROS_INFO_STREAM("Des pan pos " << des_pan_pos_ << " tilt " << des_tilt_pos_);
    return true;
  }

*/

public:
/*
  double max_pan_, max_tilt_, min_tilt_;
  int axis_pan_, axis_tilt_;
  double pan_scale_, tilt_scale_;

  double des_pan_pos_;
  double des_tilt_pos_;

  double vel_val_pan_;
  double vel_val_tilt_;

  double des_vx_;
  double des_vy_;
  double des_vw_;

  double vx_scale_;
  double vy_scale_;
  double vw_scale_;

  double arm_x_scale_;
  double arm_y_scale_;
  double arm_z_scale_;

  double right_arm_vx_;
  double right_arm_vy_;
  double right_arm_vz_;

  double left_arm_vx_;
  double left_arm_vy_;
  double left_arm_vz_;

  double tuck_shoulder_pan_;

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

  bool head_init_;
  bool torso_init_;

  double req_torso_vel_;
  double req_torso_pos_;

  double des_torso_pos_;
  double des_torso_vel_;
  double torso_step_;
  double min_torso_;
  double max_torso_;

  double wrist_velocity_;
  double des_right_wrist_vel_;  
  double des_left_wrist_vel_;

  double walk_along_x_speed_scale_;
  double walk_along_y_speed_scale_;
  double walk_along_w_speed_scale_;
  double walk_along_thresh_;
  double walk_along_x_dist_max_;
  double walk_along_y_dist_max_;

  bool walk_along_init_waiting_;
  bool set_walk_along_mode_;

  std::string prosilica_namespace_;

  bool proj_toggle_com_; 
 
  int projector_toggle_button_;
  int tilt_toggle_button_;
  int switch_head_control_mode_button_;


  ros::Time joy_deadman_;

  bool first_callback_;

*/
  ros::NodeHandle n_;
  ros::Subscriber joy_sub_;

  ros::Time last_projector_toggle_;
  ros::Time last_laser_toggle_;
  ros::Time last_head_toggle_;

  ros::Time last_walk_along_time_;
  GeneralCommander* gc;
  sensor_msgs::JoyConstPtr last_joy_;
};

static const double FastHz = 100;
static const double SlowHz = 20;

void spin_function() {
  ros::spin();
}

// void quit(int sig)
// {
//   delete generaljoy;
//   ros::shutdown();
//   spin_thread.join();
//   exit(0);
// }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "prlite_telep_general_joystick", ros::init_options::NoSigintHandler);
  //signal(SIGINT,quit);

  boost::thread spin_thread(boost::bind(&spin_function));

  Pr2TeleopGeneralJoystick generaljoy;
  generaljoy.init();
  
ROS_INFO("init done");
  ros::Rate pub_rate(FastHz);
    
  unsigned int counter_limit = (unsigned int)(FastHz/SlowHz);

  unsigned int counter = 0;
/*
  generaljoy.des_vx_ = 0;
  generaljoy.des_vy_ = 0;
  generaljoy.des_vw_ = 0;
*/
  des_vx_ = 0;
  des_vy_ = 0;
  des_vw_ = 0;

  ros::Time beforeCall = ros::Time::now();
  ros::Time afterCall = ros::Time::now();
  while (ros::ok()) 
  {
    // ROS_INFO_STREAM("Time since last " << (ros::Time::now()-beforeCall).toSec());
    beforeCall = ros::Time::now();

    /* ARD */
    if(layout == LAYOUT_KINECT) {
      if (kinect_arms_up()) {
        generaljoy.gc->mimicArms(GeneralCommander::ARMS_LEFT);
        generaljoy.gc->mimicArms(GeneralCommander::ARMS_RIGHT);
      }
/*
      kinect_get_pos();
*/
      double x,y,z;
      double dx,dz;
      kinect_get_pos(&x,&y,&z);
      if (kinect_follow_ > 0) 
      {
       static int stopped = 0;

       // rotation
       if (fabs(x) > .01)
	 dz = x * 1.0;
       else
	 dz = 0;
       if (dz > 0.6) dz = 0.6;
       if (dz < -0.6) dz = -0.6;

       // forward/backward
       if (fabs(z - 1.50) > .1 && kinect_follow_ == 2)
	 dx = (z-1.50)*1.0;
       else
	 dx = 0;
       if (dx > 0.5) dx = 0.5;
       if (dx < -0.5) dx = -0.5;

       if (x != 0 || y != 0 || z != 0) {
         generaljoy.gc->sendBaseCommand(dx, 0.0, dz);
         ROS_INFO_STREAM("kinect x= " << x << " y= " << y << " z= " << z);
         ROS_INFO_STREAM("prlite x= " << dx << " y= " << 0 << " z= " << dz);
         stopped = 0;
       } else if (!stopped) {
         generaljoy.gc->sendBaseCommand(0.0, 0.0, 0.0);
         ROS_INFO_STREAM("kinect x= " << x << " y= " << y << " z= " << z);
         ROS_INFO_STREAM("prlite x= " << dx << " y= " << 0 << " z= " << dz);
         stopped = 1;
       }
     }
    } else 
    {
      // generaljoy.gc->sendHeadCommand( generaljoy.vel_val_pan_, generaljoy.vel_val_tilt_);
      generaljoy.gc->sendHeadCommand( vel_val_pan_, vel_val_tilt_);
       // ROS_INFO_STREAM("prlite x= " <<  generaljoy.des_vy_ << " y= " << generaljoy.des_vx_ << " z= " << generaljoy.des_vw_);
      // generaljoy.gc->sendBaseCommand(generaljoy.des_vy_, generaljoy.des_vx_, generaljoy.des_vw_);
      generaljoy.gc->sendBaseCommand(des_vy_, des_vx_, des_vw_);
/*
    if(!generaljoy.gc->isWalkAlongOk() && !generaljoy.set_walk_along_mode_ && !generaljoy.walk_along_init_waiting_) {
      if(generaljoy.convertCurrentVelToDesiredHeadPos(FastHz)) {
        generaljoy.gc->sendHeadCommand(generaljoy.des_pan_pos_, generaljoy.des_tilt_pos_);
      } 
      // ROS_INFO("HeadTrack");
      generaljoy.gc->sendHeadTrackCommand();
      // ROS_INFO("sendBase");
      // ARD: until lin act
      // generaljoy.gc->sendBaseCommand(generaljoy.des_vx_, generaljoy.des_vy_, generaljoy.des_vw_);
    }
*/
      
    if((counter % counter_limit) == 0) {
      
      counter = 0;
      
      if(set_walk_along_mode_) {
        bool ok = generaljoy.gc->moveToWalkAlongArmPose();
        //if we didn't get a select while moving the arms
        if(ok && set_walk_along_mode_) {
          ROS_INFO("Arms in walk position");
          walk_along_init_waiting_ = true;
        } else {
          ROS_INFO("Arms not in walk position");
        }
        set_walk_along_mode_ = false;
      }
      
      if(generaljoy.gc->isWalkAlongOk()) {
       
        ROS_INFO("WalkAlong");
        generaljoy.gc->sendWalkAlongCommand(walk_along_thresh_, 
                                            walk_along_x_dist_max_,
                                            walk_along_x_speed_scale_,
                                            walk_along_y_dist_max_,
                                            walk_along_y_speed_scale_,
                                            walk_along_w_speed_scale_);
      } 
/*
      else 
        if(generaljoy.convertCurrentVelToDesiredTorsoPos(SlowHz)) 
*/
      {
          // ROS_INFO("Torso");
          generaljoy.gc->sendTorsoCommand(des_torso_pos_, des_torso_vel_);
        }
        //generaljoy.gc->updateCurrentWristPositions();
        generaljoy.gc->sendWristVelCommands(des_right_wrist_vel_, des_left_wrist_vel_, SlowHz);
	//ROS_INFO_STREAM("Robert: about to update shoulder " << des_left_shoulder_tilt_vel_);
        generaljoy.gc->sendShoulderCommand(des_right_shoulder_tilt_vel_, 
                                               des_right_shoulder_pan_vel_, 
			                       des_left_shoulder_tilt_vel_, 
			                       des_left_shoulder_pan_vel_, 
					       SlowHz);
        // ROS_INFO("SendARmVelCom");
        
        generaljoy.gc->sendArmVelCommands(right_arm_vx_, right_arm_vy_, right_arm_vz_, 0.0,
                                          left_arm_vx_, left_arm_vy_, left_arm_vz_, 0.0,
                                          SlowHz);
    }
    }
    // ROS_INFO_STREAM("Everything took " << (afterCall-beforeCall).toSec());
    counter++;
    pub_rate.sleep();
  }
  
  // kinect_shutdown();
  ros::shutdown();
  spin_thread.join();
  return 0;
}
