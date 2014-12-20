/*
* prlite_base
* Copyright (c) 2011-2012, Andrew Downing, Alan Downing, Robert Ou
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "packets_485net/packet_485net_dgram.h"
#include "net_485net_id_handler/SearchID.h"
#include "pr2lite_moveit_config/ReturnJointStates.h"
#include <unistd.h>
#include <cstdlib>

#define MAX_LINACT 0x3FF

// const double X_MULT = 7.51913116; // speed is ticks per interval and interval is 1/10 sec, so should be WHEEL_TICKS_PER_METER / 10
// (1 meter) / ((6 * pi inches) / 36) / 10 sec = 7.51913117
// const double X_MULT = 7.51913117; // speed is ticks per interval and interval is 1/10 sec, so should be WHEEL_TICKS_PER_METER / 10
const double X_MULT = 7.51913117; // speed is ticks per interval and interval is 1/10 sec, so should be WHEEL_TICKS_PER_METER / 10
// const double X_MULT = 1.25318853;
// const double TH_MULT = 5; // tuned manually (is about right)
const double TH_MULT = 15; // tuned manually (is about right)
#define INCHES_TO_METERS 0.0254
#define linact_len(x) (x * 4.0 / 1000.0 * INCHES_TO_METERS)

const uint16_t EXTEND_LINACT_0 = 1000;
const uint16_t LINACT_0 = 915;
const uint16_t LINACT_45 = 530;
const uint16_t LINACT_90 = 70;
const uint16_t EXTEND_LINACT_90 = 0;
const uint16_t LINACT_PRECISION = 15;
const ros::Duration WHEEL_TO(5.0);
const ros::Duration LINACT_TO(5.0);
const ros::Duration LINACT_FINISH_TO(5.0);
const ros::Duration LINACT_EXTEND_TO(2.0);
extern bool get_linact_state();

ros::Publisher cmd_pub;
ros::Publisher linact_pub;

ros::ServiceClient idlookup;

// TODO: encapsolate globals into objects

// linact_commands
uint16_t linact_goal = LINACT_0;
uint16_t linact_goal_last = 10000;

// wheel_commands
int16_t cmd_l = 0;
int16_t cmd_l_last = 10000;
int16_t cmd_r = 0;
int16_t cmd_r_last = 10000;
bool cmd_90_last = false;
bool force_wheel_update = false;

int pid_p = 0;
int pid_i = 0;
int pid_d = 0;

bool rosinfodbg = true;

// base states
enum base_state_enum {ready, linact_start, linact_moving, linact_extend, wheel_start, wheel_moving};
std::string base_state_name[] = {"ready", "linact_start", "linact_moving", "linact_extend", "wheel_start", "wheel_moving"};
base_state_enum base_state = ready;
ros::Time base_state_time; // time of last transition

// wheel_state (sent in wheelCallback)
bool wheel_stopped = true;
//fl, fr, bl, br
bool wheel_started[4] = {false, false, false, false};

//front left, front right, back left, back right
unsigned char wheel_debug_bits[4] = {0};

// linact state (set in linactCallback)
bool linact_arrived = true;
bool linact_goal_arrived = true;

#define DIST_FROM_LIMIT	110
#define MOVING_LAST_N	5
#define NOT_MOVING_VAL	1
bool linact_does_not_seem_to_be_moving = false;

unsigned char already_in_state_change = 0;

void cmdPublishLinact(void);
void cmdPublishLinactStop(void);
void cmdPublishWheel(void);

bool state_timeout(base_state_enum state, ros::Time *now, ros::Duration timeout)
{
       if (base_state != state)
         return false;

       // Compute the time remaining to wait.
       if (*now - base_state_time > timeout) {
         base_state_time = *now;
         return true;
       }

       return false;
}

// TODO: Add mutex???
void state_change(void)
{  
  ros::Time now;
  base_state_enum base_state_last;
  
  if(already_in_state_change)
    return;
  
  already_in_state_change = 1;
  if (!get_linact_state()) {
    ROS_INFO("ERR: GET LINACT STATE FAILED");
    return;
  }
  
  //debug
  //linact_arrived = true;
  //linact_goal_arrived = true;

  // Get the current time
  now = ros::Time::now();
  do {
    base_state_last = base_state;
    // TODO: 
    // if linact in unknown state or not LINACT_0,
    // then don't do FWD (not sideways)
    if (base_state == ready) {
      if (!linact_arrived) {
        // shouldn't happen
        ROS_INFO("ERR: LINACT MOVING");
        base_state = linact_moving;
      } else if (!wheel_stopped) {
        ROS_INFO("ERR: WHEELS MOVING");
        // shouldn't happen
        base_state = wheel_moving;
      } else if (linact_goal == linact_goal_last && (cmd_l != 0 || cmd_r != 0)) {
        if (cmd_l * cmd_r < 0 && linact_goal != LINACT_45) {
          ROS_INFO("ERR: WHEEL ROTATION NOT AT LINACT_45");
          // illegal state; stop and refuse to go
          cmd_l = 0;
          cmd_r = 0;
          cmdPublishWheel();
          base_state = ready;
        } else if (cmd_l*cmd_r > 0 && linact_goal != EXTEND_LINACT_0 
                                   && linact_goal != EXTEND_LINACT_90) { 
          if (linact_goal == LINACT_0 || linact_goal == LINACT_90) {
            // initial condition, allow to extend
            ROS_INFO("linact not extended");
            base_state = linact_moving;
          } else {
            ROS_INFO("ERR: WHEELS WANT STRAIGHT WHEN LINACT NOT READY");
            // illegal state; stop and refuse to go
            cmd_l = 0;
            cmd_r = 0;
            cmdPublishWheel();
            base_state = ready;
          }
        } else {
          // publish wheel commands
          if (rosinfodbg) ROS_INFO("WHEEL START");
          base_state = wheel_start;
          cmdPublishWheel();
        }
      } else if (linact_goal != linact_goal_last) {
        // publish new linear actuator command
        base_state = linact_start;
        if (rosinfodbg) ROS_INFO("WHEEL  LINACT START");
        cmdPublishLinact();
      }
    } else if (base_state == linact_start) {
        if (!get_linact_state()) {
          ROS_INFO("ERR: GET LINACT STATE FAILED");
          return;
        }
        if (!linact_arrived || linact_goal_arrived) {  // linact is moving
          base_state = linact_moving;
        } else if (state_timeout(linact_start, &now, LINACT_TO)) {
          // timeout without linact moving
          if (cmd_l != 0 || cmd_r != 0)
            ROS_INFO("ERR: LINACT START TIMEOUT");

          // need to check all base_state settings to ready ; 
          //  cmdPublishLinactStop();
          //  base_state = ready;
        }
        else if (linact_does_not_seem_to_be_moving)
        {
          if (rosinfodbg) ROS_INFO("RESEND LINACT CMD ");
          cmdPublishLinact();
        }
    } else if (base_state == linact_moving) {
        if (!get_linact_state()) {
          ROS_INFO("ERR: GET LINACT STATE FAILED");
          return;
        }
        if (!wheel_stopped) {
          // shouldn't happen but...
          // wheels are moving but linear actuator didn't arrive yet, so stop!
          ROS_INFO("ERR: WHEELS MOVING WITH LINACT!");
          base_state = wheel_moving;
          // Stop, and then restore command
          int cmd_l_tmp = cmd_l;
          int cmd_r_tmp = cmd_r;
          cmd_l = 0;
          cmd_r = 0;
          cmdPublishWheel();
          cmd_l = cmd_l_tmp;
          cmd_r = cmd_r_tmp;
        } else if (!linact_arrived) {  // linact is moving
          base_state = linact_moving;  // TODO: any emergency stop?
          if (rosinfodbg) ROS_INFO("LINACT MOVING");
          if (linact_does_not_seem_to_be_moving)
          {
            linact_does_not_seem_to_be_moving = false;
            ROS_INFO("Want to place a bet that linear actuator is stopped");
            // cmdPublishLinactStop();
            // base_state = ready;
          }
        } else if (state_timeout(linact_moving, &now, LINACT_FINISH_TO)) {
          // TODO: make timeout more precise based on starting / end point
          ROS_INFO("ERR: LINACT FINISH TIMEOUT");
          // base_state = ready;
          now = ros::Time::now();
        } else if (linact_goal != linact_goal_last) {
          // publish new linear actuator command
          if (rosinfodbg) ROS_INFO("NEW LINACT CMD ");
          cmdPublishLinact();
        } 
        else if (linact_arrived && linact_goal_arrived)
        {
          if (rosinfodbg) ROS_INFO("BASE LINACT EXTEND");

          // done with linact
          if (linact_goal == LINACT_0) {
            linact_goal = EXTEND_LINACT_0;
            cmdPublishLinact();
            base_state = linact_extend;
            now = ros::Time::now();
          } else if (linact_goal == LINACT_90) {
            linact_goal = EXTEND_LINACT_90;
            cmdPublishLinact();
            base_state = linact_extend;
            now = ros::Time::now();
          } else
            base_state = ready;
        }
    } else if (state_timeout(linact_extend, &now, LINACT_EXTEND_TO)) {
            uint16_t tmp_linact_goal = linact_goal;
            ROS_INFO("linact extend timeout (expected)");
            cmdPublishLinactStop();
            // fake out linact stopping
            linact_arrived = true;
            linact_goal = tmp_linact_goal;
            linact_goal_last = linact_goal;
            ROS_INFO("linact extend timeout (expected)");
            base_state = ready;
    } else if (base_state == wheel_start) {
        if ((cmd_l == 0 || (wheel_started[0] && wheel_started[2])) &&
            (cmd_r == 0 || (wheel_started[1] && wheel_started[3]))) {
          base_state = wheel_moving;
          if (rosinfodbg) ROS_INFO("WHEEL moving");
        } else if (state_timeout(wheel_start, &now, WHEEL_TO)) {
          ROS_INFO("ERR: WHEEL START TIMEOUT");
          force_wheel_update = true;
          //base_state = ready;
          cmdPublishWheel();
        }
    } else if (base_state == wheel_moving) {
        if (wheel_stopped) {
          base_state = ready;
          if (rosinfodbg) ROS_INFO("WHEEL stopped; ready");
        } else if (linact_goal != linact_goal_last) { 
          // Change in command that requires new linact pos
          // Stop, and then restore command
          // Are we multithreaded?  If so, a potential bug
          int cmd_l_tmp = cmd_l;
          int cmd_r_tmp = cmd_r;
          cmd_l = 0;
          cmd_r = 0;
          force_wheel_update = true;
          if (rosinfodbg) ROS_INFO("stop wheels (new linact pos)");
          cmdPublishWheel();
          // restorre cmds after stopping
          cmd_l = cmd_l_tmp;
          cmd_r = cmd_r_tmp;
          base_state = ready;
          // force_wheel_update = true;
          // Change in wheel command with same linact pos
          // if (rosinfodbg) ROS_INFO("WHEEL cmd change, same linact");
          // cmdPublishWheel();
          // if (rosinfodbg) ROS_INFO("WHEEL cmd change, diff linact");
          // cmdPublishLinact();
        } 
        // Note: did Robert change this logic?  Revert.
        // else if (linact_goal != linact_goal_last) 
        else if (cmd_l != cmd_l_last || cmd_r != cmd_r_last)
        { 
          if (rosinfodbg) ROS_INFO("wheel cmd change");
          // base_state = ready;
          // cmdPublishLinact();
          if (rosinfodbg) ROS_INFO("WHEEL cmd change, same linact");
          cmdPublishWheel();
          // base_state = wheel_start;
        } else if (cmd_l == 0 && cmd_r == 0 && !wheel_stopped) {
          // try to stop again
          force_wheel_update = true;
          if (rosinfodbg) ROS_INFO("wheels not stopped");
          cmdPublishWheel();
          base_state = ready;
        }
     }
     if (base_state != base_state_last) {
       ROS_INFO_STREAM("state = " << base_state_name[base_state]);
       base_state_time = now;  // only place base_state_time can be set after init
     }
  
  } while (base_state != base_state_last);
  
  already_in_state_change = 0;
}

uint8_t lookup_id(const char *type, const char *desc)
{
  net_485net_id_handler::SearchID search;
  
  search.request.type = type;
  search.request.desc = desc;
  if(idlookup.call(search))
  {
    return search.response.res;
  }
  else
  {
    //ERROR
    return 0xFF;
  }
}

bool get_linact_state()
{
  static bool do_init = true;
  static int cnt = 0;
  static int notmoving = 0;
  static pr2lite_moveit_config::ReturnJointStates srv;
  static ros::ServiceClient client;

  if (do_init) { 
    ros::NodeHandle n;
    client = n.serviceClient<pr2lite_moveit_config::ReturnJointStates>("return_joint_states");
    do_init = false;
  }
  srv.request.name.push_back("wheel_linear_actuator_joint");
  if (cnt != 0 && base_state != linact_start && base_state != linact_moving) {
    cnt--;
    return true;
  }
  cnt = 4;
  if (client.call(srv))
  {
    ROS_INFO("wheel linact: velocity %f pos %f goal %d", srv.response.velocity[0], srv.response.position[0],linact_goal);
    if (srv.response.velocity[0] != 0)
    {
        linact_arrived = false;
        linact_does_not_seem_to_be_moving = false;
        notmoving = 0;
    } else {
        linact_arrived = true;
        notmoving++;
        // we're supposed to timeout for Linact extension goals
        if(notmoving >= MOVING_LAST_N && linact_goal != EXTEND_LINACT_0 && linact_goal != EXTEND_LINACT_90)
        {
          ROS_INFO("By the alignment of the planets, I conclude that the linact isn't actually moving.");
          linact_does_not_seem_to_be_moving = true;
        }
    }
    ROS_INFO("wheel linact: goal %f dif %f prec %f pos %f", linact_len(linact_goal), fabs(srv.response.position[0] - linact_len(linact_goal)), linact_len(LINACT_PRECISION), srv.response.position[0]);
    if (fabs(srv.response.position[0] - linact_len(linact_goal)) < linact_len(LINACT_PRECISION)
       || ((linact_goal == LINACT_0 || linact_goal == EXTEND_LINACT_0) && srv.response.position[0] > linact_len(LINACT_0))
       || ((linact_goal == LINACT_90 || linact_goal == EXTEND_LINACT_0) && srv.response.position[0] < linact_len(LINACT_90))) 
    {
        ROS_INFO("linact goal arrived");
        linact_goal_arrived = true;
        notmoving = 0;
    } else {
        linact_goal_arrived = false;
    }

    return true;
  }
  else
  {
    ROS_ERROR("Failed to call service ReturnJointState");
    do_init = true;
    return false;
  }
  return false;
}

// only routine that can change linact_goal_last except for extend timeout
void cmdPublishLinact(void)
{
  uint16_t itmp;
  packets_485net::packet_485net_dgram linact_cmd;
  linact_cmd.destination = lookup_id("lin-act", "wheel rotate");
  linact_cmd.source = 0xF0;
  linact_cmd.sport = 7;
  linact_cmd.dport = 1;
  
  itmp = linact_goal > LINACT_PRECISION ? linact_goal - LINACT_PRECISION : 0;
  linact_cmd.data.push_back(itmp & 0xFF);
  linact_cmd.data.push_back((itmp >> 8) & 0xFF);
  itmp = linact_goal + LINACT_PRECISION < MAX_LINACT ? linact_goal + LINACT_PRECISION : MAX_LINACT;
  linact_cmd.data.push_back(itmp & 0xFF);
  linact_cmd.data.push_back((itmp >> 8) & 0xFF);
  
  ROS_INFO("base linact %d", linact_goal);
  linact_pub.publish(linact_cmd);
  linact_goal_last = linact_goal;
}

void cmdPublishLinactStop(void)
{
  packets_485net::packet_485net_dgram linact_cmd;
  // linact_cmd.destination = lookup_id("lin-act", "wheel rotate");
  linact_cmd.destination = 13;
  linact_cmd.source = 0xF0;
  linact_cmd.sport = 7;
  linact_cmd.dport = 1;
  
  linact_cmd.data.push_back(0);
  linact_cmd.data.push_back(0);
  linact_cmd.data.push_back(0xff);
  linact_cmd.data.push_back(0x03);
  
  ROS_INFO("stopping linact");
  linact_pub.publish(linact_cmd);
  linact_goal_last = linact_goal;
}

static void initOneWheelPID(unsigned char id, int32_t p, int32_t i, int32_t d, unsigned char dir)
{
  if(id > 3)
  {
    ROS_INFO("ERROR: tried to send pid for invalid wheel %d\n", id);
    return;
  }
  
  const char * const names[] = {"front left", "front right", "back left", "back right"};

  packets_485net::packet_485net_dgram pid_gains;
  
  pid_gains.source = 0xF0;
  pid_gains.sport = 7;
  pid_gains.dport = 1;
    
  pid_gains.data.push_back((p >>  0) & 0xFF);
  pid_gains.data.push_back((p >>  8) & 0xFF);
  pid_gains.data.push_back((p >> 16) & 0xFF);
  pid_gains.data.push_back((p >> 24) & 0xFF);
  pid_gains.data.push_back((i >>  0) & 0xFF);
  pid_gains.data.push_back((i >>  8) & 0xFF);
  pid_gains.data.push_back((i >> 16) & 0xFF);
  pid_gains.data.push_back((i >> 24) & 0xFF);
  pid_gains.data.push_back((d >>  0) & 0xFF);
  pid_gains.data.push_back((d >>  8) & 0xFF);
  pid_gains.data.push_back((d >> 16) & 0xFF);
  pid_gains.data.push_back((d >> 24) & 0xFF);
  
  pid_gains.data.push_back(dir);
  
  pid_gains.destination = lookup_id("wheel-cnt", names[id]);
  
  int w = 10;
  do
  {
    ROS_INFO("Trying to send pid to %d %s %d\n", pid_gains.destination, names[id],wheel_debug_bits[id]);
    cmd_pub.publish(pid_gains);
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    // w--;
  }
  // ARD debugging hack
  while((wheel_debug_bits[id] & 4) == 0 && w > 0);
  // while(false);
  ROS_INFO("WC 1 : %d %d %d %d", lookup_id("wheel-cnt", "front left"), lookup_id("wheel-cnt", "front right") , lookup_id("wheel-cnt", "back right"), lookup_id("wheel-cnt", "back left"));
  ROS_INFO("pid_p %d pid_i %d pid_p %d\n", pid_p, pid_i, pid_d);
}

static void initWheelPID(void)
{
  /* TUNING THE PID
     First set K_i and K_d values to zero. 
     Increase the K_p until the output of the loop oscillates, 
     Then the K_p should be set to approximately half of that value 
         for a "quarter amplitude decay" type response. 
     Then increase K_i until any offset is corrected 
          Too much K_i will cause instability. 
     Finally, increase K_d, if required, until the loop is acceptably quick 
          to reach its reference after a load disturbance. i
          Too much K_d will cause excessive response and overshoot. 
     A fast PID loop tuning usually overshoots slightly to reach the setpoint 
     If overshoot undesirable set K_p significantly less than half that of 
          the K_p setting that was causing oscillation.

Effects of increasing a parameter independently
Parameter Rise time 	Overshoot 	Settling time 	Steady-state?   Stabile
K_p 	Decrease 	Increase 	Small change 	Decrease 	Degrade
K_i 	Decrease 	Increase 	Increase 	Eliminate 	Degrade
K_d 	Minor change 	Decrease 	Decrease 	No effect in theory 	Improve if K_d small
  */ 
  int16_t p = pid_p;
  int16_t i = pid_i;
  int16_t d = pid_d;

  //front left
  initOneWheelPID(0, p << 16, i << 16, d << 16, 1);
  //front right
  initOneWheelPID(1, p << 16, i << 16, d << 16, 255);
  //back left
  initOneWheelPID(2, p << 16, i << 16, d << 16, 1);
  //back right
  initOneWheelPID(3, p << 16, i << 16, d << 16, 255);
}

static void multicastSetWheelSpeeds(int16_t fl, int16_t fr, int16_t bl, int16_t br)
{
  unsigned char idfl = lookup_id("wheel-cnt", "front left");
  unsigned char idfr = lookup_id("wheel-cnt", "front right");
  unsigned char idbl = lookup_id("wheel-cnt", "back left");
  unsigned char idbr = lookup_id("wheel-cnt", "back right");

  packets_485net::packet_485net_dgram wheel_cmd;
  
  wheel_cmd.source = 0xF0;
  wheel_cmd.sport = 7;
  wheel_cmd.dport = 6;
  wheel_cmd.destination = lookup_id("multicast", "all wheels");
  
  wheel_cmd.data.push_back(5);
  wheel_cmd.data.push_back(idfl);
  wheel_cmd.data.push_back(1);
  wheel_cmd.data.push_back((fl >> 0) & 0xFF);
  wheel_cmd.data.push_back((fl >> 8) & 0xFF);
  
  wheel_cmd.data.push_back(5);
  wheel_cmd.data.push_back(idfr);
  wheel_cmd.data.push_back(1);
  wheel_cmd.data.push_back((fr >> 0) & 0xFF);
  wheel_cmd.data.push_back((fr >> 8) & 0xFF);
  
  wheel_cmd.data.push_back(5);
  wheel_cmd.data.push_back(idbl);
  wheel_cmd.data.push_back(1);
  wheel_cmd.data.push_back((bl >> 0) & 0xFF);
  wheel_cmd.data.push_back((bl >> 8) & 0xFF);
  
  wheel_cmd.data.push_back(5);
  wheel_cmd.data.push_back(idbr);
  wheel_cmd.data.push_back(1);
  wheel_cmd.data.push_back((br >> 0) & 0xFF);
  wheel_cmd.data.push_back((br >> 8) & 0xFF);
  
  do
  {
    ROS_INFO("Sending command to wheels (%hd, %hd, %hd, %hd)\n", fl, fr, bl, br);
    cmd_pub.publish(wheel_cmd);
    ros::Duration(1.0).sleep();
    ros::spinOnce();
  }
  while((wheel_debug_bits[0] & 0x10) != 0x10 || (wheel_debug_bits[1] & 0x10) != 0x10 || (wheel_debug_bits[2] & 0x10) != 0x10 || (wheel_debug_bits[3] & 0x10) != 0x10);
  
  //all got commands now
  wheel_cmd.data.clear();
  wheel_cmd.data.push_back(0xa5);
  wheel_cmd.data.push_back(0x5a);
  wheel_cmd.data.push_back(0xcc);
  wheel_cmd.data.push_back(0x33);
  wheel_cmd.dport = 3;
  
  bool needsfl, needsfr, needsbl, needsbr;
  needsfl = needsfr = needsbl = needsbr = true;
  
  do
  {
    if(needsfl)
    {
      ROS_INFO("Sending sync deassert to fl\n");
      wheel_cmd.destination = idfl;
      cmd_pub.publish(wheel_cmd);
    }
    if(needsfr)
    {
      ROS_INFO("Sending sync deassert to fr\n");
      wheel_cmd.destination = idfr;
      cmd_pub.publish(wheel_cmd);
    }
    if(needsbl)
    {
      ROS_INFO("Sending sync deassert to bl\n");
      wheel_cmd.destination = idbl;
      cmd_pub.publish(wheel_cmd);
    }
    if(needsbr)
    {
      ROS_INFO("Sending sync deassert to br\n");
      wheel_cmd.destination = idbr;
      cmd_pub.publish(wheel_cmd);
    }
    
    ros::Duration(1.0).sleep();
    
    ros::spinOnce();
    
    needsfl = (wheel_debug_bits[0] & 0x10) != 0;
    needsfr = (wheel_debug_bits[1] & 0x10) != 0;
    needsbl = (wheel_debug_bits[2] & 0x10) != 0;
    needsbr = (wheel_debug_bits[3] & 0x10) != 0;
  }
  while(needsfl || needsfr || needsbl || needsbr);
}

// only routine that can change cmd_l_last, cmd_r_last
void cmdPublishWheel(void)
{
  if (cmd_l != cmd_l_last || cmd_r != cmd_r_last || (LINACT_90 == linact_goal || EXTEND_LINACT_90 == linact_goal) != cmd_90_last || force_wheel_update)
  {
    // publish wheel commands
    packets_485net::packet_485net_dgram wheel_cmd;
    
    ROS_INFO("Wheel update forced");
    force_wheel_update = false;
    
    wheel_cmd.source = 0xF0;
    wheel_cmd.sport = 7;
    wheel_cmd.dport = 6;
    if (LINACT_90 == linact_goal || EXTEND_LINACT_90 == linact_goal)
    {
      ROS_INFO("LINACT_90");
      // move sideways (positive cmd_l/cmd_r means go left)
      
      multicastSetWheelSpeeds(-cmd_r, cmd_r, cmd_l, -cmd_l);
    }
    else
    {
      ROS_INFO("MOVE F/B");
      // move forwards/backwards or spin in place
      
      multicastSetWheelSpeeds(cmd_l, cmd_r, cmd_l, cmd_r);
    }
    cmd_l_last = cmd_l;
    cmd_r_last = cmd_r;
    cmd_90_last = (LINACT_90 == linact_goal || EXTEND_LINACT_90 == linact_goal);
    if (rosinfodbg) ROS_INFO("l %d r %d", cmd_l, cmd_r);
  }
}

void cmdCallback(const geometry_msgs::Twist& cmd_vel)
{
  if (fabs(cmd_vel.linear.x) >= fabs(cmd_vel.linear.y))
  {
    // move forwards/backwards
    cmd_l = cmd_vel.linear.x * X_MULT - cmd_vel.angular.z * TH_MULT;
    cmd_r = cmd_vel.linear.x * X_MULT + cmd_vel.angular.z * TH_MULT;
    if (linact_goal != EXTEND_LINACT_0)
      linact_goal = LINACT_0;
  }
  else
  {
    // move sideways
    // Note: cmdPublishWheel will make the wheels move in opposite directions
    // to make sideways wheels go fwd/bck.
    cmd_l = cmd_vel.linear.y * X_MULT - cmd_vel.angular.z * TH_MULT;
    cmd_r = cmd_vel.linear.y * X_MULT + cmd_vel.angular.z * TH_MULT;
    if (linact_goal != EXTEND_LINACT_90)
      linact_goal = LINACT_90;
  }
  if (cmd_l * cmd_r < 0)
  {
    // wheels are told to go opposite directions, so spin in place
    cmd_l = -cmd_vel.angular.z * TH_MULT;
    cmd_r = cmd_vel.angular.z * TH_MULT;
    linact_goal = LINACT_45;
    ROS_INFO("LINACT_45");
  }
  ROS_INFO("x %f y %f th %f cmd_l %d cmd_r %d linact_goal %d", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z, cmd_l, cmd_r, linact_goal);
  state_change();
}

// only place to set wheel_stopped
void wheelCallback(const packets_485net::packet_485net_dgram& ws)
{
  static int16_t vl[2];
  static int16_t vr[2];

  uint16_t itmp;
// ROS_INFO("WC 1 : %d  %d %d %d %d", ws.source, lookup_id("wheel-cnt", "front left"), lookup_id("wheel-cnt", "front right") , lookup_id("wheel-cnt", "front right"), lookup_id("wheel-cnt", "back left"));
  if(ws.source != lookup_id("wheel-cnt", "front left") && ws.source != lookup_id("wheel-cnt", "front right") && ws.source != lookup_id("wheel-cnt", "back left") && ws.source != lookup_id("wheel-cnt", "back right"))
    return;
// ROS_INFO("WC 2");
  if(!(ws.destination == 0xF0 || ws.destination == 0x00))
    return;
// ROS_INFO("WC 3");
  if(ws.dport != 7)
    return;
// ROS_INFO("WC 4");
  if(ws.data.size() != 23)
    return;
// ROS_INFO("WC 5");
    
  itmp = ws.data[4] | ((ws.data[5]) << 8);
  ROS_INFO("ws data[4,5] = %d", itmp); 
  if(ws.source == lookup_id("wheel-cnt", "front left"))
  {
    vl[0] = itmp;
    wheel_debug_bits[0] = ws.data[22];
  }
  else if(ws.source == lookup_id("wheel-cnt", "front right"))
  {
    vr[0] = itmp;
    wheel_debug_bits[1] = ws.data[22];
  }
  else if(ws.source == lookup_id("wheel-cnt", "back left"))
  {
    vl[1] = itmp;
    wheel_debug_bits[2] = ws.data[22];
  }
  else if(ws.source == lookup_id("wheel-cnt", "back right"))
  {
    vr[1] = itmp;
    wheel_debug_bits[3] = ws.data[22];
  }
  itmp = ws.data[20] | ((ws.data[21]) << 8);
  ROS_INFO("ws data[20,21] (want 3k) = %d", itmp); 


  if(rosinfodbg)
    ROS_INFO("debug bits %02X %02X %02X %02X", wheel_debug_bits[0], wheel_debug_bits[1], wheel_debug_bits[2], wheel_debug_bits[3]);
  //int addr = ws.srcaddr / 2 - 1;
  //vl[addr] = ws.ticks0_interval;
  //vr[addr] = ws.ticks1_interval;  
  wheel_stopped = (0 == vl[0] && 0 == vl[1] && 0 == vr[0] && 0 == vr[1]);
  if (rosinfodbg) ROS_INFO("wheel_stopped %d %d %d %d", vl[0], vl[1], vr[0], vr[1]);
  wheel_started[0] = vl[0] != 0;    //fl
  wheel_started[1] = vr[0] != 0;    //fr
  wheel_started[2] = vl[1] != 0;    //bl
  wheel_started[3] = vr[1] != 0;    //br
  state_change();
}

// only place to set linact_arrived and linact_goal_arrived
void linactCallback(const packets_485net::packet_485net_dgram& linear_actuator_status)
{
  uint16_t itmp;
  static int notmoving = 0;
  static int last_pos = -1;
  if(linear_actuator_status.source != lookup_id("lin-act", "wheel rotate"))
    return;
  if(!(linear_actuator_status.destination == 0xF0 || linear_actuator_status.destination == 0x00))
    return;
  if(linear_actuator_status.dport != 7)
    return;
  if(linear_actuator_status.data.size() != 7)
    return;
  linact_arrived = linear_actuator_status.data[4+2];
  itmp = linear_actuator_status.data[4] | ((linear_actuator_status.data[5]) << 8);
  //linact_arrived = true; // hack for if linear actuator isn't working
  linact_goal_arrived = (itmp >= linact_goal - LINACT_PRECISION && itmp <= linact_goal + LINACT_PRECISION);
  
  ROS_INFO("base: linact is at %d\n", itmp);
  
  //if we are within +- NOT_MOVING_VAL of the previous value, we probably aren't moving
  if((itmp >= last_pos - NOT_MOVING_VAL) && (itmp <= last_pos + NOT_MOVING_VAL))
    notmoving++;
  else
    notmoving = 0;
  last_pos = itmp;
  ROS_INFO("base: not moving for %d\n", notmoving);
  
  if(notmoving >= MOVING_LAST_N && ((itmp > 0x400 - DIST_FROM_LIMIT) || (itmp < DIST_FROM_LIMIT)))
  {
    ROS_INFO("By the alignment of the planets, I conclude that the linact isn't actually moving.");
    linact_does_not_seem_to_be_moving = true;
    notmoving = 0;
  }
  else
    linact_does_not_seem_to_be_moving = false;
  
  state_change();
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "prlite_base_controller");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
  */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */

  base_state_time = ros::Time::now();
  
  idlookup = n.serviceClient<net_485net_id_handler::SearchID>("/search_id", true);
  idlookup.waitForExistence();

  ros::Subscriber cmd_sub = n.subscribe("cmd_vel", 1000, cmdCallback);
  ros::Subscriber wheel_sub = n.subscribe("net_485net_incoming_dgram", 1000, wheelCallback);
  ros::Subscriber linact_sub = n.subscribe("net_485net_incoming_dgram", 1000, linactCallback);

  cmd_pub = n.advertise<packets_485net::packet_485net_dgram>("net_485net_outgoing_dgram", 1000);
  linact_pub = n.advertise<packets_485net::packet_485net_dgram>("net_485net_outgoing_dgram", 1000);
  
  n.param("/base_controller/pid_p", pid_p, 11);
  n.param("/base_controller/pid_i", pid_i, 6);
  n.param("/base_controller/pid_d", pid_d, 0);
  initWheelPID();

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}

