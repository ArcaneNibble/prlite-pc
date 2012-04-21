/*
* prlite_base
* Copyright (c) 2011, Andrew Downing, Alan Downing
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
#include <unistd.h>

#define MAX_LINACT 0x3FF

const double X_MULT = 7.51913116; // speed is ticks per interval and interval is 1/10 sec, so should be WHEEL_TICKS_PER_METER / 10
const double TH_MULT = 5; // tuned manually (is about right)
const uint16_t LINACT_0 = 940;
const uint16_t LINACT_45 = 500;
const uint16_t LINACT_90 = 62;
const uint16_t LINACT_PRECISION = 15;
const ros::Duration WHEEL_TO(5.0);
const ros::Duration LINACT_TO(5.0);
const ros::Duration LINACT_FINISH_TO(5.0);

ros::Publisher pid_pub;
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

// base states
enum base_state_enum {ready, linact_start, linact_moving, wheel_start, wheel_moving};
std::string base_state_name[] = {"ready", "linact_start", "linact_moving", "wheel_start", "wheel_moving"};
base_state_enum base_state = ready;
ros::Time base_state_time; // time of last transition

// wheel_state (sent in wheelCallback)
bool wheel_stopped = true;
//fl, fr, bl, br
bool wheel_started[4] = {false, false, false, false};

// linact state (set in linactCallback)
bool linact_arrived = true;
bool linact_goal_arrived = true;

void cmdPublishLinact(void);
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
  
  //debug
  //linact_arrived = true;
  //linact_goal_arrived = true;

  // Get the current time
  now = ros::Time::now();
  do {
    base_state_last = base_state;
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
        // publish wheel commands
        base_state = wheel_start;
        cmdPublishWheel();
      } else if (linact_goal != linact_goal_last) {
        // publish new linear actuator command
        base_state = linact_start;
        cmdPublishLinact();
      }
    } else if (base_state == linact_start) {
        if (!linact_arrived || linact_goal_arrived) {  // linact is moving
          base_state = linact_moving;
        } else if (state_timeout(linact_start, &now, LINACT_TO)) {
          // timeout without linact moving
          if (cmd_l != 0 || cmd_r != 0)
            ROS_INFO("ERR: LINACT START TIMEOUT");
          base_state = ready;
        }
    } else if (base_state == linact_moving) {
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
        } else if (state_timeout(linact_moving, &now, LINACT_FINISH_TO)) {
          // TODO: make timeout more precise based on starting / end point
          ROS_INFO("ERR: LINACT FINISH TIMEOUT");
          base_state = ready;
        } else if (linact_goal != linact_goal_last) {
          // publish new linear actuator command
          cmdPublishLinact();
        } 
		else if (linact_arrived && linact_goal_arrived)
		{
			//done with linact
			base_state = ready;
		}
    } else if (base_state == wheel_start) {
        if ((cmd_l == 0 || (wheel_started[0] && wheel_started[2])) &&
		    (cmd_r == 0 || (wheel_started[1] && wheel_started[3]))) {
          base_state = wheel_moving;
        } else if (state_timeout(wheel_start, &now, WHEEL_TO)) {
          ROS_INFO("ERR: WHEEL START TIMEOUT");
		  force_wheel_update = true;
          //base_state = ready;
          cmdPublishWheel();
        }
    } else if (base_state == wheel_moving) {
        if (wheel_stopped) {
          base_state = ready;
        } else if (linact_goal != linact_goal_last) { 
          // Change in command that requires new linact pos
          // Stop, and then restore command
          // Are we multithreaded?  If so, a potential bug
          int cmd_l_tmp = cmd_l;
          int cmd_r_tmp = cmd_r;
          cmd_l = 0;
          cmd_r = 0;
		  force_wheel_update = true;
          cmdPublishWheel();
          cmd_l = cmd_l_tmp;
          cmd_r = cmd_r_tmp;
		  force_wheel_update = true;
        } else if (cmd_l != cmd_l_last || cmd_r != cmd_r_last) {
          // Change in wheel command with same linact pos
          cmdPublishWheel();
		  base_state = wheel_start;
        } else if (cmd_l == 0 && cmd_r == 0 && !wheel_stopped) {
          // try to stop again
		  force_wheel_update = true;
          cmdPublishWheel();
        }
     }
     if (base_state != base_state_last) {
       ROS_INFO_STREAM("state = " << base_state_name[base_state]);
       base_state_time = now;  // only place base_state_time can be set after init
     }
  
  } while (base_state != base_state_last);
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

// only routine that can change linact_goal_last
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
	
  ROS_INFO("linact %d", linact_goal);
  linact_pub.publish(linact_cmd);
  linact_goal_last = linact_goal;
}

// only routine that can change cmd_l_last, cmd_r_last
void cmdPublishWheel(void)
{
  static bool init = true;
  if (init) {
    // publish pid gains
    packets_485net::packet_485net_dgram pid_gains;
	
	pid_gains.source = 0xF0;
	pid_gains.sport = 7;
	pid_gains.dport = 1;
	  
	pid_gains.data.push_back(0);
	pid_gains.data.push_back(0);
	pid_gains.data.push_back(5);
	pid_gains.data.push_back(0);		//this is 5.0 (p)
	pid_gains.data.push_back(0);
	pid_gains.data.push_back(0);
	pid_gains.data.push_back(25);
	pid_gains.data.push_back(0);		//this is 25.0 (i)
	pid_gains.data.push_back(0);
	pid_gains.data.push_back(0);
	pid_gains.data.push_back(3);
	pid_gains.data.push_back(0);		//this is 3.0 (d)
	
	pid_gains.data.push_back(0xFF);		//do reverse
	pid_gains.destination = lookup_id("wheel-cnt", "front right");
    pid_pub.publish(pid_gains);
	pid_gains.data[12] = 0xFF;		//do reverse	//this isn't push_back
	pid_gains.destination = lookup_id("wheel-cnt", "back right");
    pid_pub.publish(pid_gains);
	pid_gains.data[12] = 1;		//do not reverse
	pid_gains.destination = lookup_id("wheel-cnt", "front left");
    pid_pub.publish(pid_gains);
	pid_gains.data[12] = 1;		//do not reverse
	pid_gains.destination = lookup_id("wheel-cnt", "back left");
    pid_pub.publish(pid_gains);
    init = false;	//robert: I have no idea why this was commented out
  }
  if (cmd_l != cmd_l_last || cmd_r != cmd_r_last || (LINACT_90 == linact_goal) != cmd_90_last || force_wheel_update)
  {
    // publish wheel commands
    packets_485net::packet_485net_dgram wheel_cmd;
	
	ROS_INFO("Wheel update forced");
	force_wheel_update = false;
	
	wheel_cmd.source = 0xF0;
	wheel_cmd.sport = 7;
	wheel_cmd.dport = 2;
    if (LINACT_90 == linact_goal)
    {
      ROS_INFO("LINACT_90");
      // move sideways (positive cmd_l/cmd_r means go left)
      // back wheels
      wheel_cmd.data.push_back(cmd_l & 0xFF);
      wheel_cmd.data.push_back((cmd_l >> 8) & 0xFF);
	  wheel_cmd.destination = lookup_id("wheel-cnt", "front right");
      cmd_pub.publish(wheel_cmd);
	  
      wheel_cmd.data[0] = (-cmd_l) & 0xFF;
      wheel_cmd.data[1] = ((-cmd_l) >> 8) & 0xFF;
	  wheel_cmd.destination = lookup_id("wheel-cnt", "front left");
      cmd_pub.publish(wheel_cmd);
	  
      // front wheels
      wheel_cmd.data[0] = cmd_r & 0xFF;
      wheel_cmd.data[1] = (cmd_r >> 8) & 0xFF;
	  wheel_cmd.destination = lookup_id("wheel-cnt", "back right");
      cmd_pub.publish(wheel_cmd);
	  
      wheel_cmd.data[0] = (-cmd_r) & 0xFF;
      wheel_cmd.data[1] = ((-cmd_r) >> 8) & 0xFF;
	  wheel_cmd.destination = lookup_id("wheel-cnt", "back left");
      cmd_pub.publish(wheel_cmd);
    }
    else
    {
      ROS_INFO("MOVE F/B");
      // move forwards/backwards or spin in place
	  
	  wheel_cmd.dport = 6;
	  wheel_cmd.data.push_back(5);
	  wheel_cmd.data.push_back(lookup_id("wheel-cnt", "front left"));
	  wheel_cmd.data.push_back(1);
      wheel_cmd.data.push_back(cmd_l & 0xFF);
      wheel_cmd.data.push_back((cmd_l >> 8) & 0xFF);
	  
	  wheel_cmd.data.push_back(5);
	  wheel_cmd.data.push_back(lookup_id("wheel-cnt", "front right"));
	  wheel_cmd.data.push_back(1);
      wheel_cmd.data.push_back(cmd_r & 0xFF);
      wheel_cmd.data.push_back((cmd_r >> 8) & 0xFF);
	  
	  wheel_cmd.data.push_back(5);
	  wheel_cmd.data.push_back(lookup_id("wheel-cnt", "back left"));
	  wheel_cmd.data.push_back(1);
      wheel_cmd.data.push_back(cmd_l & 0xFF);
      wheel_cmd.data.push_back((cmd_l >> 8) & 0xFF);
	  
	  wheel_cmd.data.push_back(0);
	  wheel_cmd.data.push_back(lookup_id("wheel-cnt", "back right"));
	  wheel_cmd.data.push_back(1);
      wheel_cmd.data.push_back(cmd_r & 0xFF);
      wheel_cmd.data.push_back((cmd_r >> 8) & 0xFF);
	  
	  wheel_cmd.destination = lookup_id("multicast", "all wheels");
      cmd_pub.publish(wheel_cmd);
    }
    cmd_l_last = cmd_l;
    cmd_r_last = cmd_r;
    cmd_90_last = (LINACT_90 == linact_goal);
    ROS_INFO("l %d r %d", cmd_l, cmd_r);
  }
}

void cmdCallback(const geometry_msgs::Twist& cmd_vel)
{
  if (fabs(cmd_vel.linear.x) >= fabs(cmd_vel.linear.y))
  {
    // move forwards/backwards
    cmd_l = cmd_vel.linear.x * X_MULT - cmd_vel.angular.z * TH_MULT;
    cmd_r = cmd_vel.linear.x * X_MULT + cmd_vel.angular.z * TH_MULT;
    linact_goal = LINACT_0;
  }
  else
  {
    // move sideways
    cmd_l = cmd_vel.linear.y * X_MULT - cmd_vel.angular.z * TH_MULT;
    cmd_r = cmd_vel.linear.y * X_MULT + cmd_vel.angular.z * TH_MULT;
    linact_goal = LINACT_90;
  }
  if (cmd_l * cmd_r < 0)
  {
    // wheels are told to go opposite directions, so spin in place
    cmd_l = -cmd_vel.angular.z * TH_MULT;
    cmd_r = cmd_vel.angular.z * TH_MULT;
    linact_goal = LINACT_45;
  }
  //ROS_INFO("x %f y %f th %f", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
  state_change();
}

// only place to set wheel_stopped
void wheelCallback(const packets_485net::packet_485net_dgram& ws)
{
  static int16_t vl[2];
  static int16_t vr[2];

	uint16_t itmp;
	if(ws.source != lookup_id("wheel-cnt", "front left") && ws.source != lookup_id("wheel-cnt", "front right") && ws.source != lookup_id("wheel-cnt", "back left") && ws.source != lookup_id("wheel-cnt", "back right"))
		return;
	if(!(ws.destination == 0xF0 || ws.destination == 0x00))
		return;
	if(ws.dport != 7)
		return;
	if(ws.data.size() != 22)
		return;
		
	itmp = ws.data[4] | ((ws.data[5]) << 8);
	
	if(ws.source == lookup_id("wheel-cnt", "front left"))
		vl[0] = itmp;
	else if(ws.source == lookup_id("wheel-cnt", "front right"))
		vr[0] = itmp;
	else if(ws.source == lookup_id("wheel-cnt", "back left"))
		vl[1] = itmp;
	else if(ws.source == lookup_id("wheel-cnt", "back right"))
		vr[1] = itmp;
		
  //int addr = ws.srcaddr / 2 - 1;
  //vl[addr] = ws.ticks0_interval;
  //vr[addr] = ws.ticks1_interval;  
  wheel_stopped = (0 == vl[0] && 0 == vl[1] && 0 == vr[0] && 0 == vr[1]);
  wheel_started[0] = vl[0] != 0;	//fl
  wheel_started[1] = vr[0] != 0;	//fr
  wheel_started[2] = vl[1] != 0;	//bl
  wheel_started[3] = vr[1] != 0;	//br
  state_change();
}

// only place to set linact_arrived and linact_goal_arrived
void linactCallback(const packets_485net::packet_485net_dgram& linear_actuator_status)
{
	uint16_t itmp;
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
  state_change();
}

int main(int argc, char **argv)
{
	system("./yourmom.sh");

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
  
  idlookup = n.serviceClient<net_485net_id_handler::SearchID>("search_id", true);
  idlookup.waitForExistence();

  ros::Subscriber cmd_sub = n.subscribe("cmd_vel", 1000, cmdCallback);
  ros::Subscriber wheel_sub = n.subscribe("net_485net_incoming_dgram", 1000, wheelCallback);
  ros::Subscriber linact_sub = n.subscribe("net_485net_incoming_dgram", 1000, linactCallback);

  pid_pub = n.advertise<packets_485net::packet_485net_dgram>("net_485net_outgoing_dgram", 1000);
  cmd_pub = n.advertise<packets_485net::packet_485net_dgram>("net_485net_outgoing_dgram", 1000);
  linact_pub = n.advertise<packets_485net::packet_485net_dgram>("net_485net_outgoing_dgram", 1000);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}

