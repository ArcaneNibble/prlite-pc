/*
* prlite_odom
* Copyright (c) 2011, Andrew Downing (modified from code by Willow Garage, Inc.)
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

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include "packets_485net/packet_485net_dgram.h"

const double WHEEL_TICKS_PER_METER = 75.1913116; // from magellan-v2 code: 36 clicks/circumference / pi diameter/circumference / 6 inches/diameter * 39.3700787 inches/meter
const double TURN_DIAMETER_METERS = 0.2; // found by tuning (is approximate)

ros::Publisher odom_pub;
ros::Time current_time, last_time;

// wheel velocities
int16_t vl[2];
int16_t vr[2];

// calculated robot velocities
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

// estimated position
double x = 0.0;
double y = 0.0;
double th = 0.0;

void odometryUpdate(void)
{
  static tf::TransformBroadcaster odom_broadcaster;
  boost::array<double, 36> odom_pose_covariance = {
    {1e-3, 0, 0, 0, 0, 0, 
     0, 1e-3, 0, 0, 0, 0, 
     0, 0, 1e6, 0, 0, 0, 
     0, 0, 0, 1e6, 0, 0, 
     0, 0, 0, 0, 1e6, 0, 
     0, 0, 0, 0, 0, 1e3}};

  boost::array<double, 36> odom_pose_covariance2 = {
     {1e-9, 0, 0, 0, 0, 0, 
     0, 1e-3, 1e-9, 0, 0, 0, 
     0, 0, 1e6, 0, 0, 0,
     0, 0, 0, 1e6, 0, 0, 
     0, 0, 0, 0, 1e6, 0, 
     0, 0, 0, 0, 0, 1e-9}};
    
  boost::array<double, 36> odom_twist_covariance = {
    {1e-3, 0, 0, 0, 0, 0, 
     0, 1e-3, 0, 0, 0, 0, 
     0, 0, 1e6, 0, 0, 0, 
     0, 0, 0, 1e6, 0, 0, 
     0, 0, 0, 0, 1e6, 0, 
     0, 0, 0, 0, 0, 1e3}};

  boost::array<double, 36> odom_twist_covariance2 = {
    {1e-9, 0, 0, 0, 0, 0, 
     0, 1e-3, 1e-9, 0, 0, 0, 
     0, 0, 1e6, 0, 0, 0, 
     0, 0, 0, 1e6, 0, 0, 
     0, 0, 0, 0, 1e6, 0, 
     0, 0, 0, 0, 0, 1e-9}};

  current_time = ros::Time::now();

  //compute odometry in a typical way given the velocities of the robot
  double dt = (current_time - last_time).toSec();

  x += (vx * cos(th) - vy * sin(th)) * dt;
  y += (vx * sin(th) + vy * cos(th)) * dt;
  th += vth * dt;
  if (th > M_PI) th -= M_PI * 2;
  if (th < -M_PI) th += M_PI * 2;

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "wheelodom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  odom_broadcaster.sendTransform(odom_trans);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "wheelodom";

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  odom.pose.covariance = odom_pose_covariance;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;
  odom.twist.covariance = odom_twist_covariance;

  //publish the message
  odom_pub.publish(odom);

  last_time = current_time;

  ROS_INFO("x %f y %f theta %f vx %f vy %f vtheta %f", x, y, th, vx, vy, vth);
}

void wheelStatusCallback(const packets_485net::packet_485net_dgram& ws)
{
	uint16_t itmp;
	if(ws.source != 0x08 && ws.source != 0x09 && ws.source != 0x0A && ws.source != 0x0B)
		return;
	if(!(ws.destination == 0xF0 || ws.destination == 0x00))
		return;
	if(ws.dport != 7)
		return;
		
  // update wheel velocity
  //int addr = ws.srcaddr / 2 - 1;
  //vl[addr] = ws.ticks0_interval * 10; // 10 intervals per second
  //vr[addr] = ws.ticks1_interval * 10;
	itmp = ws.data[4] | ((ws.data[5]) << 8);
	
	switch(ws.source)
	{
	case 0x08:
		vr[1] = itmp;
		break;
	case 0x09:
		vr[0] = itmp;
		break;
	case 0x0A:
		vl[1] = itmp;
		break;
	case 0x0B:
		vl[0] = itmp;
		break;
	}
  // update forward and rotation velocities (algorithm based on magellan-v2 odometryupdate)
  double lavg = ((double)(vl[0] + vl[1])) * 0.5 / WHEEL_TICKS_PER_METER;
  double ravg = ((double)(vr[0] + vr[1])) * 0.5 / WHEEL_TICKS_PER_METER;
  vx = (lavg + ravg) * 0.5;
  vth = (ravg - lavg) / TURN_DIAMETER_METERS;
  // update position and publish odometry
  odometryUpdate();
}

void cmdVelCallback(const geometry_msgs::Twist& cmd_vel)
{
  // fake odometry by directly using the velocity commands
  vx = cmd_vel.linear.x;
  vy = cmd_vel.linear.y;
  vth = cmd_vel.angular.z;
}

int main(int argc, char** argv)
{
  bool fake_localization = false; // to set to true on command line use "rosrun prlite_odom odom _fake_localization:=true"

  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::param::get("~fake_localization", fake_localization);
  odom_pub = n.advertise<nav_msgs::Odometry>("wheelodom", 50);
  ros::Subscriber odom_sub;
  if (fake_localization)
    odom_sub = n.subscribe("cmd_vel", 50, cmdVelCallback);
  else
    odom_sub = n.subscribe("net_485net_outgoing_dgram", 50, wheelStatusCallback);

  vl[0] = 0;
  vl[1] = 0;
  vr[0] = 0;
  vr[1] = 0;

  current_time = ros::Time::now();
  last_time = ros::Time::now();

  if (fake_localization)
  {
    ros::Rate r(5);

    while(n.ok()){
      odometryUpdate();

      ros::spinOnce();
      r.sleep();
    }
  }
  else
  {
    ros::spin();
  }
  ros::param::del("~fake_localization");
}

