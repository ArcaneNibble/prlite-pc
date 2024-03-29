#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"

#include <sstream>
#include <cstdlib>
#include <cmath>

#include "std_msgs/String.h"
#include "packets_485net/packet_485net_dgram.h"

#define BASE_LENGTH 9.9f
#define SWING 4.0f

const uint16_t TORSO_UP = 1000;
const uint16_t TORSO_MID= 500;
const uint16_t TORSO_DOWN = 0;
const uint16_t ARM_DOWN = 1000;
const uint16_t ARM_UP = 0;
const uint16_t LINACT_PRECISION = 15;

#define AX12_FEEDBACK 0

ros::Publisher linact_pub;
ros::Subscriber linact_sub;

#define LINACT_TORSO        0
#define LINACT_RIGHT_ARM    1
#define LINACT_LEFT_ARM     2
// #define LINACT_WHEELS       3
#define NUM_LINACT          3
#define LINACT_TORSO_ID     0x0C 
#define LINACT_RIGHT_ARM_ID 0x0E 
#define LINACT_LEFT_ARM_ID  0x0F 
#define LINACT_WHEELS_ID    0xAA 

int  linact_id[NUM_LINACT] = {LINACT_TORSO_ID, LINACT_RIGHT_ARM_ID, LINACT_LEFT_ARM_ID};
bool linact_arrived[NUM_LINACT];
bool linact_new_goal[NUM_LINACT];
bool linact_goal_last[NUM_LINACT];
int  linact_goal[NUM_LINACT] = {TORSO_DOWN, ARM_DOWN, ARM_DOWN};

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
        ROS_INFO("[ACTUATOR ID %d] herp derp", this_linact);
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
    
    ROS_INFO("[ACTUATOR ID %d] publish!", this_linact); 
    LinactPublish(this_linact);
}


/*void LinactInit(void)
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
}*/

int main (int argc, char** argv)
{
    ros::init(argc, argv, "linact_test");
    ros::NodeHandle n;
    int i;

    linact_sub = n.subscribe("net_485net_incoming_dgram", 1000, LinactCallback);
    linact_pub = n.advertise<packets_485net::packet_485net_dgram>("net_485net_outgoing_dgram", 1000);
    for (i = 0 ; i < NUM_LINACT; i++) {
        linact_goal_last[i] = 10000;
        linact_arrived[i] = false; 
        linact_new_goal[i] = false; 
    }

    /*while(ros::ok())
    {
        for(float f = 25.0; f < 65.0; f += 5.0)
        {
            double length = sqrt(-162.2 * cos((87.58 - f) * M_PI / 180.0) + 256.0) - BASE_LENGTH;
            length *= (1000.0f / SWING);

            if(length < 10.0f) length = 10.0f;
            if(length >  1000.0f) length = 1000.0f;

            linact_goal[LINACT_LEFT_ARM] = (int) length;
            linact_goal[LINACT_RIGHT_ARM] = (int) length;
            LinactPublish(LINACT_LEFT_ARM);
            LinactPublish(LINACT_RIGHT_ARM);

            ros::spinOnce();
            ros::Duration(2.0).sleep();
       }

        for(float f = 65.0; f > 25.0; f -= 5.0)
        {
            double length = sqrt(-162.2 * cos((87.58 - f) * M_PI / 180.0) + 256.0) - BASE_LENGTH;
            length *= (1000.0f / SWING);

            if(length < 10.0f) length = 10.0f;
            if(length >  1000.0f) length = 1000.0f;

            linact_goal[LINACT_LEFT_ARM] = (int) length;
            linact_goal[LINACT_RIGHT_ARM] = (int) length;
            LinactPublish(LINACT_LEFT_ARM);    
            LinactPublish(LINACT_RIGHT_ARM);

            ros::spinOnce();
            ros::Duration(2.0).sleep();
        }
    }*/

    double length = sqrt(-162.2 * cos((87.58 - atof(argv[1])) * M_PI / 180.0) + 256.0) - BASE_LENGTH;
    length *= (1000.0f / SWING);

    if(length < 10.0f) length = 10.0f;
    if(length >  1000.0f) length = 1000.0f;

    linact_goal[LINACT_LEFT_ARM] = (int) length;
    linact_goal[LINACT_RIGHT_ARM] = (int) length;
    LinactPublish(LINACT_LEFT_ARM);    
    LinactPublish(LINACT_RIGHT_ARM);

    ros::spin();

    return 0;
}

