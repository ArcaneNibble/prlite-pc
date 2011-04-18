
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

// usually call cmdPublish instead of cmdPublishWheel

void cmdPublish(void)
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

void torsoCallback(const i2c_net_packets::linact_position& linear_actuator_status)
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
  cmdPublish();
}

void torso_init(void)
{
  ros::NodeHandle n;

  ros::Subscriber linact_sub = n.subscribe("linear_actuator_status", 1000, linactCallback);
  linact_pub = n.advertise<i2c_net_packets::linact_target>("linear_actuator_target", 1000);

}

