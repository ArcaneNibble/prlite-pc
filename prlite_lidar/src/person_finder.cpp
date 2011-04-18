/*
find local minimum and maximum distances
2 highest local maximums are corners of room
lowest local minimum that is not end is person
	later try to make sure has large enough jump close to this local min
*/

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_broadcaster.h"

ros::Publisher pub;

void scanCallback(const sensor_msgs::LaserScan& scan)
{
  static tf::TransformBroadcaster broadcaster;
  int minrangeid = -1;
  float minrange = 0;
  // find person (currently closest object > 0.5 meters)
  for (int i = 0; i < (scan.angle_max - scan.angle_min) / scan.angle_increment; i++)
  {
    if (scan.ranges[i] > 0.5 && (scan.ranges[i] < minrange || minrangeid < 0))
    {
      minrangeid = i;
      minrange = scan.ranges[i];
    }
  }
  if (minrangeid >= 0)
  {
    // seeing person, broadcast position
    geometry_msgs::TransformStamped trans;
    trans.header.stamp = ros::Time::now();
    trans.header.frame_id = "base_laser";
    trans.child_frame_id = "person";
    trans.transform.translation.x = minrange * cos(scan.angle_min + (float)minrangeid * scan.angle_increment);
    trans.transform.translation.y = minrange * sin(scan.angle_min + (float)minrangeid * scan.angle_increment);
    trans.transform.translation.z = 0.0;
    trans.transform.rotation.x = 0.0;
    trans.transform.rotation.y = 0.0;
    trans.transform.rotation.z = 0.0;
    trans.transform.rotation.w = 1.0;
    broadcaster.sendTransform(trans);
    pub.publish(true);
    ROS_INFO("angle %d dist %f x %f y %f", minrangeid, minrange, trans.transform.translation.x, trans.transform.translation.y);
  }
  else
  {
    // not seeing person
    pub.publish(false);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "prlite_person_finder");
  ros::NodeHandle n;
  pub = n.advertise<std_msgs::Bool>("prlite_seeing_person", 1000);
  ros::Subscriber sub = n.subscribe("scan", 1000, scanCallback);
  ros::spin();
  return 0;
}
