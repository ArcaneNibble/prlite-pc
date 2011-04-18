#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
//#include <boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
int pclCount = 0;

void callback(const PointCloud::ConstPtr& msg)
{
  pclCount++;
  //ROS_INFO("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  /*BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);*/
}

void timerCallback(const ros::TimerEvent& evt) {
    ROS_INFO("Count = %d", pclCount);
    pclCount = 0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("/camera/depth/points2", 1, callback);
  ros::Timer timer = nh.createTimer(ros::Duration(1.0), timerCallback);
  ros::spin();
}
