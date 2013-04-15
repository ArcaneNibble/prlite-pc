/**

\author Michael Ferguson

@b Offers a service to release a single depth/rgb image onto an alternate topic.

**/

#include "ros/ros.h"

#include "std_srvs/Empty.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl/io/io.h"

class CameraTurnpike
{
  public:
    CameraTurnpike(ros::NodeHandle & n):n_ (n)
    {
        ros::NodeHandle nh("~");

        rgb_sub_ = n.subscribe("/camera/rgb/image_color", 10, &CameraTurnpike::rgb_cb, this);
        depth_sub_ = n.subscribe("/camera/rgb/points", 10, &CameraTurnpike::depth_cb, this);

        rgb_pub_ = nh.advertise<sensor_msgs::Image>("image", 10);
        depth_pub_ = nh.advertise<sensor_msgs::PointCloud2>("points", 10);

        // advertise service to copy from input to output topics
        service_ = nh.advertiseService("trigger", &CameraTurnpike::service_callback, this);
    }

    /*
     * Capture latest point cloud
     */
    void depth_cb ( const sensor_msgs::PointCloud2ConstPtr& cloud )
    {
        //pcl::copyPointCloud(*cloud, depth_);
        depth_ = cloud;
    }

    /*
     * Capture latest rgb image
     */
    void rgb_cb ( const sensor_msgs::Image& image )
    {
        rgb_ = image;
    }

    /*
     * Service which copies latest message to other topics
     */
    bool service_callback ( std_srvs::Empty::Request& request, std_srvs::Empty::Response& response )
    {
        rgb_pub_.publish(rgb_);
        depth_pub_.publish(depth_);
        return true;
    }

  private:
    sensor_msgs::Image          rgb_;
    ros::Subscriber             rgb_sub_;
    ros::Publisher              rgb_pub_;
    sensor_msgs::PointCloud2ConstPtr    depth_;
    ros::Subscriber             depth_sub_;
    ros::Publisher              depth_pub_;
    ros::NodeHandle             n_;
    ros::ServiceServer          service_;
};

int main (int argc, char **argv)
{
  ros::init (argc, argv, "camera_turnpike");
  ros::NodeHandle n;
  CameraTurnpike turnpike(n);
  ros::spin ();
  return 0;
}

