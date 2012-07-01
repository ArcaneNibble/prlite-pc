#include "ros/ros.h"
#include "packets_485net/packet_485net_dgram.h"

#define LINACT_PRECISION 15

// PR2 Lite namespace
namespace pr2lite
{
    // Actuators namespace
    namespace actuators
    {
        // Class to encapsulate the control of a single linear actuator
        class LinearActuator
        {
        protected: 
            // ROS objects
            ros::NodeHandle m_nh;
            
        public:
            // Construct the linear actuator object
            LinearActuator(ros::NodeHandle& nh, int id, int initialGoal = 1000);
            
            // Deconstruct (i.e. release the actuator from our control)
            ~LinearActuator();
            
            // Set the position
            void setPosition(int position);
            
        private:
            // ROS objects
            ros::Publisher  m_publisher;
            ros::Subscriber m_subscriber;
        
            // Internal data storage
            int  m_id;             // stores the 485net node id of the actuator
            int  m_goal;           // the target position for the actuator
            bool m_goallast;
            bool m_arrived;        
            bool m_newgoal;
            
            // ROS responses
            void actuator_callback(const packets_485net::packet_485net_dgram& linear_actuator_status);
            void actuator_publish();
        };
    }
}
