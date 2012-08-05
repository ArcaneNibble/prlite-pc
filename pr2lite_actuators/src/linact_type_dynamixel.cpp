// Include linear actuator definition
#include "pr2lite_actuators/LinearActuator.hpp"
#include "std_msgs/Float64.h"
#include "dynamixel_msgs/JointState.h"

namespace pr2lite
{
    namespace actuators
    {
        namespace plugins
        {
            class LinactDynamixel : public pr2lite::actuators::LinearActuator
            {
            private:
                // Store its name
                std::string     m_name;
                
                // ROS objects
                ros::Subscriber m_command;
                ros::Publisher  m_state; 
                
                // Base and extent of the actuator
                double          m_base_length;
                double          m_extent;
                
                // Command callback
                void command_callback(const std_msgs::Float64::ConstPtr& command)
                {
                    // Since this is just the raw, provide a simple write to service
                    ROS_INFO("Setting \"%s\" to %lf radians", m_name.c_str(), command->data);
                    
                    // Calculate from the angle the length of the actuator, then the number to sent to it
                    double length = sqrt(-162.2 * cos(1.44129 - command->data) + 256.0) - m_base_length;
                    length *= (1000.0f / m_extent);

                    // Bound the value
                    if(length < 10.0f) length = 10.0f;
                    if(length >  1000.0f) length = 1000.0f;
                    
                    // Set the position
		            this->setPosition(length);
                }
            public:
                LinactDynamixel(ros::NodeHandle& nh, int id, std::string& name)
                    : LinearActuator(nh, id, 1000), m_name(name)
                {
                    // Load the linear actuator parameters
                    nh.getParam(name + "/base_length", m_base_length);
                    nh.getParam(name + "/extent", m_extent);
                
                    // Here is where to set up a subscriber
                    m_command = nh.subscribe(name + "/command", 1000, &LinactDynamixel::command_callback, this);
                    m_state = nh.advertise<dynamixel_msgs::JointState>(name + "/state", 5);
                }
            };
        }
    }
}

extern "C" 
{
    // Allocate an actuator object
    pr2lite::actuators::LinearActuator* createActuator(ros::NodeHandle& nh, std::string name)
    {
        int id;
        nh.getParam(name + "/id", id);
        ROS_INFO("Registering \"%s\"(%d) of type LinactDynamixel", name.c_str(), id);
        return new pr2lite::actuators::plugins::LinactDynamixel(nh, id, name);
    }
    
    // Return the type of actuator this is
    std::string pluginType()
    {
        return "pr2lite.actuator.linactdynamixel";
    }
}

