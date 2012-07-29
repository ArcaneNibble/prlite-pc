// Include linear actuator definition
#include "pr2lite_actuators/LinearActuator.hpp"
#include "std_msgs/Int32.h"

namespace pr2lite
{
    namespace actuators
    {
        namespace plugins
        {
            class LinactRaw : public pr2lite::actuators::LinearActuator
            {
            private:
                // Store its name
                std::string m_name;
                
                // ROS objects
                ros::Subscriber m_position_source;
                
                // Position callback
                void position_callback(const std_msgs::Int32::ConstPtr& position)
                {
                    // Since this is just the raw, provide a simple write to service
                    ROS_INFO("Setting \"%s\" to %d", m_name.c_str(), position->data);
		    this->setPosition(position->data);
                }
            public:
                LinactRaw(ros::NodeHandle& nh, int id, std::string& name)
                    : LinearActuator(nh, id, 1000), m_name(name)
                {
                    // Here is where to set up a subscriber
                    m_position_source = nh.subscribe(name + "/position", 1000, &LinactRaw::position_callback, this);
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
        //ROS_INFO("DERP");
        int id;
        nh.getParam(name + "/id", id);
        ROS_INFO("Registering \"%s\"(%d) of type LinactRaw", name.c_str(), id);
        return new pr2lite::actuators::plugins::LinactRaw(nh, id, name);
    }
    
    // Return the type of actuator this is
    std::string pluginType()
    {
        return "pr2lite.actuator.linactraw";
    }
}

