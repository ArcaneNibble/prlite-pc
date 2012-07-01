// Implementation of our simple linear actuator control object
#include "pr2lite_actuators/LinearActuator.hpp"

using namespace pr2lite::actuators;

LinearActuator::LinearActuator(ros::NodeHandle& nh, int id, int initialGoal) : m_nh(nh), m_id(id), m_goal(initialGoal)
{
    // Initalize the internal state fo the actuator object
    m_goallast = 10000;
    m_arrived = false;
    m_newgoal = false;

    // Subscribe to the callback and advertise the published commands
    m_subscriber = m_nh.subscribe("net_485net_incoming_dgram", 1000, &LinearActuator::actuator_callback, this);
    m_publisher = m_nh.advertise<packets_485net::packet_485net_dgram>("net_485net_outgoing_dgram", 1000);
}

LinearActuator::~LinearActuator()
{
    
}

void LinearActuator::actuator_callback(const packets_485net::packet_485net_dgram& linear_actuator_status) 
{
    // Ignore message if this packet is not targeted at the actuator we are controlling
    if(linear_actuator_status.source != m_id)
    {
        // Ignore
        return;
    }
    
    // Ignore if the destination is out of bounds?? ask robert what the hell these do
    if(!(linear_actuator_status.destination == 0xF0 || linear_actuator_status.destination == 0x00))
    {
        // Ignore
        return;
    }
    
    // Check for an invalid dport
    if(linear_actuator_status.dport != 7)
    {
        ROS_WARN("[ACTUATOR ID %d] invalid dport", m_id);
        return;
    }

    // Check for an invalid size
    if(linear_actuator_status.data.size() != 7) 
    {
        ROS_WARN("[ACTUATOR ID %d] invalid size", m_id);
        return;
    }
    
    // Store if it arrived
    m_arrived = linear_actuator_status.data[4+2];
    
    // Adjust stuff according to the initial code
    if(m_newgoal)
    {
        if(!m_arrived)
            m_newgoal = false;
        else
            m_arrived = false;
    }
    
    //ROS_INFO("[ACTUATOR ID %d] publish!", m_id);
    actuator_publish();
}

void LinearActuator::actuator_publish()
{
    if(m_arrived && m_goal == m_goallast)
    {
        ROS_WARN("[ACTUATOR ID %d] Same goal at %d", m_id, m_goal);
        return;
    }
    
    if(m_goal != m_goallast)
    {
        // publish linear actuator command
        //ROS_INFO("[ACTUATOR ID %d] herp derp", m_id);
        packets_485net::packet_485net_dgram cmd;
        cmd.destination = m_id;
        cmd.source = 0xF0;
        cmd.sport = 7;
        cmd.dport = 1;

        uint16_t itmp = m_goal > LINACT_PRECISION ? m_goal - LINACT_PRECISION : 0;
        cmd.data.push_back(itmp & 0xFF);
        cmd.data.push_back((itmp >> 8) & 0xFF);
        itmp = m_goal < (1023 - LINACT_PRECISION) ? m_goal + LINACT_PRECISION : 1023;
        cmd.data.push_back(itmp & 0xFF);
        cmd.data.push_back((itmp >> 8) & 0xFF);

        m_publisher.publish(cmd);
        m_goallast = m_goal;
        m_newgoal = true;
        m_arrived = false;
        ROS_INFO("[ACTUATOR ID %d] At %d", m_id, m_goal);
    }
}

void LinearActuator::setPosition(int position)
{
    m_goal = position;
    actuator_publish();
}
