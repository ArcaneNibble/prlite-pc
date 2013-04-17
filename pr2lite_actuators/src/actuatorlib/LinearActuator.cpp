// Implementation of our simple linear actuator control object
#include "pr2lite_actuators/LinearActuator.hpp"
#include <sensor_msgs/JointState.h>
#include "dynamixel_msgs/JointState.h"
#include "ros/time.h"

using namespace pr2lite::actuators;

LinearActuator::LinearActuator(ros::NodeHandle& nh, int id, int initialGoal) : m_nh(nh), m_id(id), m_goal(initialGoal)
{
    // Initalize the internal state fo the actuator object
    m_goallast = 10000;
    m_arrived = false;
    m_newgoal = false;
    m_name = "";

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
        // ROS_WARN("bad source %d expected %d", linear_actuator_status.source, m_id);
        return;
    }
    
    // Ignore if the destination is out of bounds?? ask robert what the hell these do
    if(!(linear_actuator_status.destination == 0xF0 || linear_actuator_status.destination == 0x00))
    {
        // Ignore
        ROS_WARN("bad dest %d", linear_actuator_status.destination);
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
    
    // 0,1,2,3 seq num for packet
    // 4, 5 (int) current position
    // 6 (eight bit boolean) arrived
    // Store if it arrived
    int cur_pos = linear_actuator_status.data[4];
    m_arrived = linear_actuator_status.data[4+2];
    
    // Adjust stuff according to the initial code
    if(m_newgoal)
    {
        if(!m_arrived)
            m_newgoal = false;
        else
            m_arrived = false;
    }
    
    // ROS_INFO("[ACTUATOR ID %d] publish!", m_id);
    actuator_publish();
    // if (m_name == "left_shoulder_tilt_joint" || m_name == "right_shoulder_tilt_joint")
    if (m_id == 15 || m_id == 14) {
        publish_dyna_state(cur_pos);
    // else if (m_name == "torso_lift_joint")
    } else if (m_id == 13) {
        publish_torso_state(cur_pos);
    // else if (m_name == "wheel_linear_actuator_joint")
    } else if (m_id == 12) {
        publish_base_state(cur_pos);
    } else
        ROS_INFO("no actuator registered");
}

void LinearActuator::publish_dyna_state(int cur_pos)
{
    // Since this is just the raw, provide a simple write to service
    
    // ROS_INFO("dynamixel ");
    double length = (1000.0f - (double)cur_pos) * 4.0 / 1000.0f;
    // Calculate from the angle the length of the actuator
    // double length = sqrt(-162.2 * cos(1.44129 - command->data) + 256.0) - m_base_length;
    double angle = 1.44129 - acos((pow((length + 9.9), 2.0) - 256.0)/(-162.2));
#define BAR_LEN (4.3 - .315)   /* Upper arm bar that LinAct connects to */
#define HYPOTENUSE (14.968)    /* Len from bottom LinAct hole to Shoulder Joint
    // double angle = (1.44129 - pow(acos((length + 9.9), 2.0) - 256.0)/(-162.2));

    // ROS_INFO("current length of \"%s\" dynamixel: %lf radians %d cur_pos", m_name.c_str(), angle, cur_pos);

    // Bound the value
    if(angle >  3.14159f) angle = 3.14159f;
    if(angle <  -3.14159f) angle = -3.14159f;


/* constants in inches */
#define LINACT_DWN (9.9)       /* length of LinAct when retracted */
#define LINACT_UP (13.9)       /* length of LinAct when extended */
*/
#define LINACT_TO_TOP (14.715) /* len from bottom LinAct hole to top lazy susan
*/
#define BRACKET_TO_BAR (1.26)  /* From LinAct hole to upper arm bar */
/* additional angle caused by LinAct bracket */
#define ADD_BRACKET_ANGLE (atan(BRACKET_TO_BAR / BAR_LEN))
/* extra angle caused by LinAct bottom and Shoulder hinge not lining up */
#define FIXED_LA_ANGLE  (acos(LINACT_TO_TOP / HYPOTENUSE))
#define FIXED_LA_ANGLE2  (asin(LINACT_TO_TOP / HYPOTENUSE))
// tan(θ) = Opposite / Adjacent
#define FIXED_LA_ANGLE3  (atan(BAR_LEN / HYPOTENUSE))
/* for shorthand */
#define B BAR_LEN
#define H HYPOTENUSE
#define L (length + LINACT_DWN)
#define ONE_PI (3.14159265359)
#define TWO_PI (2.0*3.14159265359)
#define INCHES_TO_METERS (0.0254)

#define TMP_ANGLE (acos( (B*B - L*L + H*H) / H))
/* shoulder angle in radians based on linact_len */
#define SHLDR_ANGLE (TMP_ANGLE + ADD_BRACKET_ANGLE - FIXED_LA_ANGLE)

/* linact_len based on shoulder angle */
#define desired_linact_len(shdrangle) sqrt(B*B + H*H - H*cos(shdrangle - ADD_BRACKET_ANGLE + FIXED_LA_ANGLE))

#define TMP2_ANGLE (asin( (B*B - L*L + H*H) / H))
/* linact_angle based on shoulder angle */
#define LINACT_ANGLE (TMP2_ANGLE + FIXED_LA_ANGLE2)

#define LINACT_VEL (.5)  /* inches per second */
#define FAST_LINACT_VEL (2)  /* inches per second */
/* Let u = arccos(x) then du = -1/(1-x²) dx in radians per sec */
#define SHLDR_ANGLE_VEL (-1/(1-pow(((B*B-L*L+H*H)/H),2))*(-LINACT_VEL*LINACT_VEL/H))

    // double linact_length = length * INCHES_TO_METERS;
    // double linact_cyl_angle = LINACT_ANGLE;
    // double upper_arm_angle = TWO_PI - linact_cyl_angle - angle;
    // sin rule : a / sin(A) = b/ sin(B) = c/ sin(C)
    // cos rule : c^2 = a^2 + b^2 - 2ab cos(C)
    // sin rule : A = asin((sin(B) * a) / b)
    // double linact_cyl_angle = asin( (BAR_LEN*INCHES_TO_METERS) * sin(ONE_PI - angle) / (linact_length + (LINACT_DWN*INCHES_TO_METERS)));
    // double linact_cyl_angle = LINACT_ANGLE;
    // double upper_arm_angle = ONE_PI - linact_cyl_angle - angle;
    // double upper_arm_angle = angle - ONE_PI;
    // tan(θ) = Opposite / Adjacent
    // angle -= ADD_BRACKET_ANGLE;
    double linact_length = length * INCHES_TO_METERS;
    double linact_cyl_angle = asin( (BAR_LEN*INCHES_TO_METERS) * sin(ONE_PI - angle) / (linact_length + (LINACT_DWN*INCHES_TO_METERS))) + FIXED_LA_ANGLE3;
    double upper_arm_angle = -1 * angle;
    linact_length -= (.5 * INCHES_TO_METERS); // avoid false collision

    // return the angle
    /*
	Header header
	    uint32 seq
	    time stamp
	    string frame_id
	string[] name
	float64[] position
	float64[] velocity
	float64[] effort

    TODO: we can set the positions of the other passive joints in the linact joint
          by pushing back more names/positions
    */
    // dynamixel_msgs::JointState dynastate;
    // dynastate.position.resize(1);
    // dynastate.name[0] = strcat(m_name, "/state");
    // std::string tmpstr = m_name + "/state";
    // dynastate.position.resize(1);
    // dynastate.position[0] = angle;
    // dynastate.set_motor_ids_size((uint32_t)1);
    // int motor_id = m_id + 10;
    // dynastate.set_motor_ids_vec(&motor_id);
    // dynastate.velocity.push_back(0);
    // dynastate.effort.push_back(0);

    sensor_msgs::JointState dynastate;
    dynastate.header.stamp = ros::Time::now();
    dynastate.name.resize(4);
    dynastate.name[0] = m_name;
    dynastate.position.push_back(angle);   // shoulder_tilt_joint
    // if (m_name == "left_shoulder_tilt_joint")
    if (m_id == 15)
    {
        dynastate.name[1] = "left_linear_actuator_joint";
        dynastate.name[2] = "left_lin_act_cyl_joint";
        dynastate.name[3] = "left_upper_arm_hinge_joint";
        // ROS_INFO("left_shoulder_tilt_joint ");
    } else
    {
        dynastate.name[1] = "right_linear_actuator_joint";
        dynastate.name[2] = "right_lin_act_cyl_joint";
        dynastate.name[3] = "right_upper_arm_hinge_joint";
        // ROS_INFO("right_shoulder_tilt_joint ");
    }

    dynastate.position.push_back(linact_length);  // linear_actuator_joint
    dynastate.position.push_back(linact_cyl_angle);  // lin_act_cyl_joint
    dynastate.position.push_back(upper_arm_angle);  // upper_arm_hinge_joint
    if (m_arrived) {
      dynastate.velocity.push_back(0);
      dynastate.velocity.push_back(0);
      dynastate.velocity.push_back(0);
      dynastate.velocity.push_back(0);
    } else {
      dynastate.velocity.push_back(LINACT_VEL*INCHES_TO_METERS);
      dynastate.velocity.push_back(LINACT_VEL*INCHES_TO_METERS);
      dynastate.velocity.push_back(LINACT_VEL*INCHES_TO_METERS);
      dynastate.velocity.push_back(LINACT_VEL*INCHES_TO_METERS);
    }

    m_state.publish(dynastate);
    // ROS_INFO("Published %s : %f %f %f %f", m_name.c_str(), angle, linact_length, linact_cyl_angle, upper_arm_angle);
}

void LinearActuator::publish_torso_state(int cur_pos)
{
    sensor_msgs::JointState torsostate;
    // ROS_INFO("torso ");
    torsostate.header.stamp = ros::Time::now();
    torsostate.name.resize(1);
    torsostate.name[0] = "torso_lift_joint";
    double length = (1000.0f - (double)cur_pos) * 12.0 / 1000.0f * INCHES_TO_METERS;
    torsostate.position.push_back(length);
    if (m_arrived) {
      torsostate.velocity.push_back(0);
    } else {
      torsostate.velocity.push_back(FAST_LINACT_VEL*INCHES_TO_METERS);
    }

    m_state.publish(torsostate);
    ROS_INFO("Published torso: %d %f", cur_pos, length);
}

void LinearActuator::publish_base_state(int cur_pos)
{

#define PUSH_ROD_LEN 0.1778
// #define CASTER_LEN (0.1016 / 2.0)
#define CASTER_LEN (0.089 / 2.0)

/*
 ---------
 |linact_side_angle
 |pushrod
 | 
 ---
  casterlen
*/
  
    sensor_msgs::JointState basestate;
    basestate.header.stamp = ros::Time::now();

    double length = (1000.0f - (double)cur_pos) * 4.0 / 1000.0f * INCHES_TO_METERS;
    // length = 0 * INCHES_TO_METERS; // for debugging
    // basestate.position.push_back(length);
    // sin rule : a / sin(A) = b/ sin(B) = c/ sin(C)
    // cos rule : c^2 = a^2 + b^2 - 2ab cos(C)
    // a = pushrod ; b = casterlen ; c = side ; C = casterangle
    // cos rule : cos(C) = (a^2 + b^2 + c^2) / 2ab
    double retracted_linact_side_angle = ONE_PI;  // straight from linact rod to caster
    double retracted_caster_angle = ONE_PI / 2.0;

    // a = pushrod ; b = casterlen ; c = side ; C = casterangle
    // cos rule : c = sqrt(a^2 + b^2 - 2ab cos(C))
    double retracted_side = sqrt( pow(PUSH_ROD_LEN,2.0) + pow(CASTER_LEN, 2.0) 
                            + 2 * PUSH_ROD_LEN * CASTER_LEN * cos(retracted_caster_angle));
    // double retracted_pushrod_angle = asin((sin(retracted_caster_angle) * retracted_side) / PUSH_ROD_LEN);
    double retracted_pushrod_angle = 0;

    // a = side ; b = pushrod_len ; B = caster_angle
    // sin rule : A = asin((sin(B) * a) / b)
    //--------------------------------------------------

    double mid_caster_angle = ONE_PI / 4.0;
    double mid_side = sqrt( pow(PUSH_ROD_LEN,2.0) + pow(CASTER_LEN, 2.0) 
                            + 2 * PUSH_ROD_LEN * CASTER_LEN * cos(mid_caster_angle));
    double mid_pushrod_angle = asin((sin(mid_caster_angle) * mid_side) / PUSH_ROD_LEN);
    double mid_linact_side_angle = TWO_PI - mid_pushrod_angle - mid_caster_angle + (ONE_PI / 2.0);

    //--------------------------------------------------
    double extended_linact_side_angle = 2*ONE_PI;  // straight
    double extended_caster_angle = 0; 
    double extended_side = sqrt( pow(PUSH_ROD_LEN,2.0) + pow(CASTER_LEN, 2.0) 
                            + 2 * PUSH_ROD_LEN * CASTER_LEN * cos(extended_caster_angle));
    double extended_pushrod_angle = asin((sin(extended_caster_angle) * extended_side) / PUSH_ROD_LEN);
    // extended_linact_side_angle = ONE_PI / 4.0;
    // extended_pushrod_angle = ONE_PI / 4.0;

    basestate.name.resize(13);
    basestate.name[0] = "wheel_linear_actuator_joint";
    basestate.name[1] = "fl_anchor_rod_joint";
    basestate.name[2] = "fl_push_rod_joint";
    basestate.name[3] = "fl_caster_rotation_joint";
    basestate.name[4] = "fr_anchor_rod_joint";
    basestate.name[5] = "fr_push_rod_joint";
    basestate.name[6] = "fr_caster_rotation_joint";
    basestate.name[7] = "bl_anchor_rod_joint";
    basestate.name[8] = "bl_push_rod_joint";
    basestate.name[9] = "bl_caster_rotation_joint";
    basestate.name[10] = "br_anchor_rod_joint";
    basestate.name[11] = "br_push_rod_joint";
    basestate.name[12] = "br_caster_rotation_joint";
    basestate.position.push_back(length);
    ROS_INFO("Published base: %d %f %f %f %f %f %f %f %f %f %f", 
      cur_pos, length, retracted_linact_side_angle, retracted_pushrod_angle, 
      retracted_caster_angle,
      mid_linact_side_angle, mid_pushrod_angle, mid_caster_angle,
      extended_linact_side_angle, extended_pushrod_angle, extended_caster_angle
      );
    if (length <= 1 * INCHES_TO_METERS)
    {
      double inverse = 1;
      basestate.position.push_back(retracted_linact_side_angle*inverse); // fl
      basestate.position.push_back(retracted_pushrod_angle*inverse);
      inverse = -1;
      basestate.position.push_back(retracted_caster_angle*inverse);
      inverse = 1;
      basestate.position.push_back(retracted_linact_side_angle*inverse); // fr
      basestate.position.push_back(retracted_pushrod_angle*inverse);
      basestate.position.push_back(retracted_caster_angle*inverse);
      inverse = 1;
      basestate.position.push_back(retracted_linact_side_angle*inverse); // bl
      basestate.position.push_back(retracted_pushrod_angle*inverse);
      basestate.position.push_back(retracted_caster_angle*inverse);
      basestate.position.push_back(retracted_linact_side_angle*inverse); // br
      basestate.position.push_back(retracted_pushrod_angle*inverse);
      inverse = -1;
      basestate.position.push_back(retracted_caster_angle*inverse);
    } else if (length <= 3 * INCHES_TO_METERS)
    {
      double inverse = 1;
      basestate.position.push_back(mid_linact_side_angle*inverse); // fl
      basestate.position.push_back(mid_pushrod_angle*inverse);
      inverse = -1;
      basestate.position.push_back(mid_caster_angle*inverse);
      inverse = -1;
      basestate.position.push_back(mid_linact_side_angle*inverse); // fr
      basestate.position.push_back(mid_pushrod_angle*inverse);
      inverse = 1;
      basestate.position.push_back(mid_caster_angle*inverse);
      basestate.position.push_back(mid_linact_side_angle*inverse); // bl
      basestate.position.push_back(mid_pushrod_angle*inverse);
      basestate.position.push_back(mid_caster_angle*inverse);
      inverse = -1;
      basestate.position.push_back(mid_linact_side_angle*inverse); // br
      basestate.position.push_back(mid_pushrod_angle*inverse);
      basestate.position.push_back(mid_caster_angle*inverse);
    } else
    {
      double inverse = 1;
      basestate.position.push_back(extended_linact_side_angle*inverse); // fl
      basestate.position.push_back(extended_pushrod_angle*inverse);
      basestate.position.push_back(extended_caster_angle*inverse);
      basestate.position.push_back(extended_linact_side_angle*inverse); // fr
      basestate.position.push_back(extended_pushrod_angle*inverse);
      basestate.position.push_back(extended_caster_angle*inverse);
      basestate.position.push_back(extended_linact_side_angle*inverse); // bl
      basestate.position.push_back(extended_pushrod_angle*inverse);
      basestate.position.push_back(extended_caster_angle*inverse);
      basestate.position.push_back(extended_linact_side_angle*inverse); // br
      basestate.position.push_back(extended_pushrod_angle*inverse);
      basestate.position.push_back(extended_caster_angle*inverse);
    }
    if (m_arrived) {
      basestate.velocity.push_back(0);
      basestate.velocity.push_back(0);
      basestate.velocity.push_back(0);
      basestate.velocity.push_back(0);
      basestate.velocity.push_back(0);
      basestate.velocity.push_back(0);
      basestate.velocity.push_back(0);
      basestate.velocity.push_back(0);
      basestate.velocity.push_back(0);
      basestate.velocity.push_back(0);
      basestate.velocity.push_back(0);
      basestate.velocity.push_back(0);
      basestate.velocity.push_back(0);
    } else
    {
      basestate.velocity.push_back(FAST_LINACT_VEL*INCHES_TO_METERS);
      basestate.velocity.push_back(FAST_LINACT_VEL*INCHES_TO_METERS);
      basestate.velocity.push_back(FAST_LINACT_VEL*INCHES_TO_METERS);
      basestate.velocity.push_back(FAST_LINACT_VEL*INCHES_TO_METERS);
      basestate.velocity.push_back(FAST_LINACT_VEL*INCHES_TO_METERS);
      basestate.velocity.push_back(FAST_LINACT_VEL*INCHES_TO_METERS);
      basestate.velocity.push_back(FAST_LINACT_VEL*INCHES_TO_METERS);
      basestate.velocity.push_back(FAST_LINACT_VEL*INCHES_TO_METERS);
      basestate.velocity.push_back(FAST_LINACT_VEL*INCHES_TO_METERS);
      basestate.velocity.push_back(FAST_LINACT_VEL*INCHES_TO_METERS);
      basestate.velocity.push_back(FAST_LINACT_VEL*INCHES_TO_METERS);
      basestate.velocity.push_back(FAST_LINACT_VEL*INCHES_TO_METERS);
      basestate.velocity.push_back(FAST_LINACT_VEL*INCHES_TO_METERS);
    }
    m_state.publish(basestate);
    // ROS_INFO("Published torso %f", length);
}
/*
"fl_wheel_joint", 
"fr_wheel_joint", 
"bl_wheel_joint", 
"br_wheel_joint", 
*/

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
        // ROS_INFO("[ACTUATOR ID %d] herp derp", m_id);
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
        // ROS_INFO("[ACTUATOR ID %d] At %d", m_id, m_goal);
    }
}

void LinearActuator::setPosition(int position)
{
    m_goal = position;
    actuator_publish();
}

void LinearActuator::setName(std::string name)
{
    m_name = name;
    // m_state = m_nh.advertise<dynamixel_msgs::JointState>(name + "/state", 5);
    // m_state = m_nh.advertise<sensor_msgs::JointState>(name + "/state", 5);
    m_state = m_nh.advertise<sensor_msgs::JointState>("joint_states", 5);
    // m_state = state;
    ROS_INFO_STREAM("setName " <<  name);
}
