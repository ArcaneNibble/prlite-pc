#include "dynamixel_msgs/MotorStateList.h"
#include "dynamixel_msgs/JointState.h"
//#include "dynamixel_msgs/JointStateList.h"


/* enumerated types */
enum left_right {left, right};
enum pan_tilt {pan, tilt};


#define HEAD_TILT_MIDDLE  128
#define HEAD_TILT_UP  128
#define HEAD_TILT_DOWN  128
#define HEAD_PAN_MIDDLE  128
#define HEAD_PAN_LEFT  128
#define HEAD_PAN_RIGHT  128
#define MAXLOAD  128
#define MAXVOLT  128
#define MAXTEMP  128
  
/*
typedef struct motor_states {
    double  timestamp;
    int id;
    int goal;
    int position;
    int error;
    int speed;
    double  load;
    double  voltage;
    int temperature;
    bool moving;
    } motor_state_t;

typedef struct jointstate_s {
    struct Header
    {
        unsigned int seq;
        double stamp;
        std::string frame_id;
    } header;
    std::string name;
    int motor_ids[];
    int motor_temps[];
    double goal_pos;
    double current_pos;
    double error;
    double velocity;
    double load;
    bool is_moving;
} JointState;
*/


class prlite_ax12commander {
public:
enum prlite_ax12jointtype {
   shoulderpan, shouldertilt, elbowtilt, wristrot, lfinger, rfinger,
   shoulderpanR, shouldertiltR, elbowtiltR, wristrotR, lfingerR, rfingerR, 
   kinectpan, kinecttilt, lasertilt, numjoints
   };

enum mode {UNCHANGED_MODE, MANNEQUIN_MODE, POSITION_MODE, NO_CONTROLLER_MODE};

void get_params();
int get_joint_by_name(std::string name);
int get_joint_by_id(int id); 
// void motor_state_callback(const ax12_driver_core::MotorStateList& motor_state_list);
//void joint_state_callback(const ua_controller_msgs::JointState& joint_msg_ptr);
void init(void);
void setTorsoGoal(int goal);
void go_directly_to_pos(void);
int move_to_next_pos(int joint, double desired_pos, int dir);
void move_to_desired_pos();
void set_desired_pos(int joint, double desired_pos);
void tuck();
void untuck();
void kinect_callobration_pos();
void arm_head_mode(int left_arm_control_mode, int right_arm_control_mode, int head_control_mode);
void set_arm_goal(double rx, double ry, double rz, double lx, double ly, double lz);
void JointCommand(int joint, double vel);
void WristCommand(double right_wrist_vel, double left_wrist_vel);
void ArmCommand(
       double r_x_vel, double r_y_vel, double r_z_vel,
       double l_x_vel, double l_y_vel, double l_z_vel);
void HeadCommand(double pan_vel, double tilt_vel);
};
