#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class LeftRobotArm
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  TrajClient* traj_client_;

public:
  //! Initialize the action client and wait for action server to come up
  LeftRobotArm() 
  {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("l_arm_controller/joint_trajectory_action", true);

    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  //! Clean up the action client
  ~LeftRobotArm()
  {
    delete traj_client_;
  }

  //! Sends the command to start a given trajectory
  void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal)
  {
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);
  }

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
  pr2_controllers_msgs::JointTrajectoryGoal armExtensionTrajectory()
  {
    //our goal variable
    pr2_controllers_msgs::JointTrajectoryGoal goal;
    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
    goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
    goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
    goal.trajectory.joint_names.push_back("l_wrist_roll_joint");
    goal.trajectory.points.resize(1);

    // Positions
    goal.trajectory.points[0].positions.resize(7);
    ros::NodeHandle ph("~");
    for(int i = 0; i < 7; i++)
      goal.trajectory.points[0].positions[i] = 0.0;

    ph.getParam("l_shoulder_pan", goal.trajectory.points[0].positions[0]);
    ph.getParam("l_shoulder_lift", goal.trajectory.points[0].positions[1]);
    ph.getParam("l_upper_arm_roll", goal.trajectory.points[0].positions[2]);
    ph.getParam("l_elbow_flex", goal.trajectory.points[0].positions[3]);
    ph.getParam("l_forearm_roll", goal.trajectory.points[0].positions[4]);
    ph.getParam("l_wrist_flex", goal.trajectory.points[0].positions[5]);
    ph.getParam("l_wrist_roll", goal.trajectory.points[0].positions[6]);

    // Velocities
    goal.trajectory.points[0].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal.trajectory.points[0].velocities[j] = 0.0;
    }
    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[0].time_from_start = ros::Duration(2.0);

    //we are done; return the goal
    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }
 
};

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "send_left_arm_to_position");

  LeftRobotArm arm;
  // Start the trajectory
  arm.startTrajectory(arm.armExtensionTrajectory());
  // Wait for trajectory completion
  while(!arm.getState().isDone() && ros::ok())
  {
    usleep(50000);
  }
}
