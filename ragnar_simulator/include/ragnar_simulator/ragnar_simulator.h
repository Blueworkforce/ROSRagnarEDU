#ifndef RAGNAR_SIMULATOR_H
#define RAGNAR_SIMULATOR_H

#include <vector>
#include <string>

#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/node_handle.h>
#include <actionlib/server/action_server.h>

namespace ragnar_simulator
{

class RagnarSimulator
{
public:
  RagnarSimulator(const std::vector<double>& seed_pose,
                  const std::vector<std::string>& joint_names,
                  ros::NodeHandle &nh);

  const std::vector<std::string>& getJointNames() const { return joint_names_; }

  // Initializes trajectory, start time, and position fields
  bool setTrajectory(const trajectory_msgs::JointTrajectory& new_trajectory);
  // Computes the robot position at a given time based on the currently active
  // trajectory
  bool computeTrajectoryPosition(const ros::Time& tm, std::vector<double>& output) const;

  void pollAction();

private:
  // Configuration
  std::vector<std::string> joint_names_;

  // Action server
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JointTractoryActionServer;

  void goalCB(JointTractoryActionServer::GoalHandle & gh);
  void cancelCB(JointTractoryActionServer::GoalHandle & gh);

  JointTractoryActionServer action_server_;
  JointTractoryActionServer::GoalHandle active_goal_;
  bool has_active_goal_;

  // State
  trajectory_msgs::JointTrajectory traj_;
  std::vector<double> traj_start_position_;
  ros::Time traj_start_time_;
};

}

#endif
