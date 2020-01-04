#ifndef RAGNAR_JOINT_TRAJECTORY_STREAMER
#define RAGNAR_JOINT_TRAJECTORY_STREAMER

#include "industrial_robot_client/joint_trajectory_streamer.h"
#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace ragnar_drivers
{

class RagnarTrajectoryStreamer
    : public industrial_robot_client::joint_trajectory_streamer::
          JointTrajectoryStreamer
{
public:
  RagnarTrajectoryStreamer();

  using JointTrajectoryInterface::init;

  virtual bool transform(const trajectory_msgs::JointTrajectoryPoint& pt_in,
                         trajectory_msgs::JointTrajectoryPoint* pt_out);

  virtual bool trajectory_to_msgs(
      const trajectory_msgs::JointTrajectoryConstPtr& traj,
      std::vector<industrial::joint_traj_pt_message::JointTrajPtMessage>* msgs);

  //
  // Action Server Interface
  //

  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JointTractoryActionServer;

  virtual void jointStateCB(const sensor_msgs::JointStateConstPtr &msg);
  void goalCB(JointTractoryActionServer::GoalHandle gh);
  void cancelCB(JointTractoryActionServer::GoalHandle gh);

  JointTractoryActionServer action_server_;
  JointTractoryActionServer::GoalHandle active_goal_;
  trajectory_msgs::JointTrajectoryPoint target_pt_;
  bool has_active_goal_;


};
}

#endif
