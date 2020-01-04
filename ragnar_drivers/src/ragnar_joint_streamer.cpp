#include "ragnar_drivers/ragnar_joint_streamer.h"
#include <ragnar_kinematics/ragnar_kinematics.h>

// To simplify the massively complicated namespaces
using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial::tcp_client::TcpClient;
using industrial::joint_traj_pt_message::JointTrajPtMessage;
using industrial::simple_message::SimpleMessage;
using industrial::joint_traj_pt_message::JointTrajPtMessage;

typedef industrial::joint_traj_pt::JointTrajPt rbt_JointTrajPt;
typedef trajectory_msgs::JointTrajectoryPoint ros_JointTrajPt;

namespace TransferStates =
    industrial_robot_client::joint_trajectory_streamer::TransferStates;

// Constants
// NOTE that the robot takes velocity values in millimeters per minute
const static double RAGNAR_DEFAULT_VELOCITY = 100.0 * 60.0; // 100 mm/s
// The error from the nominal point that the robot can be at
// and be considered finished with its goal. It's large, but it seems
// the FK differs a bit between platforms.
const static double JOINT_TOL_EPS = 0.05;

// helper function
static JointTrajPtMessage create_message(int seq,
                                         const std::vector<double>& joint_pos,
                                         double velocity, double duration)
{
  industrial::joint_data::JointData pos;
  ROS_ASSERT(joint_pos.size() <= (unsigned int)pos.getMaxNumJoints());

  for (size_t i = 0; i < joint_pos.size(); ++i)
    pos.setJoint(i, joint_pos[i]);

  rbt_JointTrajPt pt;
  pt.init(seq, pos, velocity, duration);

  JointTrajPtMessage msg;
  msg.init(pt);

  return msg;
}

static bool pose_in_range(const double(&vec)[4])
{
  const static double MIN_X = -0.4;
  const static double MAX_X = 0.4;
  const static double MIN_Y = -0.4;
  const static double MAX_Y = 0.4;
  const static double MIN_Z = -0.55;
  const static double MAX_Z = 0.0;

  if (vec[0] > MAX_X || vec[0] < MIN_X)
    return false;
  if (vec[1] > MAX_Y || vec[1] < MIN_Y)
    return false;
  if (vec[2] > MAX_Z || vec[2] < MIN_Z)
    return false;
  return true;
}

static bool calc_ragnar_velocity(const std::vector<double>& start_pose,
                                 const std::vector<double>& stop_pose,
                                 double dt, double& output_vel)
{
  if (dt < 0.0)
  {
    return false;
  }
  if (dt == 0.0)
  {
    dt = RAGNAR_DEFAULT_VELOCITY;
  }

  double start[4];
  if (!ragnar_kinematics::forward_kinematics(start_pose.data(), start))
  {
    ROS_INFO_STREAM("Forward kinematics failure");
    return false;
  }

  if (!pose_in_range(start))
  {
    ROS_INFO_STREAM("Pose out of allowable work-volume");
    return false;
  }

  double stop[4];
  if (!ragnar_kinematics::forward_kinematics(stop_pose.data(), stop))
  {
    ROS_INFO_STREAM("Forward kinematics failure");
    return false;
  }

  if (!pose_in_range(stop))
  {
    ROS_INFO_STREAM("Pose out of allowable work-volume");
    return false;
  }
  // calculate euclidean distance
  double dist = std::sqrt(std::pow(stop[0] - start[0], 2) +
                          std::pow(stop[1] - start[1], 2) +
                          std::pow(stop[2] - start[2], 2));
  double vel = dist / dt;

  // Ragnar G-Code translation requires units of mm per minute
  output_vel = vel * 1000.0 * 60.0;

  return true;
}

bool ragnar_drivers::RagnarTrajectoryStreamer::transform(
    const trajectory_msgs::JointTrajectoryPoint& pt_in,
    trajectory_msgs::JointTrajectoryPoint* pt_out)
{
  const static double RAD_TO_DEG = 180.0 / M_PI;
  // Copy the input
  *pt_out = pt_in;
  // Radians -> Degrees
  for (size_t i = 0; i < pt_out->positions.size(); ++i)
  {
    pt_out->positions[i] *= RAD_TO_DEG;
  }

  // Velocity/Acceleration/Effort values are ignored in this version of the
  // controller
  // Duration values are used elsewhere to calculate a cartesian tool trajectory

  return true;
}

//
bool ragnar_drivers::RagnarTrajectoryStreamer::trajectory_to_msgs(
    const trajectory_msgs::JointTrajectoryConstPtr& traj,
    std::vector<JointTrajPtMessage>* msgs)
{
  msgs->clear();

  // check for valid trajectory
  if (!is_valid(*traj))
    return false;

  for (size_t i = 0; i < traj->points.size(); ++i)
  {
    ros_JointTrajPt rbt_pt, xform_pt;
    double vel, duration;

    // select / reorder joints for sending to robot
    if (!select(traj->joint_names, traj->points[i], this->all_joint_names_,
                &rbt_pt))
      return false;

    // transform point data (e.g. for joint-coupling)
    if (!transform(rbt_pt, &xform_pt))
      return false;

    // If not the starting point, we compute the actual tool velocity between
    // subsiquent points
    if (i > 0)
    {
      const std::vector<double>& start_pose = traj->points[i - 1].positions;
      const std::vector<double>& stop_pose = traj->points[i].positions;
      double start_time = traj->points[i - 1].time_from_start.toSec();
      double stop_time = traj->points[i].time_from_start.toSec();
      // Compute the tool velocity between these two points; stored in 'vel' arg
      // Can fail if the FK is bad
      if (!calc_ragnar_velocity(start_pose, stop_pose, stop_time - start_time,
                                vel))
      {
        return false;
      }
    }
    else
    {
      // Starting point velocity
      vel = RAGNAR_DEFAULT_VELOCITY;
    }
    duration = traj->points[i].time_from_start.toSec();

    JointTrajPtMessage msg =
        ::create_message(i, xform_pt.positions, vel, duration);
    msgs->push_back(msg);
  }

  return true;
}

// RAGNAR action server stuff

ragnar_drivers::RagnarTrajectoryStreamer::RagnarTrajectoryStreamer()
  : action_server_(node_, "joint_trajectory_action", boost::bind(&RagnarTrajectoryStreamer::goalCB, this, _1),
                   boost::bind(&RagnarTrajectoryStreamer::cancelCB, this, _1), false)
  , has_active_goal_(false)
{
   action_server_.start();
}

void ragnar_drivers::RagnarTrajectoryStreamer::goalCB(JointTractoryActionServer::GoalHandle gh)
{
  ROS_INFO("Recieved new goal request");
  if (has_active_goal_)
  {
    ROS_WARN("Received new goal, canceling current one");
    trajectoryStop();
    active_goal_.setAborted();
    has_active_goal_ = false;
  }

  gh.setAccepted();
  active_goal_ = gh;
  has_active_goal_ = true;

  const trajectory_msgs::JointTrajectory& traj = active_goal_.getGoal()->trajectory;
  if (!traj.points.empty())
  {
    target_pt_ = traj.points.back();
  }
  jointTrajectoryCB( trajectory_msgs::JointTrajectoryConstPtr(new trajectory_msgs::JointTrajectory(traj)) );
}

void ragnar_drivers::RagnarTrajectoryStreamer::cancelCB(JointTractoryActionServer::GoalHandle gh)
{
  ROS_INFO("Cancelling goal");
  if (active_goal_ == gh)
  {
    // stop the controller
    trajectoryStop();
    // mark the goal as canceled
    active_goal_.setCanceled();
    has_active_goal_ = false;
  }
}

static bool inRange(const std::vector<double>& a, const std::vector<double>& b, double eps)
{
  ROS_ASSERT(a.size() == b.size());
  for (size_t i = 0; i < a.size(); ++i)
  {
    if (std::abs(a[i] - b[i]) > eps) return false;
  }
  return true;
}

void ragnar_drivers::RagnarTrajectoryStreamer::jointStateCB(const sensor_msgs::JointStateConstPtr &msg)
{
  this->cur_joint_pos_ = *msg;
  if (has_active_goal_)
  {
    if (state_ == TransferStates::IDLE && inRange(cur_joint_pos_.position, target_pt_.positions, JOINT_TOL_EPS))
    {
      ROS_INFO("Action succeeded");
      active_goal_.setSucceeded();
      has_active_goal_ = false;
    }
  }
}
