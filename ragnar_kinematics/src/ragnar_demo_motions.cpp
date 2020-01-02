#include <ros/ros.h>
#include <ragnar_kinematics/ragnar_kinematics.h>
#include <trajectory_msgs/JointTrajectory.h>

static void populateHeader(std_msgs::Header& header)
{
  header.frame_id = "base_link";
  header.stamp = ros::Time::now();
}

static trajectory_msgs::JointTrajectory makeCircleTrajectory()
{
  using namespace trajectory_msgs;
  // Header
  JointTrajectory traj;
  populateHeader(traj.header);

  // Create circle points
  const double r = 0.15;
  const double dt = 0.05;

  double pose[4];
  double joints[4];

  double total_t = dt;

  for (int i = 0; i < 360; ++i)
  {
    pose[0] = r * std::cos(i * M_PI / 180.0);
    pose[1] = r * std::sin(i * M_PI / 180.0);
    pose[2] = -0.35;
    pose[3] = 0.0;

    JointTrajectoryPoint pt;
    if (!ragnar_kinematics::inverse_kinematics(pose, joints))
    {
      ROS_WARN_STREAM("Could not solve for: " << pose[0] << " " << pose[1] << " " << pose[2] << " " << pose[3]);
    }
    else
    {
      pt.positions.assign(joints, joints+4);
      pt.time_from_start = ros::Duration(total_t);
      total_t += dt;
      traj.points.push_back(pt);
    }
  }
  return traj;
}

// Linear trajectory helpers
struct RagnarPoint {
  double joints[4];
};

struct RagnarPose {
  RagnarPose() {}

  RagnarPose(double x, double y, double z)
  {
    pose[0] = x;
    pose[1] = y;
    pose[2] = z;
    pose[3] = 0.0;
  }

  double pose[4];
};

RagnarPose interpPose(const RagnarPose& start, const RagnarPose& stop, double ratio)
{
  RagnarPose result;
  result.pose[0] = start.pose[0] + ratio * (stop.pose[0] - start.pose[0]);
  result.pose[1] = start.pose[1] + ratio * (stop.pose[1] - start.pose[1]);
  result.pose[2] = start.pose[2] + ratio * (stop.pose[2] - start.pose[2]);
  result.pose[3] = start.pose[3] + ratio * (stop.pose[3] - start.pose[3]);
  return result;
}

bool linearMove(const RagnarPose& start, const RagnarPose& stop, double ds, std::vector<RagnarPoint>& out)
{
  std::vector<RagnarPoint> pts;
  double delta_x = stop.pose[0] - start.pose[0];
  double delta_y = stop.pose[1] - start.pose[1];
  double delta_z = stop.pose[2] - start.pose[2];
  double delta_s = std::sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);

  unsigned steps = static_cast<unsigned>(delta_s / ds) + 1;

  for (unsigned i = 0; i <= steps; i++)
  {
    double ratio = static_cast<double>(i) / steps;
    RagnarPose pose = interpPose(start, stop, ratio);
    RagnarPoint pt;
    if (!ragnar_kinematics::inverse_kinematics(pose.pose, pt.joints))
    {
      return false;
    }
    pts.push_back(pt);
  }

  out = pts;
  return true;
}

std::vector<RagnarPoint> linearMove(const RagnarPose& start, const RagnarPose& stop, double ds)
{
  std::vector<RagnarPoint> pts;
  if (!linearMove(start, stop, ds, pts))
  {
    throw std::runtime_error("Couldn't plan for linear move");
  }
  return pts;
}

typedef std::vector<trajectory_msgs::JointTrajectoryPoint> TrajPointVec;

TrajPointVec toTrajPoints(const std::vector<RagnarPoint>& points, double time)
{
  TrajPointVec vec;
  double dt = time / points.size();
  double total_t = dt;

  for (size_t i = 0; i < points.size(); ++i)
  {
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions.assign(points[i].joints, points[i].joints + 4);
    pt.time_from_start = ros::Duration(total_t);
    total_t += dt;

    vec.push_back(pt);
  }

  return vec;
}

// append b to a
TrajPointVec append(const TrajPointVec& a, const TrajPointVec& b)
{
  TrajPointVec result;
  result.reserve(a.size() + b.size());
  // insert a
  result.insert(result.end(), a.begin(), a.end());

  ros::Duration time_end_a = a.back().time_from_start;

  for (size_t i = 0; i < b.size(); ++i)
  {
    trajectory_msgs::JointTrajectoryPoint pt = b[i];
    pt.time_from_start += time_end_a;
    result.push_back(pt);
  }

  return result;
}



static trajectory_msgs::JointTrajectory makeLineTrajectory()
{
  using namespace trajectory_msgs;
  // Header
  JointTrajectory traj;
  populateHeader(traj.header);

  std::vector<RagnarPoint> points;
  RagnarPose start (-0.1, -0.1, -0.3);
  RagnarPose stop (0.1, 0.1, -0.4);

  if (!linearMove(start, stop, 0.01, points))
  {
    throw std::runtime_error("Linear movement planning failed");
  }

  const double dt = 0.1;
  double total_t = dt;

  for (unsigned i = 0; i < points.size(); ++i)
  {
    JointTrajectoryPoint pt;
    pt.positions.assign(points[i].joints, points[i].joints+4);
    pt.time_from_start = ros::Duration(total_t);
    total_t += dt;
    traj.points.push_back(pt);
  }
  return traj;
}

TrajPointVec singlePoint(const RagnarPose& pose, double dt)
{
  RagnarPoint joints;
  if (!ragnar_kinematics::inverse_kinematics(pose.pose, joints.joints))
  {
    throw std::runtime_error("Couldn't plan to point");
  }

  TrajPointVec v;
  trajectory_msgs::JointTrajectoryPoint pt;
  pt.positions.assign(joints.joints, joints.joints + 4);
  pt.time_from_start = ros::Duration(dt);
  v.push_back(pt);
  return v;
}

static trajectory_msgs::JointTrajectory makePickPlaceTrajectory()
{
  trajectory_msgs::JointTrajectory traj;
  populateHeader(traj.header);

  const double LINEAR_MOVE_TIME = 2.0;
  const double VERTICAL_MOVE_TIME = 2.0;
  const double WAIT_PERIOD = 0.5;

  // Home position
  RagnarPose home_pt (0.0, 0.0, -0.2);
  TrajPointVec vec = singlePoint(home_pt, 5.0);

  // Pick spot 1
  RagnarPose pick1 (0.15, -0.3, -0.25);
  vec = append(vec, toTrajPoints(linearMove(home_pt, pick1, 0.01), LINEAR_MOVE_TIME));

  // Down
  RagnarPose pick1_down (0.15, -0.3, -0.4);
  vec = append(vec, toTrajPoints(linearMove(pick1, pick1_down, 0.01), VERTICAL_MOVE_TIME));

  // Wait & Up
  vec = append(vec, singlePoint(pick1_down, WAIT_PERIOD));
  vec = append(vec, toTrajPoints(linearMove(pick1_down, pick1, 0.01), VERTICAL_MOVE_TIME));
  // back home
  vec = append(vec, toTrajPoints(linearMove(pick1, home_pt, 0.01), LINEAR_MOVE_TIME));

  /*// Place spot 1
  RagnarPose place1 (-0.15, 0.3, -0.1);
  vec = append(vec, toTrajPoints(linearMove(pick1, home_pt, 0.01), LINEAR_MOVE_TIME));
  vec = append(vec, toTrajPoints(linearMove(home_pt, place1, 0.01), LINEAR_MOVE_TIME));

  // Down
  RagnarPose place1_down (-0.15, 0.35, -0.3);
  vec = append(vec, toTrajPoints(linearMove(place1, place1_down, 0.01), VERTICAL_MOVE_TIME));

  // Wait & Up
  vec = append(vec, singlePoint(place1_down, WAIT_PERIOD));
  vec = append(vec, toTrajPoints(linearMove(place1_down, place1, 0.01), VERTICAL_MOVE_TIME));
  vec = append(vec, toTrajPoints(linearMove(place1, home_pt, 0.01), LINEAR_MOVE_TIME));
  */
  /*//
  // CYCLE 2
  //
  // Pick spot 2
  RagnarPose pick2 (-0.1, -0.4, -0.2);
  vec = append(vec, toTrajPoints(linearMove(home_pt, pick2, 0.01), LINEAR_MOVE_TIME));

  // Down
  RagnarPose pick2_down (-0.1, -0.4, -0.50);
  vec = append(vec, toTrajPoints(linearMove(pick2, pick2_down, 0.01), VERTICAL_MOVE_TIME*1.2));

  // Wait & Up
  vec = append(vec, singlePoint(pick2_down, WAIT_PERIOD));
  vec = append(vec, toTrajPoints(linearMove(pick2_down, pick2, 0.01), VERTICAL_MOVE_TIME));

  // Place spot 2
  RagnarPose place2 (-0.1, 0.4, -0.2);
  vec = append(vec, toTrajPoints(linearMove(pick2, home_pt, 0.01), LINEAR_MOVE_TIME));
  vec = append(vec, toTrajPoints(linearMove(home_pt, place2, 0.01), LINEAR_MOVE_TIME));

  // Down
  RagnarPose place2_down (-0.1, 0.4, -0.4);
  vec = append(vec, toTrajPoints(linearMove(place2, place2_down, 0.01), VERTICAL_MOVE_TIME));

  // Wait & Up
  vec = append(vec, singlePoint(place2_down, WAIT_PERIOD));
  vec = append(vec, toTrajPoints(linearMove(place2_down, place2, 0.01), VERTICAL_MOVE_TIME));
  vec = append(vec, toTrajPoints(linearMove(place2, home_pt, 0.01), LINEAR_MOVE_TIME));

  // Forward and back
  RagnarPose forward1 (0.4, 0, -0.4);
  RagnarPose backward1 (-0.4, 0, -0.4);
  vec = append(vec, toTrajPoints(linearMove(home_pt, forward1, 0.01), LINEAR_MOVE_TIME));
  vec = append(vec, toTrajPoints(linearMove(forward1, backward1, 0.01), LINEAR_MOVE_TIME));
  vec = append(vec, toTrajPoints(linearMove(backward1, forward1, 0.01), LINEAR_MOVE_TIME));
  vec = append(vec, toTrajPoints(linearMove(forward1, home_pt, 0.01), LINEAR_MOVE_TIME));
*/
  traj.points = vec;
  return traj;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ragnar_demo_motions");

  ros::NodeHandle nh;
  ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);

  // trajectory_msgs::JointTrajectory traj = makeLineTrajectory();
  // trajectory_msgs::JointTrajectory traj = makeCircleTrajectory();
  trajectory_msgs::JointTrajectory traj = makePickPlaceTrajectory();
  //
  std::vector<std::string> names;
  names.push_back("joint_1");
  names.push_back("joint_2");
  names.push_back("joint_3");
  names.push_back("joint_4");

  traj.joint_names = names;
  ros::Duration(0.5).sleep();

  traj_pub.publish(traj);

  ros::spin();
}
