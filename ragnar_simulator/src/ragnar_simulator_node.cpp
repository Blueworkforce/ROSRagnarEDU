#include <ros/ros.h>
#include <sensor_msgs/JointState.h> // for publishing robot current position
#include <trajectory_msgs/JointTrajectory.h>

#include "ragnar_simulator/ragnar_simulator.h"

void publishCurrentState(const ros::TimerEvent& timer,
                         ros::Publisher& pub,
                         ragnar_simulator::RagnarSimulator& sim)
{
  sensor_msgs::JointState joint_state;
  joint_state.header.frame_id = "world";
  joint_state.header.stamp = ros::Time::now();
  joint_state.name = sim.getJointNames();
  // compute current position
  sim.computeTrajectoryPosition(timer.current_real, joint_state.position);
  sim.pollAction();

  std::reverse(joint_state.position.begin(), joint_state.position.end());
  for (unsigned i = 0; i < joint_state.position.size(); ++i)
    joint_state.position[i] *= -1.0;

  pub.publish(joint_state);
}

void setCurrentTrajectory(const trajectory_msgs::JointTrajectoryConstPtr& traj,
                          ragnar_simulator::RagnarSimulator& sim)
{
  ROS_INFO("Setting new trajectory");
  sim.setTrajectory(*traj);
}

int main(int argc, char** argv)
{
  const static double default_position[] = {-0.07979196, 0.07044869,
                                            -0.07044869, 0.07979196};

  ros::init(argc, argv, "ragnar_simulator_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh ("~");

  // nh loads joint names if possible
  std::vector<std::string> joint_names;
  if (!nh.getParam("controller_joint_names", joint_names))
  {
    // otherwise, it loads defaults
    joint_names.push_back("joint_1");
    joint_names.push_back("joint_2");
    joint_names.push_back("joint_3");
    joint_names.push_back("joint_4");
  }
  // pnh loads configuration parameters
  std::vector<double> seed_position;
  if (!pnh.getParam("initial_position", seed_position))
  {
    seed_position.assign(default_position, default_position + 4);
  }

  double publish_rate;
  pnh.param<double>("rate", publish_rate, 30.0);

  // instantiate simulation
  ragnar_simulator::RagnarSimulator sim (seed_position, joint_names, nh);

  // create pub/subscribers and wire them up
  ros::Publisher current_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::Subscriber command_state_sub =
      nh.subscribe<trajectory_msgs::JointTrajectory>("joint_path_command",
                                                     1,
                                                     boost::bind(setCurrentTrajectory,
                                                                 _1,
                                                                 boost::ref(sim)));
  ros::Timer state_publish_timer =
      nh.createTimer(ros::Duration(1.0/publish_rate), boost::bind(publishCurrentState,
                                                     _1,
                                                     boost::ref(current_state_pub),
                                                     boost::ref(sim)));

  ROS_INFO("Simulator service spinning");
  ros::spin();
  return 0;
}
