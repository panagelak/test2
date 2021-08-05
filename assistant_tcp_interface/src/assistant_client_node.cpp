/**
 * @file assistant_motion_client_node.cpp
 * @author LMS ()
 * @brief Test node for using motion client node
 * @version 0.1
 * @date 30-06-2020
 *
 * @copyright Copyright (c) 2020
 *
 */
#include <csignal>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include "assistant_tcp_interface/client/assistant_client.h"

using namespace assistant_tcp_interface;

boost::shared_ptr<AssistantClient> assistant_client;

[[noreturn]] void signalHandler(int signum) {
  ROS_WARN_STREAM("[assistant_client_node] Interrupt signal (" << signum << ") received.\n");

  assistant_client.reset();

  ros::shutdown();
  exit(signum);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "assistant_client_node", ros::init_options::NoRosout);
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  // register signal SIGINT and signal handler
  signal(SIGINT, signalHandler);

  AssistantTcpClientInterfaceParameters params;

  // Read parameters through ros parameter server
  nh.param<std::string>("robot_ip", params.server_ip_address, "127.0.0.1");
  nh.param<std::string>("server_port", params.server_port, "1234");
  params.log_tag = "[assistant_client_node] ";

  sleep(3);

  // pluginlib::ClassLoader<AssistantClientBase> client_loader("assistant_tcp_interface", "assistant_tcp_interface::AssistantClientBase");

  try {
    assistant_client.reset(new AssistantClient());

    if (assistant_client->initialize(params)) {
      ;
      // utils::vector6f_t joint_values = {0.0, 0.0, -90.0, 0.0, 0.0, 0.0};
      // utils::trajectoryf_t traj = {joint_values};

      // joint_values = {25.0, 0.0, -90.0, 0.0, 0.0, 0.0};
      // traj.push_back(joint_values);

      // joint_values = {50.0, 0.0, -90.0, 0.0, 0.0, 0.0};
      // traj.push_back(joint_values);

      // joint_values = {75.0, 0.0, -90.0, 0.0, 0.0, 0.0};
      // traj.push_back(joint_values);

      // joint_values = {90.0, 0.0, -90.0, 0.0, 0.0, 0.0};
      // traj.push_back(joint_values);

      // joint_values = {75.0, 0.0, -60.0, 0.0, 0.0, 0.0};
      // traj.push_back(joint_values);

      // joint_values = {50.0, 0.0, -60.0, 0.0, 0.0, 0.0};
      // traj.push_back(joint_values);

      // joint_values = {0.0, 0.0, -60.0, 0.0, 0.0, 0.0};
      // traj.push_back(joint_values);

      // joint_values = {-20.0, 0.0, -60.0, 0.0, 0.0, 0.0};
      // traj.push_back(joint_values);

      // joint_values = {-40.0, 0.0, -60.0, 0.0, 0.0, 0.0};
      // traj.push_back(joint_values);

      // joint_values = {-60.0, 0.0, -60.0, 0.0, 0.0, 0.0};
      // traj.push_back(joint_values);

      // joint_values = {-80.0, 0.0, -60.0, 0.0, 0.0, 0.0};
      // traj.push_back(joint_values);

      // // motion_client->sendJointTrajectoryMessage(traj);

      // sleep(15);
    } else {
      ROS_ERROR_STREAM("[assistant_client_node] Error at client initialize");
    }
  } catch (pluginlib::PluginlibException &e) {
    ROS_ERROR_STREAM("[assistant_client_node]" << e.what());
  }

  assistant_client.reset();

  ros::shutdown();
  return 0;
}
