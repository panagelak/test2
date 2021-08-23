#include "zed_publisher.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "zed_publisher");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle nh("");

  ROS_INFO("ZED Publisher");
  // Create the zed publisher object
  ZEDPublisher ZEDPublisher(nh);

  ros::waitForShutdown();
  ROS_INFO("Exiting.");
  return 0;
}