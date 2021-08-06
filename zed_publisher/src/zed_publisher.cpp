#include "Publisher.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "zed_publisher");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle nh("");

  // Set the ROS logging level (for better debugging)
  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
  //     ros::console::notifyLoggerLevelsChanged();
  // }

  ROS_INFO("ZED Publisher");

  // Create the zed publisher object
  Publisher ZEDPublisher(nh);

  ZEDPublisher.run();
  //   double Frequency = 30;
  // nh.param<int>("Frequency", Frequency, 30);
  //   ros::Rate rate(Frequency);

  //   while (ros::ok()) {
  //     ZEDPublisher.Publish();
  //     ros::spinOnce();
  //     rate.sleep();
  //   }

  ROS_INFO("Exiting.");
  return 0;
}