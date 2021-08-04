#include <ros/ros.h>

#include <comau_driver/comau_driver.h>

boost::shared_ptr<comau_driver::ComauRobot> robot_ptr_;

int main(int argc, char **argv) {
  ros::init(argc, argv, "send_node");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh("");

  // parameters
  bool is_receiver = false;
  // std::string ip_address = "192.168.1.101";
  // std::string port = "1102";
  std::string ip_address = "192.168.56.2";
  std::string port = "1104";
  std::string log_tag = "robot_tcp_server";

  // Initialize Robot driver
  try {
    robot_ptr_.reset(new comau_driver::ComauRobot(nh));
    if (!robot_ptr_->initialize(ip_address, port, log_tag, is_receiver)) {
      ROS_ERROR_STREAM("[comau_hw_interface] Failed to initialize robot driver");
      return false;
    }
  } catch (...) {
    ROS_ERROR_STREAM("[comau_hw_interface] Failed to initialize robot driver");
    return false;
  }

  uint32_t image_size = 120;
  std::vector<uint8_t> image_array;
  image_array.resize(120);
  for (size_t i = 0; i < 120; i++)
    image_array[i] = 0;

  while (true) {
    robot_ptr_->writeImage(image_size, image_array);
    image_array[0] += 1;
    image_array[1] += 1;
    image_array[2] += 1;
    ros::Duration(0.1).sleep();
  }

  return 0;
}
