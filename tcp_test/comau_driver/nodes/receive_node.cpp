#include <ros/ros.h>

#include <comau_driver/comau_driver.h>

std::unique_ptr<comau_driver::ComauRobot> robot_ptr_;

int main(int argc, char **argv) {
  ros::init(argc, argv, "receive_node");
  // ros::AsyncSpinner spinner(2);
  // spinner.start();

  ros::NodeHandle nh("");

  // parameters
  bool is_receiver = true;
  std::string ip_address = "192.168.1.110";
  // std::string port = "1104"; // 54000
  // std::string ip_address = "192.168.56.2";
    // std::string ip_address = "150.140.148.219";
  std::string port = "1104";
  std::string log_tag = "state_tcp_server";

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

  uint32_t image_size;
  std::vector<uint8_t> image_array;

  while (true) {
    if (robot_ptr_->readMessagePackage()) {
      robot_ptr_->getImage(image_size, image_array);
      ROS_INFO("Image Size is : %d Received : %d", image_size, image_array.size());
      ROS_INFO("First 3 Ints %d %d %d", image_array[0], image_array[1], image_array[2]);
    }
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }

  return 0;
}
