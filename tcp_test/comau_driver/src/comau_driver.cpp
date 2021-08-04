/**
 * @file comau_driver.cpp
 * @author Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 * @brief The ROS node that publishes the robot information
 * @version 0.1
 * @date 25-02-2020
 *
 * @copyright (c) 2020 Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 *
 */

#include <comau_driver/comau_driver.h>
// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>
// pluginlib
#include <pluginlib/class_loader.h>

using namespace comau_tcp_interface;
using namespace comau_tcp_interface::utils;

namespace comau_driver {
ComauRobot::ComauRobot(ros::NodeHandle &nh) : name_("comau_driver"), nh_(nh) {
  // Load rosparams
  ros::NodeHandle nh_priv_("~");
  // Read parameters through ros parameter server
  // std::size_t error = 0;
  // error += !rosparam_shortcuts::get(name_, nh_priv_, "robot_ip", state_params_.server_ip_address);
  // error += !rosparam_shortcuts::get(name_, nh_priv_, "server_port", state_params_.server_port);
  // error += !rosparam_shortcuts::get(name_, nh_priv_, "log_tag", state_params_.log_tag);
  // rosparam_shortcuts::shutdownIfError(name_, error);
  // nh_priv_.getParam("din_pins", din_pins_);
}

ComauRobot::~ComauRobot() {
  if (is_receiver_) {
    state_client_ptr_->close();
    state_client_ptr_.reset();
  } else {
    robot_client_ptr_->close();
    robot_client_ptr_.reset();
  }
}

bool ComauRobot::initialize(std::string ip_address, std::string port, std::string log_tag, bool is_receiver) {

  pluginlib::ClassLoader<comau_tcp_interface::ComauClientBase> client_loader("comau_tcp_interface",
                                                                             "comau_tcp_interface::ComauClientBase");

  state_params_.server_ip_address = ip_address;
  state_params_.server_port = port;
  state_params_.log_tag = log_tag;
  robot_params_.server_ip_address = state_params_.server_ip_address;
  robot_params_.server_port = state_params_.server_port;
  robot_params_.log_tag = state_params_.log_tag;
  is_receiver_ = is_receiver;
  if (is_receiver_) {
    // tcp interface state client ptr
    try {
      state_client_ptr_.reset(new comau_tcp_interface::StateClient());
      if (!state_client_ptr_->initialize(state_params_)) {
        ROS_ERROR_STREAM("[comau_driver] State Client could not initialized ");
        return false;
      }
    } catch (pluginlib::PluginlibException &e) {
      ROS_ERROR_STREAM("[comau_driver] " << e.what());
      return false;
    }
  } else {

    // tcp interface motion client ptr
    try {
      robot_client_ptr_.reset(new comau_tcp_interface::RobotClient());
      if (!robot_client_ptr_->initialize(robot_params_)) {
        ROS_ERROR_STREAM("[comau_driver] Robot Command Client could not initialized ");
        return false;
      }
    } catch (pluginlib::PluginlibException &e) {
      ROS_ERROR_STREAM("[comau_driver] " << e.what());
      return false;
    }
  }

  return true;
}

void ComauRobot::getImage(uint32_t image_size, std::vector<uint8_t> &image_array) {
  msg->getData("image_size", image_size_);
  msg->getData("image_array", image_array_);
  image_array.assign(image_array_.begin(), image_array_.end());
}

bool ComauRobot::readMessagePackage() {
  msg = dynamic_cast<comau_tcp_interface::utils::MessagePackage *>(
      new comau_tcp_interface::utils::MessagePackage(state_client_ptr_->getRecvRecipe()));
  if (state_client_ptr_->getLastMessage(*msg)) {
    return true;
  }
  ROS_ERROR_STREAM("[comau_robot] Could not get Last Message Package");
  return false;
}

bool ComauRobot::writeImage(uint32_t image_size, std::vector<uint8_t> &image_array) {

  return robot_client_ptr_->sendImageMessage(image_size, image_array);
}

} // namespace comau_driver
