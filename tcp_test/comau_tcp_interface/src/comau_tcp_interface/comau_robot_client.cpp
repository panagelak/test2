/**
 * @file comau_robot_client.cpp
 * @author Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 * @brief The ROS node that publishes the robot information
 * @version 0.1
 * @date 25-02-2020
 *
 * @copyright (c) 2020 Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 *
 */

#include <chrono>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "comau_tcp_interface/comau_robot_client.h"

using namespace comau_tcp_interface::utils;

namespace comau_tcp_interface {

RobotClient::~RobotClient() {
  close();
}

bool RobotClient::initialize(const ComauTcpInterfaceParameters &params) {
  is_connected_ = false;
  // Assign parameters
  params_ptr_.reset(new ComauTcpInterfaceParameters(params));

  // Describe the incoming message
  incoming_msg_motion_descr_.push_back("image_size");
  last_recv_msg_.reset(new MessagePackage(incoming_msg_motion_descr_));

  // Describe the outgoing messages
  // initialize msg
  image_msg_descr_.push_back("image_size");
  image_msg_descr_.push_back("image_array");

  try {
    tcp_interface_ptr_.reset(new ComauTcpConnection(*params_ptr_));
  } catch (const boost::system::system_error &e) {
    ROS_ERROR_STREAM("[" << params_ptr_->log_tag << "] " << e.what());
    return false;
  }

  return true;
}

void RobotClient::close() {
  std::this_thread::sleep_for(std::chrono::seconds(1));
  is_connected_ = false;
  std::this_thread::sleep_for(std::chrono::seconds(3));
}

bool RobotClient::isConnected() {
  return is_connected_;
}

bool RobotClient::getLastMessage(MessagePackage &msg) {
  MessagePackage *last = dynamic_cast<MessagePackage *>(last_recv_msg_.get());
  if (last != nullptr) {
    msg = *last;
    return true;
  }
  return false;
}

void RobotClient::receive(std::chrono::milliseconds timeout) {

  std::vector<uint8_t> recv_raw_data(last_recv_msg_->getCapacity());
  size_t read_len = 0;

  tcp_interface_ptr_->read(recv_raw_data, sizeof(recv_raw_data), read_len);
  if (read_len == 0) {
    ROS_ERROR_STREAM("[" << params_ptr_->log_tag << "] Received zero bytes message");
    return;
  }

  MessageParser mp(recv_raw_data.data(), read_len);
  if (!last_recv_msg_->parseWith(mp)) {
    ROS_ERROR_STREAM("[" << params_ptr_->log_tag << "] "
                         << "Received message could not parsed");
    return;
  }

  ROS_DEBUG_STREAM("[" << params_ptr_->log_tag << "] Received message : " << last_recv_msg_->toString());
}

bool RobotClient::send(MessagePackage &msg) {
  size_t written_size = 0;
  std::vector<uint8_t> send_buffer(msg.getSize());
  size_t serielized_buffer_size = msg.serializePackage(send_buffer.data());
  if (serielized_buffer_size > msg.getSize()) {
    send_buffer.resize(serielized_buffer_size);
  }

  tcp_interface_ptr_->write(send_buffer, serielized_buffer_size, written_size);

  if (written_size != serielized_buffer_size) {
    ROS_ERROR_STREAM("[" << params_ptr_->log_tag << "] Send message failed buffer size: " << serielized_buffer_size
                         << "  written: " << written_size);
    return false;
  }
  ROS_DEBUG_STREAM("[" << params_ptr_->log_tag << "] " << msg.toString());
  return true;
}

bool RobotClient::sendImageMessage(uint32_t image_size,std::vector<uint8_t> &image_array) {
  utils::MessagePackage msg(getSendImageRecipe());
  vectorImage_t image_vec;
  for(size_t i=0; i<120; i++){
    image_vec[i] = image_array[i];
  }
  bool com1 = msg.setData("image_size", image_size);
  bool com2 = msg.setData("image_array", image_vec);
  if (com1 && com2) {
    return send(msg);
  } else {
    ROS_ERROR_STREAM("[" << params_ptr_->log_tag << "] Send message failed ");
    return false;
  }
}

} // namespace comau_tcp_interface

PLUGINLIB_EXPORT_CLASS(comau_tcp_interface::RobotClient, comau_tcp_interface::ComauClientBase)
