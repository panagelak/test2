/**
 * @file assistant_motion_client.cpp
 * @author LMS ()
 * @brief
 * @version 0.1
 * @date  20-6-2020
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <chrono>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "assistant_tcp_interface/client/assistant_client.h"

using namespace assistant_tcp_interface::utils;

namespace assistant_tcp_interface {

AssistantClient::~AssistantClient() {
  close();
  receiving_thread_->detach();
}

bool AssistantClient::initialize(const AssistantTcpClientInterfaceParameters &params) {
  is_connected_ = false;
  // Assign parameters
  params_ptr_.reset(new AssistantTcpClientInterfaceParameters(params));

  // Describe the incoming message
  incoming_msg_descr_.push_back("timestamp");
  incoming_msg_descr_.push_back("confirm_receive");
  // initialize incoming message pack structure
  last_recv_msg_.reset(new MessagePackage(incoming_msg_descr_));

  // Describe initialization message
  outgoing_msg_descr_.push_back("timestamp");
  outgoing_msg_descr_.push_back("image_size");
  // outgoing_msg_descr_.push_back("image_data");

  // Fetch std::future object associated with promise
  future_obj_for_exit_ = exit_promise_signal_.get_future();
  // Starting Thread & move the future object in callback function by reference
  try {
    receiving_thread_.reset(new std::thread(&AssistantClient::callback, this, std::move(future_obj_for_exit_)));
  } catch (const std::bad_alloc &e) {
    ROS_ERROR_STREAM("[" << params_ptr_->log_tag << "] "
                         << "Callback thread allocation failed: " << e.what());
    return false;
  } catch (const boost::system::system_error &e) {
    ROS_ERROR_STREAM("[" << params_ptr_->log_tag << "] "
                         << "Callback thread instantion throws : " << e.what());
    return false;
  } catch (...) {
    ROS_ERROR_STREAM("[" << params_ptr_->log_tag << "] "
                         << "Callback thread instantion throws unexpected error ");
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  return true;
}

void AssistantClient::close() {
  is_connected_ = false;
  exit_promise_signal_.set_value();

  // TODO send terminate message
  // MessagePackage terminate_msg(terminate_msg_descr_);
  // bool trigger_termination = true;
  // if (terminate_msg.setData("terminate", trigger_termination)) {
  //   send(terminate_msg);
  // }

  std::this_thread::sleep_for(std::chrono::seconds(3));
}

bool AssistantClient::isConnected() {
  return is_connected_;
}

bool AssistantClient::getLastMessage(MessagePackage &msg) {
  MessagePackage *last = dynamic_cast<MessagePackage *>(last_recv_msg_.get());
  if (last != nullptr) {
    msg = *last;
    return true;
  }
  return false;
}

void AssistantClient::receive(std::chrono::milliseconds timeout) {

  std::vector<uint8_t> recv_raw_data(last_recv_msg_->getCapacity());
  size_t read_len = 0;

  tcp_client_interface_ptr_->read(recv_raw_data, sizeof(recv_raw_data), read_len);
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

bool AssistantClient::send(MessagePackage &msg) {
  size_t written_size = 0;
  std::vector<uint8_t> send_buffer(msg.getSize());
  size_t serielized_buffer_size = msg.serializePackage(send_buffer.data());
  if (serielized_buffer_size > msg.getSize()) {
    send_buffer.resize(serielized_buffer_size);
  }

  tcp_client_interface_ptr_->write(send_buffer, serielized_buffer_size, written_size);

  if (written_size != serielized_buffer_size) {
    ROS_ERROR_STREAM("[" << params_ptr_->log_tag << "] Send message failed buffer size: " << serielized_buffer_size
                         << "  written: " << written_size);
    return false;
  }
  ROS_DEBUG_STREAM("[" << params_ptr_->log_tag << "] " << msg.toString());
  return true;
}

bool AssistantClient::validate() {

  const int UTC_diff = 7199; // The time difference between the two timestamps (1h59m59s) constant due to SNTP sync.
  const int validation_threshold = 1; // The threshold (in seconds) between send and receive.

  std::int32_t time;
  if (last_recv_msg_->getData("timestamp", time)) {
    if (time - ros::Time::now().toSec() - UTC_diff > validation_threshold ||
        time - ros::Time::now().toSec() - UTC_diff < -validation_threshold) {
      return false;
      ROS_ERROR_STREAM(
          "[" << params_ptr_->log_tag << "] "
              << "Message validation error: Time difference between ASSISTANT CONTROLLER and ROS MASTER is over "
              << validation_threshold);
    } else {
      return true;
    }
  } else {
    return false;
    ROS_ERROR_STREAM("[" << params_ptr_->log_tag << "] "
                         << "Message validation error: ASSISTANT CONTROLLER timestamp could not parsed correctly");
  }
}
void AssistantClient::callback(std::future<void> exit_signal) {
  ROS_INFO_STREAM("[" << params_ptr_->log_tag << "] "
                      << "Receiving callback starting for host ip : " << params_ptr_->server_ip_address << ":"
                      << params_ptr_->server_port);

  // TODO remove try to connect it is not working with the current server
  while (exit_signal.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout) {
    if (!is_connected_) {
      try {
        tcp_client_interface_ptr_.reset(new AssistantTcpClientInterface(*params_ptr_));
        is_connected_ = true;
      } catch (const boost::system::system_error &e) {
        ROS_ERROR_STREAM("[" << params_ptr_->log_tag << "] " << e.what());
        is_connected_ = false;
        size_t timeout = 15;
        ROS_ERROR_STREAM("[" << params_ptr_->log_tag << "] "
                             << "Trying again in " << timeout << " seconds");
        std::this_thread::sleep_for(std::chrono::seconds(timeout));
        continue;
      }
    } else {
      try {
        receive(std::chrono::seconds(1));
        if (!validate()) {
          // TODO uncomment the line below when validate is fully tested
          // is_connected_ = false;
        }
      } catch (const boost::system::system_error &e) {
        ROS_ERROR_STREAM("[" << params_ptr_->log_tag << "] " << e.what());
        is_connected_ = false;
        continue;
      }
    }
  }

  ROS_INFO_STREAM("[" << params_ptr_->log_tag << "] "
                      << "Receiving callback ending ");
}

bool AssistantClient::sendMessage() {
  utils::MessagePackage msg(getRecvOutgoingRecipe());
  // bool com1 = msg.setData("robot_command", RobotCommand::INITIALIZE);
  // bool  com2 = msg.setData("verbose", verb);
  uint32_t timestamp = 10000;
  uint32_t image_size = 30000;
  bool com1 = msg.setData("timestamp", timestamp);
  bool com2 = msg.setData("image_size", image_size);
  if (com1 && com2) {
    return send(msg);
  } else {
    ROS_ERROR_STREAM("[" << params_ptr_->log_tag << "] Send message failed ");
    return false;
  }
}

} // namespace assistant_tcp_interface

PLUGINLIB_EXPORT_CLASS(assistant_tcp_interface::AssistantClient, assistant_tcp_interface::AssistantClientBase)
