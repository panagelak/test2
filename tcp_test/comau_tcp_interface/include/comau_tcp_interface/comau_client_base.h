/**
 * @file comau_client_base.h
 * @author Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 * @brief The ROS node that publishes the robot information
 * @version 0.1
 * @date 25-02-2020
 *
 * @copyright (c) 2020 Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 *
 */

#ifndef COMAU_TCP_INTERFACE__COMAU_CLIENT_BASE_H
#define COMAU_TCP_INTERFACE__COMAU_CLIENT_BASE_H

#include "comau_tcp_interface/comau_tcp_interface.h"
#include "comau_tcp_interface/utils/custom_data_type.h"
#include "comau_tcp_interface/utils/message_package.h"

namespace comau_tcp_interface {

class ComauClientBase {

protected:
  ComauClientBase() {}

public:
  virtual ~ComauClientBase() {}

  virtual bool initialize(const ComauTcpInterfaceParameters &params) = 0;
  virtual void close() = 0;
  virtual bool isConnected() = 0;
  virtual bool getLastMessage(utils::MessagePackage &msg) = 0;
  utils::vectorstr_t getRecvRecipe() {
    return incoming_msg_descr_;
  }
  utils::vectorstr_t getSendImageRecipe() {
    return image_msg_descr_;
  }
  utils::vectorstr_t getSendIncomingRecipe() {
    return incoming_msg_descr_;
  }

protected:
  virtual void receive(std::chrono::milliseconds timeout) = 0;
  virtual bool send(utils::MessagePackage &msg) = 0;

  std::unique_ptr<utils::MessagePackage> last_recv_msg_;
  std::vector<std::string> incoming_msg_descr_;
  std::vector<std::string> incoming_msg_motion_descr_;
  std::vector<std::string> image_msg_descr_;
};

} // namespace comau_tcp_interface

#endif
