/**
 * @file assistant_server_base.h
 * @author LMS ()
 * @brief
 * @version 0.1
 * @date 29-04-2020
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef ASSISTANT_TCP_INTERFACE__ASSISTANT_SERVER_BASE_H
#define ASSISTANT_TCP_INTERFACE__ASSISTANT_SERVER_BASE_H

#include "assistant_tcp_interface/server/assistant_tcp_server_interface.h"
#include "assistant_tcp_interface/utils/custom_data_type.h"
#include "assistant_tcp_interface/utils/message_package.h"

namespace assistant_tcp_interface {

class AssistantServerBase {

protected:
  AssistantServerBase() {}

public:
  virtual ~AssistantServerBase() {}

  virtual bool initialize(const AssistantTcpServerInterfaceParameters &params) = 0;
  virtual void close() = 0;
  virtual bool isConnected() = 0;
  virtual bool getLastMessage(utils::MessagePackage &msg) = 0;
  utils::vectorstr_t getRecvIncomingRecipe() {
    return incoming_msg_descr_;
  }
  utils::vectorstr_t getRecvOutgoingRecipe() {
    return outgoing_msg_descr_;
  }

protected:
  virtual void receive(std::chrono::milliseconds timeout) = 0;
  virtual bool send(utils::MessagePackage &msg) = 0;

  std::unique_ptr<utils::MessagePackage> last_recv_msg_;
  std::vector<std::string> incoming_msg_descr_;
  std::vector<std::string> outgoing_msg_descr_;
};

} // namespace assistant_tcp_interface

#endif
