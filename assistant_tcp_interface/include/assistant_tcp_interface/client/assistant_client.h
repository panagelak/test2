/**
 * @file assistant_motion_client.h
 * @author LMS ()
 * @brief
 * @version 0.1
 * @date  20-5-2020
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef ASSISTANT_TCP_INTERFACE__ASSISTANT_CLIENT_H
#define ASSISTANT_TCP_INTERFACE__ASSISTANT_CLIENT_H

#include <boost/shared_ptr.hpp>
#include <future>
#include <thread>

#include "assistant_tcp_interface/client/assistant_client_base.h"

namespace assistant_tcp_interface {

class AssistantClient : public AssistantClientBase {
public:
  AssistantClient() {}
  ~AssistantClient();

  /**
   * @brief Read the params to connect with the server and initiates the receiving callback thread
   *
   * @param params AssistantTcpInterfaceParameters for the State Server
   * @return true for success connection
   */
  bool initialize(const AssistantTcpClientInterfaceParameters &params);
  /**
   * @brief Closes the connection with the server
   *
   */
  void close();
  /**
   * @brief Returns the connetion state
   *
   * @return true for connected
   * @return false for not connected
   */
  bool isConnected();
  /**
   * @brief
   *
   * @param msg
   * @return true
   * @return false
   */
  bool getLastMessage(utils::MessagePackage &msg);

  bool sendMessage();

private:
  /**
   * @brief Read the message and parse it into the MessagePackage last_recv_msg_
   *
   * @param timeout  TODO implement the timeout for receive
   */
  void receive(std::chrono::milliseconds timeout);
  /**
   * @brief Send the serialized message package
   *
   * @param msg
   */
  bool send(utils::MessagePackage &msg);
  /**
   * @brief Validate message based on timestamp
   *
   * @param none
   */
  bool validate();
  /**
   * @brief Callback function running in separate thread for non blocking receiving
   *
   * @param exit
   */
  void callback(std::future<void> exit_signal);
  std::promise<void> exit_promise_signal_;          /**< */
  std::future<void> future_obj_for_exit_;           /**< */
  boost::shared_ptr<std::thread> receiving_thread_; /**<  */

  boost::shared_ptr<AssistantTcpClientInterfaceParameters> params_ptr_; /**< private pointer for client parameters*/
  bool is_connected_;                                             /**< private variable holding the connection info*/

  boost::shared_ptr<AssistantTcpClientInterface>
      tcp_client_interface_ptr_; /**< AssistantTcpConnection responsible for the TCP connection */
};

} // namespace assistant_tcp_interface

#endif // ASSISTANT_CLIENT_H
