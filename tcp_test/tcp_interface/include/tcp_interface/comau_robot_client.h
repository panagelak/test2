/**
 * @file comau_robot_client.h
 * @author Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 * @brief The ROS node that publishes the robot information
 * @version 0.1
 * @date 25-02-2020
 *
 * @copyright (c) 2020 Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 *
 */

#ifndef COMAU_TCP_INTERFACE__COMAU_ROBOT_CLIENT_H
#define COMAU_TCP_INTERFACE__COMAU_ROBOT_CLIENT_H

#include <boost/shared_ptr.hpp>
#include <future>
#include <thread>

#include "comau_tcp_interface/comau_client_base.h"

namespace comau_tcp_interface {

class RobotClient : public ComauClientBase {
public:
  RobotClient() {}
  ~RobotClient();

  /**
   * @brief Read the params to connect with the server and initiates the receiving callback thread
   *
   * @param params ComauTcpInterfaceParameters for the State Server
   * @return true for success connection
   */
  bool initialize(const ComauTcpInterfaceParameters &params);
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
   * @brief get the last message of the robot
   *
   * @param msg
   * @return true
   * @return false
   */
  bool getLastMessage(utils::MessagePackage &msg);
  /**
   * @brief  Sends the robot to initialize the robot driver parameters
   *
   * @param
   * @return true
   * @return false
   */
  bool sendImageMessage(uint32_t image_size, std::vector<uint8_t> &image_array);

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

  boost::shared_ptr<ComauTcpInterfaceParameters> params_ptr_; /**< private pointer for client parameters*/
  bool is_connected_;                                         /**< private variable holding the connection info*/

  boost::shared_ptr<ComauTcpConnection>
      tcp_interface_ptr_; /**< ComauTcpConnection responsible for the TCP connection */
};

} // namespace comau_tcp_interface

#endif // COMAU_ROBOT_CLIENT_H
