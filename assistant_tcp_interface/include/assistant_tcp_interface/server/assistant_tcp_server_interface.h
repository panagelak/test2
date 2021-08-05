/**
 * @file assistant_tcp_interface.h
 * @author LMS ()
 * @brief
 * @version 0.1
 * @date 15-02-2020
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef ASSISTANT_TCP_SERVER_INTERFACE_H
#define ASSISTANT_TCP_SERVER_INTERFACE_H

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <chrono>
#include <future>
#include <string>
#include <thread>

#include <ros/ros.h>

#include <assistant_tcp_interface/utils/message_parser.h>

namespace assistant_tcp_interface {

/**
 * @brief This is the structure that contains all the parameters that are used from
 * AssistantTcpInterface class
 *
 */
struct AssistantTcpServerInterfaceParameters {
  std::string server_ip_address; /**< Address of the Server that will be connected */
  std::string server_port;       /**< Port of the Server that will be connected */
  std::string log_tag;           /**< Tag that presented at output stream in brackets []*/
};

/**
 * @brief This is the main class that handles the TCP connection with the ASSISTANT C5G contoller
 *
 */
class AssistantTcpServerInterface {
public:
  /**
   * @brief Construct a new Assistant Tcp Interface object
   *
   * @param params as defined in struct AssistantTcpServerInterfaceParameters
   */
  AssistantTcpServerInterface(const AssistantTcpServerInterfaceParameters &params);
  /**
   * @brief Destroy the Assistant Tcp Interface object
   *
   */
  ~AssistantTcpServerInterface();

  /**
   * @brief The main function that connects with the PDL TCP server using the AssistantTcpServerInterfaceParameters
   *
   */
  void acceptServer();

  /**
   * @brief Reads data from the socket
   *
   * @param buf [out] buf Buffer where the data shall be stored
   * @param buf_len [in] buf_len Number of bytes in the buffer
   * @param read  [out] read Number of bytes actually read
   */
  void read(std::vector<uint8_t> &buf, const size_t buf_len, size_t &read);

  /**
   * @brief Writes a buffer to the TCP socket
   *
   * @param buf The buffer to write from
   * @param buf_len The length to write
   * @param written A reference used to indicate how many bytes were written
   */
  void write(const std::vector<uint8_t> &buf, const size_t buf_len, size_t &written);

  void start_accept();
  void handle_accept(const boost::system::error_code &err);

private:
  const AssistantTcpServerInterfaceParameters &params_; /**< private variable for parameters*/

  boost::shared_ptr<boost::asio::io_service>
      io_service_ptr_; /**< The io_context class provides the core I/O functionality for users of the asynchronous I/O
                          objects */
  boost::shared_ptr<boost::asio::ip::tcp::socket>
      socket_ptr_; /**< The basic_stream_socket class template provides asynchronous and blocking stream-oriented socket
                      functionality */
  boost::shared_ptr<boost::asio::ip::tcp::acceptor> acceptor_ptr_; /**< Acceptor connection from client */
};

} // namespace assistant_tcp_interface

#endif // ASSISTANT_TCP_SERVER_INTERFACE_H
