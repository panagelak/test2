/**
 * @file assistant_tcp_interface.cpp
 * @author LMS ()
 * @brief
 * @version 0.1
 * @date 15-02-2020
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <assistant_tcp_interface/server/assistant_tcp_server_interface.h>

using namespace assistant_tcp_interface::utils;
using namespace boost::asio::ip;

namespace assistant_tcp_interface {

AssistantTcpServerInterface::AssistantTcpServerInterface(const AssistantTcpServerInterfaceParameters &params)
    : params_(params) {
  acceptServer();
}

AssistantTcpServerInterface::~AssistantTcpServerInterface() {
  if (socket_ptr_) {
    socket_ptr_->close();
  }
  if (io_service_ptr_) {
    io_service_ptr_->reset();
  }
  if (acceptor_ptr_) {
    acceptor_ptr_->close();
  }
}

void AssistantTcpServerInterface::acceptServer() {
  ROS_INFO_STREAM("[" << params_.log_tag << "] Connecting to " << params_.server_ip_address << ":"
                      << params_.server_port);
  try {
    io_service_ptr_.reset(new boost::asio::io_service());
    tcp::resolver resolver(*io_service_ptr_);
    tcp::resolver::query query(params_.server_ip_address, params_.server_port);
    tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

    socket_ptr_.reset(new tcp::socket(*io_service_ptr_));
    // socket_ptr_->open(tcp::v4());
    // boost::asio::connect(*socket_ptr_, endpoint_iterator);
    // intantiate acceptor pointer at 1234 port
    acceptor_ptr_.reset(new tcp::acceptor(*io_service_ptr_, tcp::v4(), 1234));

    // start accepting
    start_accept();

  } catch (const boost::system::system_error &e) {
    // leave the treatment to the initialize function of each client class
    throw e;
  }
}
void AssistantTcpServerInterface::start_accept() {
  try {
    acceptor_ptr_->accept(*socket_ptr_);

    // acceptor_ptr_->async_accept(
    // *socket_ptr_, boost::bind(&AssistantTcpServerInterface::handle_accept, this, boost::asio::placeholders::error));
  } catch (const boost::system::system_error &e) {
    // leave the treatment to the initialize function of each client class
    throw e;
  }
}
void AssistantTcpServerInterface::handle_accept(const boost::system::error_code &err) {
  ;
}

void AssistantTcpServerInterface::read(std::vector<uint8_t> &buf, const size_t buf_len, size_t &read) {
  boost::system::error_code error;
  read = socket_ptr_->read_some(boost::asio::buffer(buf), error);
  ROS_DEBUG_STREAM(params_.log_tag << "Read from socket " << read << " bytes");
  if (error) {
    throw boost::system::system_error(error);
  }
}

void AssistantTcpServerInterface::write(const std::vector<uint8_t> &buf, const size_t buf_len, size_t &written) {
  boost::system::error_code error;
  written = socket_ptr_->write_some(boost::asio::buffer(buf), error);

  if (error) {
    throw boost::system::system_error(error);
  }
}

} // namespace assistant_tcp_interface
