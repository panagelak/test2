#include <boost/asio.hpp>
#include <iostream>
#include <ros/ros.h>

using namespace boost::asio;
using ip::tcp;
using std::cout;
using std::endl;
using std::string;

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_client", ros::init_options::NoRosout);
  ros::NodeHandle nh("");

  boost::asio::io_service io_service;
  // socket creation
  tcp::socket socket(io_service);
  // connection
  socket.connect(tcp::endpoint(boost::asio::ip::address::from_string("127.0.0.1"), 1240));
  //   const string msg = "Hello from Client!\n";
  string msg = "";
  for (size_t i = 0; i < 2000000; i++) {
    msg += "t";
  }
  msg += "\n";

  while (true) {
    // request/message from client
    ros::Time now = ros::Time::now();

    boost::system::error_code error;
    boost::asio::write(socket, boost::asio::buffer(msg), error);
    if (!error) {
      cout << "Client sent hello message!" << endl;
    } else {
      cout << "send failed: " << error.message() << endl;
    }
    // getting response from server
    boost::asio::streambuf receive_buffer;
    boost::asio::read_until(socket, receive_buffer, "\n");
    string data = boost::asio::buffer_cast<const char *>(receive_buffer.data());
    cout << data << endl;

    ROS_INFO("Delay is : %f", ros::Time::now().toSec() - now.toSec());

    // boost::asio::read(socket, receive_buffer, boost::asio::transfer_all(), error);
    // if (error && error != boost::asio::error::eof) {
    //   cout << "receive failed: " << error.message() << endl;
    // } else {
    //   const char *data = boost::asio::buffer_cast<const char *>(receive_buffer.data());
    //   cout << data << endl;
    // }
  }

  ros::shutdown();
  return 0;
}