#include <boost/asio.hpp>
#include <iostream>
#include <ros/ros.h>

using namespace boost::asio;
using ip::tcp;
using std::cout;
using std::endl;
using std::string;

string read_(tcp::socket &socket) {
  boost::asio::streambuf buf;
  boost::asio::read_until(socket, buf, "\n");
  string data = boost::asio::buffer_cast<const char *>(buf.data());
  return data;
}
void send_(tcp::socket &socket, const string &message) {
  const string msg = message + "\n";
  boost::asio::write(socket, boost::asio::buffer(message));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_server", ros::init_options::NoRosout);
  ros::NodeHandle nh("");

  boost::asio::io_service io_service;
  // listen for new connection
  tcp::acceptor acceptor_(io_service, tcp::endpoint(tcp::v4(), 1244));
  // socket creation
  tcp::socket socket_(io_service);
  // waiting for connection
  acceptor_.accept(socket_);
  while (true) {

    // read operation
    string message = read_(socket_);
    cout << message.size() << endl;
    // write operation
    send_(socket_, "OK From Server!\n");
    // cout << "Servent sent Hello message to Client!" << endl;
  }

  ros::shutdown();
  return 0;
}