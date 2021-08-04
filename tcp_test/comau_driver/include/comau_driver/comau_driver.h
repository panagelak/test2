/**
 * @file comau_driver.h
 * @author Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 * @brief The ROS node that publishes the robot information
 * @version 0.1
 * @date 25-02-2020
 *
 * @copyright (c) 2020 Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 *
 */

#ifndef COMAU_DRIVER_H
#define COMAU_DRIVER_H

#include "comau_tcp_interface/comau_robot_client.h"
#include "comau_tcp_interface/comau_state_client.h"
#include "comau_tcp_interface/comau_tcp_interface.h"
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>

using namespace comau_tcp_interface;
using namespace comau_tcp_interface::utils;

namespace comau_driver {
/**
 * @brief This is the main class for interfacing the Robot controller.
 * It sets up all the neacessary socket connections and handles the data exchange with the robot.
 */
class ComauRobot {
public:
  /**
   * @brief Construct a new Comau Robot object
   * @param nh global ROS NodeHandle
   */
  ComauRobot(ros::NodeHandle &nh);
  /**
   * @brief Destroy the Comau Robot object
   */
  ~ComauRobot();
  /**
   * @brief Initializes the robot driver
   * @param is_receiver if we are using the state server
   * @return true If comau robot initialized correctly
   */
  bool initialize(std::string ip_address, std::string port, std::string log_tag, bool is_receiver);

  // ================ READ =================
  /**
   * @brief Getter for the robot Message Package
   * @return true
   * @return false
   */
  bool readMessagePackage();

  void getImage(uint32_t image_size, std::vector<uint8_t> &image_array);

  bool writeImage(uint32_t image_size, std::vector<uint8_t> &image_array);


private:
  // Name of the driver
  std::string name_;              /**< Name of this class -> comau_driver */
  ros::NodeHandle nh_; /**< ROS NodeHandle objects required for parameters reading */

  boost::shared_ptr<comau_tcp_interface::StateClient> state_client_ptr_; /**< StateClient object */
  boost::shared_ptr<comau_tcp_interface::RobotClient> robot_client_ptr_; /**< RobotClient object */
  comau_tcp_interface::utils::MessagePackage *msg;                       /**< State message */
  comau_tcp_interface::ComauTcpInterfaceParameters state_params_, robot_params_; /**< Net parameters */
  // State parameters
  comau_tcp_interface::utils::vectorImage_t image_array_;
  uint32_t image_size_;
  // comau_tcp_interface::utils::vector6i_t pins_state_out_;     
  // std::vector<int> din_pins_;                                  /**< DIN pins */

  bool is_receiver_;
};

} // namespace comau_driver

#endif // COMAU_DRIVER_H
