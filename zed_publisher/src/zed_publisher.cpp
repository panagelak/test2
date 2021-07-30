#include "Publisher.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "zed_publisher");
    ros::NodeHandle nh("zed_publisher");

    // Set the ROS logging level (for better debugging)
    // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
    //     ros::console::notifyLoggerLevelsChanged();
    // }

    ROS_INFO("ZED Publisher");

    // Create the tracker objects
    Publisher ZEDPublisher(&nh);

    int Frequency;
    nh.param<int>("Frequency", Frequency, 30);
    ros::Rate rate(Frequency);

    while (ros::ok()) {
        ZEDPublisher.Publish();
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Exiting.");
    return 0;
}