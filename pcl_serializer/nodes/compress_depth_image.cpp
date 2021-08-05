#include <chrono>
#include <cmath>
#include <compress_depth_image/compress_depth.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <vector>

using namespace std::chrono;

class CompressDeptImagePub {
public:
  CompressDeptImagePub() : nh_("") {

    depth_image_sub_ =
        nh_.subscribe("/camera/depth_registered/image_raw", 1, &CompressDeptImagePub::depth_image_callback, this);
    depth_compress_pub_ = nh_.advertise<sensor_msgs::CompressedImage>("/my_depth_image/compressed", 1, true);
  }
  void depth_image_callback(const sensor_msgs::Image::ConstPtr &msg) {
    sensor_msgs::CompressedImage compress_msg = depth_image_compressor_.encodeDepthImage(*msg, "png", 10., 100., 9);
    ROS_INFO("Original size is : %d", msg->data.size());
    ROS_INFO("Compressed size is : %d", compress_msg.data.size());
    ROS_INFO("Format is : %s", compress_msg.format.c_str());
    depth_compress_pub_.publish(compress_msg);
  }

protected:
  ros::NodeHandle nh_;
  ros::Publisher depth_compress_pub_;
  ros::Subscriber depth_image_sub_;
  compress_depth_image::CompressDepth depth_image_compressor_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "compress_depth_image");
  CompressDeptImagePub handler;
  ros::spin();
  return 0;
}
