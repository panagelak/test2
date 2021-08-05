#include <chrono>
#include <cmath>
#include <compress_depth_image/decompressed_depth.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <vector>

using namespace std::chrono;

class DeCompressDeptImagePub {
public:
  DeCompressDeptImagePub() : nh_("") {

    depth_compress_sub_ =
        nh_.subscribe("/my_depth_image/compressed", 1, &DeCompressDeptImagePub::depth_image_callback, this);
    depth_image_pub_ = nh_.advertise<sensor_msgs::Image>("/my_depth_image", 1, true);
  }
  void depth_image_callback(const sensor_msgs::CompressedImage::ConstPtr &msg) {
    sensor_msgs::Image depth_image_msg = depth_image_decompressor_.decodeDepthImage(*msg);
    ROS_INFO("Compressed size is : %d", msg->data.size());
    ROS_INFO("frame is : %s", depth_image_msg.header.frame_id.c_str());
    ROS_INFO("encoding is : %s", depth_image_msg.encoding.c_str());
    ROS_INFO("height is : %d", depth_image_msg.height);
    ROS_INFO("width is : %d", depth_image_msg.width);
    ROS_INFO("step is : %d", depth_image_msg.step);
    ROS_INFO("bigedian is : %d", depth_image_msg.is_bigendian);

    // ROS_INFO("Decompress is : %s", depth_image_msg.data.size());
    depth_image_pub_.publish(depth_image_msg);
  }

protected:
  ros::NodeHandle nh_;
  ros::Subscriber depth_compress_sub_;
  ros::Publisher depth_image_pub_;
  compress_depth_image::DeCompressDepth depth_image_decompressor_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "decompress_depth_image");
  DeCompressDeptImagePub handler;
  ros::spin();
  return 0;
}
