#include <chrono>
#include <cmath>
#include <compress_depth_image/decompressed_depth.h>
#include <compress_image/decompress_image.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <vector>
#include <zed_msgs/ZedTransfer.h>
#include <zed_msgs/ZedTransferService.h>

// // pcl
// #include <pcl/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <sensor_msgs/PointCloud2.h>

using namespace std::chrono;

class ZedSubscriber {
public:
  ZedSubscriber() : nh_(""), first_(true), got_camera_info_(true), got_msg_(false) {
    // transfer_sub_ = nh_.subscribe("input_zed_transfer", 30, &ZedSubscriber::callback, this);
    server_ = nh_.advertiseService("/zed_transfer_service", &ZedSubscriber::getTransferCB, this);
    im_pub_ = nh_.advertise<sensor_msgs::Image>("image_out", 0, false);
    depth_pub_ = nh_.advertise<sensor_msgs::Image>("depth_out", 0, false);
    camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info_out", 0, false);
    double frequency_ = 15.;
    Timer = nh_.createWallTimer(ros::WallDuration(1. / double(frequency_)), &ZedSubscriber::callback, this);
    // wait for first camera_info msg
    // auto msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/zed2/zed_node/depth/camera_info",
    // ros::Duration(0.0)); camera_info_msg_ = *msg;
  }
  bool getTransferCB(zed_msgs::ZedTransferService::Request &req, zed_msgs::ZedTransferService::Response &res) {
    comp_depth_image_msg_ = req.zed_transfer.depth_image;
    comp_image_msg_ = req.zed_transfer.rgb_image;
    res.success = true;
    got_msg_ = true;
    return res.success;
  }
  // void callback(const zed_msgs::ZedTransfer::ConstPtr &msg) {
  void callback(const ros::WallTimerEvent &event) {
    if (!got_msg_)
      return;
    // ros::Time now = ros::Time::now();
    // ROS_INFO("Network Delay is %f", now.toSec() - msg->header.stamp.toSec());
    // ROS_INFO("Image Delay is %f", now.toSec() - msg->rgb_image.header.stamp.toSec());
    // ROS_INFO("Depth Image Delay is %f", now.toSec() - msg->depth_image.header.stamp.toSec());
    // depth_image_msg_ = depth_image_decompressor_.decodeDepthImage(msg->depth_image);
    // image_msg_ = image_decompressor_.decodeImage(msg->rgb_image, "unchanged");
    depth_image_msg_ = depth_image_decompressor_.decodeDepthImage(comp_depth_image_msg_);
    image_msg_ = image_decompressor_.decodeImage(comp_image_msg_, "unchanged");
    // ROS_INFO("Decompress Delay is %f", ros::Time::now().toSec() - now.toSec());

    if (first_) {
      // Construct Zed Camera Info Message
      camera_info_msg_.header.frame_id = "zed2_left_camera_optical_frame";
      camera_info_msg_.height = image_msg_.height;
      camera_info_msg_.width = image_msg_.width;
      camera_info_msg_.distortion_model = "plumb_bob";
      camera_info_msg_.D = {0.0, 0.0, 0.0, 0.0, 0.0};
      camera_info_msg_.K = {
          262.87078857421875, 0.0, 315.916748046875, 0.0, 262.87078857421875, 183.6972198486328, 0.0, 0.0, 1.0};
      camera_info_msg_.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
      camera_info_msg_.P = {262.87078857421875,
                            0.0,
                            315.916748046875,
                            0.0,
                            0.0,
                            262.87078857421875,
                            183.6972198486328,
                            0.0,
                            0.0,
                            0.0,
                            1.0,
                            0.0};
      camera_info_msg_.binning_x = 0;
      camera_info_msg_.binning_y = 0;
      camera_info_msg_.roi.x_offset = 0;
      camera_info_msg_.roi.y_offset = 0;
      camera_info_msg_.roi.height = 0;
      camera_info_msg_.roi.width = 0;
      camera_info_msg_.roi.do_rectify = false;
      first_ = false;
    }

    image_msg_.header.stamp = ros::Time::now();
    depth_image_msg_.header.stamp = ros::Time::now();
    camera_info_msg_.header.stamp = ros::Time::now();
    camera_info_msg_.header.stamp = depth_image_msg_.header.stamp;
    im_pub_.publish(image_msg_);
    depth_pub_.publish(depth_image_msg_);

    // camera_info_msg_.header.stamp = depth_image_msg_.header.stamp;
    camera_info_pub_.publish(camera_info_msg_);
  }

protected:
  ros::NodeHandle nh_;
  ros::Publisher im_pub_;
  ros::Publisher camera_info_pub_;
  ros::Publisher depth_pub_;
  // ros::Subscriber transfer_sub_;
  ros::ServiceServer server_;
  ros::WallTimer Timer;
  sensor_msgs::Image depth_image_msg_;
  sensor_msgs::Image image_msg_;
  sensor_msgs::CompressedImage comp_depth_image_msg_;
  sensor_msgs::CompressedImage comp_image_msg_;
  sensor_msgs::CameraInfo camera_info_msg_;
  compress_depth_image::DeCompressDepth depth_image_decompressor_;
  decompress_image::DeCompressImage image_decompressor_;
  bool first_, got_camera_info_;
  bool got_msg_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "zed_subscriber");
  ZedSubscriber handler;
  ros::spin();
  return 0;
}