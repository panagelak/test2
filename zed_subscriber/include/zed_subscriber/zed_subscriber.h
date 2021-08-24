#pragma once

#include <chrono>
#include <cmath>
#include <compress_depth_image/decompressed_depth.h>
#include <compress_image/decompress_image.h>
#include <depth_image_to_pc2/point_cloud_xyz.h>
#include <iostream>
#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <vector>
#include <zed_msgs/ZedTransfer.h>
#include <zed_msgs/ZedTransferService.h>
// // pcl
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <zed_msgs/HtpInput.h>

using namespace std::chrono;

class ZedSubscriber {
public:
  ZedSubscriber();

protected:
  void getTransferCB(const zed_msgs::ZedTransfer::ConstPtr &msg);
  void callback(const ros::WallTimerEvent &event);
  std::string name_;
  ros::NodeHandle nh_;

  ros::Publisher im_pub_;
  ros::Publisher camera_info_pub_;
  ros::Publisher depth_pub_;
  ros::Publisher pcl_pub_;
  ros::Publisher htp_input_pub_;
  ros::Subscriber sub_;
  // ros::ServiceServer server_;
  ros::WallTimer Timer;

  sensor_msgs::Image depth_image_msg_;
  sensor_msgs::Image image_msg_;
  sensor_msgs::PointCloud2 pcl_xyz_msg_;
  sensor_msgs::CompressedImage comp_depth_image_msg_;
  sensor_msgs::CompressedImage comp_image_msg_;
  std_msgs::Header transfer_header_;
  sensor_msgs::CameraInfo camera_info_msg_;
  compress_depth_image::DeCompressDepth depth_image_decompressor_;
  decompress_image::DeCompressImage image_decompressor_;
  depth_image_to_pc2::PointCloudXyz pcl_xyz_proc_;
  bool got_msg_, verbose_;
  bool first_;
  ros::Time last_;
  // htp input
  zed_msgs::HtpInput htp_input_;
};