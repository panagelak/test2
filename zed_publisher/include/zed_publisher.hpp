#pragma once

#include <ros/ros.h>
#include <string>

#include <chrono>
#include <cmath>
#include <compress_depth_image/compress_depth.h>
#include <compress_image/compress_image.h>
#include <dynamic_reconfigure/server.h>
#include <iostream>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sl/Camera.hpp>
#include <std_msgs/Header.h>
#include <zed_msgs/ZedTransfer.h>
#include <zed_publisher/ZEDPublisherConfig.h>

// Boost headers
// #include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>

class ZEDPublisher {
public:
  ZEDPublisher(ros::NodeHandle &nh);
  virtual ~ZEDPublisher();

  void Shutdown();
  void Publish(const ros::WallTimerEvent &event);

private:
  void Reconfigure(zed_publisher::ZEDPublisherConfig &config, uint32_t level);
  void ToROSImage(sensor_msgs::ImagePtr MsgPtr, sl::Mat *Image, std::string Frame, ros::Time t);
  void retrieveAndCompressImage();
  void retrieveAndCompressDepthImage();

  std::string name_; // Name of this class (for parameter loading)
  ros::NodeHandle nh_;
  // ZED SDK Objects
  sl::Camera ZED;
  sl::Mat Image;
  sl::Mat depth_map;
  // Lens Selection
  sl::VIEW Lens;
  // sl::MEASURE LensDepth;
  // TF Frames
  std::string LensFrame;
  // ROS publishers
  ros::Publisher PubImage;
  ros::Publisher PubCompressImage;
  ros::Publisher PubDepthImage;
  ros::Publisher PubDepthCompressImage;
  ros::Publisher PubTransferCompressCombined;

  zed_msgs::ZedTransfer TransferMsg_;

  compress_depth_image::CompressDepth depth_image_compressor_;
  compress_image::CompressImage image_compressor_;
  double frequency_;
  // zed parametes
  double min_depth_;
  int resolution_;
  int mode_;
  int LensMode_, LensDepthMode_;
  std::string LensFrame_;
  // publisher option
  bool PubOnlyTransfer_;
  bool verbose_img_, verbose_depth_;
  // timer callback
  ros::WallTimer Timer;
  // Initialize dynamic reconfigure
  dynamic_reconfigure::Server<zed_publisher::ZEDPublisherConfig> server_;
  dynamic_reconfigure::Server<zed_publisher::ZEDPublisherConfig>::CallbackType f_;
  // compress rgb and depth image config
  zed_publisher::ZEDPublisherConfig config_;
  // Topic names
  std::string ZedImage_, ZedImageComp_, ZedDepthImage_, ZedDepthImageComp_, ZedTransfer_;
  // image pointers
  sensor_msgs::ImagePtr ImgMsg, DepthImgMsg;
};
