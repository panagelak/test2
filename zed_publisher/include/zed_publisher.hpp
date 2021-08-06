#pragma once

#include <ros/ros.h>
#include <string>

#include <chrono>
#include <cmath>
#include <compress_depth_image/compress_depth.h>
#include <compress_image/compress_image.h>
#include <iostream>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sl/Camera.hpp>
#include <std_msgs/Header.h>
#include <zed_msgs/ZedTransfer.h>

class ZEDPublisher {
public:
  ZEDPublisher(ros::NodeHandle &nh);
  virtual ~ZEDPublisher();

  void Shutdown();
  bool Publish();
  void run();

private:
  std::string name_; // Name of this class (for parameter loading)
  void ToROSImage(sensor_msgs::ImagePtr MsgPtr, sl::Mat *Image, std::string Frame, ros::Time t);
  ros::NodeHandle nh_;
  // ZED SDK Objects
  sl::Camera ZED;
  // sl::Objects Bodies;
  sl::Mat Image;
  sl::Mat depth_map;
  // Floor Plane Detection
  // bool		  NeedsFloor = true;
  // sl::Plane	  Floor;
  // sl::Transform FloorTransform;
  // Detection Parameters
  // sl::ObjectDetectionRuntimeParameters ZedParam;
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
  bool verbose_;
  // compression image parameters
  std::string image_format_;
  int jpeg_quality_;
  bool jpeg_progressive_;
  bool jpeg_optimize_;
  int jpeg_restart_interval_;
  int image_png_level_;
  // compression depth image parameters
  std::string depth_format_;
  double depth_max_;
  double depth_quantization_;
  int depth_png_level_;
};
