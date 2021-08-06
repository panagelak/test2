#pragma once

#include <ros/ros.h>
#include <string>

#include <sl/Camera.hpp>

#include <chrono>
#include <cmath>
#include <compress_depth_image/compress_depth.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

class Publisher {
public:
  Publisher(ros::NodeHandle &nh);
  virtual ~Publisher();

  void Shutdown();
  bool Publish();
  void run();

private:
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
  // std::string SkeletonFrame;
  // ROS publishers
  // ros::Publisher PubSkeletons;
  ros::Publisher PubImage;
  ros::Publisher PubDepthImage;
  ros::Publisher PubDepthCompressImage;
  double frequency_;
  compress_depth_image::CompressDepth depth_image_compressor_;
};
