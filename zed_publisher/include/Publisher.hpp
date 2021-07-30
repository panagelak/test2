#pragma once

#include <string>
#include <ros/ros.h>

#include <sl/Camera.hpp>

#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

// #include <zed_human_tracking/Skeleton.h>
// #include <zed_human_tracking/SkeletonArray.h>

class Publisher {
public:
	Publisher(ros::NodeHandle *nh);
	virtual ~Publisher();

	void Shutdown();
	bool Publish();

private:
	void ToROSImage(sensor_msgs::ImagePtr MsgPtr, sl::Mat *Image, std::string Frame, ros::Time t);

	// ZED SDK Objects
	sl::Camera	ZED;
	// sl::Objects Bodies;
	sl::Mat		Image;
	// Floor Plane Detection
	// bool		  NeedsFloor = true;
	// sl::Plane	  Floor;
	// sl::Transform FloorTransform;
	// Detection Parameters
	//sl::ObjectDetectionRuntimeParameters ZedParam;
	// Lens Selection
	sl::VIEW Lens;
	// TF Frames
	std::string LensFrame;
	//std::string SkeletonFrame;
	// ROS publishers
	//ros::Publisher PubSkeletons;
	ros::Publisher PubImage;
};