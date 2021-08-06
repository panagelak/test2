#include <Publisher.hpp>

Publisher::Publisher(ros::NodeHandle &nh) : nh_(nh), frequency_(30.) {

  // Load and apply the configuration of the publishers
  std::string TopicImage = "zed_image", TopicCompressImage = "zed_image/compressed",
              TopicDepthImage = "zed_depth_image", TopicCompressDepthImage = "zed_depth_image/compressed";
  // nh_.param<std::string>("TopicImage", TopicImage, "image");
  // nh_.param<std::string>("TopicDepthImage", TopicImage, "depth_image");
  PubImage = nh_.advertise<sensor_msgs::Image>(TopicImage, 1);
  PubCompressImage = nh_.advertise<sensor_msgs::CompressedImage>(TopicCompressImage, 1);
  PubDepthImage = nh_.advertise<sensor_msgs::Image>(TopicDepthImage, 1);
  PubDepthCompressImage = nh_.advertise<sensor_msgs::CompressedImage>(TopicCompressDepthImage, 1);

  // Load and set the parameters of the camera
  sl::InitParameters ZEDParam;
  int Resolution = 3;
  // nh_.param<int>("Resolution", Resolution, 3); // Default VGA
  ZEDParam.camera_resolution = static_cast<sl::RESOLUTION>(Resolution);
  int Mode = 2;
  // nh_.param<int>("Mode", Mode, 1); // Default Quality
  ZEDParam.depth_mode = static_cast<sl::DEPTH_MODE>(Mode);
  ZEDParam.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
  ZEDParam.coordinate_units = sl::UNIT::METER;

  // Depth Image parameters
  ZEDParam.depth_minimum_distance = 0.25;

  // Open the camera
  sl::ERROR_CODE returned_state = ZED.open(ZEDParam);
  if (returned_state != sl::ERROR_CODE::SUCCESS) {
    ROS_ERROR("Cannot open camera. Make sure another process is not using it.");
    Shutdown();
  }
  //   ZED.setDepthMaxRangeValue(40);

  // Load the image lens parameter
  int LensMode = 0; //, LensDepthMode;
  // nh_.param<int>("LensMode", LensMode, 0);           // Default 0
  // nh_.param<int>("LensDepthMode", LensDepthMode, 9); // Default 0
  Lens = static_cast<sl::VIEW>(LensMode);
  // LensDepth = static_cast<sl::MEASURE>(LensDepthMode);
  // Load the frame parameters
  LensFrame = "zed2_left_camera_frame";
  // nh_.param<std::string>("LensFrame", LensFrame, "zed2_camera_center"); // Default left lens
}

Publisher::~Publisher() {
  Shutdown();
}

void Publisher::run() {
  while (ros::ok()) {
    this->Publish();
    ros::Duration(1. / frequency_).sleep();
  }
}

bool Publisher::Publish() {
  // Grab images
  if (ZED.grab() == sl::ERROR_CODE::SUCCESS) {
    // Retrieve and publish the image
    ros::Time now_retr_img = ros::Time::now();
    ZED.retrieveImage(Image, Lens, sl::MEM::CPU);
    // ROS_INFO("Retreiving Image takes : %f", ros::Time::now().toSec() - now_retr_img.toSec());
    ros::Time now_conv_img = ros::Time::now();
    sensor_msgs::ImagePtr ImgMsg = boost::make_shared<sensor_msgs::Image>();
    ToROSImage(ImgMsg, &Image, LensFrame, ros::Time::now());
    // ROS_INFO("Converting Image takes : %f", ros::Time::now().toSec() - now_conv_img.toSec());
    ros::Time now_pub_img = ros::Time::now();
    PubImage.publish(ImgMsg);
    // ROS_INFO("Publishing Image takes : %f", ros::Time::now().toSec() - now_pub_img.toSec());

    ros::Time now_compress_img = ros::Time::now();
    sensor_msgs::CompressedImage compress_image_msg =
        image_compressor_.encodeImage(*ImgMsg, "jpeg", 80, false, false, 0, 9);
    ROS_INFO("Compressing Image takes : %f", ros::Time::now().toSec() - now_compress_img.toSec());
    ROS_INFO("Original Image size is : %d", ImgMsg->data.size());
    ROS_INFO("Compress Image size is : %d", compress_image_msg.data.size());
    PubCompressImage.publish(compress_image_msg);
    // DEPTH =================
    // Retrieve and publish depth image

    ros::Time now_retr_dimg = ros::Time::now();
    ZED.retrieveMeasure(depth_map, sl::MEASURE::DEPTH, sl::MEM::CPU);
    // ROS_INFO("Retreiving Depth Image takes : %f", ros::Time::now().toSec() - now_retr_dimg.toSec());
    ros::Time now_conv_dimg = ros::Time::now();
    sensor_msgs::ImagePtr DepthImgMsg = boost::make_shared<sensor_msgs::Image>();
    ToROSImage(DepthImgMsg, &depth_map, LensFrame, ros::Time::now());
    // ROS_INFO("Converting Depth Image takes : %f", ros::Time::now().toSec() - now_conv_dimg.toSec());
    ros::Time now_pub_dimg = ros::Time::now();
    PubDepthImage.publish(DepthImgMsg);
    // ROS_INFO("Publishing Depth Image takes : %f", ros::Time::now().toSec() - now_pub_dimg.toSec());

    // compress and publish depth Image
    ros::Time now_compress_dimg = ros::Time::now();
    // sensor_msgs::CompressedImage compress_msg =
    // depth_image_compressor_.encodeDepthImage(*DepthImgMsg, "png", 10., 100., 9);
    sensor_msgs::CompressedImage compress_depth_image_msg =
        depth_image_compressor_.encodeDepthImage(*DepthImgMsg, "png", 5., 40., 1);
    ROS_INFO("Compressing Depth Image takes : %f", ros::Time::now().toSec() - now_compress_dimg.toSec());
    ROS_INFO("Original Depth Image size is : %d", DepthImgMsg->data.size());
    ROS_INFO("Compress Depth Image size is : %d", compress_depth_image_msg.data.size());
    PubDepthCompressImage.publish(compress_depth_image_msg);
    return true;
  }
  return false;
}

void Publisher::Shutdown() {
  // Release objects
  Image.free();
  depth_map.free();
  // Floor.clear();
  // Bodies.object_list.clear();
  // Disable modules
  // ZED.disableObjectDetection();
  // ZED.disablePositionalTracking();
  ZED.close();
  ROS_WARN("Shutting down");
  ros::shutdown();
}

void Publisher::ToROSImage(sensor_msgs::ImagePtr MsgPtr, sl::Mat *Image, std::string Frame, ros::Time t) {
  if (!MsgPtr) {
    return;
  }
  MsgPtr->header.stamp = t;
  MsgPtr->header.frame_id = Frame;
  MsgPtr->height = Image->getHeight();
  MsgPtr->width = Image->getWidth();
  int num = 1; // for endianness detection
  MsgPtr->is_bigendian = !(*(char *)&num == 1);
  MsgPtr->step = Image->getStepBytes(sl::MEM::CPU);
  size_t Size = MsgPtr->step * MsgPtr->height;
  MsgPtr->data.resize(Size);
  sl::MAT_TYPE DataType = Image->getDataType();
  switch (DataType) {
  case sl::MAT_TYPE::F32_C1: /**< float 1 channel.*/
    MsgPtr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    memcpy((char *)(&MsgPtr->data[0]), Image->getPtr<sl::float1>(sl::MEM::CPU), Size);
    break;
  case sl::MAT_TYPE::F32_C2: /**< float 2 channels.*/
    MsgPtr->encoding = sensor_msgs::image_encodings::TYPE_32FC2;
    memcpy((char *)(&MsgPtr->data[0]), Image->getPtr<sl::float2>(sl::MEM::CPU), Size);
    break;
  case sl::MAT_TYPE::F32_C3: /**< float 3 channels.*/
    MsgPtr->encoding = sensor_msgs::image_encodings::TYPE_32FC3;
    memcpy((char *)(&MsgPtr->data[0]), Image->getPtr<sl::float3>(sl::MEM::CPU), Size);
    break;
  case sl::MAT_TYPE::F32_C4: /**< float 4 channels.*/
    MsgPtr->encoding = sensor_msgs::image_encodings::TYPE_32FC4;
    memcpy((char *)(&MsgPtr->data[0]), Image->getPtr<sl::float4>(sl::MEM::CPU), Size);
    break;
  case sl::MAT_TYPE::U8_C1: /**< unsigned char 1 channel.*/
    MsgPtr->encoding = sensor_msgs::image_encodings::MONO8;
    memcpy((char *)(&MsgPtr->data[0]), Image->getPtr<sl::uchar1>(sl::MEM::CPU), Size);
    break;
  case sl::MAT_TYPE::U8_C2: /**< unsigned char 2 channels.*/
    MsgPtr->encoding = sensor_msgs::image_encodings::TYPE_8UC2;
    memcpy((char *)(&MsgPtr->data[0]), Image->getPtr<sl::uchar2>(sl::MEM::CPU), Size);
    break;
  case sl::MAT_TYPE::U8_C3: /**< unsigned char 3 channels.*/
    MsgPtr->encoding = sensor_msgs::image_encodings::BGR8;
    memcpy((char *)(&MsgPtr->data[0]), Image->getPtr<sl::uchar3>(sl::MEM::CPU), Size);
    break;
  case sl::MAT_TYPE::U8_C4: /**< unsigned char 4 channels.*/
    MsgPtr->encoding = sensor_msgs::image_encodings::BGRA8;
    memcpy((char *)(&MsgPtr->data[0]), Image->getPtr<sl::uchar4>(sl::MEM::CPU), Size);
  }
}