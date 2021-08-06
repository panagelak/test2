#include <zed_publisher.hpp>

ZEDPublisher::ZEDPublisher(ros::NodeHandle &nh) : name_("zed_publisher"), nh_(nh) {

  // Parameters
  // Load parameters
  ros::NodeHandle nh_priv(name_);
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, nh_priv, "Frequency", frequency_);
  // Zed Parameters
  error += !rosparam_shortcuts::get(name_, nh_priv, "Mode", mode_);
  error += !rosparam_shortcuts::get(name_, nh_priv, "LensMode", LensMode_);
  error += !rosparam_shortcuts::get(name_, nh_priv, "LensDepthMode", LensDepthMode_);
  error += !rosparam_shortcuts::get(name_, nh_priv, "LensFrame", LensFrame_);
  error += !rosparam_shortcuts::get(name_, nh_priv, "Resolution", resolution_);
  error += !rosparam_shortcuts::get(name_, nh_priv, "MinimumDepth", min_depth_);
  // Publisher options
  error += !rosparam_shortcuts::get(name_, nh_priv, "PubOnlyTransfer", PubOnlyTransfer_);
  error += !rosparam_shortcuts::get(name_, nh_priv, "Verbose", verbose_);
  // Compression Image parameters
  error += !rosparam_shortcuts::get(name_, nh_priv, "image_format", image_format_);
  error += !rosparam_shortcuts::get(name_, nh_priv, "jpeg_quality", jpeg_quality_);
  error += !rosparam_shortcuts::get(name_, nh_priv, "jpeg_progressive", jpeg_progressive_);
  error += !rosparam_shortcuts::get(name_, nh_priv, "jpeg_optimize", jpeg_optimize_);
  error += !rosparam_shortcuts::get(name_, nh_priv, "jpeg_restart_interval", jpeg_restart_interval_);
  error += !rosparam_shortcuts::get(name_, nh_priv, "image_png_level", image_png_level_);
  // Compression Depth Image parameters
  error += !rosparam_shortcuts::get(name_, nh_priv, "depth_format", depth_format_);
  error += !rosparam_shortcuts::get(name_, nh_priv, "depth_max", depth_max_);
  error += !rosparam_shortcuts::get(name_, nh_priv, "depth_quantization", depth_quantization_);
  error += !rosparam_shortcuts::get(name_, nh_priv, "depth_png_level", depth_png_level_);
  rosparam_shortcuts::shutdownIfError(name_, error);

  // Publishers
  // if (!PubOnlyTransfer_) {
  PubImage = nh_.advertise<sensor_msgs::Image>("zed_image", 1);
  PubCompressImage = nh_.advertise<sensor_msgs::CompressedImage>("zed_image/compressed", 1);
  PubDepthImage = nh_.advertise<sensor_msgs::Image>("zed_depth_image", 1);
  PubDepthCompressImage = nh_.advertise<sensor_msgs::CompressedImage>("zed_depth_image/compressed", 1);
  // }
  PubTransferCompressCombined = nh_.advertise<zed_msgs::ZedTransfer>("zed_transfer", 1);
  // Set the parameters of the camera
  sl::InitParameters ZEDParam;
  ZEDParam.camera_resolution = static_cast<sl::RESOLUTION>(resolution_);
  ZEDParam.depth_mode = static_cast<sl::DEPTH_MODE>(mode_);
  ZEDParam.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
  ZEDParam.coordinate_units = sl::UNIT::METER;
  // Depth Image parameters
  ZEDParam.depth_minimum_distance = min_depth_;

  // Open the camera
  sl::ERROR_CODE returned_state = ZED.open(ZEDParam);
  if (returned_state != sl::ERROR_CODE::SUCCESS) {
    ROS_ERROR("Cannot open camera. Make sure another process is not using it.");
    Shutdown();
  }
  //   ZED.setDepthMaxRangeValue(40);
  Lens = static_cast<sl::VIEW>(LensMode_);
  // LensDepth = static_cast<sl::MEASURE>(LensDepthMode);
  // Load the frame parameters
  // nh_.param<std::string>("LensFrame_", LensFrame_, "zed2_camera_center"); // Default left lens
}

ZEDPublisher::~ZEDPublisher() {
  Shutdown();
}

void ZEDPublisher::run() {
  while (ros::ok()) {
    this->Publish();
    ros::Duration(1. / frequency_).sleep();
  }
}

bool ZEDPublisher::Publish() {
  // Grab images
  if (ZED.grab() == sl::ERROR_CODE::SUCCESS) {
    // Retrieve and publish the image
    // ros::Time now_retr_img = ros::Time::now();
    ZED.retrieveImage(Image, Lens, sl::MEM::CPU);
    // ROS_INFO("Retreiving Image takes : %f", ros::Time::now().toSec() - now_retr_img.toSec());
    // ros::Time now_retr_dimg = ros::Time::now();
    ZED.retrieveMeasure(depth_map, sl::MEASURE::DEPTH, sl::MEM::CPU);
    // ROS_INFO("Retreiving Depth Image takes : %f", ros::Time::now().toSec() - now_retr_dimg.toSec());

    // ros::Time now_conv_img = ros::Time::now();
    sensor_msgs::ImagePtr ImgMsg = boost::make_shared<sensor_msgs::Image>();
    ToROSImage(ImgMsg, &Image, LensFrame_, ros::Time::now());
    // ROS_INFO("Converting Image takes : %f", ros::Time::now().toSec() - now_conv_img.toSec());

    // ros::Time now_conv_dimg = ros::Time::now();
    sensor_msgs::ImagePtr DepthImgMsg = boost::make_shared<sensor_msgs::Image>();
    ToROSImage(DepthImgMsg, &depth_map, LensFrame_, ros::Time::now());
    // ROS_INFO("Converting Depth Image takes : %f", ros::Time::now().toSec() - now_conv_dimg.toSec());

    // Image
    // ros::Time now_compress_img = ros::Time::now();
    TransferMsg_.rgb_image = image_compressor_.encodeImage(*ImgMsg, image_format_, jpeg_quality_, jpeg_progressive_,
                                                           jpeg_optimize_, jpeg_restart_interval_, image_png_level_);
    // ROS_INFO("Compressing Image takes : %f", ros::Time::now().toSec() - now_compress_img.toSec());
    // ROS_INFO("Original Image size is : %d", ImgMsg->data.size());
    // ROS_INFO("Compress Image size is : %d", TransferMsg_.rgb_image.data.size());

    // DEPTH =================
    // compress and publish depth Image
    ros::Time now_compress_dimg = ros::Time::now();
    TransferMsg_.depth_image = depth_image_compressor_.encodeDepthImage(*DepthImgMsg, depth_format_, depth_max_,
                                                                        depth_quantization_, depth_png_level_);
    if (verbose_) {
      ROS_INFO("Compressing Depth Image takes : %f", ros::Time::now().toSec() - now_compress_dimg.toSec());
      ROS_INFO("Original Depth Image size is : %d", DepthImgMsg->data.size());
      ROS_INFO("Compress Depth Image size is : %d", TransferMsg_.depth_image.data.size());
    }

    if (!PubOnlyTransfer_) {
      PubImage.publish(ImgMsg);
      PubDepthImage.publish(DepthImgMsg);
      PubCompressImage.publish(TransferMsg_.rgb_image);
      PubDepthCompressImage.publish(TransferMsg_.depth_image);
    }

    // Publish Transfer Topic
    TransferMsg_.header.frame_id = "";
    TransferMsg_.header.stamp = ros::Time::now();
    PubTransferCompressCombined.publish(TransferMsg_);
    return true;
  }
  return false;
}

void ZEDPublisher::Shutdown() {
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

void ZEDPublisher::ToROSImage(sensor_msgs::ImagePtr MsgPtr, sl::Mat *Image, std::string Frame, ros::Time t) {
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