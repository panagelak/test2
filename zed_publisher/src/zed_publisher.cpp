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
  error += !rosparam_shortcuts::get(name_, nh_priv, "VerboseImg", verbose_img_);
  error += !rosparam_shortcuts::get(name_, nh_priv, "VerboseDepth", verbose_depth_);
  // Topic names
  error += !rosparam_shortcuts::get(name_, nh_priv, "ZedImage", ZedImage_);
  error += !rosparam_shortcuts::get(name_, nh_priv, "ZedImageComp", ZedImageComp_);
  error += !rosparam_shortcuts::get(name_, nh_priv, "ZedDepthImage", ZedDepthImage_);
  error += !rosparam_shortcuts::get(name_, nh_priv, "ZedDepthImageComp", ZedDepthImageComp_);
  error += !rosparam_shortcuts::get(name_, nh_priv, "ZedTransfer", ZedTransfer_);

  rosparam_shortcuts::shutdownIfError(name_, error);

  // Publishers
  if (!PubOnlyTransfer_) {
    PubImage = nh_.advertise<sensor_msgs::Image>(ZedImage_, 1);
    PubCompressImage = nh_.advertise<sensor_msgs::CompressedImage>(ZedImageComp_, 1);
    PubDepthImage = nh_.advertise<sensor_msgs::Image>(ZedDepthImage_, 1);
    PubDepthCompressImage = nh_.advertise<sensor_msgs::CompressedImage>(ZedDepthImageComp_, 1);
  }
  PubTransferCompressCombined = nh_.advertise<zed_msgs::ZedTransfer>(ZedTransfer_, 1);
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

  // Enable the dynamic reconfigure
  f_ = boost::bind(&ZEDPublisher::Reconfigure, this, _1, _2);
  server_.setCallback(f_);

  // Create Timer Callback
  Timer = nh_priv.createWallTimer(ros::WallDuration(1. / double(frequency_)), &ZEDPublisher::Publish, this);
  DepthImgMsg = boost::make_shared<sensor_msgs::Image>();
  ImgMsg = boost::make_shared<sensor_msgs::Image>();
}

ZEDPublisher::~ZEDPublisher() {
  Shutdown();
}

void ZEDPublisher::retrieveAndCompressImage() {

  // === Retrieving
  // ros::Time now_retr_img = ros::Time::now();
  ZED.retrieveImage(Image, Lens, sl::MEM::CPU);
  // ros::Time end_retr_img = ros::Time::now();

  // === Converting to ros message
  ToROSImage(ImgMsg, &Image, LensFrame_, ros::Time::now());

  // === Compressing Image
  ros::Time now_compress_img = ros::Time::now();
  TransferMsg_.rgb_image = image_compressor_.encodeImage(*ImgMsg, config_.img_format, config_.img_jpeg_quality,
                                                         config_.img_jpeg_progressive, config_.img_jpeg_optimize,
                                                         config_.img_jpeg_restart_interval, config_.img_png_level);
  if (verbose_img_)
    ROS_INFO("Image -> Compr Time : %f Size : %d", ros::Time::now().toSec() - now_compress_img.toSec(),
             TransferMsg_.rgb_image.data.size());
}
void ZEDPublisher::retrieveAndCompressDepthImage() {

  // === Retrieving
  // ros::Time now_retr_dimg = ros::Time::now();
  ZED.retrieveMeasure(depth_map, sl::MEASURE::DEPTH, sl::MEM::CPU);
  // ros::Time end_retr_dimg = ros::Time::now();

  // === Converting to ros message
  ToROSImage(DepthImgMsg, &depth_map, LensFrame_, ros::Time::now());

  // === Compressing Depth Image
  ros::Time now_compress_dimg = ros::Time::now();
  TransferMsg_.depth_image = depth_image_compressor_.encodeDepthImage(
      *DepthImgMsg, config_.depth_format, config_.depth_max, config_.depth_quantization, config_.depth_png_level);
  if (verbose_depth_)
    ROS_INFO("Depth -> Compr Time : %f Size : %d", ros::Time::now().toSec() - now_compress_dimg.toSec(),
             TransferMsg_.depth_image.data.size());
}

void ZEDPublisher::Publish(const ros::WallTimerEvent &event) {
  // Grab images
  if (ZED.grab() == sl::ERROR_CODE::SUCCESS) {

    boost::thread retrieve_img_thread(&ZEDPublisher::retrieveAndCompressImage, this);
    boost::thread retrieve_depth_img_thread(&ZEDPublisher::retrieveAndCompressDepthImage, this);
    retrieve_img_thread.join();
    retrieve_depth_img_thread.join();

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
  }
}

void ZEDPublisher::Shutdown() {
  // Release objects
  Image.free();
  depth_map.free();
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

void ZEDPublisher::Reconfigure(zed_publisher::ZEDPublisherConfig &config, uint32_t level) {
  config_ = config;
}