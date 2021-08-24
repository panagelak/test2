#include <zed_subscriber/zed_subscriber.h>
using namespace std::chrono;

ZedSubscriber::ZedSubscriber() : name_("zed_subscriber"), nh_(""), got_msg_(false), first_(true) {
  // Parameters
  // Load parameters
  ros::NodeHandle nh_priv(name_);
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, nh_priv, "Verbose", verbose_);
  rosparam_shortcuts::shutdownIfError(name_, error);

  server_ = nh_.advertiseService("input_zed_transfer", &ZedSubscriber::getTransferCB, this);
  im_pub_ = nh_.advertise<sensor_msgs::Image>("image_out", 0, false);
  depth_pub_ = nh_.advertise<sensor_msgs::Image>("depth_out", 0, false);
  camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info_out", 0, false);
  pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_out", 0, false);
  htp_input_pub_ = nh_.advertise<zed_msgs::HtpInput>("htp_input", 0, false);
  double frequency_ = 15.;
  Timer = nh_.createWallTimer(ros::WallDuration(1. / double(frequency_)), &ZedSubscriber::callback, this);
  last_ = ros::Time::now();

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
}

bool ZedSubscriber::getTransferCB(zed_msgs::ZedTransferService::Request &req,
                                  zed_msgs::ZedTransferService::Response &res) {
  transfer_header_ = req.zed_transfer.header;
  comp_depth_image_msg_ = req.zed_transfer.depth_image;
  comp_image_msg_ = req.zed_transfer.rgb_image;
  res.success = true;
  got_msg_ = true;
  // ROS_INFO("Service delay is : %f", ros::Time::now().toSec() - last_.toSec());
  last_ = ros::Time::now();
  return res.success;
}
void ZedSubscriber::callback(const ros::WallTimerEvent &event) {
  if (!got_msg_)
    return;
  if (verbose_) {
    ros::Time now = ros::Time::now();
    ROS_INFO("Network Delay is %f", now.toSec() - transfer_header_.stamp.toSec());
    ROS_INFO("Image Delay is %f", now.toSec() - comp_image_msg_.header.stamp.toSec());
    ROS_INFO("Depth Image Delay is %f", now.toSec() - comp_depth_image_msg_.header.stamp.toSec());
  }
  depth_image_msg_ = depth_image_decompressor_.decodeDepthImage(comp_depth_image_msg_);
  image_msg_ = image_decompressor_.decodeImage(comp_image_msg_, "unchanged");
  // ROS_INFO("Decompress Delay is %f", ros::Time::now().toSec() - now.toSec());

  image_msg_.header.stamp = ros::Time::now();
  depth_image_msg_.header.stamp = ros::Time::now();
  camera_info_msg_.header.stamp = ros::Time::now();
  sensor_msgs::ImageConstPtr depth_ptr_(new sensor_msgs::Image(depth_image_msg_));
  sensor_msgs::CameraInfoConstPtr cam_info_ptr_(new sensor_msgs::CameraInfo(camera_info_msg_));

  // pcl pointcloud
  pcl_xyz_msg_ = pcl_xyz_proc_.getPointCloudXyz(depth_ptr_, cam_info_ptr_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(pcl_xyz_msg_, *pcl_cloud);

  // htp input
  htp_input_.header = transfer_header_;
  htp_input_.image = image_msg_;
  if (first_) {
    htp_input_.point_cloud.height = pcl_cloud->height;
    htp_input_.point_cloud.width = pcl_cloud->width;
    htp_input_.point_cloud.points.resize(pcl_cloud->points.size());
    first_ = false;
  }
  for (size_t i = 0; i < pcl_cloud->points.size(); i++) {
    htp_input_.point_cloud.points[i].x = pcl_cloud->points[i].x;
    htp_input_.point_cloud.points[i].y = pcl_cloud->points[i].y;
    htp_input_.point_cloud.points[i].z = pcl_cloud->points[i].z;
  }

  im_pub_.publish(image_msg_);
  depth_pub_.publish(depth_image_msg_);
  camera_info_pub_.publish(camera_info_msg_);
  pcl_pub_.publish(pcl_xyz_msg_);

  htp_input_pub_.publish(htp_input_);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "zed_subscriber");
  ZedSubscriber handler;
  ros::spin();
  return 0;
}