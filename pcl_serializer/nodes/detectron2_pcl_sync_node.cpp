// This code synchronizes the 3d point cloud and 2D image and publishes 3d locations of human skeletons
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

// #include <pcl/PCLPointCloud2.h>
// #include <pcl/point_types.h>
// #include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/filters/passthrough.h>
// #include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <iostream>
#include <pcl/common/common.h>

#include <chrono>
#include <cmath>
#include <vector>
using namespace std::chrono;

#include <detectron2_ros/GetDetectron.h>
#include <detectron2_ros/Result.h>

// Declare pcl publisher
ros::Publisher pcl_publisher;

// Function to make the average of a vector
double Average(std::vector<double> v) {
  double total = 0.0;
  double size = 0.0;
  for (int n = 0; n < v.size(); n++) {
    total += v[n];
    size++;
  }
  return total / size;
}

// Function to get 3d detections
/*
mediapipe_msgs::BodypartDetection get3dcoordinates(const mediapipe_msgs::BodypartDetection bodypart_2d,
                                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud,
                                                  const std::string bodypart_name) {

  // If detected bodypart
  else {
    uint width = temp_cloud->width;
    uint height = temp_cloud->height;
    // Get keypoint pixel coordinates
    uint x_pixel = uint(bodypart_2d.x * width);
    uint y_pixel = uint(bodypart_2d.y * height);

    // Vector for storing the keypoint index and the surrounding indices ones
    std::vector<unsigned long long int> indices;
    int index = 0;

    // Number of colums and rows of indices surrounding keypoint to get (both must be even)
    int rows = 5;
    int columns = 5;

    // Store in the vector the indices surrounding the keypoint
    for (int i = -(rows - 1) / 2; i <= (rows - 1) / 2; i++) {
      for (int j = -(columns - 1) / 2; j <= (columns - 1) / 2; j++) {
        index = width * (y_pixel + i) + x_pixel + j + 1;
        indices.push_back(index);
      }
    }

    // Vector for storing possible world coordinates of indices in the cluster
    std::vector<double> possible_x;
    std::vector<double> possible_y;
    std::vector<double> possible_z;
    // Get coordinates if are valid

    for (int n = 0; n < indices.size(); n++) {
      if (not std::isnan(temp_cloud->points[indices[n]].x) && not std::isnan(temp_cloud->points[indices[n]].y) &&
          not std::isnan(temp_cloud->points[indices[n]].z) && not std::isinf(temp_cloud->points[indices[n]].x) &&
          not std::isinf(temp_cloud->points[indices[n]].y) && not std::isinf(temp_cloud->points[indices[n]].z)) {

        possible_x.push_back(temp_cloud->points[indices[n]].x);
        possible_y.push_back(temp_cloud->points[indices[n]].y);
        possible_z.push_back(temp_cloud->points[indices[n]].z);
      }
    }


}
*/

pcl::PointCloud<pcl::PointXYZRGBA>
merge_pointclouds(const std::vector<pcl::PointCloud<pcl::PointXYZRGBA>> &segment_clouds) {
  pcl::PointCloud<pcl::PointXYZRGBA> merged_cloud;
  merged_cloud.height = 1;
  merged_cloud.width = 0;
  for (size_t i = 0; i < segment_clouds.size(); i++) {
    merged_cloud.header = segment_clouds[i].header;
    merged_cloud.width += segment_clouds[i].width;
  }
  merged_cloud.points.resize(merged_cloud.width);
  size_t k = 0;
  for (size_t i = 0; i < segment_clouds.size(); i++) {
    for (size_t j = 0; j < segment_clouds[i].width; j++) {
      merged_cloud.points[k].x = segment_clouds[i].points[j].x;
      merged_cloud.points[k].y = segment_clouds[i].points[j].y;
      merged_cloud.points[k].z = segment_clouds[i].points[j].z;
      merged_cloud.points[k].rgba = segment_clouds[i].points[j].rgba;
      k++;
    }
  }
  return merged_cloud;
}

pcl::PointCloud<pcl::PointXYZRGBA> getSegCloudfromMask(const pcl::PointCloud<pcl::PointXYZRGBA> &temp_cloud,
                                                       const sensor_msgs::Image &mask) {
  uint width = mask.width;
  uint height = mask.height;

  pcl::PointCloud<pcl::PointXYZRGBA> seg_cloud;
  seg_cloud.header = temp_cloud.header;
  seg_cloud.height = 1;

  std::size_t num_of_points = 0;
  for (uint i = 0; i < height; i++)
    for (uint j = 0; j < width; j++)
      if (mask.data[i * width + j] == 255)
        num_of_points += 1;

  seg_cloud.width = num_of_points;
  seg_cloud.points.resize(num_of_points);
  std::size_t k = 0;
  for (uint i = 0; i < height; i++)
    for (uint j = 0; j < width; j++)
      if (mask.data[i * width + j] == 255) {
        seg_cloud.points[k].x = temp_cloud.points[i * width + j].x;
        seg_cloud.points[k].y = temp_cloud.points[i * width + j].y;
        seg_cloud.points[k].z = temp_cloud.points[i * width + j].z;
        seg_cloud.points[k].r = temp_cloud.points[i * width + j].r;
        seg_cloud.points[k].g = temp_cloud.points[i * width + j].g;
        seg_cloud.points[k].b = temp_cloud.points[i * width + j].b;
        seg_cloud.points[k].a = temp_cloud.points[i * width + j].a;
        k++;
      }

  return seg_cloud;
}

// std::vector<pcl::PointCloud<pcl::PointXYZRGBA>> getSegmentClouds(const pcl::PointCloud<pcl::PointXYZRGBA> temp_cloud,
//                                                             const std::vector<sensor_msgs::Image> masks) {
//  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>> segment_clouds;
//  for (sensor_msgs::Image mask : masks) {
//    pcl::PointCloud<pcl::PointXYZRGBA> seg_cloud = getSegCloudfromMask(temp_cloud, mask);
//    segment_clouds.push_back(seg_cloud);
//  }
//  return segment_clouds;
//}

// Declare Service Client
ros::ServiceClient client;
detectron2_ros::GetDetectron srv;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;

// Declare Callback
void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg, const sensor_msgs::ImageConstPtr &image_msg) {
  ROS_INFO("Cloud and Image Messages Received!");
  ROS_INFO("    Cloud Time Stamp: %f", cloud_msg->header.stamp.toSec());
  ROS_INFO("    Image Time Stamp: %f", image_msg->header.stamp.toSec());

  // Call Service
  srv.request.with_visualization = true;
  srv.request.input = *image_msg;
  auto start = high_resolution_clock::now();
  bool call_success = client.call(srv);
  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<nanoseconds>(stop - start);
  std::cerr << "FOR SERVICE CALL " << duration.count() << std::endl;
  if (!call_success) {
    ROS_ERROR("Failed to call service ");
    return;
  }
  // Call Successfull

  // Declare and fill pcl<xyz> pointcloud from ros message
  pcl::PointCloud<pcl::PointXYZRGBA> temp_cloud;
  start = high_resolution_clock::now();
  pcl::fromROSMsg(*cloud_msg, temp_cloud);
  stop = high_resolution_clock::now();
  duration = duration_cast<nanoseconds>(stop - start);
  std::cerr << "FOR PCL TRANSFORM " << duration.count() << std::endl;
  static tf2_ros::TransformBroadcaster br_;
  geometry_msgs::TransformStamped transformStamped_;
  transformStamped_.header.stamp = ros::Time::now();
  transformStamped_.header.frame_id = "zed2_left_camera_frame"; // temp_cloud.header.frame_id;
  transformStamped_.child_frame_id = "cologne";
  ROS_INFO("HERE1");
  // segment clouds
  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>> segment_clouds;
  for (sensor_msgs::Image mask : srv.response.detectron_result.masks) {
    pcl::PointCloud<pcl::PointXYZRGBA> seg_cloud = getSegCloudfromMask(temp_cloud, mask);
    segment_clouds.push_back(seg_cloud);
    // get mean point
    double m_x = 0.0, m_y = 0.0, m_z = 0.0;
    double k = 0.0;
    for (auto point : seg_cloud.points) {
      if ((not std::isnan(point.x) && not std::isnan(point.y) &&
          not std::isnan(point.z))  && (not std::isinf(point.x) && not std::isinf(point.y) &&
          not std::isinf(point.z))) {
        // ROS_INFO("HERE2");
        m_x += point.x;
        m_y += point.y;
        m_z += point.z;
        // ROS_INFO("x y z : %f %f %f", m_x, m_y, m_z);
        k++;
      }
    }
    ROS_INFO("x y z : %f %f %f", m_x, m_y, m_z);
    m_x = m_x * (1. / k);
    m_y = m_y * (1. / k);
    m_z = m_z * (1. / k);
    transformStamped_.transform.translation.x = m_x;
    transformStamped_.transform.translation.y = m_y;
    transformStamped_.transform.translation.z = m_z;
    transformStamped_.transform.rotation.x = 0.0;
    transformStamped_.transform.rotation.y = 0.0;
    transformStamped_.transform.rotation.z = 0.0;
    transformStamped_.transform.rotation.w = 1.0;
    br_.sendTransform(transformStamped_);
  }

  // merge
  pcl::PointCloud<pcl::PointXYZRGBA> merged = merge_pointclouds(segment_clouds);

  // result
  sensor_msgs::PointCloud2 result;
  pcl::toROSMsg(merged, result);
  pcl_publisher.publish(result);
}

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "detectron_pcl_sync_node");

  // Declare Node Handle
  ros::NodeHandle nh("");

  // Declare Subscribers
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/input_pointcloud", 1);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/input_image", 1);
  client = nh.serviceClient<detectron2_ros::GetDetectron>("/get_detectron_results");
  client.waitForExistence();

  pcl_publisher = nh.advertise<sensor_msgs::PointCloud2>("/result_cloud", 0, false);
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, image_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  // Spin Forever
  ros::spin();

  return 0;
}