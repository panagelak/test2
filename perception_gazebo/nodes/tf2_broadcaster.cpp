#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

class Tf2Handler {

public:
  Tf2Handler() {
    cylinder1_sub_ = nh_.subscribe("cylinder_1/odom", 10, &Tf2Handler::Cylinder1OdomCB, this);
    cylinder2_sub_ = nh_.subscribe("cylinder_2/odom", 10, &Tf2Handler::Cylinder2OdomCB, this);
    cylinder3_sub_ = nh_.subscribe("cylinder_3/odom", 10, &Tf2Handler::Cylinder3OdomCB, this);
    cylinder4_sub_ = nh_.subscribe("cylinder_4/odom", 10, &Tf2Handler::Cylinder4OdomCB, this);
    cylinder5_sub_ = nh_.subscribe("cylinder_5/odom", 10, &Tf2Handler::Cylinder5OdomCB, this);
    cylinder6_sub_ = nh_.subscribe("cylinder_6/odom", 10, &Tf2Handler::Cylinder6OdomCB, this);
    cylinder7_sub_ = nh_.subscribe("cylinder_7/odom", 10, &Tf2Handler::Cylinder7OdomCB, this);
    cylinder8_sub_ = nh_.subscribe("cylinder_8/odom", 10, &Tf2Handler::Cylinder8OdomCB, this);
    cylinder9_sub_ = nh_.subscribe("cylinder_9/odom", 10, &Tf2Handler::Cylinder9OdomCB, this);
    cylinder_base_sub_ = nh_.subscribe("cylinder_base/odom", 10, &Tf2Handler::CylinderBaseOdomCB, this);
    timerPublish_ = nh_.createTimer(ros::Duration(0.05), &Tf2Handler::update, this);
  }
  void Cylinder1OdomCB(const nav_msgs::Odometry::ConstPtr &msg) {
    cylinder1_odom_ = msg;
    first_1 = true;
  }
  void Cylinder2OdomCB(const nav_msgs::Odometry::ConstPtr &msg) {
    cylinder2_odom_ = msg;
    first_2 = true;
  }
  void Cylinder3OdomCB(const nav_msgs::Odometry::ConstPtr &msg) {
    cylinder3_odom_ = msg;
    first_3 = true;
  }
  void Cylinder4OdomCB(const nav_msgs::Odometry::ConstPtr &msg) {
    cylinder4_odom_ = msg;
    first_4 = true;
  }
  void Cylinder5OdomCB(const nav_msgs::Odometry::ConstPtr &msg) {
    cylinder5_odom_ = msg;
    first_5 = true;
  }
  void Cylinder6OdomCB(const nav_msgs::Odometry::ConstPtr &msg) {
    cylinder6_odom_ = msg;
    first_6 = true;
  }
  void Cylinder7OdomCB(const nav_msgs::Odometry::ConstPtr &msg) {
    cylinder7_odom_ = msg;
    first_7 = true;
  }
  void Cylinder8OdomCB(const nav_msgs::Odometry::ConstPtr &msg) {
    cylinder8_odom_ = msg;
    first_8 = true;
  }
  void Cylinder9OdomCB(const nav_msgs::Odometry::ConstPtr &msg) {
    cylinder9_odom_ = msg;
    first_9 = true;
  }
  void CylinderBaseOdomCB(const nav_msgs::Odometry::ConstPtr &msg) {
    cylinder_base_odom_ = msg;
    first_10 = true;
  }
  void update(const ros::TimerEvent &e) {
    if (!(first_1 && first_2 && first_3 && first_4 && first_5 && first_6 && first_7 && first_8 && first_9 && first_10))
      return;
    transformStamped_.header.stamp = ros::Time::now();
    transformStamped_.header.frame_id = "world";
    // cylinder 1
    transformStamped_.child_frame_id = "cylinder_1";
    transformStamped_.transform.translation.x = cylinder1_odom_->pose.pose.position.x;
    transformStamped_.transform.translation.y = cylinder1_odom_->pose.pose.position.y;
    transformStamped_.transform.translation.z = cylinder1_odom_->pose.pose.position.z;
    transformStamped_.transform.rotation.x = cylinder1_odom_->pose.pose.orientation.x;
    transformStamped_.transform.rotation.y = cylinder1_odom_->pose.pose.orientation.y;
    transformStamped_.transform.rotation.z = cylinder1_odom_->pose.pose.orientation.z;
    transformStamped_.transform.rotation.w = cylinder1_odom_->pose.pose.orientation.w;
    br_.sendTransform(transformStamped_);
    // cylinder 2
    transformStamped_.child_frame_id = "cylinder_2";
    transformStamped_.transform.translation.x = cylinder2_odom_->pose.pose.position.x;
    transformStamped_.transform.translation.y = cylinder2_odom_->pose.pose.position.y;
    transformStamped_.transform.translation.z = cylinder2_odom_->pose.pose.position.z;
    transformStamped_.transform.rotation.x = cylinder2_odom_->pose.pose.orientation.x;
    transformStamped_.transform.rotation.y = cylinder2_odom_->pose.pose.orientation.y;
    transformStamped_.transform.rotation.z = cylinder2_odom_->pose.pose.orientation.z;
    transformStamped_.transform.rotation.w = cylinder2_odom_->pose.pose.orientation.w;
    br_.sendTransform(transformStamped_);
    // cylinder 3
    transformStamped_.child_frame_id = "cylinder_3";
    transformStamped_.transform.translation.x = cylinder3_odom_->pose.pose.position.x;
    transformStamped_.transform.translation.y = cylinder3_odom_->pose.pose.position.y;
    transformStamped_.transform.translation.z = cylinder3_odom_->pose.pose.position.z;
    transformStamped_.transform.rotation.x = cylinder3_odom_->pose.pose.orientation.x;
    transformStamped_.transform.rotation.y = cylinder3_odom_->pose.pose.orientation.y;
    transformStamped_.transform.rotation.z = cylinder3_odom_->pose.pose.orientation.z;
    transformStamped_.transform.rotation.w = cylinder3_odom_->pose.pose.orientation.w;
    br_.sendTransform(transformStamped_);
    // cylinder 4
    transformStamped_.child_frame_id = "cylinder_4";
    transformStamped_.transform.translation.x = cylinder4_odom_->pose.pose.position.x;
    transformStamped_.transform.translation.y = cylinder4_odom_->pose.pose.position.y;
    transformStamped_.transform.translation.z = cylinder4_odom_->pose.pose.position.z;
    transformStamped_.transform.rotation.x = cylinder4_odom_->pose.pose.orientation.x;
    transformStamped_.transform.rotation.y = cylinder4_odom_->pose.pose.orientation.y;
    transformStamped_.transform.rotation.z = cylinder4_odom_->pose.pose.orientation.z;
    transformStamped_.transform.rotation.w = cylinder4_odom_->pose.pose.orientation.w;
    br_.sendTransform(transformStamped_);
    // cylinder 5
    transformStamped_.child_frame_id = "cylinder_5";
    transformStamped_.transform.translation.x = cylinder5_odom_->pose.pose.position.x;
    transformStamped_.transform.translation.y = cylinder5_odom_->pose.pose.position.y;
    transformStamped_.transform.translation.z = cylinder5_odom_->pose.pose.position.z;
    transformStamped_.transform.rotation.x = cylinder5_odom_->pose.pose.orientation.x;
    transformStamped_.transform.rotation.y = cylinder5_odom_->pose.pose.orientation.y;
    transformStamped_.transform.rotation.z = cylinder5_odom_->pose.pose.orientation.z;
    transformStamped_.transform.rotation.w = cylinder5_odom_->pose.pose.orientation.w;
    br_.sendTransform(transformStamped_);
    // cylinder 6
    transformStamped_.child_frame_id = "cylinder_6";
    transformStamped_.transform.translation.x = cylinder6_odom_->pose.pose.position.x;
    transformStamped_.transform.translation.y = cylinder6_odom_->pose.pose.position.y;
    transformStamped_.transform.translation.z = cylinder6_odom_->pose.pose.position.z;
    transformStamped_.transform.rotation.x = cylinder6_odom_->pose.pose.orientation.x;
    transformStamped_.transform.rotation.y = cylinder6_odom_->pose.pose.orientation.y;
    transformStamped_.transform.rotation.z = cylinder6_odom_->pose.pose.orientation.z;
    transformStamped_.transform.rotation.w = cylinder6_odom_->pose.pose.orientation.w;
    br_.sendTransform(transformStamped_);
    // cylinder 7
    transformStamped_.child_frame_id = "cylinder_7";
    transformStamped_.transform.translation.x = cylinder7_odom_->pose.pose.position.x;
    transformStamped_.transform.translation.y = cylinder7_odom_->pose.pose.position.y;
    transformStamped_.transform.translation.z = cylinder7_odom_->pose.pose.position.z;
    transformStamped_.transform.rotation.x = cylinder7_odom_->pose.pose.orientation.x;
    transformStamped_.transform.rotation.y = cylinder7_odom_->pose.pose.orientation.y;
    transformStamped_.transform.rotation.z = cylinder7_odom_->pose.pose.orientation.z;
    transformStamped_.transform.rotation.w = cylinder7_odom_->pose.pose.orientation.w;
    br_.sendTransform(transformStamped_);
    // cylinder 8
    transformStamped_.child_frame_id = "cylinder_8";
    transformStamped_.transform.translation.x = cylinder8_odom_->pose.pose.position.x;
    transformStamped_.transform.translation.y = cylinder8_odom_->pose.pose.position.y;
    transformStamped_.transform.translation.z = cylinder8_odom_->pose.pose.position.z;
    transformStamped_.transform.rotation.x = cylinder8_odom_->pose.pose.orientation.x;
    transformStamped_.transform.rotation.y = cylinder8_odom_->pose.pose.orientation.y;
    transformStamped_.transform.rotation.z = cylinder8_odom_->pose.pose.orientation.z;
    transformStamped_.transform.rotation.w = cylinder8_odom_->pose.pose.orientation.w;
    br_.sendTransform(transformStamped_);
    // cylinder 9
    transformStamped_.child_frame_id = "cylinder_9";
    transformStamped_.transform.translation.x = cylinder9_odom_->pose.pose.position.x;
    transformStamped_.transform.translation.y = cylinder9_odom_->pose.pose.position.y;
    transformStamped_.transform.translation.z = cylinder9_odom_->pose.pose.position.z;
    transformStamped_.transform.rotation.x = cylinder9_odom_->pose.pose.orientation.x;
    transformStamped_.transform.rotation.y = cylinder9_odom_->pose.pose.orientation.y;
    transformStamped_.transform.rotation.z = cylinder9_odom_->pose.pose.orientation.z;
    transformStamped_.transform.rotation.w = cylinder9_odom_->pose.pose.orientation.w;
    br_.sendTransform(transformStamped_);
    // cylinder base 1
    float offset = 0.1875;
    transformStamped_.child_frame_id = "cylinder_base_1";
    transformStamped_.transform.translation.x = cylinder_base_odom_->pose.pose.position.x - offset;
    transformStamped_.transform.translation.y = cylinder_base_odom_->pose.pose.position.y - offset;
    transformStamped_.transform.translation.z = cylinder_base_odom_->pose.pose.position.z;
    transformStamped_.transform.rotation.x = cylinder_base_odom_->pose.pose.orientation.x;
    transformStamped_.transform.rotation.y = cylinder_base_odom_->pose.pose.orientation.y;
    transformStamped_.transform.rotation.z = cylinder_base_odom_->pose.pose.orientation.z;
    transformStamped_.transform.rotation.w = cylinder_base_odom_->pose.pose.orientation.w;
    br_.sendTransform(transformStamped_);
    // cylinder base 2
    transformStamped_.child_frame_id = "cylinder_base_2";
    transformStamped_.transform.translation.x = cylinder_base_odom_->pose.pose.position.x - offset;
    transformStamped_.transform.translation.y = cylinder_base_odom_->pose.pose.position.y;
    transformStamped_.transform.translation.z = cylinder_base_odom_->pose.pose.position.z;
    transformStamped_.transform.rotation.x = cylinder_base_odom_->pose.pose.orientation.x;
    transformStamped_.transform.rotation.y = cylinder_base_odom_->pose.pose.orientation.y;
    transformStamped_.transform.rotation.z = cylinder_base_odom_->pose.pose.orientation.z;
    transformStamped_.transform.rotation.w = cylinder_base_odom_->pose.pose.orientation.w;
    br_.sendTransform(transformStamped_);
    // cylinder base 3
    transformStamped_.child_frame_id = "cylinder_base_3";
    transformStamped_.transform.translation.x = cylinder_base_odom_->pose.pose.position.x - offset;
    transformStamped_.transform.translation.y = cylinder_base_odom_->pose.pose.position.y + offset;
    transformStamped_.transform.translation.z = cylinder_base_odom_->pose.pose.position.z;
    transformStamped_.transform.rotation.x = cylinder_base_odom_->pose.pose.orientation.x;
    transformStamped_.transform.rotation.y = cylinder_base_odom_->pose.pose.orientation.y;
    transformStamped_.transform.rotation.z = cylinder_base_odom_->pose.pose.orientation.z;
    transformStamped_.transform.rotation.w = cylinder_base_odom_->pose.pose.orientation.w;
    br_.sendTransform(transformStamped_);
    // cylinder base 4
    transformStamped_.child_frame_id = "cylinder_base_4";
    transformStamped_.transform.translation.x = cylinder_base_odom_->pose.pose.position.x;
    transformStamped_.transform.translation.y = cylinder_base_odom_->pose.pose.position.y - offset;
    transformStamped_.transform.translation.z = cylinder_base_odom_->pose.pose.position.z;
    transformStamped_.transform.rotation.x = cylinder_base_odom_->pose.pose.orientation.x;
    transformStamped_.transform.rotation.y = cylinder_base_odom_->pose.pose.orientation.y;
    transformStamped_.transform.rotation.z = cylinder_base_odom_->pose.pose.orientation.z;
    transformStamped_.transform.rotation.w = cylinder_base_odom_->pose.pose.orientation.w;
    br_.sendTransform(transformStamped_);
    // cylinder base 5
    transformStamped_.child_frame_id = "cylinder_base_5";
    transformStamped_.transform.translation.x = cylinder_base_odom_->pose.pose.position.x;
    transformStamped_.transform.translation.y = cylinder_base_odom_->pose.pose.position.y;
    transformStamped_.transform.translation.z = cylinder_base_odom_->pose.pose.position.z;
    transformStamped_.transform.rotation.x = cylinder_base_odom_->pose.pose.orientation.x;
    transformStamped_.transform.rotation.y = cylinder_base_odom_->pose.pose.orientation.y;
    transformStamped_.transform.rotation.z = cylinder_base_odom_->pose.pose.orientation.z;
    transformStamped_.transform.rotation.w = cylinder_base_odom_->pose.pose.orientation.w;
    br_.sendTransform(transformStamped_);
    // cylinder base 6
    transformStamped_.child_frame_id = "cylinder_base_6";
    transformStamped_.transform.translation.x = cylinder_base_odom_->pose.pose.position.x;
    transformStamped_.transform.translation.y = cylinder_base_odom_->pose.pose.position.y + offset;
    transformStamped_.transform.translation.z = cylinder_base_odom_->pose.pose.position.z;
    transformStamped_.transform.rotation.x = cylinder_base_odom_->pose.pose.orientation.x;
    transformStamped_.transform.rotation.y = cylinder_base_odom_->pose.pose.orientation.y;
    transformStamped_.transform.rotation.z = cylinder_base_odom_->pose.pose.orientation.z;
    transformStamped_.transform.rotation.w = cylinder_base_odom_->pose.pose.orientation.w;
    br_.sendTransform(transformStamped_);
    // cylinder base 7
    transformStamped_.child_frame_id = "cylinder_base_7";
    transformStamped_.transform.translation.x = cylinder_base_odom_->pose.pose.position.x + offset;
    transformStamped_.transform.translation.y = cylinder_base_odom_->pose.pose.position.y - offset;
    transformStamped_.transform.translation.z = cylinder_base_odom_->pose.pose.position.z;
    transformStamped_.transform.rotation.x = cylinder_base_odom_->pose.pose.orientation.x;
    transformStamped_.transform.rotation.y = cylinder_base_odom_->pose.pose.orientation.y;
    transformStamped_.transform.rotation.z = cylinder_base_odom_->pose.pose.orientation.z;
    transformStamped_.transform.rotation.w = cylinder_base_odom_->pose.pose.orientation.w;
    br_.sendTransform(transformStamped_);
    // cylinder base 8
    transformStamped_.child_frame_id = "cylinder_base_8";
    transformStamped_.transform.translation.x = cylinder_base_odom_->pose.pose.position.x + offset;
    transformStamped_.transform.translation.y = cylinder_base_odom_->pose.pose.position.y;
    transformStamped_.transform.translation.z = cylinder_base_odom_->pose.pose.position.z;
    transformStamped_.transform.rotation.x = cylinder_base_odom_->pose.pose.orientation.x;
    transformStamped_.transform.rotation.y = cylinder_base_odom_->pose.pose.orientation.y;
    transformStamped_.transform.rotation.z = cylinder_base_odom_->pose.pose.orientation.z;
    transformStamped_.transform.rotation.w = cylinder_base_odom_->pose.pose.orientation.w;
    br_.sendTransform(transformStamped_);
    // cylinder base 9
    transformStamped_.child_frame_id = "cylinder_base_9";
    transformStamped_.transform.translation.x = cylinder_base_odom_->pose.pose.position.x + offset;
    transformStamped_.transform.translation.y = cylinder_base_odom_->pose.pose.position.y + offset;
    transformStamped_.transform.translation.z = cylinder_base_odom_->pose.pose.position.z;
    transformStamped_.transform.rotation.x = cylinder_base_odom_->pose.pose.orientation.x;
    transformStamped_.transform.rotation.y = cylinder_base_odom_->pose.pose.orientation.y;
    transformStamped_.transform.rotation.z = cylinder_base_odom_->pose.pose.orientation.z;
    transformStamped_.transform.rotation.w = cylinder_base_odom_->pose.pose.orientation.w;
    br_.sendTransform(transformStamped_);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber cylinder1_sub_, cylinder2_sub_, cylinder3_sub_, cylinder4_sub_, cylinder5_sub_, cylinder6_sub_,
      cylinder7_sub_, cylinder8_sub_, cylinder9_sub_, cylinder_base_sub_;
  nav_msgs::OdometryConstPtr cylinder1_odom_, cylinder2_odom_, cylinder3_odom_, cylinder4_odom_, cylinder5_odom_,
      cylinder6_odom_, cylinder7_odom_, cylinder8_odom_, cylinder9_odom_, cylinder_base_odom_;
  tf2_ros::TransformBroadcaster br_;
  geometry_msgs::TransformStamped transformStamped_;
  ros::Timer timerPublish_;
  bool first_1 = false, first_2 = false, first_3 = false, first_4 = false, first_5 = false, first_6 = false,
       first_7 = false, first_8 = false, first_9 = false, first_10 = false;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "cylinder_tf2_broadcaster");
  Tf2Handler tf2_handler;
  ros::spin();
  return 0;
}