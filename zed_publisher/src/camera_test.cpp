
#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
// #include <sensor_msgs/PointCloud2.h>

class CameraRelay
{
public:
    CameraRelay()
    {
        // pc_sub = nh.subscribe("/zed2/zed_node/point_cloud/cloud_registered", 2, &CameraRelay::pc_CB, this);
        pc_sub = nh.subscribe("/zed2/zed_node/rgb/camera_info", 2, &CameraRelay::pc_CB, this);
        im_sub = nh.subscribe("/zed2/zed_node/depth/depth_registered", 2, &CameraRelay::im_CB, this);

        pc_pub = nh.advertise<sensor_msgs::CameraInfo>("/cam_info", 2);
        im_pub = nh.advertise<sensor_msgs::Image>("/my_depth", 2);
        // Timer = nh.createTimer(ros::Duration(0.1),  &CameraRelay::Timer_CB, this)
    }
    void im_CB(const sensor_msgs::Image::ConstPtr &inf)
    {
        pc_pub.publish(info_msg);
        im_pub.publish(inf);
    }
    void pc_CB(const sensor_msgs::CameraInfo::ConstPtr &inf)
    {
        info_msg = *inf;
    }
    // void Timer_CB(const time::CameraInfo::ConstPtr &inf)
    // {
    //     info_msg = *inf;
    // }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pc_sub, im_sub;
    ros::Publisher pc_pub, im_pub;
    ros::Timer Timer;
    sensor_msgs::CameraInfo info_msg;
    int count;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vel_to_arduino");
    CameraRelay handler;
    ros::spin();
    return 0;
}