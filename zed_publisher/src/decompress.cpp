#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

ros::Publisher PubImage;

void depthCallback(const sensor_msgs::Image::ConstPtr &msg)
{
    PubImage.publish(msg);
    // sensor_msgs::Image image;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth");
    ros::NodeHandle nh;
    PubImage = nh.advertise<sensor_msgs::Image>("test", 1);
    image_transport::ImageTransport it(nh);
    const ros::VoidPtr obj;
    image_transport::Subscriber itSub = it.subscribe("/zed2/zed_node/rgb/image_rect_color", 1, depthCallback, obj, image_transport::TransportHints("compressed"));
    // image_transport::Subscriber itSub = it.subscribe("/zed2/zed_node/depth/depth_registered", 1, depthCallback, obj, image_transport::TransportHints("compressed"));
    // image_transport::Subscriber sub = it.subscribe("/zed2/zed_node/rgb/image_rect_color", 1, depthCallback);
    ros::spin();

    return 0;
}