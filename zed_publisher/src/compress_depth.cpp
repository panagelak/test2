#include <compressed_depth_image_transport/CompressedDepthPublisherConfig.h>
#include <compressed_depth_image_transport/codec.h>
#include <compressed_depth_image_transport/compressed_depth_publisher.h>
#include <compressed_depth_image_transport/compressed_depth_subscriber.h>
#include <compressed_depth_image_transport/compression_common.h>
#include <image_transport/image_transport.h>

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

ros::Subscriber SubImage;
compressed_depth_image_transport::CompressedDepthPublisher pub;
void depthCallback(const sensor_msgs::Image::ConstPtr &msg)
{
    // pub.publish(msg);
    // sensor_msgs::Image image;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth");
    ros::NodeHandle nh;
    // SubImage = nh.advertise<sensor_msgs::Image>("test", 1);
    SubImage = nh.subscribe("/zed2/zed_node/depth/depth_registered", 1000, depthCallback);
    // compressed_depth_image_transport::encodeCompressedDepthImage()

    image_transport::ImageTransport it(nh);
    const ros::VoidPtr obj;

    
    // pub.advertiseImpl(nh,"test",1);
    // image_transport::Subscriber itSub = it.subscribe("/zed2/zed_node/rgb/image_rect_color", 1, depthCallback, obj, image_transport::TransportHints("compressed"));
    // image_transport::Subscriber itSub = it.subscribe("/zed2/zed_node/depth/depth_registered", 1, depthCallback, obj, image_transport::TransportHints("compressed"));
    // image_transport::Subscriber sub = it.subscribe("/zed2/zed_node/rgb/image_rect_color", 1, depthCallback);
    ros::spin();

    return 0;
}