#include <chrono>
#include <cmath>
#include <integration/PclTransfer.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <vector>

using namespace std::chrono;

typedef unsigned short ushort;
typedef unsigned int uint;

typedef unsigned char uchar;

class PcDeserializer
{
public:
    PcDeserializer() : nh_(""), first_(true), got_camera_info_(false)
    {
        transfer_sub_ = nh_.subscribe("input_pcl_transfer", 1, &PcDeserializer::callback, this);
        im_pub_ = nh_.advertise<sensor_msgs::CompressedImage>("image_out", 0, false);
        depth_pub_ = nh_.advertise<sensor_msgs::Image>("depth_out", 0, false);
        camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info_out", 0, false);
        // wait for first camera_info msg
        auto msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/zed2/zed_node/depth/camera_info", ros::Duration(0.0));
        camera_info_msg_ = *msg;
    }
    uint as_uint(const float x)
    {
        return *(uint *)&x;
    }
    float as_float(const uint x)
    {
        return *(float *)&x;
    }
    float half_to_float(const ushort x)
    {                                                                                                                                                        // IEEE-754 16-bit floating-point format (without infinity): 1-5-10, exp-15, +-131008.0, +-6.1035156E-5, +-5.9604645E-8, 3.311 digits
        const uint e = (x & 0x7C00) >> 10;                                                                                                                   // exponent
        const uint m = (x & 0x03FF) << 13;                                                                                                                   // mantissa
        const uint v = as_uint((float)m) >> 23;                                                                                                              // evil log2 bit hack to count leading zeros in denormalized format
        return as_float((x & 0x8000) << 16 | (e != 0) * ((e + 112) << 23 | m) | ((e == 0) & (m != 0)) * ((v - 37) << 23 | ((m << (150 - v)) & 0x007FE000))); // sign : normalized : denormalized
    }
    ushort float_to_half(const float x)
    {                                                                                                                                                                                       // IEEE-754 16-bit floating-point format (without infinity): 1-5-10, exp-15, +-131008.0, +-6.1035156E-5, +-5.9604645E-8, 3.311 digits
        const uint b = as_uint(x) + 0x00001000;                                                                                                                                             // round-to-nearest-even: add last bit after truncated mantissa
        const uint e = (b & 0x7F800000) >> 23;                                                                                                                                              // exponent
        const uint m = b & 0x007FFFFF;                                                                                                                                                      // mantissa; in line below: 0x007FF000 = 0x00800000-0x00001000 = decimal indicator flag - initial rounding
        return (b & 0x80000000) >> 16 | (e > 112) * ((((e - 112) << 10) & 0x7C00) | m >> 13) | ((e < 113) & (e > 101)) * ((((0x007FF000 + m) >> (125 - e)) + 1) >> 1) | (e > 143) * 0x7FFF; // sign : normalized : denormalized : saturate
    }
    void callback(const integration::PclTransfer::ConstPtr &msg)
    {
        if (first_) {
            depth_image_msg_.header = msg->depth_image.header;
            depth_image_msg_.width = msg->depth_image.width;
            depth_image_msg_.step = msg->depth_image.step * 2;
            depth_image_msg_.encoding = msg->depth_image.encoding;
            depth_image_msg_.is_bigendian = msg->depth_image.is_bigendian;
            depth_image_msg_.height = msg->depth_image.height;
            depth_image_msg_.width = msg->depth_image.width;
            depth_image_msg_.data.resize(msg->depth_image.height * msg->depth_image.width * 4);
            first_ = false;
        }
        for (size_t row = 0; row < msg->depth_image.height; row++) {
            for (size_t col = 0; col < msg->depth_image.width; col++) {
                ushort fs;
                memcpy((uchar *)(&fs), &msg->depth_image.data[row * msg->depth_image.step + 2 * col], 2);
                float d = half_to_float(fs);
                d *= 10;
                if (d == 0 || d > 100)
                    d = std::nanf("");
                memcpy((uchar *)(&depth_image_msg_.data[row * depth_image_msg_.step + 4 * col]), &d, 4);
            }
        }

        im_pub_.publish(msg->rgb_image);
        depth_pub_.publish(depth_image_msg_);
        camera_info_msg_.header = depth_image_msg_.header;
        camera_info_pub_.publish(camera_info_msg_);
    }

protected:
    ros::NodeHandle nh_;
    ros::Publisher im_pub_;
    ros::Publisher camera_info_pub_;
    ros::Publisher depth_pub_;
    ros::Publisher pcl_pub_;
    ros::Subscriber transfer_sub_;
    sensor_msgs::Image depth_image_msg_;
    // sensor_msgs::CompressedImage image_msg_;
    sensor_msgs::CameraInfo camera_info_msg_;
    // sensor_msgs::PointCloud2 pcl_msg_;
    bool first_, got_camera_info_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_deserializer");
    PcDeserializer handler;
    ros::spin();
    return 0;
}