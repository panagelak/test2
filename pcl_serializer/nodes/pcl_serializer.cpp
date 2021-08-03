#include <chrono>
#include <cmath>
#include <integration/PclTransfer.h>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <vector>

using namespace std::chrono;

typedef unsigned short ushort;
typedef unsigned int uint;

typedef unsigned char uchar;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CompressedImage> MySyncPolicy;

class PcSerializer
{
public:
    PcSerializer() : nh_(""), depth_sub_(nh_, "input_depth", 1), image_sub_(nh_, "input_image", 1), sync_(MySyncPolicy(1), depth_sub_, image_sub_), first_(true)
    {
        sync_.registerCallback(boost::bind(&PcSerializer::callback, this, _1, _2));
        pc_comp_pub_ = nh_.advertise<integration::PclTransfer>("transfer_topic", 0, false);
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
    void callback(const sensor_msgs::ImageConstPtr &depth_msg, const sensor_msgs::CompressedImageConstPtr &image_msg)
    {
        compress_msg_.header.stamp = ros::Time::now();
        compress_msg_.header.frame_id = "";
        compress_msg_.rgb_image = *image_msg;

        compress_msg_.depth_image.header = depth_msg->header;
        compress_msg_.depth_image.width = depth_msg->width;
        compress_msg_.depth_image.step = depth_msg->step / 2.;
        compress_msg_.depth_image.encoding = depth_msg->encoding;
        compress_msg_.depth_image.is_bigendian = depth_msg->is_bigendian;
        compress_msg_.depth_image.height = depth_msg->height;
        compress_msg_.depth_image.width = depth_msg->width;
        if (first_)
            compress_msg_.depth_image.data.resize(compress_msg_.depth_image.height * compress_msg_.depth_image.width * 2);

        for (size_t row = 0; row < depth_msg->height; row++) {
            for (size_t col = 0; col < depth_msg->width; col++) {
                float d;
                memcpy((uchar *)(&d), &depth_msg->data[row * depth_msg->step + 4 * col], 4);
                d *= 0.1;
                ushort fs = float_to_half(d);
                memcpy((uchar *)(&compress_msg_.depth_image.data[row * compress_msg_.depth_image.step + 2 * col]), &fs, 2);
            }
        }
        pc_comp_pub_.publish(compress_msg_);
    }

protected:
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::CompressedImage> image_sub_;
    message_filters::Synchronizer<MySyncPolicy> sync_;
    ros::Publisher pc_comp_pub_;
    integration::PclTransfer compress_msg_;
    bool first_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_serializer");
    PcSerializer handler;
    ros::spin();
    return 0;
}