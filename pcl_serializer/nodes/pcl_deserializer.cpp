#include <chrono>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vector>

using namespace std::chrono;

typedef unsigned short ushort;
typedef unsigned int uint;

typedef unsigned char uchar;

class PcDeserializer
{
public:
    PcDeserializer()
    {
        im_depth_sub_ = nh_.subscribe("input_image", 1, &PcDeserializer::im_CB, this);
        im_depth_pub_ = nh_.advertise<sensor_msgs::Image>("result_image", 0, false);
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
    uint as_uint(const float x)
    {
        return *(uint *)&x;
    }
    float as_float(const uint x)
    {
        return *(float *)&x;
    }
    float bytesToFloat(uchar b0, uchar b1, uchar b2, uchar b3)
    {
        float output;

        *((uchar *)(&output) + 0) = b0;
        *((uchar *)(&output) + 1) = b1;
        *((uchar *)(&output) + 2) = b2;
        *((uchar *)(&output) + 3) = b3;

        return output;
    }
    float bytesToFloat2(uchar b0, uchar b1)
    {
        float output;

        *((uchar *)(&output) + 0) = b0;
        *((uchar *)(&output) + 1) = b1;

        return output;
    }
    void im_CB(const sensor_msgs::Image::ConstPtr &msg)
    {
        // if (!first) {
        //     return;
        // }
        depth_compress_msg_.header = msg->header;
        depth_compress_msg_.width = msg->width;
        depth_compress_msg_.step = msg->step * 2;
        depth_compress_msg_.encoding = msg->encoding;
        depth_compress_msg_.is_bigendian = msg->is_bigendian;
        depth_compress_msg_.height = msg->height;
        depth_compress_msg_.width = msg->width;
        depth_compress_msg_.data.resize(depth_compress_msg_.height * depth_compress_msg_.width * 4);

        //ROS_INFO("Height is %d", msg->height);
        //ROS_INFO("Width is %d", msg->width);
        //ROS_INFO("Encoding is %d", msg->encoding);
        //ROS_INFO("Is bidedian %d", msg->is_bigendian);
        //ROS_INFO("Step is %d", msg->step);
        //ROS_INFO("Data Size is %d", msg->data.size());

        for (size_t row = 0; row < msg->height; row++) {
            for (size_t col = 0; col < msg->width; col++) {

                // uchar b0 = msg->data[row * msg->step + 2 * col + 0];
                // uchar b1 = msg->data[row * msg->step + 2 * col + 1];
                ushort fs;
                memcpy((uchar *)(&fs), &msg->data[row * msg->step + 2 * col], 2);
                // *((uchar *)(&fs) + 0) = b0;
                // *((uchar *)(&fs) + 1) = b1;
                float d = half_to_float(fs);
                if (first)
                    ROS_INFO("The Float at %d %d is %f", row, col, d);
                d *= 10;
                if (d == 0 || d > 100)
                    d = std::nanf("");
                memcpy((uchar *)(&depth_compress_msg_.data[row * depth_compress_msg_.step + 4 * col]), &d, 4);
                // uchar b00, b01, b02, b03;
                // b00 = *(uchar *)((&d) + 0);
                // b01 = *(uchar *)((&d) + 1);
                // b02 = *(uchar *)((&d) + 2);
                // b03 = *(uchar *)((&d) + 3);
                // depth_compress_msg_.data[row * depth_compress_msg_.step + 4 * col + 0] = b00;
                // depth_compress_msg_.data[row * depth_compress_msg_.step + 4 * col + 1] = b01;
                // depth_compress_msg_.data[row * depth_compress_msg_.step + 4 * col + 2] = b02;
                // depth_compress_msg_.data[row * depth_compress_msg_.step + 4 * col + 3] = b03;
            }
        }
        im_depth_pub_.publish(depth_compress_msg_);

        if (first) {
            first = false;
            ROS_INFO("Height is %d", msg->height);
            ROS_INFO("Width is %d", msg->width);
            ROS_INFO("Encoding is %d", msg->encoding);
            ROS_INFO("Is bidedian %d", msg->is_bigendian);
            ROS_INFO("Step is %d", msg->step);
            ROS_INFO("Data Size is %d", msg->data.size());
            for (size_t row = 0; row < msg->height; row++) {
                for (size_t col = 0; col < msg->width; col++) {
                    uchar b0 = msg->data[row * msg->step + 2 * col + 0];
                    uchar b1 = msg->data[row * msg->step + 2 * col + 1];
                    ushort fs;
                    *((uchar *)(&fs) + 0) = b0;
                    *((uchar *)(&fs) + 1) = b1;

                    // if (row == 479 && col == 639) {
                    // ROS_INFO("The bytes are %d %d", b0, b1);
                    // float d = half_to_float(fs);
                    // ROS_INFO("The Float at %d %d is %f", row, col, d);
                    // }
                }
            }
        }
    }

protected:
    ros::NodeHandle nh_;
    ros::Publisher im_depth_pub_;
    ros::Subscriber im_depth_sub_;
    sensor_msgs::Image depth_compress_msg_;
    bool first = true;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_deserializer");
    PcDeserializer handler;
    ros::spin();
    return 0;
}