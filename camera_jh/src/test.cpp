#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
// #include <librealsense2/rs.hpp>
#include <iostream>
#include <typeinfo>
#include <vector>
#include <math.h>
#include <std_msgs/Int16MultiArray.h>

int n = 2;
int countt  = 0;

// RGV2HSV function [k] 

#define min_f(a, b, c)  (fminf(a, fminf(b, c)))
#define max_f(a, b, c)  (fmaxf(a, fmaxf(b, c)))

#define Cluster_number 8 // [jh]

#define img_width 1280
#define img_height 720

int dst_hsv_for_msg[3];

void rgb2hsv(const unsigned int &src_r, const unsigned int &src_g, const unsigned int &src_b)
{
    float r = src_r / 255.0f;
    float g = src_g / 255.0f;
    float b = src_b / 255.0f;

    float h, s, v; // h:0-360.0, s:0.0-1.0, v:0.0-1.0

    float max = max_f(r, g, b);
    float min = min_f(r, g, b);

    v = max;

    if (max == 0.0f) {
        s = 0;
        h = 0;
    }
    else if (max - min == 0.0f) {
        s = 0;
        h = 0;
    }
    else {
        s = (max - min) / max;

        if (max == r) {
            h = 60 * ((g - b) / (max - min)) + 0;
        }
        else if (max == g) {
            h = 60 * ((b - r) / (max - min)) + 120;
        }
        else {
            h = 60 * ((r - g) / (max - min)) + 240;
        }
    }

    if (h < 0) h += 360.0f;

    int dst_h = (unsigned int)(h / 2);   
    int dst_s = (unsigned int)(s * 255); 
    int dst_v = (unsigned int)(v * 255); 
    dst_hsv_for_msg[0] = dst_h;
    dst_hsv_for_msg[1] = dst_s;
    dst_hsv_for_msg[2] = dst_v;
    // dst_hsv=dst_h,dst_s,dst_v;

    // cout << "h:" << dst_h << "s:" <<  dst_s << "v:"<< dst_v << endl;
    std::cout << "h:" << dst_h << "s:" <<  dst_s << "v:"<< dst_v << std::endl;
   
    // return dst_hsv;
    return;
}

int main(int argc, char **argv)
{
//   set_array();
  ros::init(argc, argv, "ros_realsense_opencv_tutorial");
  ros::NodeHandle nh;

  rgb2hsv(128, 90, 23);
}