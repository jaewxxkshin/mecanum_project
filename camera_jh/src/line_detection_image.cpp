#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <typeinfo>
#include <vector>
#include <math.h>
#include <std_msgs/Int16MultiArray.h>

#define img_width 1280
#define img_height 720

// message for array of waypoint [jh]
std_msgs:: Int16MultiArray arr_wp;

//using namespace cv;
using namespace std;

// function for resizing topic [jh]
// void set_array()
// {
//   dst_hsv.data.resize(18);
// }

// // function for org_x -> converted x [jh]
// int img_x ( const unsigned int &x)
// { 
//   int converted_x = 0;
//   converted_x= x-img_width/2;
//   return converted_x;
// }
// // function for org_y -> converted y [jh]
// int img_y ( const unsigned int &y)
// {
//   int converted_y = 0;
//   converted_y= -(y-img_height);
//   return converted_y;
// }

int main(int argc, char **argv)
{
//   set_array();
  ros::init(argc, argv, "ros_realsense_opencv_tutorial");
  ros::NodeHandle nh;
  
  // Publisher for arr_wp [jh]
//   ros::Publisher  = nh.advertise<std_msgs::Int16MultiArray>("msg_hsv", 1000);

  while(ros::ok())
  {
    cv::Mat src,hsv,red_mask, red_image, gray;

    // arr_wp publish [jh]
    // msg_hsv.publish(dst_hsv);
    
    // get image [jh]
    src = cv::imread("Image2.png");
    cv::imshow("src", src);
    int width, height,nPoints;

    // get info of img [jh]
    width = src.cols, height = src.rows;
    nPoints = width * height;

    // change img bgr to hsv [jh]
    cv::cvtColor(src,hsv,cv::COLOR_BGR2HSV); 

    // Red_HSV_range (based on dataset)
    // cv::Scalar lower_red = cv::Scalar(170, 100, 100);
    // cv::Scalar upper_red = cv::Scalar(180, 255, 255);

    // Red_HSV_range (based on dataset_1)
    cv::Scalar lower_red = cv::Scalar(10, 100, 100);
    cv::Scalar upper_red = cv::Scalar(30, 255, 255);

    // Blue_HSV_range (based on dataset_ 1)
    cv::Scalar lower_blue = cv::Scalar(110, 100, 100);
    cv::Scalar upper_blue = cv::Scalar(130, 255, 255);

    // red_mask [jh]
    cv::inRange(hsv, lower_red, upper_red, red_mask);    
    cv::bitwise_and(src, src, red_image, red_mask);
    cv::imshow("red_mask",red_mask);
    if(cv::waitKey(10)==27) break;
    ros::spinOnce();
  }
  return 0;
}