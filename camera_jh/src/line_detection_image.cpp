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
// std_msgs:: Int16MultiArray arr_wp;

//using namespace cv;
using namespace std;

int color = 1;

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
  cv::Mat img_bin = cv::Mat::zeros(cv::Size(1280, 720), CV_8UC3);

  while(ros::ok())
  {
  cv::Mat src, hsv, gray, image, mask, mask_sc, blur, edge;
  
  vector<cv::Vec4i> linesP;
  src = cv::imread("/home/mrl/catkin_ws/src/mecanum_project/camera_jh/src/Image2.png");
  cv::imshow("src", src);
  // arr_wp publish [jh]
  // msg_hsv.publish(dst_hsv);
  
  // get image [jh]
  
  
  int width, height, nPoints;
  // get info of img [jh]
  width = src.cols, height = src.rows;
  nPoints = width * height;
  // change img bgr to hsv [jh]
  cvtColor(src,hsv,cv::COLOR_BGR2HSV); 
  // Red_HSV_range (based on dataset)
  cv::Scalar lower_red = cv::Scalar(160, 100, 100);
  cv::Scalar upper_red = cv::Scalar(180, 255, 255);
  // Red_HSV_range (hsv of red - special case)
  cv::Scalar lower_red_sc = cv::Scalar(0, 100, 100);
  cv::Scalar upper_red_sc = cv::Scalar(20, 255, 255);
  // Blue_HSV_range (based on dataset_ 1)
  cv::Scalar lower_blue = cv::Scalar(90, 100, 100);
  cv::Scalar upper_blue = cv::Scalar(130, 255, 255);
  // hsv detect
  // mask : color 1 is red / color 2 is blue
  //========================================================================
  if(color ==1)
  {
    inRange(hsv, lower_red, upper_red, mask);  
    inRange(hsv, lower_red_sc, upper_red_sc, mask_sc);
    bitwise_or(mask, mask_sc, mask);
    bitwise_and(src, src, image, mask);
    cv::imshow("red", image);
  }
  else if (color ==2)
  {
    inRange(hsv, lower_blue, upper_blue, mask);  
    bitwise_and(src, src, image, mask);
    cv::imshow("blue", image);
  }
  //========================================================================
  // line detect[W]
  //========================================================================
  cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  GaussianBlur( gray, blur, cv::Size(7, 7), 0);
  Canny(blur, edge, 10, 150);
  cv::imshow("edge", edge);
  HoughLinesP(edge, linesP, 1, (CV_PI / 180), 50, 100, 100);
  
  for (size_t i = 0; i < linesP.size(); i++)
  {
    cv::Vec4i line = linesP[i];
    cv::line(img_bin, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(255, 255, 255), 2, 8);
    cout << linesP[i] << endl;      
  }
  cv::imshow("img_bin", img_bin);
  
  if(cv::waitKey(10)==27) break;
  ros::spinOnce();
  }
  return 0;
}