#include <vector>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <librealsense2/rs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int16MultiArray.h>
#include <opencv2/highgui/highgui.hpp>

// constant value[JH]
#define distance_of_pixel 0.075  //cm
#define pixels 72
#define left_width 310
#define right_width 885
#define img_height 720
#define PI 3.141592

// function for coordinate conversion [JH]
// -------------------------------------------
int convert_x (int &x)
{ 
  int converted_x = 0;
  converted_x= x-((right_width-left_width)/2 + left_width);
  return converted_x;
}

int convert_y (int &y)
{
  int converted_y = 0;
  converted_y= -(y-img_height);
  return converted_y;
}

int inv_convert_x( const int &x)
{
  int inv_converted_x=0;
  inv_converted_x = x+ ((right_width-left_width)/2 + left_width);
  return inv_converted_x;
}

int inv_convert_y( const int &y)
{
  int inv_converted_y=0;
  inv_converted_y = -y+img_height;
  return inv_converted_y;
}
// -------------------------------------------

// resize function [HW]
void set_array(std_msgs::Int16MultiArray &arr, const int &n)
{
  arr.data.resize(n);
  return 0;
}

// variation declaration [W]
  // -------------------------------------------
  // Matrix for img_proc [W]
  Mat src, hsv, gray, image, mask, mask_sc, blur, edge;

  // vector for finding contour [W]
  vector<Vec4i> linesP, hierachy;
  vector<vector<Point>> contours; 

  // if system detect more than 2 contours -> connect every contours [JH]
  vector<Point> contours_sum;

  // vector for fitLine() function[W]
  Vec4f detected_line;

  // vector that contains all of contours's y value [W]
  vector<int> y_val;

  // vector that contains waypoint's position [W]
  vector<int> wp_y;
  vector<int> wp_x;
  vector<int> wp_y_c;
  vector<int> wp_x_c;

  // variation after applying fitLine() function [W]
  float vx, vy;
  int upper_x, lower_x, top_y, x, y, converted_x, converted_y;

  // 2-D vector for way points [W]
  vector<pair<int, int>> wp_xy;

  // x, y positions [W]
  float x_pos;
  float y_pos;
  // -------------------------------------------

  // Red_HSV_range (based on dataset)
  Scalar lower_red = Scalar(160, 100, 100);
  Scalar upper_red = Scalar(180, 255, 255);
  // Red_HSV_range (hsv of red - special case)
  Scalar lower_red_sc = Scalar(0, 100, 100);
  Scalar upper_red_sc = Scalar(20, 255, 255);
  // Blue_HSV_range (based on dataset_ 1)
  Scalar lower_blue = Scalar(90, 100, 100);
  Scalar upper_blue = Scalar(130, 255, 255);
