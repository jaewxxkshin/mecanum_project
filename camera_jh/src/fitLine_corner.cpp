#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>
#include <math.h>
#include <numeric>

// constant value[JH]
#define left_width 310
#define right_width 885
#define img_height 720
#define distance_of_pixel 0.075  //cm
#define pixels 72
#define PI 3.141592
#define corner_threshold 700
using namespace cv;
using namespace std;

// select line which we want to tracking(by color) - demo [HW]
int color = 1;

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

int main()
{
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
  vector<int> y_val, x_val;

  // vector that contains waypoint's position [W]
  vector<int> wp_y;
  vector<int> wp_x;
  vector<int> wp_y_c;
  vector<int> wp_x_c;
  vector<int> bottom_x;

  // variation after applying fitLine() function [W]
  float vx, vy;
  int upper_x, lower_x, top_y, x, y, converted_x, converted_y, right_x, left_x;

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

  

  // read image [JH]
  src = imread("/home/mrl/catkin_ws/src/mecanum_project/camera_jh/src/Image_line_right.png");  

  // BGR -> HSV [HW]
  cvtColor(src,hsv,COLOR_BGR2HSV); 
  
  // HSV detect [W]
  // mask : color 1 is red / color 2 is blue - demo version [W]
  //========================================================================
  if(color ==1)
  {
    inRange(hsv, lower_red, upper_red, mask);  
    inRange(hsv, lower_red_sc, upper_red_sc, mask_sc);
    bitwise_or(mask, mask_sc, mask);
    bitwise_and(src, src, image, mask);
  }
  else if (color ==2)
  {
    inRange(hsv, lower_blue, upper_blue, mask);  
    bitwise_and(src, src, image, mask);
  }

  //========================================================================

  // classical edge detection [W]
  //========================================================================
  // cvtColor(image, gray, COLOR_BGR2GRAY);
  // GaussianBlur( gray, blur, Size(7, 7), 0);
  // Canny(blur, edge, 10, 150);
  //========================================================================

  // find contours [HW]
  findContours(mask, contours, hierachy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); 
  drawContours(image, contours, -1, Scalar(255, 0, 0), 5);  

  // setting y threshold [JH]
  //========================================================================
  for ( int i = 0; i < contours.size(); i++)
  {
    for ( int j = 0; j < contours[i].size(); j++)
    {
      y_val.push_back(contours[i][j].y);
    }
  }
  for ( int i = 0; i < contours.size(); i++)
  {
    for ( int j = 0; j < contours[i].size(); j++)
    {
      x_val.push_back(contours[i][j].x);
    }
  }

  int min = *min_element(y_val.begin(),y_val.end()); 
  int max = *max_element(y_val.begin(),y_val.end()); 
  int x_left = *min_element(x_val.begin(),x_val.end());
  int x_right = *max_element(x_val.begin(),x_val.end());
  //========================================================================
  for ( int i = 0; i < contours.size(); i++)
  {
    for ( int j = 0; j < contours[i].size(); j++)
    {
      if(contours[i][j].y==max)
      {
      bottom_x.push_back(contours[i][j].x);
      }
    }
  }
  int sum_bottom_x = accumulate(bottom_x.begin(),bottom_x.end(),0);
  int mean_bottom_x = sum_bottom_x/bottom_x.size();

  // if system detect more than 2 contours -> connect every contours [JH]
  for (int i=0; i < contours.size(); i++)
  {
    for (int j=0; j < contours[i].size() ; j++)
    {
      contours_sum.push_back(contours[i][j]);
    }
  }
  
  // fitLine() function to detect representive line [W]
  fitLine(contours_sum, detected_line, CV_DIST_L2, 0, 0.01, 0.01);
  vx = detected_line[0];
  vy = detected_line[1];
  x = int(detected_line[2]);
  y = int(detected_line[3]);

  // get (x,y) of detected line in converted coordinate [JH]
  converted_x = convert_x(x);
  converted_y = convert_y(y);
  top_y = convert_y(min);
  left_x = convert_x(x_left);
  right_x = convert_x(x_right);
  // cout << "vx : " << vx << "vy : " << vy <<endl;

  upper_x = int(vx/vy*(top_y-converted_y)+converted_x);
  lower_x = int(vx/vy*(-1*converted_y) + converted_x);

    // visualization representive line [W]
  line(image, Point(inv_convert_x(upper_x),inv_convert_y(top_y)), Point(inv_convert_x(lower_x),inv_convert_y(0)),Scalar(0,0,255), 3);

  // cout << "upper point : " << upper_x << "," << top_y << endl;
  // cout << "lower point : " << lower_x << "," << "0" << endl;

  // waypoint visualization [W]
  if(top_y > corner_threshold) // go straight
  {
    for(int i=0; i<10; i++)
    {
      wp_y.push_back(top_y/10*(i+1));
      wp_x.push_back((vx/vy*(wp_y[i]-converted_y)+converted_x));
      // use make_pair [W]
      // wp_xy.push_back(make_pair(wp_x[i], wp_y[i]));
    }
  }
  else if(top_y<corner_threshold) // turn right / left
  {
    for(int i=0; i<10; i++) // circle waypoint y
    {
      float theta = (i+1) * 10 * PI / 180;
      wp_y.push_back(top_y * sin(theta));
      // cout << top_y << " " << theta << endl;
      // cout << wp_y[i] << endl;
    }
    if (vy/vx > 0) // turn left - waypoint x
    {
        for(int i=0; i<10; i++)
      {
        float theta = (i+1)*10 * PI / 180;
        wp_x.push_back(convert_x(mean_bottom_x)- top_y + top_y * cos(theta));
        // wp_x_c.push_back(lower_x - top_y * cos(theta));
        // cout << wp_x[i] << endl;
      }
    } 
    else if (vy/vx<0) // turn right - waypoint x
    {
        for(int i=0; i<10; i++)
      {
        float theta = (i+1)*10 * PI / 180;
        wp_x.push_back(convert_x(mean_bottom_x) + top_y - top_y * cos(theta));
        // wp_x_c.push_back(lower_x - top_y * cos(theta));
        // cout << wp_x[i] << endl;
      }
    }
  }

    for(int i=0; i<10; i++)
    {
      wp_xy.push_back(make_pair(wp_x[i], wp_y[i]));
    }

    for(int i=0; i<10; i++)
    {
    circle(src, Point(inv_convert_x(wp_x[i]), inv_convert_y(wp_y[i])), 5, Scalar(255,255,255), 3);
    }

  // 2022/07/15 temporay comment [W]
  // waypoint visualization on corner [HW]
  

  // print x,y position of way points [W]
  for(int i = 0; i < wp_xy.size(); i++)
  {
    x_pos = wp_xy[i].first*distance_of_pixel*0.01; // m
    y_pos =  wp_xy[i].second*distance_of_pixel*0.01; //m
    cout <<"way point" << i + 1<< "'s x value : " << x_pos << "m, way point"
    << i + 1 << "'s y value: " << y_pos<< "m" <<endl;
    
  }



//   for(int i=0; i<10; i++)
//   {
//     circle(src, Point(inv_convert_x(wp_x_c[i]), inv_convert_y(wp_y_c[i])), 5, Scalar(255,255,255), 3);
//   }

  // visualization camera's origin(on converted coordinate) [JH]
  circle(src, Point(597,720), 10, Scalar(255,0,0), 5);

  // 2022/07/15 temporay comment [W]
  // visualization rotation Circle[HW]

//   circle(src, Point(inv_convert_x(lower_x - top_y),inv_convert_y(0)),top_y,Scalar(0,255,0),5);

  // save image [JH]
  imwrite("mask_hw.png", mask);  
  imwrite("contours_hw.png", image);
  imwrite("result_jh.png",src);  

  return 0;
}

