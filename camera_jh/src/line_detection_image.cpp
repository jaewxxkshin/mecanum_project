#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <typeinfo>
#include <vector>
#include <math.h>
#include <algorithm>


#define img_width 1280
#define img_height 720

// message for array of waypoint [jh]
// std_msgs:: Int16MultiArray arr_wp;

//using namespace cv;
using namespace std;

int color = 2;

// function for resizing topic [jh]
// void set_array()
// {
//   dst_hsv.data.resize(18);
// }

// // function for org_x -> converted x [jh]
int convert_x (int &x)
{ 
  int converted_x = 0;
  converted_x= x-img_width/2;
  return converted_x;
}
// function for org_y -> converted y [jh]
int convert_y (int &y)
{
  int converted_y = 0;
  converted_y= -(y-img_height);
  return converted_y;
}

int inv_convert_x( const int &x)
{
  int inv_converted_x=0;
  inv_converted_x = x+img_width/2;
  return inv_converted_x;
}

int inv_convert_y( const int &y)
{
  int inv_converted_y=0;
  inv_converted_y = -y+img_height;
  return inv_converted_y;
}

int main()
{
  // Publisher for arr_wp [jh]
//   ros::Publisher  = nh.advertise<std_msgs::Int16MultiArray>("msg_hsv", 1000);
  // cv::Mat img_bin = cv::Mat::zeros(cv::Size(1280, 720), CV_8UC3);
  cv::Mat src, hsv, gray, image, mask, mask_sc, blur, edge;

  vector<cv::Vec4i> linesP, hierachy;
  
  vector<vector<cv::Point>> contours; 
  vector<cv::Point> fitLine_contours;
  src = cv::imread("/home/mrl/catkin_ws/src/mecanum_project/camera_jh/src/Image2.png");

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

  }
  else if (color ==2)
  {
    inRange(hsv, lower_blue, upper_blue, mask);  
    bitwise_and(src, src, image, mask);
   
  }
  //========================================================================
  // line detect[W]
  //========================================================================
  cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  
  GaussianBlur( gray, blur, cv::Size(7, 7), 0);
  Canny(blur, edge, 10, 150);
  
  
  // cv::imshow("edge", edge); 
  //===============================================================
  // houghlinesP [k]
  // HoughLinesP(edge, linesP, 1, (CV_PI / 180), 50, 100, 100);

  // for (size_t i = 0; i < linesP.size(); i++)
  // {
  //   cv::Vec4i line = linesP[i];
  //   cv::line(img_bin, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(255, 255, 255), 2, 8);
  //   cout << linesP[i] << endl;      
  // }
  // cv::imshow("img_bin", img_bin);
  //===============================================================

  //find contours [k]
  findContours(mask, contours, hierachy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); 
  // drawContours(image, contours, -1, cv::Scalar(255, 0, 0), 5);  


  // vector<vector<Point>> => voctor<Point>
  // for (int i = 0; i < contours.size(); i++)
  //   for ( int j = 0; j < contours[i].size(); j++)
  //     fitLine_contours.push_back(contours[i][j]);
  
  // cout << fitLine_contours <<endl;


  // insert into a vector<Point>
	vector<cv::Point> points;

	//Iterate over the pixels to obtain all point positions
	for (int y = 0; y < edge.rows; y++) {
		// row y
		uchar* rowPtr = edge.ptr<uchar>(y);

	  for (int x = 0; x < edge.cols; x++) {
			// column x
			if (rowPtr[x]) {	// if on a contour
				points.push_back(cv::Point(x, y));
			}
		}
	}
  
  vector<int> y_val;
  // int count_n = 0;
  // for (auto it = contours[0].begin(); it < contours[0].end(); it++)
  // {
  //   for (auto it2 = *it; it2 < *it; it2++)
  //   { 
  //     cout << *it2 << endl;
  //   }
  // }
  // cout << contours[0].size() <<endl;
  // cout << contours[0] << "hi " << endl;
  // cout << contours[0][0] << "hello" << endl;
  // cout << contours[0][0].x << "h" << endl;
  for ( int i =0; i<contours.size(); i++)
  {
    for ( int j = 0; j <contours[i].size(); j++)
    {
      //cout << contours[i][j].y << endl;
      y_val.push_back(contours[i][j].y);
    }
  }
  for (auto it = y_val.begin(); it < y_val.end(); it++)
  {
    //cout << *it << endl;
  }
  int min = *min_element(y_val.begin(),y_val.end()); 
  cout << "min val : " << min << endl;

  cv::Vec4f line;

  vector<cv::Point> contours_sum;
  //============================================ 
  // for (auto it = contours.begin(); it < contours.end(); it++)
  // {
  //   contours_sum.push_back(*it);
  //   cout << *it << "h" << endl;
    
  // }
  for (int i=0; i < contours.size(); i++)
  {
    for (int j=0; j < contours[i].size() ; j++)
    {
      contours_sum.push_back(contours[i][j]);
    }
  }
  // cout << contours_sum << endl;
  // cout << contours[0] << "s"<< endl;
  
  // auto end_0 = contours[0].end();
  // auto start_1 = contours[1].begin();
  // auto end_1 = contours[1].end() - 1;

  // contours[0].insert(end_0, start_1, end_1);

  // contours[0].insert(end_0, 


  // cout << contours[0] << "ss" << endl;
  //==========================================
  
  //cout << contours.size() <<endl;
  //cout << contours_sum.size() <<endl;

  fitLine(contours_sum, line, CV_DIST_L2, 0, 0.01, 0.01);
  
  float vx, vy;
  int upper_x, lower_x, top_y, x, y, converted_x, converted_y;

  vx = line[0];
  vy = line[1];
  x = int(line[2]);
  y = int(line[3]);

  converted_x = convert_x(x);
  converted_y = convert_y(y);
  top_y = convert_y(min);

  upper_x = int(vx/vy*(top_y-converted_y)+converted_x);
  lower_x = int(vx/vy*(-1*converted_y) + converted_x);

  cout << "upper point : " << upper_x << "," << top_y << endl;
  cout << "lower point : " << lower_x << "," << "0" << endl;

  cv::line(src, cv::Point(inv_convert_x(upper_x),inv_convert_y(top_y)),cv::Point(inv_convert_x(lower_x),inv_convert_y(0)),cv::Scalar(0,0,255),3);

  cv::imwrite("result.png",src);  



  // vector<cv::Point> points;

	// Iterate over the pixels to obtain all point positions
	// for (int y = 0; y < edge.rows; y++) {
	// 	// row y
	// 	uchar* rowPtr = edge.ptr<uchar>(y);

	// 	for (int x = 0; x < edge.cols; x++) {
	// 		// column x
	// 		if (rowPtr[x]) {	// if on a contour
	// 			points.push_back(cv::Point(x, y));
	// 		}
	// 	}
	// }
  // cout << points <<endl;
  // vector<cv::Vec4i> linesP, hierachy;
  // vector<cv::Vec4f> line;
  // vector<vector<cv::Point>> contours;
  // vector<cv::Point> fitLine_contours;
  // fitLine_contours = contours[0];
  // Using fitline function [W]

  //int x0, y0;
  //cout << line[0] <<endl; 
  // cout << contours[0] << endl;  
  // cout << contours.size() << endl; 
  // dst.at<cv::Vec3b>(y, x)[0];
  // cout << contours[0] << endl;
  // cout << contours[1] << endl;


  return 0;
}

// void BlockEdgeDetectorT::testag(vector<cv::Point2f> contour)
// {
// 	vector<double> partk;
// 	const int partspan = 100;
// 	partk.push_back(p_block->UpLine.k);
// 	for (int i = 0; i + partspan < contour.size(); i += partspan)
// 	{
// 		vector<cv::Point> points;
// 		for (int j = i; j < i + partspan; j+= 10)
// 		{
// 			points.push_back(contour[j]);
// 		}
// 		cv::Vec4f line4f;
// 		fitLine(cv::Mat(points), line4f, CV_DIST_L2, 0, 0.01, 0.01);

// 		double dx = line4f[0];
// 		double dy = line4f[1];
// 		partk.push_back(dy / dx);
// 	}