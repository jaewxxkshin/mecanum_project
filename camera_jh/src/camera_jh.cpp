#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <typeinfo>
#include <vector>
#include <math.h>
#include <std_msgs/Int16MultiArray.h>
#include <string>


std_msgs::Int16MultiArray dst_hsv;

//using namespace cv;
using namespace std;
cv::Mat img_roi;



int n = 2;

// variation To save K-means result Image[W]
string img_name = "/home/mrl/bag_image/Image";
string type = ".png";
string temp = "";

int countt  = 0;
int dst_hsv_for_msg[3];


#define min_f(a, b, c)  (fminf(a, fminf(b, c)))
#define max_f(a, b, c)  (fmaxf(a, fmaxf(b, c)))


// resize function [k]

void set_array()
{
  dst_hsv.data.resize(24);
}

// RGV2HSV function [k] 

void bgr2hsv(const unsigned int &src_b, const unsigned int &src_g, const unsigned int &src_r)
{
    float b = src_b / 255.0f;
    float g = src_g / 255.0f;
    float r = src_r / 255.0f;

    float h, s, v; // h:0-360.0, s:0.0-1.0, v:0.0-1.0

    float max = max_f(b, g, r);
    float min = min_f(b, g, r);

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
    
    // return dst_hsv;
    return;
}


int main(int argc, char **argv)
{
  set_array();
  ros::init(argc, argv, "ros_realsense_opencv_tutorial");
  ros::NodeHandle nh;
  ros::Publisher msg_hsv = nh.advertise<std_msgs::Int16MultiArray>("msg_hsv", 1000);


  rs2::pipeline pipe;
  rs2::config cfg;
  rs2::frameset frames;
  rs2::frame color_frame;

  int bgr2hsv(const unsigned int &src_b, const unsigned int &src_g, const unsigned int &src_r);

  cfg.enable_stream(RS2_STREAM_COLOR, 1280,720, RS2_FORMAT_BGR8, 30);
  pipe.start(cfg);

  for(int i=0; i < 30; i ++)
  {
    frames = pipe.wait_for_frames();
  }

  ros::Rate loop_rate(30);

  while(ros::ok())
  {
    frames = pipe.wait_for_frames();
    color_frame = frames.get_color_frame();
    cv::Mat dst;
    cv::Point2f src_p[4], dst_p[4];
    cout << "count : " << countt << endl;
    //imshow("Display Image", src);

    // [jaeho]
    src_p[0] = cv::Point2f(443,478);
    src_p[1] = cv::Point2f(336,720);
    src_p[2] = cv::Point2f(855,474);
    src_p[3] = cv::Point2f(940,720);

    dst_p[0] = cv::Point2f(604-268/n,720-268/n*3/2);
    dst_p[1] = cv::Point2f(604-268/n,720);
    dst_p[2] = cv::Point2f(604+268/n,720-268/n*3/2);
    dst_p[3] = cv::Point2f(604+268/n,720);

    cv::Mat perspective_mat = cv::getPerspectiveTransform(src_p, dst_p);

    // get image every 5s[W]
    if(countt== 0 || countt ==120)
    { 
      // variations [W]
      //------------------------------------------------------------------------------------------
      // Image generation variation[W]
      cv::Mat src(cv::Size(1280,720), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
      // 2022.07.06 kmeans Matrixc variations[W]
      cv::Mat res, points, labels, centers;
      // color detect and binarization Matrix variations[W]
      cv::Mat hsv,red_mask, red_image, gray, blur;
      // edge Matrix variation[W]
      cv::Mat edge;
      // varaitons of kmeans algorithm
      int width, height, x, y, n, nPoints, cIndex, iTemp, i;
      const int cluster_k = 8;
      //------------------------------------------------------------------------------------------
    
      cv::warpPerspective(src, dst, perspective_mat, cv::Size(1280,720));
      // cv::imshow("src", src); // original image[W]
      //cv::imshow("dst", dst); // perspective transformation image[W]

      // apply filter and get edge [W]
      cvtColor(dst,gray, cv::COLOR_BGR2GRAY);
      GaussianBlur( gray, blur, cv::Size(7, 7), 0);
      Canny(blur, edge, 10, 150);
      //cv::imshow("edge", edge); // edge image[W]

      // initailization of renewal flag variation[W]
      countt=0;
    
      // recognize image informations[W]
      width = dst.cols, height = dst.rows;
      nPoints = width * height;
 
      // initialization (create)[W]
      points.create(nPoints, 1, CV_32FC3);        // input data[W]
      centers.create(cluster_k, 1, points.type());        // results of k means[W]
      res.create(height, width, dst.type());      // results images[W]

      // data transform to fitting kmeasn algortihm[W]
      // there's no way to save computing time????[W]
      for(y = 0, n = 0; y < height; y++)
      {
          for(x = 0; x < width; x++, n++)
          {
              points.at<cv::Vec3f>(n)[0] = dst.at<cv::Vec3b>(y, x)[0];
              points.at<cv::Vec3f>(n)[1] = dst.at<cv::Vec3b>(y, x)[1];
              points.at<cv::Vec3f>(n)[2] = dst.at<cv::Vec3b>(y, x)[2];
          } 
      }
      
      // k-means clustering[W]
      kmeans(points, cluster_k, labels, cv::TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0), 
             20, cv::KMEANS_PP_CENTERS, centers);
    
      // visualization of result of kmeans algorithm[W]
      // if We don't need to visualization ...??[W]

      // test v_ 1 2022.07.12 [W]
      //---------------------------------------------------------
      for(y = 0, n = 0; y < height; y++)
      {
          for(x = 0; x < width; x++, n++)
          {
              cIndex = labels.at<int>(n);
              iTemp = cvRound(centers.at<cv::Vec3f>(cIndex)[0]);
              iTemp = iTemp > 255 ? 255 : iTemp < 0 ? 0 : iTemp;
              res.at<cv::Vec3b>(y, x)[0] = (uchar)iTemp;

              iTemp = cvRound(centers.at<cv::Vec3f>(cIndex)[1]);
              iTemp = iTemp > 255 ? 255 : iTemp < 0 ? 0 : iTemp;
              res.at<cv::Vec3b>(y, x)[1] = (uchar)iTemp;

              iTemp = cvRound(centers.at<cv::Vec3f>(cIndex)[2]);
              iTemp = iTemp > 255 ? 255 : iTemp < 0 ? 0 : iTemp;
              res.at<cv::Vec3b>(y, x)[2] = (uchar)iTemp;
          } 
      }
      //---------------------------------------------------------

      // after kmeans image generate, we need to arrange hsv space[W]
      for (i=0; i<cluster_k; i++)
      {
        // bgr2hsv(int(centers.at<cv::Vec3f>(i)[0]),int(centers.at<cv::Vec3f>(i)[1]),int(centers.at<cv::Vec3f>(i)[2]));
        bgr2hsv(int(centers.at<cv::Vec3f>(i)[0]),int(centers.at<cv::Vec3f>(i)[1]),int(centers.at<cv::Vec3f>(i)[2]));
        // cout << "hsv : " << centers.at<cv::Vec3f>(i) << endl;
        // cout << "h:" << dst_hsv[0] << " s:" <<  dst_hsv[1] << " v:"<< dst_hsv[2] << endl;
        cout << "hsv : " << dst_hsv_for_msg[0]<<","<<dst_hsv_for_msg[1]<<","<<dst_hsv_for_msg[2]<< endl;
        dst_hsv.data[3*i+0] = dst_hsv_for_msg[0];
        dst_hsv.data[3*i+1] = dst_hsv_for_msg[1];
        dst_hsv.data[3*i+2] = dst_hsv_for_msg[2];
      } 
      // May be we don't need to see this[W]
      //cv::imshow("Result", res);

      // save result of K-means Image[W]
      img_name = img_name + type;
      //cout << img_name << endl;
      cv::imwrite(img_name, res);
    

      // publish topic every 5s
      msg_hsv.publish(dst_hsv); 
      
    }
    // image renewal flag increasing[W]
    countt++;
    
    // if press 'esc' stop releasing image
    if(cv::waitKey(10)==27) break;
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}

