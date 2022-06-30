#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <librealsense2/rs.hpp>
#include <iostream>

//using namespace cv;
using namespace std;
cv::Mat img_roi;

int n = 2;
int countt  = 0;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_realsense_opencv_tutorial");
  ros::NodeHandle nh;

  cout << "OpenCV version : " << CV_VERSION << endl;
  cout << "Major version : "  << CV_MAJOR_VERSION << endl;

  rs2::pipeline pipe;
  rs2::config cfg;
  rs2::frameset frames;
  rs2::frame color_frame;


  cfg.enable_stream(RS2_STREAM_COLOR, 1280,720, RS2_FORMAT_BGR8, 30);
  pipe.start(cfg);

  for(int i=0; i < 30; i ++)
  {
    frames = pipe.wait_for_frames();
  }

  //cv::namedWindow("Display Imagee", cv::WINDOW_AUTOSIZE);

  ros::Rate loop_rate(30);

  while(ros::ok())
  {
    frames = pipe.wait_for_frames();
    color_frame = frames.get_color_frame();
    cv::Mat dst;
    cv::Point2f src_p[4], dst_p[4];
    cout << "count : " << countt << endl;
    //imshow("Display Image", src);

    //perspective transform [W]
    ///////////////////////////////////////////////////
    

    //img_roi's shape = (560, 560)[W]
    //img_roi = src(cv::Rect(400, 160, 560, 560));
    // cv::imwrite("output.png", img_roi);
    //cv::imwrite("original.png", src);

    // [jaeho]
    src_p[0] = cv::Point2f(443,478);
    src_p[1] = cv::Point2f(336,720);
    src_p[2] = cv::Point2f(855,474);
    src_p[3] = cv::Point2f(940,720);

    dst_p[0] = cv::Point2f(604-268/n,720-268/n*3/2);
    dst_p[1] = cv::Point2f(604-268/n,720);
    dst_p[2] = cv::Point2f(604+268/n,720-268/n*3/2);
    dst_p[3] = cv::Point2f(604+268/n,720);

    //[jaewook]
    // src_p[0] = cv::Point2f(565,351);
    // src_p[1] = cv::Point2f(445,720);
    // src_p[2] = cv::Point2f(887,351);
    // src_p[3] = cv::Point2f(1065,720);

    // dst_p[0] = cv::Point2f(755-310/n,720-310/n*6/2);
    // dst_p[1] = cv::Point2f(755-310/n,720);
    // dst_p[2] = cv::Point2f(755+310/n,720-310/n*6/2);
    // dst_p[3] = cv::Point2f(755+310/n,720);


    cv::Mat perspective_mat = cv::getPerspectiveTransform(src_p, dst_p);

    //cv::warpPerspective(src, dst, perspective_mat, cv::Size(1280,720));
  // jaewook's code 
    if(countt== 0 || countt ==150)
  { 
    cv::Mat src(cv::Size(1280,720), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::warpPerspective(src, dst, perspective_mat, cv::Size(1280,720));
    cv::imshow("src", src);
    cv::imshow("dst", dst);
    cv::imwrite("kmeans.png", dst);
    // color detect and binarization[W]
    ///////////////////////////////////////////////////
    cv::Mat hsv,red_mask, red_image, gray, blur;
    //cvtColor(img_roi, hsv, cv::COLOR_BGR2HSV);
    //GaussianBlur( src, dst, kernel_size, sigma_x, sigma_y, borderType) 
    cvtColor(dst,gray, cv::COLOR_BGR2GRAY);
    GaussianBlur( gray, blur, cv::Size(7, 7), 0);
    // cvtColor(blur, hsv, cv::COLOR_BGR2HSV);
    // cv::Scalar lower_red = cv::Scalar(-10, 100, 100);
    // cv::Scalar upper_red = cv::Scalar(10, 255, 255);
    // inRange(hsv, lower_red, upper_red, red_mask);
    // bitwise_and(dst, dst, red_image, red_mask);
    // cv::imshow("mask", red_mask);
    cv::imshow("blur", blur);
    cv::Mat edge;
    Canny(blur, edge, 10, 150);
    cv::imshow("edge", edge);
    countt=0;
   }
    // image renewal preiod [W]
    countt++;
  // jaeho's code
    // if(countt > 150)
    // {
    // countt=0;
    // frames = pipe.wait_for_frames();
    // color_frame = frames.get_color_frame();
    // cv::Mat src(cv::Size(1280,720), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    // cv::warpPerspective(src, dst, perspective_mat, cv::Size(1280,720));
    // cv::imshow("src", src);
    // cv::imshow("dst", dst);
    // // color detect and binarization[W]
    // ///////////////////////////////////////////////////
    // cv::Mat hsv,red_mask, red_image;
    // //cvtColor(img_roi, hsv, cv::COLOR_BGR2HSV);
    // cvtColor(dst, hsv, cv::COLOR_BGR2HSV);
    // cv::Scalar lower_red = cv::Scalar(-10, 100, 100);
    // cv::Scalar upper_red = cv::Scalar(10, 255, 255);
    // inRange(hsv, lower_red, upper_red, red_mask);
    // bitwise_and(dst, dst, red_image, red_mask);
    // cv::imshow("mask", red_mask);
    // ////////////////////////////////////////////////// 
    // }
    // else
    // {
    // countt ++;
    // }



    if(cv::waitKey(10)==27) break;
    loop_rate.sleep();
    ros::spinOnce();
  }
  
  //validation
  //cout <<"w: "<<img_roi.size().width<<endl;
  //cout <<"h: "<<img_roi.size().height<<endl;
  //
  return 0;
}

