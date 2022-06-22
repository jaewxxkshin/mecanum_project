#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <librealsense2/rs.hpp>
#include <iostream>

//using namespace cv;
using namespace std;

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
    cv::Mat src(cv::Size(1280,720), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

    //imshow("Display Image", src);

    //perspective transform [W]
    ///////////////////////////////////////////////////
    cv::Mat dst;
    cv::Point2f src_p[4], dst_p[4];

    src_p[0] = cv::Point2f(0, 0);
    src_p[1] = cv::Point2f(1280, 0);
    src_p[2] = cv::Point2f(0, 720);
    src_p[3] = cv::Point2f(1280, 720);

    dst_p[0] = cv::Point2f(0, 0);
    dst_p[1] = cv::Point2f(1280, 0);
    dst_p[2] = cv::Point2f(0, 720);
    dst_p[3] = cv::Point2f(1280, 720);

    cv::Mat perspective_mat = cv::getPerspectiveTransform(src_p, dst_p);

    cv::warpPerspective(src, dst, perspective_mat, cv::Size(1280,720));
    cv::imshow("src", src);
    cv::imshow("dst", dst);
    ///////////////////////////////////////////////////
    if(cv::waitKey(10)==27) break;
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
