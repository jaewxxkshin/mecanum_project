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

// // array for hsv msg
// std_msgs::Int16MultiArray dst_hsv;

std_msgs::Int16MultiArray dst_hsv;

//using namespace cv;
using namespace std;
cv::Mat img_roi;

int n = 2;
int countt  = 0;

// RGV2HSV function [k] 

#define min_f(a, b, c)  (fminf(a, fminf(b, c)))
#define max_f(a, b, c)  (fmaxf(a, fmaxf(b, c)))

#define Cluster_number 8 // [jh]

#define img_width 1280
#define img_height 720

int dst_hsv_for_msg[3];

void set_array()
{
  dst_hsv.data.resize(18);
}

int img_x ( const unsigned int &x)
{ 
  int converted_x = 0;
  converted_x= x-img_width/2;
  return converted_x;
}

int img_y ( const unsigned int &y)
{
  int converted_y = 0;
  converted_y= -(y-img_height);
  return converted_y;
}

void bgr2hsv(const unsigned int &src_b, const unsigned int &src_g, const unsigned int &src_r)
{
    float b = src_b / 255.0f;
    float g = src_g / 255.0f;
    float r = src_r / 255.0f;

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
    
    // return dst_hsv;
    return;
}


int main(int argc, char **argv)
{
  set_array();
  ros::init(argc, argv, "ros_realsense_opencv_tutorial");
  ros::NodeHandle nh;

  ros::Publisher msg_hsv = nh.advertise<std_msgs::Int16MultiArray>("msg_hsv", 1000);

  cout << "OpenCV version : " << CV_VERSION << endl;
  cout << "Major version : "  << CV_MAJOR_VERSION << endl;


//   rs2::pipeline pipe;
//   rs2::config cfg;
//   rs2::frameset frames;
//   rs2::frame color_frame;
  

  int bgr2hsv(const unsigned int &src_b, const unsigned int &src_g, const unsigned int &src_r);
  
//   cfg.enable_stream(RS2_STREAM_COLOR, 1280,720, RS2_FORMAT_BGR8, 30);
//   pipe.start(cfg);

//   for(int i=0; i < 30; i ++)
//   {
//     frames = pipe.wait_for_frames();
//   }

  //cv::namedWindow("Display Imagee", cv::WINDOW_AUTOSIZE);

  ros::Rate loop_rate(30);

  while(ros::ok())
  {
    msg_hsv.publish(dst_hsv);

    // frames = pipe.wait_for_frames();
    // color_frame = frames.get_color_frame();
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
    
    // cv::Mat src(cv::Size(1280,720), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat src = cv::imread("/home/mrl/catkin_ws/original.png");
    cv::warpPerspective(src, dst, perspective_mat, cv::Size(1280,720));
    cv::imshow("src", src);
    cv::imshow("dst", dst);
    
    //function test[k]
    // int dst_h = 0;
    // int dst_s = 0;
    // int dst_v = 0;
    // dst_h,dst_s,dst_v= bgr2hsv(134,52,56);
    bgr2hsv(134,52,56);

    //////////////////
    
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

    // // 2022.07.06 kmeans first trial[W]
    // // data reshape to examine kmeans algorithm
    // cv::Mat data;
    // cv::Mat flat_label;
    // cv::Mat temp, res;
    // data = dst.reshape( 3, dst.rows * dst.cols);
    // data.convertTo (data, CV_32FC3);
    // // just check[W]
    // cout <<"shape: "<<data.size()<<data.channels()<<endl;;

    // // kmeans algorithm[W]
    // cv::Mat bestLabels, centers, clustered;
    // int K = 8;
    // cv::kmeans(data, K, bestLabels,
    //         cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0),
    //         20, cv::KMEANS_PP_CENTERS, centers);
    // centers.convertTo (centers, CV_8UC3);
    // flat_label = bestLabels.reshape(1, 1);
    // // cout << flat_label <<endl;
    // // cout <<"shape: "<<flat_label.size()<< flat_label.channels()<<endl;;
    // //data = centers(flat_label);
    // // cout << *typeid(centers).name() <<endl;
    // for (int i = 0; i < flat_label.cols; i++)
    // {
    //   uchar idx = flat_label.at<uchar>(i);
    //   uchar r = centers.at<Vec3b>(idx)[0];
    
    // }

    // // 2022.07.06 kmeans second trial[W]
    cv::Mat res, points, labels, centers;
    int width, height, x, y, n, nPoints, cIndex, iTemp,i;
    const int k = Cluster_number;
     
    // 이미지 정보 파악
    width = dst.cols, height = dst.rows;
    nPoints = width * height;
 
    // 초기화
    points.create(nPoints, 1, CV_32FC3);        // 입력 데이터
    centers.create(k, 1, points.type());        // k개의 mean 결과값들
    res.create(height, width, dst.type());      // 결과 영상

    // kmeans 함수에 맞게 데이터 변환
    for(y = 0, n = 0; y < height; y++)
    {
        for(x = 0; x < width; x++, n++)
        {
            points.at<cv::Vec3f>(n)[0] = dst.at<cv::Vec3b>(y, x)[0];
            points.at<cv::Vec3f>(n)[1] = dst.at<cv::Vec3b>(y, x)[1];
            points.at<cv::Vec3f>(n)[2] = dst.at<cv::Vec3b>(y, x)[2];
        } 
    }


    // dst.convertTo (dst, CV_32FC3);
    // k-means clustering
    kmeans(points, k, labels, cv::TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0), 
           20, cv::KMEANS_PP_CENTERS, centers);
    std::cout << centers << std::endl;
    // 클러스터링 결과물을 이미지로 보여주기 위한 변환
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

  for (i=0; i<Cluster_number; i++)
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
   // after kmeans image generate, we need to arrange hsv space[W]

   cv::imshow("Result", res); 
   //cv::imwrite("kmeans.png", res);

   //convert HSV
   
   cv::cvtColor(res,hsv,cv::COLOR_BGR2HSV); 
   cv::imshow("HSV", hsv); 
   
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

