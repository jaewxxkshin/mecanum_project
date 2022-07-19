#include <get_waypoint.hpp>

using namespace cv;
using namespace std;

// select line which we want to tracking(by color) - demo [HW]
int color = 1;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_realsense_opencv_tutorial");
    ros::NodeHandle nh;
    // ros::Publisher msg_hsv = nh.advertise<std_msgs::Int16MultiArray>("msg_hsv", 1000);  
    
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::frameset frames;
    rs2::frame color_frame; 
    
    cfg.enable_stream(RS2_STREAM_COLOR, 1280,720, RS2_FORMAT_BGR8, 30);
    pipe.start(cfg);    

    
    // BGR -> HSV [HW]
    cvtColor(src,hsv,COLOR_BGR2HSV); 
    

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

        // perspective transform [JH]
        src_p[0] = cv::Point2f(443,478);
        src_p[1] = cv::Point2f(336,720);
        src_p[2] = cv::Point2f(855,474);
        src_p[3] = cv::Point2f(940,720)
        
        dst_p[0] = cv::Point2f(604-268/n,720-268/n*3/2);
        dst_p[1] = cv::Point2f(604-268/n,720);
        dst_p[2] = cv::Point2f(604+268/n,720-268/n*3/2);
        dst_p[3] = cv::Point2f(604+268/n,720)
        
        cv::Mat perspective_mat = cv::getPerspectiveTransform(src_p, dst_p);

        // get image every 5s[W]
        if(countt== 0 || countt ==120) 
        { 
            // initailization of renewal flag variation[W]
            countt=0;
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
            cv::warpPerspective(src, dst, perspective_mat, cv::Size(1280,720));
            //------------------------------------------------------------------------------------------        
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

}












