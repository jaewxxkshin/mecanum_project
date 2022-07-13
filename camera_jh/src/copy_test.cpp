#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <iostream>

using namespace std;
using namespace cv;

#define PI 3.1415926

int main() {
	Mat input = imread("/home/mrl/catkin_ws/src/mecanum_project/camera_jh/src/Image2.png", IMREAD_GRAYSCALE);
	Mat contours;
	vector<Vec4i> lines;

	// applying canny algorithm
	Canny(input, contours, 125, 350);

	/************* Probabilistic hough transform *************/
	HoughLinesP(contours,	// InputArray
		lines,				// OutputArray
		1,					// rho
		PI / 180,			// theta (step size)
		60,					// threshold (minimum number of votes)
		100,				// minLength
		20);				// maxGap

	Mat houghLinesP;
	input.copyTo(houghLinesP);

	// Draw the lines
	Scalar color = Scalar(0, 0, 255);
	vector<Vec4i>::const_iterator it = lines.begin();
	while (it != lines.end()) {
		Point pt1((*it)[0], (*it)[1]);
		Point pt2((*it)[2], (*it)[3]);
		line(houghLinesP, pt1, pt2, color);
		++it;
	}

	imwrite("houghLinesP.bmp", houghLinesP);

	/************* Fitting a line *************/
	// black background
	Mat oneline(contours.size(), CV_8U, Scalar(0));
	
	// Select first line and draw white line
	int n = 0;	
	line(oneline,
		Point(lines[n][0], lines[n][1]),
		Point(lines[n][2], lines[n][3]),
		Scalar(255),
		3);	// line width

	imwrite("[BeforeBitwise]online.bmp", oneline);

	// contours AND white line
	bitwise_and(contours, oneline, oneline);	// src1, src2, dst

	// insert into a vector<Point>
	vector<Point> points;

	// Iterate over the pixels to obtain all point positions
	for (int y = 0; y < oneline.rows; y++) {
		// row y
		uchar* rowPtr = oneline.ptr<uchar>(y);

		for (int x = 0; x < oneline.cols; x++) {
			// column x
			if (rowPtr[x]) {	// if on a contour
				points.push_back(Point(x, y));
			}
		}
	}

	Vec4f line;
	fitLine(points, line,
		DIST_L2,		// distance type
		0,				// not used with L2 distance
		0.01, 0.01);	// accuracy

	int x0 = line[2];	// a point on the line
	int y0 = line[3];
	int x1 = x0 + 150 * line[0];
	int y1 = y0 + 150 * line[1];
	// draw the line
	Mat result;
	input.copyTo(result);
	cv::line(result, Point(x0, y0), Point(x1, y1), 
		0, 2);	// color and thickness


	/************* Image writer *************/
	imwrite("input.bmp", input);
	imwrite("contours.bmp", contours);
	imwrite("[AfterBitwise]online.bmp", oneline);
	imwrite("Fitted line.bmp", result);
}