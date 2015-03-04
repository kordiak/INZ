/*
 * CAMERA_REGULATOR.cpp
 *
 *  Created on: 4 mar 2015
 *      Author: szymon
 */




#include <opencv/cv.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video.hpp>
#include <opencv2/stitching.hpp>


int main()
{
	cv::Mat a;

	cv::VideoCapture cap(0);

	cap >> a;


	cv::imshow("OKNO",a);

	cv::waitKey();
	return 0;
}