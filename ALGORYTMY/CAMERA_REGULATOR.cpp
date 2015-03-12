/*
 * CAMERA_REGULATOR.cpp
 *
 *  Created on: 4 mar 2015
 *      Author: szymon
 */



#include "CAMERA_REGULATOR.H"

#include <opencv/cv.h>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video.hpp>
#include <opencv2/stitching.hpp>
#include <iostream>
#include "DEF.H"



void CAMERA_REGULATOR::receiveFrame()
{
		cap >> lastFrame;
		centerX = lastFrame.cols / 2;
		centerY = lastFrame.rows / 2;
}
cv::Mat& CAMERA_REGULATOR::getLastFrame()
{
		receiveFrame();
		return lastFrame;
}
bool CAMERA_REGULATOR::findChessboard() {
	getLastFrame();
	 this->BGR2GRAY(lastFrame,lastFrameGray);
	 
	bool founded = cv::findChessboardCorners(lastFrameGray, csSize,
			foundedCorners,cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

	return founded;
}
void CAMERA_REGULATOR::checkHorizontal() {
		float dif1 = foundedCorners.ptr<cv::Vec2f>(1)->val[0]
				- foundedCorners.ptr<cv::Vec2f>(0)->val[0];
		float dif2 = foundedCorners.ptr<cv::Vec2f>(
				foundedCorners.rows * foundedCorners.cols - 1)->val[0]
				- foundedCorners.ptr<cv::Vec2f>(
						foundedCorners.rows * foundedCorners.cols - 2)->val[0];
		if ((dif1) > (dif2)) {
			//std::cout << "Prawa Bliżej";
			cv::circle(lastFrame,
					cv::Point((centerX - (dif1 - dif2) * 6), centerY), 50, a,
					20);
		} else {
			//std::cout << "Lewa Bliżej";

			cv::circle(lastFrame,
					cv::Point((centerX + (dif2 - dif1) * 6), centerY), 50, a,
					20);
		}
}
void CAMERA_REGULATOR::checkVertical() {




		cv::Scalar b(0,0,200);
		//std::cout << foundedCorners;
		float dif1 = foundedCorners.ptr<cv::Vec2f>(1)->val[1]
				- foundedCorners.ptr<cv::Vec2f>(0)->val[1];

		float dif2 = foundedCorners.ptr<cv::Vec2f>(
				foundedCorners.rows * foundedCorners.cols - 2)->val[0]
				- foundedCorners.ptr<cv::Vec2f>(
						foundedCorners.rows * foundedCorners.cols - 3)->val[1];

		if ((dif1) > (dif2)) {
			//Upper is closer
			cv::circle(lastFrame,
					cv::Point(centerX, (centerY - (dif1 - dif2) * 2)), 30, b,
					20);
		} else {
			//Bottom is closer

			cv::circle(lastFrame,
					cv::Point(centerX, (centerY + (dif2 - dif1) * 2)), 30, b,
					20);
		}
	}
void CAMERA_REGULATOR::BGR2GRAY(cv::Mat & in,cv::Mat & out)
{
		cv::cvtColor(in,out,CV_BGR2GRAY);
}




bool findChessboard(cv::Mat & img)
{

	cv::Mat corners;
    CvSize csSize(8,5);

    bool an=cv::findChessboardCorners(img,csSize,corners,1);

    //if(an)
    //cv::drawChessboardCorners(img, csSize, corners, 1);



	return an;
}

int main()
{

	CAMERA_REGULATOR regulator;

	regulator.run();


//    cv::Mat img = cv::imread("Calibration_Grid.png",GRAY_SCALE);
//
//    std::cout << img.type() << CV_8UC3;
//
//    if(img.empty())
//    {
//    	std::cerr << "Blad otwarcia";
//    	return -1;
//    }
//
//
//    cv::VideoCapture cap(0);
//
//    while(true)
//    {
//    cap >> img;
//
//    if(!img.empty())
//    {
//
//    	cv::cvtColor(img,img,CV_BGR2GRAY);
//    	findChessboard(img);
//    	cv::imshow("OKNO1",img);
//    	cv::waitKey(1);
//
//    }
//
//    }





	//cv::imshow("OKNO",a);


	return 0;
}
