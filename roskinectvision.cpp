#include <stdlib.h>
#include <stdio.h>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

int CANNY_LOWTHRESHOLD = 50;
int CANNY_RATIO = 3;
int CANNY_KERNELSIZE = 3;

void DetectEdges(cv::Mat color_image);

/** @function main */
int main(int argc, char** argv) {
	// Load the images
	cv::Mat imagefull = cv::imread("images/kinect.bmp");
	//cv::Mat imgres;
	//cv::resize(imagefull, imgres, cv::Size(640, 480), 0, 0, cv::INTER_NEAREST);
	//cv::namedWindow("Original", cv::CV_)
	//cv::imshow("Original", imgres);
	//cv::resizeWindow("Original", 640, 480);
	DetectEdges(imagefull);
	cv::waitKey(0);
	return 0;
}

void DetectEdges(cv::Mat color_image) {
	cv::Mat gray_image, canny_image;
	cv::cvtColor(color_image, gray_image, cv::COLOR_BGR2GRAY);
	cv::Canny(gray_image, canny_image, CANNY_LOWTHRESHOLD,
			CANNY_LOWTHRESHOLD * CANNY_RATIO, CANNY_KERNELSIZE);
	cv::vector<cv::Vec2f> lines;
	cv::HoughLines(canny_image, lines, 45, CV_PI / 180, 100, 0, 0);
	//cv::vector<cv::Vec4i> lines;
	//cv::HoughLinesP(canny_image, lines, 1, CV_PI/180, 50, 50, 10 );

	for (size_t i = 0; i < lines.size(); i++) {
		float rho = lines[i][0], theta = lines[i][1];
		cv::Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a * rho, y0 = b * rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		cv::line(color_image, pt1, pt2, cv::Scalar(0, 0, 255), 3, CV_AA);
	}
	cv::imshow("Original", color_image);
	cv::imshow("DetectedEdges", canny_image);
	//cv::resizeWindow("DetectedEdges", 640, 480);
}
