#pragma once
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

class PrecisionTest
{

public:
	PrecisionTest();
	~PrecisionTest();
	 
	float fitPlaneBaseLeastSquares(std::vector<cv::Point3f> points_3d,std::vector<float>& plane,cv::Point3f &center_point);

	float computeTwoPointDistance(cv::Point3f p0, cv::Point3f p1);

	float computePointToPlaneDistance(cv::Point3f point, std::vector<float> plane);
 
};

