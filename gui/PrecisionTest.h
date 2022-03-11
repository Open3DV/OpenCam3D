#pragma once
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp> 
#include <iostream>
#include <fstream>

using namespace cv;

class PrecisionTest
{

public:
	PrecisionTest();
	~PrecisionTest();
	 
	float fitPlaneBaseLeastSquares(std::vector<cv::Point3f> points_3d,std::vector<float>& plane,cv::Point3f &center_point);

	float computeTwoPointDistance(cv::Point3f p0, cv::Point3f p1);

	float computePointToPlaneDistance(cv::Point3f point, std::vector<float> plane);
 
	double computeTwoPointSetDistance(std::vector<cv::Point3f> point_set_0, std::vector<cv::Point3f> point_set_1);

	double transformPoints(std::vector<cv::Point3f> org_points, std::vector<cv::Point3f>& trans_points, cv::Mat R, cv::Mat T);

	// R * pc2 + t = pc1
	// pc1: Mat (N, 3) CV_64F
	// pc2: Mat (N, 3) CV_64F
	// r: Mat (3, 3) CV_64F
	// t: Mat (3, 1) CV_64F
	void svdIcp(const Mat& pc1, const Mat& pc2, Mat& r, Mat& t);

private:


	// arr1: Mat (1, N) CV_64F
	// arr2: Mat (1, M) CV_64F
	// return: Mat (N, M) CV_64F
	Mat kronProductArr(const Mat& arr1, const Mat& arr2);

	// end == -1: take only 1 row or colume
	Rect sliceMask(int rowStart, int rowEnd, int colStart, int colEnd);
	// pc: Mat (N, 3) CV_64F
	// return: pcMean (1, 3) CV_64F
	Mat pcMean(const Mat& pc);
};

