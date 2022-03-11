#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>


class AnalyseError
{
public:
	AnalyseError();
	~AnalyseError(); 

	bool expandEdge(const cv::Mat& img, int edge[], const int edgeID);

	cv::Rect InSquare(cv::Mat& img, const cv::Point center);

	double computeError(cv::Mat err_map);

};

