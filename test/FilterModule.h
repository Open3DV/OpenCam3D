#pragma once
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp>

class FilterModule
{
public:
	FilterModule();
	~FilterModule();

	bool RadiusOutlierRemoval(cv::Mat &point_cloud_map, cv::Mat &mask, double radius, int points_num);


private:

	double computePointsDistance(cv::Vec3d p0, cv::Vec3d p1);

	double computePointsDistance(cv::Point3d p0, cv::Point3d p1);

	double computePointsDistance(cv::Point3f p0, cv::Point3f p1);

	double computePointsDistance(cv::Point2f p0, cv::Point2f p1);


	int findNearPointsnum(cv::Mat &point_cloud_map, cv::Mat &mask, cv::Point pos, double radius);

};

