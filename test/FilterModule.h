#pragma once
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp>

class FilterModule
{
public:
	FilterModule();
	~FilterModule();

	bool RadiusOutlierRemoval(cv::Mat& point_cloud_map, cv::Mat& mask, double radius, int points_num);

	bool statisticOutlierRemoval(cv::Mat& point_cloud_map, int num_neighbors, double threshold_radio);
private:

	double computePointsDistance(cv::Vec3d p0, cv::Vec3d p1);

	double computePointsDistance(cv::Point3d p0, cv::Point3d p1);

	double computePointsZDistance(cv::Point3d p0, cv::Point3d p1);

	double computePointsDistance(cv::Point3f p0, cv::Point3f p1);

	double computePointsDistance(cv::Point2f p0, cv::Point2f p1);

	int findNearPointsnum(cv::Mat& point_cloud_map, cv::Mat& mask, cv::Point pos, double radius);

	bool statisticRegion(cv::Mat& point_cloud_map, cv::Point pos, int num_neighbors, double threshold_radio);

	bool computeNormalDistribution(cv::Mat& point_cloud_map, int num_neighbors, double& mean, double& sd);

	bool getDistanctList(cv::Mat point_cloud_map, cv::Point pos, int num_neighbors, double& mean, std::vector<double>& dist_list);
};

