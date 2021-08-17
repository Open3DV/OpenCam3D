#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp> 
#include "../Firmware/camera_param.h"

class DF_Reconstruct
{
public:
	DF_Reconstruct();
	~DF_Reconstruct(); 

	bool setCalibData(CameraCalibParam param);

	bool readCalibData(cv::Mat& camera_intrinsic, cv::Mat& project_intrinsic, cv::Mat& camera_distortion,
		cv::Mat& projector_distortion, cv::Mat& rotation_matrix, cv::Mat& translation_matrix, cv::Mat& M_1, cv::Mat& M_2);

	bool setCalibrateData(cv::Mat camera_intrinsic, cv::Mat project_intrinsic, cv::Mat camera_distortion,
		cv::Mat projector_distortion, cv::Mat rotation_matrix, cv::Mat& translation_matrix);

	bool rebuildPoints(std::vector<cv::Point2d> camera_points, std::vector<cv::Point2d> dlp_points, std::vector<cv::Point3d> &rebuild_points);

	bool rebuildData(cv::Mat unwrap_map_x, cv::Mat unwrap_map_y, int period_nun, cv::Mat &deep_map);

	bool undistortedPoints(std::vector<cv::Point2d> distortPoints,cv::Mat intrinsic,cv::Mat distortion, std::vector<cv::Point2d> &undisted_points);

	bool undistortedImage(cv::Mat distort_img, cv::Mat& undistort_img);
 
	bool rebuildPoints(std::vector<cv::Point2d> camera_points, std::vector<cv::Point2d> dlp_points, std::vector<cv::Point3d> &rebuild_points,std::vector<double> &error_list);

	bool phaseToCoord(cv::Mat unwrap_map, float size, cv::Mat& coord_map);

	bool depthTransformPointcloud(cv::Mat depth_map, cv::Mat& point_cloud_map);

	bool pointError(cv::Mat point_cloud_0, cv::Mat point_cloud_1, cv::Mat& error_map);

private:

	std::string calib_path_;

	cv::Mat M_1_;
	cv::Mat M_2_;

	cv::Mat camera_intrinsic_;
	cv::Mat project_intrinsic_;
	cv::Mat camera_distortion_;
	cv::Mat projector_distortion_;
	cv::Mat rotation_matrix_;
	cv::Mat translation_matrix_;

	double dlp_width_;
	double dlp_height_;
};

