#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp> 
#include "../firmware/camera_param.h"

class Reconstruct
{
public:
	Reconstruct();
	~Reconstruct();

	bool setCalibData(CameraCalibParam param);

	bool setCameraVersion(int version);

	bool undistortedPoints(std::vector<cv::Point2f> distortPoints, cv::Mat intrinsic, cv::Mat distortion, std::vector<cv::Point2f>& undisted_points);

	bool rebuildPoints(std::vector<cv::Point2f> camera_points, std::vector<cv::Point2f> dlp_points, std::vector<cv::Point3f>& rebuild_points, std::vector<float>& error_list);

	bool rebuildData(cv::Mat unwrap_map_x, cv::Mat unwrap_map_y, int period_nun, cv::Mat& deep_map, cv::Mat& err_map);

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

