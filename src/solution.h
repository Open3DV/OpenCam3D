#pragma once

#include "iostream"
#include "encode.h" 
#include <opencv2/highgui.hpp> 
#include "../firmware/camera_param.h"


class Solution
{

public:

	Solution();
	~Solution();


	bool getCameraCalibData(std::string ip, struct CameraCalibParam& param);

	bool saveCameraCalibData(std::string path, struct CameraCalibParam& param);

	bool readCameraCalibData(std::string path, struct CameraCalibParam& param);

	bool getCameraVersion(const char* ip, int& version);

	bool setCameraVersion(int version);

	bool captureRaw01(const char* ip, std::vector<cv::Mat>& patterns);

	bool reconstructFrame01(std::vector<cv::Mat> patterns, cv::Mat& depth, cv::Mat& brightness);

	bool reconstructFrame01BaseFirmware(const char* ip, std::vector<cv::Mat>& patterns, cv::Mat& depth, cv::Mat& brightness);

	bool readPatterns(std::string dir, std::vector<cv::Mat>& patterns);

	bool savePatterns(std::string dir, std::vector<cv::Mat> patterns);

private:
	void getFiles(std::string path, std::vector<std::string>& files);

private:
	int camera_version_;
	struct CameraCalibParam calib_param_;

};

