#include <iostream>
#include <fstream>
#include "case.h"
#include "status.h"
#include "file_io.h"
#include "../src/solution.h"

using namespace std;



bool get_frame_01(const char* ip)
{
	//创建文件夹
	std::string folderPath = "..\\test_frame_01";
	std::string mkdir_cmd = std::string("mkdir ") + folderPath;
	int code = system(mkdir_cmd.c_str());

	//采集条纹
	std::string raw_path = folderPath + "\\raw_01";
	std::string calib_path = folderPath + "\\param.txt";
	//get_raw_01(ip, raw_Path.c_str());

	//读图
	struct CameraCalibParam calibration_param_;
	Solution solution_;
	std::vector<cv::Mat> patterns_;

	bool ret = solution_.captureRaw01(ip, patterns_);

	if (ret)
	{
		solution_.savePatterns(raw_path, patterns_);
	}

	ret = solution_.getCameraCalibData(ip, calibration_param_);

	if (ret)
	{
		solution_.saveCameraCalibData(calib_path, calibration_param_);
	}
	else
	{
		std::cout << "Get Camera Calib Data Failure!";
	}

	int camera_version = 0;
	ret = solution_.getCameraVersion(ip, camera_version);
	if (!ret)
	{
		std::cout << "Get Camera Version Failed!" << std::endl;
		return false;
	}

	ret = solution_.setCameraVersion(camera_version);
	if (!ret)
	{
		std::cout << "Set Camera Version Error!" << std::endl;
		return false;
	}

	cv::Mat depth;
	cv::Mat brightness;
	ret = solution_.reconstructFrame01(patterns_, depth, brightness);
	if (!ret)
	{
		std::cout << "Reconstruct Frame 01 Failed!" << std::endl;
		return false;
	}

	cv::Mat depth_remote;
	cv::Mat brightness_remote;
	ret = solution_.reconstructFrame01BaseFirmware(ip, patterns_, depth_remote, brightness_remote);
	if (!ret)
	{
		std::cout << "Reconstruct Frame01 Base Firmware Failed!" << std::endl;
		return false;
	}

	bool brightness_same = memcmp(brightness.data, brightness_remote.data, brightness.total() * brightness.elemSize());
	std::cout << "Reconstruct brightness is the same: " << brightness_same << std::endl;


	bool depth_same = memcmp(depth.data, depth_remote.data, depth.total() * depth.elemSize());
	std::cout << "Reconstruct depth is the same: " << depth_same << std::endl;

	return brightness_same && depth_same;
}


