#include <iostream>
#include <fstream>
#include "case.h"
#include "status.h" 
#include "../src/solution.h"
#include "evaluate.h"

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

	//读图
	struct CameraCalibParam calibration_param_;
	Solution solution_;
	std::vector<cv::Mat> patterns_;


	bool ret = solution_.captureRaw01(ip, patterns_);

	if (!ret)
	{
		std::cout << "Capture Raw 01 Failed!" << std::endl;
		return -1;
	}
	solution_.savePatterns(raw_path, patterns_);




	std::cout << "Capture Raw 01 Finished!" << std::endl;
	ret = solution_.getCameraCalibData(ip, calibration_param_);

	if (ret)
	{
		solution_.saveCameraCalibData(calib_path, calibration_param_);
	}
	else
	{
		std::cout << "Get Camera Calib Data Failure!";
	}

	std::cout << "Get Camera Calib Data Finished!" << std::endl;

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

	std::cout << "Reconstruct Frame 01 Finished!" << std::endl;

	std::string host_brightness_path = folderPath + "\\host_brightness.bmp";
	cv::imwrite(host_brightness_path, brightness);

	std::string host_depth_path = folderPath + "\\host_depth.tiff";
	cv::imwrite(host_depth_path, depth);


	cv::Mat depth_firmware;
	cv::Mat brightness_firmware;
	ret = solution_.reconstructFrame01BaseFirmware(ip, patterns_, depth_firmware, brightness_firmware);
	if (!ret)
	{
		std::cout << "Reconstruct Frame01 Base Firmware Failed!" << std::endl;
		return false;
	}


	std::string firmware_brightness_path = folderPath + "\\firmware_brightness.bmp";
	cv::imwrite(firmware_brightness_path, brightness_firmware);

	std::string firmware_depth_path = folderPath + "\\firmware_depth.tiff";
	cv::imwrite(firmware_depth_path, depth_firmware);


	bool brightness_compare = singlePixelCompare(brightness, brightness_firmware, 0.5);

	bool depth_compare = singlePixelCompare(depth, depth_firmware, 0.5);

	std::cout << "Reconstruct brightness memcmp code: " << brightness_compare << std::endl;
	std::cout << "Reconstruct depth memcmp code: " << depth_compare << std::endl;

	cv::Mat diff = depth - depth_firmware;

	return brightness_compare && depth_compare;
}


