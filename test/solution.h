#pragma once
  
#include "iostream"
#include "encode.h" 
#include <opencv2/highgui.hpp>
#include "reconstruct.h" 

//struct _finddata_t
//{
//	unsigned attrib;
//	time_t time_create;
//	time_t time_access;
//	time_t time_write;
//	_fsize_t size;
//	char name[_MAX_FNAME];
//};

class DfSolution
{

public:

	bool reconstructMixedVariableWavelengthPatternsBaseXYSR(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param,std::string pointcloud_path = "./");

	bool reconstructMixedVariableWavelengthXPatternsBaseTable(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string pointcloud_path = "./");

	bool testCalibrationParamBasePlane(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string err_map_path = "./");

	bool testCalibrationParamBaseBoard(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string err_map_path = "./");

	bool readCameraCalibData(std::string path, struct CameraCalibParam& param);
  
	bool getCameraCalibData(std::string ip, struct CameraCalibParam& param);

	std::string& replaceAll(std::string& str, const   std::string& old_value, const   std::string& new_value);

	bool savePatterns(std::string dir,std::vector<cv::Mat> patterns);

	bool saveCameraCalibData(std::string path, struct CameraCalibParam& param);

	bool captureMixedVariableWavelengthPatterns(std::string ip,std::vector<cv::Mat>& patterns);

	void getFiles(std::string path, std::vector<std::string>& files);

	bool readImages(std::string dir, std::vector<cv::Mat>& patterns);

 


};

