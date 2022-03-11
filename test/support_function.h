#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include<chrono>
#include<ctime>
#include <time.h>
#include<stddef.h> 
#include <io.h>

using namespace std;
using namespace std::chrono;


//struct _finddata_t
//{
//	unsigned attrib;
//	time_t time_create;
//	time_t time_access;
//	time_t time_write;
//	_fsize_t size;
//	char name[_MAX_FNAME];
//};
std::vector<std::string> vStringSplit(const std::string& s, const std::string& delim);

bool getFilesList(std::string dirs, std::vector<std::vector<std::string>>& files_list);

void getJustCurrentDir(std::string path, std::vector<std::string>& dirs);

void getFiles(std::string path, std::vector<std::string>& files);

std::string GetTimeStamp();

bool SavePointToTxt(cv::Mat deep_map, std::string path, cv::Mat texture_map);

//Ωÿ»°z-map RoiÕº
bool MaskZMap(cv::Mat& z_map, cv::Mat mask);

bool MapToColor(cv::Mat deep_map, cv::Mat& color_map, cv::Mat& grey_map, int low_z, int high_z); 

bool renderBrightnessImage(cv::Mat brightness, cv::Mat& render_brightness);

bool renderErrorMap(cv::Mat err_map, cv::Mat& color_map, cv::Mat& gray_map, float low_v, float high_v);

bool MergeTextureMap(std::vector<cv::Mat> patterns, cv::Mat& texture_map);


