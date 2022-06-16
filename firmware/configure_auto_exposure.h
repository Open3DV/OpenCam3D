#pragma once 
#include <iostream> 
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp> 
#include <opencv2/core.hpp>  
#include <opencv2/calib3d.hpp>  
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>   
#include "camera_param.h" 
#include "easylogging++.h"
 
class ConfigureAutoExposure
{
public:
	ConfigureAutoExposure();
	~ConfigureAutoExposure();

    bool evaluateBrightnessParam(cv::Mat brightness_mat, cv::Mat mask, float& average_pixel, float& over_exposure_rate);
 
private:
	  
 
};
 