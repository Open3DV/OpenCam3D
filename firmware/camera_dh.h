#pragma once
#include "GxIAPI.h" 
#include<iostream> 
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp> 
#include <opencv2/core.hpp>  
#include <opencv2/calib3d.hpp>  
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp> 
class CameraDh
{
public:
	CameraDh();
	~CameraDh();

	bool openCamera();

	bool closeCamera(); 
	
	bool warmupCamera();

	void setGenerateBrightnessParam(int model,float exposure);

	bool captureSingleImage(char* buffer);

	bool captureSingleExposureImage(float exposure,char* buffer);

	bool captureRawPhaseImages(char* buffer);

	bool captureRawTest(int num,char* buffer);

	bool CaptureSelfTest();

	bool switchToSingleShotMode();

	bool switchToScanMode();

	bool getExposure(float &value);

	bool setExposure(float value);
	
	bool setGain(float value);
	
	bool getGain(float &value);

	bool setScanExposure(float value);
	
	bool setScanGain(float value);

	bool copyBrightness(char* buffer); 
	
	void setOffsetParam(float offset){
		phase_compensate_value = offset;
	}
	
	void getOffsetParam(float &offset){
		offset = phase_compensate_value;
	}
	/********************************************************************/
	//gpu parallel
	bool captureFrame03ToGpu();
	
	bool captureFrame04ToGpu();
	
	bool captureFrame04RepetitionToGpu(int repetition_count);

	bool captureFrame03RepetitionToGpu(int repetition_count);
	
	bool compensatePhaseBaseScharr(cv::Mat& normal_phase, cv::Mat brightness, float offset_value);

private:

    GX_DEV_HANDLE hDevice_;

	bool camera_opened_state_;

	int image_shift_num_;

	float phase_compensate_value;

	//条纹扫描时，相机曝光值
	float scan_camera_exposure_;
	//条纹扫描时，相机增益值
	float scan_camera_gain_;

	int generate_brigntness_model_;
	float generate_brightness_exposure_time_;
	int buffer_size_;
	char* brightness_buff_;
};

