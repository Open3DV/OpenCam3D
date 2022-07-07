#pragma once
#include "camera_galaxy.h"
#include "camera_basler.h"
#include "lightcrafter3010.h"
#include "camera_param.h"


class Scan3D
{
public:
	Scan3D();
	~Scan3D();

	bool init();

    bool captureFrame04();
 
    bool readCalibParam();
 
    void copyBrightnessData(unsigned char* &ptr);
    
    void copyDepthData(float* &ptr);

private:
 
    Camera* camera_;
    LightCrafter3010 lc3010_;


    struct CameraCalibParam calib_param_;
 

    int image_width_;
    int image_height_;

    unsigned char* buff_brightness_;
    float* buff_depth_;
    float* buff_pointcloud_;

};
