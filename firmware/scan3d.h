#pragma once
#include "camera_galaxy.h"
#include "camera_basler.h"
#include "lightcrafter3010.h"
#include "camera_param.h"
#include "vector"

class Scan3D
{
public:
	Scan3D();
	~Scan3D();

	bool init();

    bool setParamHdr(int num,std::vector<int> led_list,std::vector<int> exposure_list);

    bool setParamExposure(float exposure);

    bool setParamGain(float gain);

    bool setParamLedCurrent(int current);

    bool setParamConfidence(float confidence);
    
    bool setParamGenerateBrightness(int model,int exposure);

    bool setCameraVersion(int version);

    void getCameraVersion(int &version);
    
    /************************************************************************/

    bool captureRaw01(unsigned char* buff);

    bool captureRaw02(unsigned char* buff);
    
    bool captureRaw03(unsigned char* buff);
    
    bool captureRaw04(unsigned char* buff);

    bool captureFrame04();

    bool captureFrame04Hdr();
    
    /************************************************************************/
 
    bool readCalibParam();

    bool loadCalibData();

    /************************************************************************/
 
    void copyBrightnessData(unsigned char* &ptr);
    
    void copyDepthData(float* &ptr);

    void getCameraResolution(int &width, int &height);

private:
 
    Camera* camera_;
    LightCrafter3010 lc3010_;

    int camera_version_;
    

    struct CameraCalibParam calib_param_;
 
 
    int hdr_num_; 
    std::vector<int> led_current_list_; 
    std::vector<int> camera_exposure_list_; 

    int max_camera_exposure_;
    int min_camera_exposure_;

    int led_current_;
    int camera_exposure_;
    float camera_gain_;

    int generate_brightness_model_;
    int generate_brightness_exposure_;

    int image_width_;
    int image_height_;

    unsigned char* buff_brightness_;
    float* buff_depth_;
    float* buff_pointcloud_;

};
