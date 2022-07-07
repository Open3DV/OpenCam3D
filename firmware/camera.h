#pragma once
#include<iostream> 

class Camera
{
public:
	Camera();
	~Camera();

	virtual bool openCamera();

	virtual bool closeCamera(); 
	
	virtual bool switchToInternalTriggerMode();

	virtual bool switchToExternalTriggerMode();

	virtual bool getExposure(float &val); 
	virtual bool setExposure(float val); 
    
	virtual bool getGain(float &val);
	virtual bool setGain(float val);
	
    virtual bool setCameraStream(bool on){};
    virtual bool getCameraBuff(unsigned char* buf){};

	bool getImageSize(int &width,int &height);

protected:
 
	bool camera_opened_state_; 

	//条纹扫描时，相机曝光值
	float scan_camera_exposure_;
	//条纹扫描时，相机增益值
	float scan_camera_gain_;

	int image_width_;
	int image_height_;
 
};

