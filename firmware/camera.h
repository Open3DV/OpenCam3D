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

	virtual bool getExposure(double &val){}; 
	virtual bool setExposure(double val){}; 
    
	virtual bool getGain(double &val){};
	virtual bool setGain(double val){};
	  
	virtual bool streamOn(){}; 
	virtual bool streamOff(){};

    virtual bool grap(unsigned char* buf){};

	bool getImageSize(int &width,int &height);

protected:
 
	bool camera_opened_state_; 
 

	long int image_width_;
	long int image_height_;
 
};

