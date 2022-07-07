#pragma once
#include "GxIAPI.h" 
#include "camera.h"

class CameraGalaxy: public Camera
{
public:
	CameraGalaxy();
	~CameraGalaxy();

	bool openCamera(); 
	bool closeCamera(); 

	bool switchToInternalTriggerMode(); 
	bool switchToExternalTriggerMode();

	bool getExposure(double &val); 
	bool setExposure(double val); 

	bool getGain(double &value);
	bool setGain(double value);  

	bool streamOn(); 
	bool streamOff();
 
    bool grap(unsigned char* buf);

private:
   
 
    GX_DEV_HANDLE hDevice_;
    
    PGX_FRAME_BUFFER pFrameBuffer_; 
};