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

	bool getExposure(float &val); 
	bool setExposure(float val); 

	bool getGain(float &value);
	bool setGain(float value);  

    bool setCameraStream(bool on);
    bool getCameraBuff(unsigned char* buf);

private:
   
 
    GX_DEV_HANDLE hDevice_;
    
    PGX_FRAME_BUFFER pFrameBuffer_; 
};