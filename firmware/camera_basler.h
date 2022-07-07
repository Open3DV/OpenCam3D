#pragma once
#include <stdlib.h>
#include <stdio.h>
#include <malloc.h>

#include <pylonc/PylonC.h>
#include "camera.h"


#define NUM_BUFFERS 5         /* Number of buffers used for grabbing. */

class CameraBasler: public Camera
{
public:
	CameraBasler();
	~CameraBasler();

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

    unsigned char*              buffers_[NUM_BUFFERS];     /* Buffers used for grabbing. */
    PYLON_STREAMBUFFER_HANDLE   bufHandles_[NUM_BUFFERS];  /* Handles for the buffers. */
   
    PYLON_DEVICE_HANDLE         hDev_;                     /* Handle for the pylon device. */
    PYLON_STREAMGRABBER_HANDLE  hGrabber_;                 /* Handle for the pylon stream grabber. */
    PYLON_WAITOBJECT_HANDLE     hWait_;                    /* Handle used for waiting for a grab to be finished. */
};