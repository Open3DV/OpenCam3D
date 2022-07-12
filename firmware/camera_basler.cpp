#include "camera_basler.h"
#include "easylogging++.h"
 

CameraBasler::CameraBasler()
{

}

CameraBasler::~CameraBasler()
{

}


bool CameraBasler::grap(unsigned char* buf)
{

        PylonGrabResult_t           grabResult;               /* Stores the result of a grab operation. */ 
        _Bool                       isReady;                  /* Used as an output parameter. */
        GENAPIC_RESULT              res;   
        size_t bufferIndex;              /* Index of the buffer */
        unsigned char min, max;
        /* Wait for the next buffer to be filled. Wait up to 1000 ms. */
        res = PylonWaitObjectWait( hWait_, 10000, &isReady );
        // CHECK( res );
        if (!isReady)
        {
            /* Timeout occurred. */
            LOG(INFO)<<"Grab timeout occurred\n";
            // fprintf( stderr, "Grab timeout occurred\n" );
            return false; /* Stop grabbing. */
        }

        /* Since the wait operation was successful, the result of at least one grab
           operation is available. Retrieve it. */
        res = PylonStreamGrabberRetrieveResult( hGrabber_, &grabResult, &isReady );
        // CHECK( res );
        if (!isReady)
        {
            /* Oops. No grab result available? We should never have reached this point.
               Since the wait operation above returned without a timeout, a grab result
               should be available. */
               LOG(INFO)<<"Failed to retrieve a grab result\n" ;
            // fprintf( stderr, "Failed to retrieve a grab result\n" );
            return false;
        }
 

        /* Get the buffer index from the context information. */
        bufferIndex = (size_t) grabResult.Context;

        /* Check to see if the image was grabbed successfully. */
        if (grabResult.Status == Grabbed)
        {
            /*  Success. Perform image processing. Since we passed more than one buffer
            to the stream grabber, the remaining buffers are filled while
            we do the image processing. The processed buffer won't be touched by
            the stream grabber until we pass it back to the stream grabber. */

            unsigned char* buffer;        /* Pointer to the buffer attached to the grab result. */

            /* Get the buffer pointer from the result structure. Since we also got the buffer index,
               we could alternatively use buffers[bufferIndex]. */
            // buffer = (unsigned char*) grabResult.pBuffer;
 
 
	        memcpy(buf, grabResult.pBuffer, image_height_*image_width_);
            //cv::imshow("img", img);
	    //cv::waitKey(1);
   
        }
        else if (grabResult.Status == Failed)
        {
            // fprintf( stderr, "Frame %d wasn't grabbed successfully.  Error code = 0x%08X\n",
            //          nGrabs, grabResult.ErrorCode );
            LOG(INFO)<<"Failed to grabbe,Error code = "<< grabResult.ErrorCode;
        }

        /* Once finished with the processing, requeue the buffer to be filled again. */
        res = PylonStreamGrabberQueueBuffer( hGrabber_, grabResult.hBuffer, (void*) bufferIndex );
        // CHECK( res );
     

 
    return true;
}



bool CameraBasler::streamOn()
{

    while (!stream_mutex_.try_lock_for(std::chrono::milliseconds(1)))
    {
        LOG(INFO) << "GXStreamOn --";
    }

        GENAPIC_RESULT              res;    
        size_t                      nStreams;                 /* The number of streams the device provides. */
        /* Image grabbing is done using a stream grabber.
        A device may be able to provide different streams. A separate stream grabber must
        be used for each stream. In this sample, we create a stream grabber for the default
        stream, i.e., the first stream ( index == 0 ).
        */

        /* Get the number of streams supported by the device and the transport layer. */
        res = PylonDeviceGetNumStreamGrabberChannels( hDev_, &nStreams);
        // CHECK( res );
        if (nStreams < 1)
        {
            LOG(INFO)<<"The transport layer doesn't support image streams\n" ;
            // fprintf( stderr, "The transport layer doesn't support image streams\n" );
            PylonTerminate();

            return false;

        }

        /* Create and open a stream grabber for the first channel. */
        res = PylonDeviceGetStreamGrabber( hDev_, 0, &hGrabber_ );
        // CHECK( res );
        res = PylonStreamGrabberOpen( hGrabber_ );
        // CHECK( res );

        /* Get a handle for the stream grabber's wait object. The wait object
        allows waiting for buffers to be filled with grabbed data. */
        res = PylonStreamGrabberGetWaitObject( hGrabber_, &hWait_ );
        // CHECK( res );


        /* Determine the minimum size of the grab buffer.
        The size is determined by the configuration of the camera
        and the stream grabber. Be aware that this may change
        by changing critical parameters after this call.*/
        size_t  payloadSize;              /* Size of an image frame in bytes. */
        res = PylonStreamGrabberGetPayloadSize( hDev_, hGrabber_, &payloadSize );
        // CHECK( res );
        LOG(INFO)<<"payloadSize: "<<payloadSize;
        /* Allocate memory for grabbing.  */
        for (size_t i = 0; i < NUM_BUFFERS; ++i)
        {
            buffers_[i] = (unsigned char*) malloc( payloadSize );
            if (NULL == buffers_[i])
            {

                LOG(INFO) << "Out of memory!\n";
                // fprintf( stderr, "Out of memory!\n" );
                PylonTerminate();
 
            }
        }

        /* We must tell the stream grabber the number and size of the buffers
            we are using. */
        /* .. We will not use more than NUM_BUFFERS for grabbing. */
        res = PylonStreamGrabberSetMaxNumBuffer( hGrabber_, NUM_BUFFERS );
        // CHECK( res );
        /* .. We will not use buffers bigger than payloadSize bytes. */
        res = PylonStreamGrabberSetMaxBufferSize( hGrabber_, payloadSize );
        // CHECK( res );


        /*  Allocate the resources required for grabbing. After this, critical parameters
            that impact the payload size must not be changed until FinishGrab() is called. */
        res = PylonStreamGrabberPrepareGrab( hGrabber_ );
        // CHECK( res );


        /* Before using the buffers for grabbing, they must be registered at
        the stream grabber. For each registered buffer, a buffer handle
        is returned. After registering, these handles are used instead of the
        raw pointers. */
        for (size_t i = 0; i < NUM_BUFFERS; ++i)
        {
            res = PylonStreamGrabberRegisterBuffer( hGrabber_, buffers_[i], payloadSize, &bufHandles_[i] );
            // CHECK( res );
        }

        /* Feed the buffers into the stream grabber's input queue. For each buffer, the API
        allows passing in a pointer to additional context information. This pointer
        will be returned unchanged when the grab is finished. In our example, we use the index of the
        buffer as context information. */
        for (size_t i = 0; i < NUM_BUFFERS; ++i)
        {
            res = PylonStreamGrabberQueueBuffer( hGrabber_, bufHandles_[i], (void*) i );
            // CHECK( res );
        }

        /* Now the stream grabber is prepared. As soon as the camera starts to acquire images,
        the image data will be grabbed into the buffers provided.  */

        /* Start the image acquisition engine. */
        res = PylonStreamGrabberStartStreamingIfMandatory( hGrabber_ );
        // CHECK( res );

        /* Let the camera acquire images. */
        res = PylonDeviceExecuteCommandFeature( hDev_, "AcquisitionStart" );
        // CHECK( res );



        stream_mutex_.unlock();
        return true;
}

void CameraBasler::streamOffThread()
{
    while (!stream_mutex_.try_lock_for(std::chrono::milliseconds(1)))
    {
        LOG(INFO) << "GXStreamOff --";
    }

 GENAPIC_RESULT              res;                      /* Return value of pylon methods. */ 
    _Bool                       isReady;                  /* Used as an output parameter. */ 
    PylonGrabResult_t           grabResult;               /* Stores the result of a grab operation. */ 
    /* Clean up. */

    /*  ... Stop the camera. */
    res = PylonDeviceExecuteCommandFeature( hDev_, "AcquisitionStop" );
    // CHECK( res );
     

    /* ... Stop the image acquisition engine. */
    res = PylonStreamGrabberStopStreamingIfMandatory( hGrabber_ );
    // CHECK( res ); 

    /* ... We must issue a flush call to ensure that all pending buffers are put into the
       stream grabber's output queue. */
    res = PylonStreamGrabberFlushBuffersToOutput( hGrabber_ );
    // CHECK( res );

    LOG(INFO)<<"PylonStreamGrabberFlushBuffersToOutput";
    /* ... The buffers can now be retrieved from the stream grabber. */
    do
    {
        res = PylonStreamGrabberRetrieveResult( hGrabber_, &grabResult, &isReady );
        // CHECK( res );
    } while (isReady);
 
    /* ... When all buffers have been retrieved from the stream grabber, they can be deregistered.
           After that, it is safe to free the memory. */

    for (size_t i = 0; i < NUM_BUFFERS; ++i)
    {
        res = PylonStreamGrabberDeregisterBuffer( hGrabber_, bufHandles_[i] );
        // CHECK( res );
        free( buffers_[i] );
    } 

    /* ... Release grabbing related resources. */
    res = PylonStreamGrabberFinishGrab( hGrabber_ );
    // CHECK( res ); 

    /* After calling PylonStreamGrabberFinishGrab(), parameters that impact the payload size (e.g.,
    the AOI width and height parameters) are unlocked and can be modified again. */

    /* ... Close the stream grabber. */
    res = PylonStreamGrabberClose( hGrabber_ );
    // CHECK( res );
 
    LOG(INFO) << "Thread GXStreamOff";
    stream_mutex_.unlock();
}


bool CameraBasler::streamOff()
{
    std::thread stop_thread(&CameraBasler::streamOffThread,this);
    stop_thread.detach(); 

    return true;
}
 

bool CameraBasler::openCamera()
{
    GENAPIC_RESULT              res;                      /* Return value of pylon methods. */
    size_t                      numDevices;               /* Number of available devices. */

    size_t                      payloadSize;              /* Size of an image frame in bytes. */
    // unsigned char*              buffers[NUM_BUFFERS];     /* Buffers used for grabbing. */
    //PYLON_STREAMBUFFER_HANDLE   bufHandles[NUM_BUFFERS];  /* Handles for the buffers. */
    PylonGrabResult_t           grabResult;               /* Stores the result of a grab operation. */
    int                         nGrabs;                   /* Counts the number of buffers grabbed. */
    size_t                      nStreams;                 /* The number of streams the device provides. */
    _Bool                       isAvail;                  /* Used for checking feature availability. */
    _Bool                       isReady;                  /* Used as an output parameter. */
    size_t                      i;                        /* Counter. */

    /* Before using any pylon methods, the pylon runtime must be initialized. */
    PylonInitialize();

    /* Enumerate all camera devices. You must call
    PylonEnumerateDevices() before creating a device. */
    res = PylonEnumerateDevices( &numDevices );
    // CHECK( res );
    if (0 == numDevices)
    {
        LOG(INFO)<<"No devices found.\n";
        // fprintf( stderr, "No devices found.\n" );
        PylonTerminate(); 
        // exit( EXIT_FAILURE );   
    }

    /* Get a handle for the first device found.  */
    res = PylonCreateDeviceByIndex( 0, &hDev_ );
    // CHECK( res );

    /* Before using the device, it must be opened. Open it for configuring
    parameters and for grabbing images. */
    res = PylonDeviceOpen( hDev_, PYLONC_ACCESS_MODE_CONTROL | PYLONC_ACCESS_MODE_STREAM );
    // CHECK( res );

    /* Print out the name of the camera we are using. */
    {
        char buf[256];
        size_t siz = sizeof( buf );
        _Bool isReadable;

        isReadable = PylonDeviceFeatureIsReadable( hDev_, "DeviceModelName" );
        if (isReadable)
        {
            res = PylonDeviceFeatureToString( hDev_, "DeviceModelName", buf, &siz );
            // CHECK( res );
            
            LOG(INFO)<<buf;
            // printf( "Using camera %s\n", buf );
        }
    }

    /* Set the pixel format to Mono8 if available, where gray values will be output as 8 bit values for each pixel. */
    isAvail = PylonDeviceFeatureIsAvailable( hDev_, "EnumEntry_PixelFormat_Mono8" );
    if (isAvail)
    {
	std::cout<<"EnumEntry_PixelFormat_Mono8"<<std::endl;
        res = PylonDeviceFeatureFromString( hDev_, "PixelFormat", "Mono8" );
        // CHECK( res );
    }

    /* Disable acquisition start trigger if available. */
    isAvail = PylonDeviceFeatureIsAvailable( hDev_, "EnumEntry_TriggerSelector_AcquisitionStart" );
    if (isAvail)
    {
	    std::cout<<"EnumEntry_TriggerSelector_AcquisitionStart"<<std::endl;
        res = PylonDeviceFeatureFromString( hDev_, "TriggerSelector", "AcquisitionStart" );
        // CHECK( res );
        res = PylonDeviceFeatureFromString( hDev_, "TriggerMode", "Off" );
        // CHECK( res );
    }

    /* Disable frame burst start trigger if available. */
    isAvail = PylonDeviceFeatureIsAvailable( hDev_, "EnumEntry_TriggerSelector_FrameBurstStart" );
    if (isAvail)
    {
        std::cout<<"EnumEntry_TriggerSelector_FrameBurstStart"<<std::endl;
        res = PylonDeviceFeatureFromString( hDev_, "TriggerSelector", "FrameBurstStart" );
        // CHECK( res );
        res = PylonDeviceFeatureFromString( hDev_, "TriggerMode", "Off" );
        // CHECK( res );
    }

    /* Disable frame start trigger if available. */
    isAvail = PylonDeviceFeatureIsAvailable( hDev_, "EnumEntry_TriggerSelector_FrameStart" );
    if (isAvail)
    {
	std::cout<<"EnumEntry_TriggerSelector_FrameStart"<<std::endl;
        res = PylonDeviceFeatureFromString( hDev_, "TriggerSelector", "FrameStart" );
        // CHECK( res );
        res = PylonDeviceFeatureFromString( hDev_, "TriggerMode", "On" );
        // CHECK( res );
        res = PylonDeviceFeatureFromString( hDev_, "TriggerSource", "Line2" );
        // CHECK( res );
        res = PylonDeviceFeatureFromString( hDev_, "ExposureMode", "Timed" );
        // CHECK( res );
        res = PylonDeviceSetFloatFeature( hDev_, "ExposureTime", 12000.0 );
        // CHECK( res );
        res = PylonDeviceSetFloatFeature( hDev_, "Gain", 0.0 );
        // CHECK( res );
    }

        res = PylonDeviceGetIntegerFeature( hDev_, "Width", &image_width_ );
        // CHECK( res );
        res = PylonDeviceGetIntegerFeature( hDev_, "Height", &image_height_ );
        // CHECK( res );

        LOG(INFO)<<"image_width_: "<<image_width_;
        LOG(INFO)<<"image_height_: "<<image_height_;

    /* We will use the Continuous frame acquisition mode, i.e., the camera delivers
    images continuously. */
    res = PylonDeviceFeatureFromString( hDev_, "AcquisitionMode", "Continuous" );
    // CHECK( res );

 
 
 
    camera_opened_state_ = true;
 
    return true;
}
bool CameraBasler::closeCamera()
{
    GENAPIC_RESULT              res;                      /* Return value of pylon methods. */
    /* ... Close and release the pylon device. The stream grabber becomes invalid
       after closing the pylon device. Don't call stream grabber related methods after
       closing or releasing the device. */
    res = PylonDeviceClose( hDev_ );
    // CHECK( res );

    /* ...The device is no longer used, destroy it. */
    res = PylonDestroyDevice( hDev_ );
    // CHECK( res );

    camera_opened_state_ = false;

    return true;
}
bool CameraBasler::switchToInternalTriggerMode()
{
    GENAPIC_RESULT res;

    res = PylonDeviceFeatureFromString(hDev_, "TriggerSelector", "FrameStart");
    // CHECK( res );
    res = PylonDeviceFeatureFromString(hDev_, "TriggerMode", "Off"); 
    // CHECK( res );
    res = PylonDeviceFeatureFromString(hDev_, "ExposureMode", "Timed");
    // CHECK( res );

    if(GENAPI_E_OK != res)
    {
        LOG(INFO)<<"Set TriggerMode On Failed!";
        return false;
    } 
 
      
    return true;
}
bool CameraBasler::switchToExternalTriggerMode()
{
    GENAPIC_RESULT res;

    res = PylonDeviceFeatureFromString(hDev_, "TriggerSelector", "FrameStart");
    // CHECK( res );
    res = PylonDeviceFeatureFromString(hDev_, "TriggerMode", "On");
    // CHECK( res );
    res = PylonDeviceFeatureFromString(hDev_, "TriggerSource", "Line2");
    // CHECK( res );
    res = PylonDeviceFeatureFromString(hDev_, "ExposureMode", "Timed");
    // CHECK( res );
    if (GENAPI_E_OK != res)
    {
        LOG(INFO) << "Set switchToExternalTriggerMode Failed!";
        return false;
    }

    return true;
    
}
bool CameraBasler::getExposure(double &val)
{
    GENAPIC_RESULT              res;                      /* Return value of pylon methods. */
    res = PylonDeviceGetFloatFeature( hDev_, "ExposureTime", &val);
    if(GENAPI_E_OK != res)
    {
        LOG(INFO)<<"Set ExposureTime Failed!";
        return false;
    } 
 

	return true;
}
bool CameraBasler::setExposure(double val)
{
    GENAPIC_RESULT              res;                      /* Return value of pylon methods. */
    res = PylonDeviceSetFloatFeature( hDev_, "ExposureTime", val);
    if(GENAPI_E_OK != res)
    {
        LOG(INFO)<<"Set ExposureTime Failed!";
        return false;
    } 
    
    LOG(INFO)<<"Set Camera ExposureTime: "<<val;
  
	return true;
}

bool CameraBasler::getGain(double &val)
{
    PylonDeviceGetFloatFeature;

    GENAPIC_RESULT              res;                      /* Return value of pylon methods. */
    res = PylonDeviceGetFloatFeature( hDev_, "Gain", &val);
    if(GENAPI_E_OK != res)
    {
        LOG(INFO)<<"Get Gain Failed!";
        return false;
    }  

	return true; 
}
bool CameraBasler::setGain(double val)
{
    GENAPIC_RESULT              res;                      /* Return value of pylon methods. */
    res = PylonDeviceSetFloatFeature( hDev_, "Gain", val);
    if(GENAPI_E_OK != res)
    {
        LOG(INFO)<<"Set Gain Failed!";
        return false;
    } 
 
 
	return true;
} 
