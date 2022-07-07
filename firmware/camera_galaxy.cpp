#include "camera_galaxy.h"
#include "easylogging++.h"

CameraGalaxy::CameraGalaxy()
{

}
CameraGalaxy::~CameraGalaxy()
{

}


bool CameraGalaxy::getCameraBuff(unsigned char* buf)
{

    LOG(INFO) << "capture:";
    GX_STATUS status = GXDQBuf(hDevice_, &pFrameBuffer_, 1000);
    LOG(INFO) << "status=" << status;
    if (status != GX_STATUS_SUCCESS)
    {
        status = GXQBuf(hDevice_, pFrameBuffer_);
        return false;
    }

    if (pFrameBuffer_->nStatus == GX_FRAME_STATUS_SUCCESS)
    {
        int img_rows = pFrameBuffer_->nHeight;
        int img_cols = pFrameBuffer_->nWidth;
        int img_size = img_rows * img_cols;

        memcpy(buf, pFrameBuffer_->pImgBuf, img_size);
    }

    status = GXQBuf(hDevice_, pFrameBuffer_);
    if (status != GX_STATUS_SUCCESS)
    { 
        return false;
    }

    return true;
}

bool CameraGalaxy::setCameraStream(bool on)
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    
    if(on)
    {
        status = GXSetEnum(hDevice_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON); 
        if(status != GX_STATUS_SUCCESS)
        {
            
            LOG(INFO) << "GXSetEnum Error: "<<status;
            return false;
        }

        status = GXStreamOn(hDevice_); 
        if(status != GX_STATUS_SUCCESS)
        {
            
            LOG(INFO) << "Stream On Error: "<<status;
            return false;
        }
    }
    else
    {
        // off
        std::thread stop_thread(GXStreamOff, hDevice_);
        stop_thread.detach();
    }
    

    return true;
}

bool CameraGalaxy::openCamera()
{

    GX_STATUS status = GX_STATUS_SUCCESS;
    uint32_t nDeviceNum = 0;

    status = GXInitLib();
    if (status != GX_STATUS_SUCCESS)
    {
        return false;
    }

    status = GXUpdateDeviceList(&nDeviceNum, 1000);
    if ((status != GX_STATUS_SUCCESS) || (nDeviceNum <= 0))
    {
        return false;
    }

    char cam_idx[8] = "0";
    if (status == GX_STATUS_SUCCESS && nDeviceNum > 0)
    {
        GX_DEVICE_BASE_INFO *pBaseinfo = new GX_DEVICE_BASE_INFO[nDeviceNum];
        size_t nSize = nDeviceNum * sizeof(GX_DEVICE_BASE_INFO);
        // Gets the basic information of all devices.
        status = GXGetAllDeviceBaseInfo(pBaseinfo, &nSize);
	for(int i=0; i<nDeviceNum; i++)
	{
	    if(GX_DEVICE_CLASS_U3V == pBaseinfo[i].deviceClass)
	    {
		//camera index starts from 1
		snprintf(cam_idx, 8, "%d", i+1);
	    }
	}

        delete []pBaseinfo;
    }

    
    GX_OPEN_PARAM stOpenParam;
    stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
    stOpenParam.openMode = GX_OPEN_INDEX;
    stOpenParam.pszContent = cam_idx;
    status = GXOpenDevice(&stOpenParam, &hDevice_);
   
    if (status != GX_STATUS_SUCCESS)
    {
   
	    LOG(INFO)<<"Open Camera Error!";
	    return false;
    }

 
    if (status == GX_STATUS_SUCCESS)
    {

        /***********************************************************************************************/
        //�� �� �� �� ֵ
        status = GXSetFloat(hDevice_, GX_FLOAT_EXPOSURE_TIME, scan_camera_exposure_);
        //�� �� �� �� �� �� �� ��
        status = GXSetEnum(hDevice_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);

        //ѡ �� �� �� ͨ �� �� ��
        status = GXSetEnum(hDevice_, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL); 
        //�� �� �� �� ֵ
        status = GXSetFloat(hDevice_, GX_FLOAT_GAIN, scan_camera_gain_); 
        //�� �� �� �� ѡ �� Ϊ Line2
        status = GXSetEnum(hDevice_, GX_ENUM_LINE_SELECTOR, GX_ENUM_LINE_SELECTOR_LINE2);
        //�� �� �� �� �� �� Ϊ �� ��
        status = GXSetEnum(hDevice_, GX_ENUM_LINE_MODE, GX_ENUM_LINE_MODE_INPUT);

        //�� �� �� �� �� �� Դ Ϊ �� �� ��,�� �� �� �� ��
        //emStatus = GXSetEnum(hDevice_, GX_ENUM_LINE_SOURCE, GX_ENUM_LINE_SOURCE_STROBE);
        status = GXSetEnum(hDevice_ ,GX_ENUM_TRIGGER_ACTIVATION, GX_TRIGGER_ACTIVATION_RISINGEDGE);
        status = GXSetEnum(hDevice_ ,GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_LINE2);

        status = GXSetAcqusitionBufferNumber(hDevice_, 72);
        camera_opened_state_ = true;

        long int width = 0,height = 0;
        status = GXGetInt(hDevice_, GX_INT_WIDTH, &width); 
        status = GXGetInt(hDevice_, GX_INT_HEIGHT, &height); 

        image_width_ = width;
        image_height_ = height;
    }
 
 
    return true;
}
bool CameraGalaxy::closeCamera()
{
    if (!camera_opened_state_)
    {
        return false;
    }

    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXCloseDevice(hDevice_);
    status = GXCloseLib();

    camera_opened_state_ = false;

    return true;
}
bool CameraGalaxy::switchToInternalTriggerMode()
{
    GX_STATUS status;
    status = GXSetEnum(hDevice_, GX_ENUM_LINE_SOURCE, GX_ENUM_LINE_SOURCE_STROBE);
    

    if(GX_STATUS_SUCCESS != status)
    {
        return false;
    }

    status = GXSetEnum(hDevice_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
    
    if(GX_STATUS_SUCCESS != status)
    {
        return false;
    }
      
    return true;
}
bool CameraGalaxy::switchToExternalTriggerMode()
{
    GX_STATUS status;
    status = GXSetEnum(hDevice_ ,GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_LINE2);

    if(GX_STATUS_SUCCESS != status)
    {
        return false;
    }
      
    return true;
    
}
bool CameraGalaxy::getExposure(float &val)
{
    GX_STATUS status = GX_STATUS_SUCCESS; 
    double exposure = 0;
    status = GXGetFloat(hDevice_, GX_FLOAT_EXPOSURE_TIME, &exposure);

    if (status != GX_STATUS_SUCCESS)
    {

        LOG(INFO) << "Error Status: " << status;
        return false;
    }

    val = exposure;

    return true;
}
bool CameraGalaxy::setExposure(float val)
{
    GX_STATUS status = GX_STATUS_SUCCESS; 
    status = GXSetFloat(hDevice_, GX_FLOAT_EXPOSURE_TIME, val);


    if(status == GX_STATUS_SUCCESS)
    {
        LOG(INFO)<<"Error Status: "<<status;
        return false;
    }
 
	return true;
}

bool CameraGalaxy::getGain(float &val)
{
    GX_STATUS status = GX_STATUS_SUCCESS; 
    double gain = 0;
    status = GXGetFloat(hDevice_, GX_FLOAT_GAIN, &gain); 

    if(status != GX_STATUS_SUCCESS)
    { 
        
        LOG(INFO)<<"Error Status: "<<status;
        return false;
    } 

    val = gain;
	return true; 
}
bool CameraGalaxy::setGain(float val)
{
    GX_STATUS status = GX_STATUS_SUCCESS; 
    status = GXSetFloat(hDevice_, GX_FLOAT_GAIN, val); 

    if(status != GX_STATUS_SUCCESS)
    {
        
        LOG(INFO)<<"Error Status: "<<status;
        return false;
    }
 
	return true;
} 
