#include "camera_dh.h"
#include "lightcrafter3010.h"
#include "easylogging++.h"
#include "encode_cuda.cuh"

extern LightCrafter3010 lc3010;

CameraDh::CameraDh()
{
    camera_opened_state_ = false;
    hDevice_ = NULL;
}


CameraDh::~CameraDh()
{
}

/******************************************************************************************************/


bool CameraDh::captureFrame03RepetitionToGpu(int repetition_count)
{
    switchToScanMode();
  
    PGX_FRAME_BUFFER pFrameBuffer; 
    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXSetEnum(hDevice_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
    status = GXStreamOn(hDevice_);

 

    int n = 0;
    int sync_serial_num = 0;
    if (status == GX_STATUS_SUCCESS)
    {
        lc3010.start_pattern_sequence();
        for (int i = 0; i < 31 + 6*(repetition_count-1); i++)
        {
            LOG(INFO) << "receiving " << i << "th image";
            status = GXDQBuf(hDevice_, &pFrameBuffer, 1000);
            LOG(INFO) << "status=" << status;
            if (status == GX_STATUS_SUCCESS)
            {
                if (pFrameBuffer->nStatus == GX_FRAME_STATUS_SUCCESS)
                {
                    int img_rows = pFrameBuffer->nHeight;
                    int img_cols = pFrameBuffer->nWidth;
                    int img_size = img_rows * img_cols;
                    // memcpy(buffer + img_size * i, pFrameBuffer->pImgBuf, img_size);
                    if(i< 12)
                    {
                        sync_serial_num = i;
                        parallel_cuda_copy_signal_patterns((unsigned char *)pFrameBuffer->pImgBuf,i);
                    }
                    else if(i> 11 && i< 12+ 6*repetition_count)
                    {

                        if(12 == i)
                        {
                            sync_serial_num = 12;
                        }
                        else
                        {
                            sync_serial_num = 13;
                        }

                        parallel_cuda_copy_repetition_signal_patterns((unsigned char *)pFrameBuffer->pImgBuf,i-12);
                        parallel_cuda_merge_repetition_patterns(i-12);
            
                        LOG(INFO) << "repetition " << i-12 << "th image";
                    }
                    else 
                    { 
                        // if(i == 12 +6*repetition_count)
                        // {
                        //     sync_serial_num = 18;
                        // }
                        // else
                        // {
                            sync_serial_num = i - 6*(repetition_count-1);

                        // }

                        parallel_cuda_copy_signal_patterns((unsigned char *)pFrameBuffer->pImgBuf,sync_serial_num);
                    }

                }

                
                status = GXQBuf(hDevice_, pFrameBuffer);

                LOG(INFO) << "sync_serial_num " << sync_serial_num;
                //copy to gpu
                switch (sync_serial_num)
                {
                case 4:
                {  
                    parallel_cuda_compute_phase(0);
                }
                break;
                case 8:
                {  
                    parallel_cuda_compute_phase(1);
                    parallel_cuda_unwrap_phase(1);
                }
                break;
                case 12:
                {  
                    parallel_cuda_compute_phase(2);
                    parallel_cuda_unwrap_phase(2);
                }
                break;
                case 18:
                {  
                    // parallel_cuda_compute_phase(3);
                    parallel_cuda_compute_merge_phase(repetition_count);
                    cudaDeviceSynchronize();
                    parallel_cuda_unwrap_phase(3);
                    
                }
                break;
                case 21:
                {  
                    parallel_cuda_compute_phase(4);
                }
                break;
                case 25:
                {  
                    parallel_cuda_compute_phase(5);
                    parallel_cuda_unwrap_phase(5);
                     
                }
                break;
                case 30:
                {  
                    parallel_cuda_compute_phase(6);
                    parallel_cuda_unwrap_phase(6);
                    
	                // cudaDeviceSynchronize();
                	parallel_cuda_reconstruct();
                }
                break;

                default:
                    break;
                }
 
            }
        }
    }
    else
    {
        false;
    }

 
    status = GXStreamOff(hDevice_); 
    return true;
}


//gpu parallel


bool CameraDh::captureFrame04ToGpu()
{
    switchToScanMode();
  
    PGX_FRAME_BUFFER pFrameBuffer; 
    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXSetEnum(hDevice_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
    status = GXStreamOn(hDevice_);

 

    int n = 0;
    if (status == GX_STATUS_SUCCESS)
    {
        lc3010.start_pattern_sequence();
        for (int i = 0; i < 19; i++)
        {
            LOG(INFO) << "receiving " << i << "th image";
            status = GXDQBuf(hDevice_, &pFrameBuffer, 1000);
            LOG(INFO) << "status=" << status;
            if (status == GX_STATUS_SUCCESS)
            {
                if (pFrameBuffer->nStatus == GX_FRAME_STATUS_SUCCESS)
                {
                    int img_rows = pFrameBuffer->nHeight;
                    int img_cols = pFrameBuffer->nWidth;
                    int img_size = img_rows * img_cols;
                    // memcpy(buffer + img_size * i, pFrameBuffer->pImgBuf, img_size);
                    parallel_cuda_copy_signal_patterns((unsigned char *)pFrameBuffer->pImgBuf,i);
                }

                
                status = GXQBuf(hDevice_, pFrameBuffer);

                //copy to gpu
                switch (i)
                {
                case 4:
                {  
                    parallel_cuda_compute_phase(0);
                }
                break;
                case 8:
                {  
                    parallel_cuda_compute_phase(1);
                    parallel_cuda_unwrap_phase(1);
                }
                break;
                case 12:
                {  
                    parallel_cuda_compute_phase(2);
                    parallel_cuda_unwrap_phase(2);
                }
                break;
                case 18:
                {  
                    parallel_cuda_compute_phase(3);
                    parallel_cuda_unwrap_phase(3);
                    
                     // cudaDeviceSynchronize();
                	generate_pointcloud_base_table();
                }
                break;
  
                break;

                default:
                    break;
                }
 
            }
        }
    }
    else
    {
        false;
    }

 
    status = GXStreamOff(hDevice_); 
    return true;
}


bool CameraDh::captureFrame03ToGpu()
{
    switchToScanMode();
  
    PGX_FRAME_BUFFER pFrameBuffer; 
    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXSetEnum(hDevice_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
    status = GXStreamOn(hDevice_);

 

    int n = 0;
    if (status == GX_STATUS_SUCCESS)
    {
        lc3010.start_pattern_sequence();
        for (int i = 0; i < 31; i++)
        {
            LOG(INFO) << "receiving " << i << "th image";
            status = GXDQBuf(hDevice_, &pFrameBuffer, 1000);
            LOG(INFO) << "status=" << status;
            if (status == GX_STATUS_SUCCESS)
            {
                if (pFrameBuffer->nStatus == GX_FRAME_STATUS_SUCCESS)
                {
                    int img_rows = pFrameBuffer->nHeight;
                    int img_cols = pFrameBuffer->nWidth;
                    int img_size = img_rows * img_cols;
                    // memcpy(buffer + img_size * i, pFrameBuffer->pImgBuf, img_size);
                    parallel_cuda_copy_signal_patterns((unsigned char *)pFrameBuffer->pImgBuf,i);
                }

                
                status = GXQBuf(hDevice_, pFrameBuffer);

                //copy to gpu
                switch (i)
                {
                case 4:
                {  
                    parallel_cuda_compute_phase(0);
                }
                break;
                case 8:
                {  
                    parallel_cuda_compute_phase(1);
                    parallel_cuda_unwrap_phase(1);
                }
                break;
                case 12:
                {  
                    parallel_cuda_compute_phase(2);
                    parallel_cuda_unwrap_phase(2);
                }
                break;
                case 18:
                {  
                    parallel_cuda_compute_phase(3);
                    parallel_cuda_unwrap_phase(3);
                    
                }
                break;
                case 21:
                {  
                    parallel_cuda_compute_phase(4);
                }
                break;
                case 25:
                {  
                    parallel_cuda_compute_phase(5);
                    parallel_cuda_unwrap_phase(5);
                     
                }
                break;
                case 30:
                {  
                    parallel_cuda_compute_phase(6);
                    parallel_cuda_unwrap_phase(6);
                    
	                // cudaDeviceSynchronize();
                	parallel_cuda_reconstruct();
                }
                break;

                default:
                    break;
                }
 
            }
        }
    }
    else
    {
        false;
    }

 
    status = GXStreamOff(hDevice_); 
    return true;
}

/******************************************************************************************************/

bool CameraDh::closeCamera()
{
	if(!camera_opened_state_)
	{
		return false;
	}

    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXCloseDevice(hDevice_);
    status = GXCloseLib();


    camera_opened_state_ = false;

	return true;
}


bool CameraDh::setExpose(double value)
{

    GX_STATUS status = GX_STATUS_SUCCESS;
    //�� �� �� �� ֵ
    status = GXSetFloat(hDevice_, GX_FLOAT_EXPOSURE_TIME, 25000);
	return true;
}

bool CameraDh::getExpose(double &value)
{

    return true;
}

bool CameraDh::switchToSingleShotMode()
{
    GX_STATUS status;
    status = GXSetEnum(hDevice_, GX_ENUM_LINE_SOURCE, GX_ENUM_LINE_SOURCE_STROBE);
    status = GXSetEnum(hDevice_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
}

bool CameraDh::switchToScanMode()
{
    GX_STATUS status;
    status = GXSetEnum(hDevice_ ,GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_LINE2);
}

bool CameraDh::openCamera()
{

    GX_STATUS status = GX_STATUS_SUCCESS;
    uint32_t nDeviceNum = 0;

        status = GXInitLib();
    if (status != GX_STATUS_SUCCESS)
    {
        return 0;
    }

    status = GXUpdateDeviceList(&nDeviceNum, 1000);
    if ((status != GX_STATUS_SUCCESS) || (nDeviceNum <= 0))
    {
        return 0;
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
   
	LOG(FATAL)<<"Open Camera Error!";
	return 0;
    }



    if (status == GX_STATUS_SUCCESS)
    {

        /***********************************************************************************************/
        //�� �� �� �� ֵ
        status = GXSetFloat(hDevice_, GX_FLOAT_EXPOSURE_TIME, 12000);
        //�� �� �� �� �� �� �� ��
        status = GXSetEnum(hDevice_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);

        //ѡ �� �� �� ͨ �� �� ��
        status = GXSetEnum(hDevice_, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);

        //�� �� �� �� ֵ
        status = GXSetFloat(hDevice_, GX_FLOAT_GAIN, 0);


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

    }


    lc3010.init();
    // float temperature = lc3010.get_temperature();
    // printf("temperature=%f deg\n", temperature);
    // LOG(INFO)<<"temperature="<<temperature;
    return true;
}


bool CameraDh::captureSingleImage(char* buffer)
{
    switchToSingleShotMode();

    //�� �� GXDQBuf �� �� �� �� ��
    PGX_FRAME_BUFFER pFrameBuffer;
    //�� ��
    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXStreamOn(hDevice_);
    if (status == GX_STATUS_SUCCESS)
    {
	status = GXSendCommand(hDevice_, GX_COMMAND_TRIGGER_SOFTWARE);
        status = GXDQBuf(hDevice_, &pFrameBuffer, 1000);
        if (status == GX_STATUS_SUCCESS)
        {
            if (pFrameBuffer->nStatus == GX_FRAME_STATUS_SUCCESS)
            {
                int img_rows = pFrameBuffer->nHeight;
                int img_cols = pFrameBuffer->nWidth;
		int img_size = img_rows*img_cols;
                LOG(TRACE)<<"H:" <<img_rows<<" W: "<<img_cols<<std::endl;
		memcpy(buffer, pFrameBuffer->pImgBuf, img_size);
	    }
            status = GXQBuf(hDevice_, pFrameBuffer);
        }
	else
	{
	    LOG(ERROR)<<"capture single image failed";
	}
        status = GXStreamOff(hDevice_);
    }
    else
    {
        false;
    }

    return true;
}


bool CameraDh::captureRawTest(int num,char* buffer)
{
    switchToScanMode();

    //�� �� GXDQBuf �� �� �� �� ��
    PGX_FRAME_BUFFER pFrameBuffer;
    //�� ��
    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXSetEnum(hDevice_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
    status = GXStreamOn(hDevice_);
    int n=0;
    if (status == GX_STATUS_SUCCESS)
    {
	lc3010.start_pattern_sequence();
	for (int i=0; i<num; i++)
	{
	    LOG(INFO)<<"receiving "<<i<<"th image";
            status = GXDQBuf(hDevice_, &pFrameBuffer, 1000);
	    LOG(INFO)<<"status="<<status;
            if (status == GX_STATUS_SUCCESS)
            {
                if (pFrameBuffer->nStatus == GX_FRAME_STATUS_SUCCESS)
                {
                    int img_rows = pFrameBuffer->nHeight;
                    int img_cols = pFrameBuffer->nWidth;
		    int img_size = img_rows*img_cols;
		    memcpy(buffer+img_size*i, pFrameBuffer->pImgBuf, img_size);
	        }
                status = GXQBuf(hDevice_, pFrameBuffer);
            }
	}
    }
    else
    {
        false;
    }

    status = GXStreamOff(hDevice_);
    return true;
}

bool CameraDh::warmupCamera()
{
    
    switchToSingleShotMode();

    
	
    PGX_FRAME_BUFFER pFrameBuffer;
    //�� ��
    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXStreamOn(hDevice_);
    if (status == GX_STATUS_SUCCESS)
    {


            status = GXDQBuf(hDevice_, &pFrameBuffer, 100);
            if (status == GX_STATUS_SUCCESS)
            {
                if (pFrameBuffer->nStatus == GX_FRAME_STATUS_SUCCESS)
                {

                    int img_rows = pFrameBuffer->nHeight;
                    int img_cols = pFrameBuffer->nWidth;
//                    std::cout<<"H:" <<img_rows<<" W: "<<img_cols<<" "<<num<<std::endl;
 //                   cv::Mat img(img_rows,img_cols,CV_8UC1,pFrameBuffer->pImgBuf);


//                    std::string save_path = "../TestData/";
//                    int i= 12-num;
//                    if (i < 10)
//                    {
//                        save_path += "0" + std::to_string(i) + ".bmp";
//                    }
//                    else
//                    {
//                        save_path += std::to_string(i) + ".bmp";
//                    }
//                    cv::imwrite(save_path,img);


                    status = GXQBuf(hDevice_, pFrameBuffer);
               }

	    }

    }
        status = GXStreamOff(hDevice_);

	return true;

}

bool CameraDh::captureRawPhaseImages(char* buffer)
{
    switchToScanMode();

    //�� �� GXDQBuf �� �� �� �� ��
    PGX_FRAME_BUFFER pFrameBuffer;
    //�� ��
    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXSetEnum(hDevice_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
    status = GXStreamOn(hDevice_);
    int n=0;
    if (status == GX_STATUS_SUCCESS)
    {

	LOG(INFO)<<"iic trigger start:";    
	lc3010.start_pattern_sequence();
	LOG(INFO)<<"iic trigger finished!";    
	
	for (int i=0; i<24; i++)
	{
	    LOG(INFO)<<"receiving "<<i<<"th image";
            status = GXDQBuf(hDevice_, &pFrameBuffer, 1000);
	    LOG(INFO)<<"status="<<status;
            if (status == GX_STATUS_SUCCESS)
            {
                if (pFrameBuffer->nStatus == GX_FRAME_STATUS_SUCCESS)
                {
                    int img_rows = pFrameBuffer->nHeight;
                    int img_cols = pFrameBuffer->nWidth;
		    int img_size = img_rows*img_cols;
		    memcpy(buffer+img_size*i, pFrameBuffer->pImgBuf, img_size);
	        }
                status = GXQBuf(hDevice_, pFrameBuffer);
            }
	}
    }
    else
    {
        false;
    }

    status = GXStreamOff(hDevice_);
    return true;
}


