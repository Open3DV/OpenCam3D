#include "camera_dh.h"
#include "lightcrafter3010.h"
#include "easylogging++.h"
#include "encode_cuda.cuh" 
#include <thread>

extern LightCrafter3010 lc3010;

CameraDh::CameraDh()
{
    camera_opened_state_ = false;
    hDevice_ = NULL;

    scan_camera_exposure_ = 12000;
    generate_brigntness_model_ = 1;
    generate_brightness_exposure_time_ = 12000;

    phase_compensate_value = 0;
    
	scan_camera_gain_ = 0;
    
    buffer_size_ = 1920*1200;
    brightness_buff_ = new char[buffer_size_];
    memset(brightness_buff_,0,buffer_size_);
}


CameraDh::~CameraDh()
{
    delete brightness_buff_;
}

/******************************************************************************************************/

void CameraDh::setGenerateBrightnessParam(int model,float exposure)
{
    generate_brigntness_model_ = model;
    generate_brightness_exposure_time_ = exposure;
}

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


bool CameraDh::compensatePhaseBaseScharr(cv::Mat& normal_phase, cv::Mat brightness, float offset_value)
{
if (normal_phase.empty() || brightness.empty())
		return false;
	 
	int nr = brightness.rows;
	int nc = brightness.cols;

	cv::Mat sobel_brightness = brightness.clone();
	cv::Mat sobel_grad_x, scharr_x;
 

	Scharr(sobel_brightness, scharr_x, CV_32F, 1, 0, 1, 0, cv::BORDER_DEFAULT);
	cv::GaussianBlur(scharr_x, scharr_x, cv::Size(5, 5), 3, 3);
  

	for (int r = 0; r < nr; r++)
	{
		float* ptr_sobel = scharr_x.ptr<float>(r); 
		float* ptr_phase_map = normal_phase.ptr<float>(r);

		for (int c = 0; c < nc; c++)
		{
			if (std::abs(ptr_sobel[c]) < 300)
			{
				ptr_sobel[c] = 0;
			}
			else
			{
				if (ptr_phase_map[c] > 0)
				{
					ptr_phase_map[c] -= ptr_sobel[c] * 0.0000001 * offset_value;
				} 
			}
		}

	}


	return true;
}


bool CameraDh::capturePhase02Repetition02ToGpu(int repetition_count)
{

    PGX_FRAME_BUFFER pFrameBuffer;
    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXSetEnum(hDevice_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
    // status = GXStreamOn(hDevice_);
    parallel_cuda_clear_repetition_02_patterns();

    for (int r = 0; r < repetition_count; r++)
    {
        int n = 0;
        if (status == GX_STATUS_SUCCESS)
        {

            LOG(INFO) << "pattern_mode04";
            lc3010.pattern_mode02();
            status = GXStreamOn(hDevice_);
            lc3010.start_pattern_sequence();
            LOG(INFO) << "start_pattern_sequence";

            for (int i = 0; i < 37; i++)
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

                        parallel_cuda_copy_signal_patterns((unsigned char *)pFrameBuffer->pImgBuf, i);
                        parallel_cuda_merge_repetition_02_patterns(i);
                    }

                    status = GXQBuf(hDevice_, pFrameBuffer);
                }
                else
                {
                    status = GXStreamOff(hDevice_);
                    return false;
                }
            }

            /*********************************************************************************************/
            std::thread stop_thread(GXStreamOff, hDevice_);
            stop_thread.detach();
            lc3010.stop_pattern_sequence();
            // status = GXStreamOff(hDevice_);
            LOG(INFO) << "GXStreamOff";
            /***********************************************************************************************/
        }
        else
        {
            return false;
        }
    }

    //关闭相机流
    // std::thread stop_thread(GXStreamOff, hDevice_);
    // stop_thread.detach();
    // LOG(INFO) << "GXStreamOff";

    parallel_cuda_compute_model_02_merge_repetition_02_phase(repetition_count);

    LOG(INFO) << "parallel_cuda_compute_mergerepetition_02_phase";
    parallel_cuda_unwrap_phase(1);
    parallel_cuda_unwrap_phase(2);
    parallel_cuda_unwrap_phase(3);
    parallel_cuda_unwrap_phase(5);
    parallel_cuda_unwrap_phase(6);
    parallel_cuda_unwrap_phase(7);
    LOG(INFO) << "parallel_cuda_unwrap_phase";

    return true;
}

bool CameraDh::captureFrame04Repetition02ToGpu(int repetition_count)
{

    PGX_FRAME_BUFFER pFrameBuffer; 
    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXSetEnum(hDevice_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
    // status = GXStreamOn(hDevice_);  
    parallel_cuda_clear_repetition_02_patterns();

    for(int r= 0;r< repetition_count;r++)
    {
    int n = 0;
    if (status == GX_STATUS_SUCCESS)
    {
 
        LOG(INFO) << "pattern_mode04";
        lc3010.pattern_mode04();
        status = GXStreamOn(hDevice_); 
        lc3010.start_pattern_sequence();  
        LOG(INFO) << "start_pattern_sequence";

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
         
                    if(i< 19)
                    { 
                        parallel_cuda_copy_signal_patterns((unsigned char *)pFrameBuffer->pImgBuf,i); 
                        parallel_cuda_merge_repetition_02_patterns(i);
                    }
 
                } 
                
                status = GXQBuf(hDevice_, pFrameBuffer);

 
            }
            else
            {
                status = GXStreamOff(hDevice_); 
                return false;
            }
        }

        /*********************************************************************************************/
        std::thread stop_thread(GXStreamOff, hDevice_);
        stop_thread.detach();
        lc3010.stop_pattern_sequence();
        // status = GXStreamOff(hDevice_); 
        LOG(INFO) << "GXStreamOff";
        /***********************************************************************************************/
   
   
    }
    else
    {
        return false;
    }
    }

        //关闭相机流
        // std::thread stop_thread(GXStreamOff, hDevice_);
        // stop_thread.detach();
        // LOG(INFO) << "GXStreamOff";
        parallel_cuda_compute_merge_repetition_02_phase(repetition_count);
        LOG(INFO) << "parallel_cuda_compute_mergerepetition_02_phase";
        parallel_cuda_unwrap_phase(1);
        parallel_cuda_unwrap_phase(2);
        parallel_cuda_unwrap_phase(3);
        LOG(INFO) << "parallel_cuda_unwrap_phase";
        generate_pointcloud_base_table();
        LOG(INFO) << "generate_pointcloud_base_table";

        return true;
}

bool CameraDh::captureFrame04RepetitionToGpu(int repetition_count)
{
// switchToScanMode();
    // LOG(INFO) << "switchToScanMode";

    PGX_FRAME_BUFFER pFrameBuffer; 
    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXSetEnum(hDevice_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON); 
    status = GXStreamOn(hDevice_); 
 
    int n = 0;
    int capture_count = 19 +6*(repetition_count-1);
    if (status == GX_STATUS_SUCCESS)
    {
        lc3010.start_pattern_sequence(); 
        for (int i = 0; i < capture_count; i++)
        {
            LOG(INFO) << "receiving " << i << "th image";
            status = GXDQBuf(hDevice_, &pFrameBuffer, 1000);
            LOG(INFO) << "status=" << status;

            int sync_serial_num = i;

            if (status == GX_STATUS_SUCCESS)
            {
                if (pFrameBuffer->nStatus == GX_FRAME_STATUS_SUCCESS)
                {
                    int img_rows = pFrameBuffer->nHeight;
                    int img_cols = pFrameBuffer->nWidth;
                    int img_size = img_rows * img_cols; 
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
                        
                        memcpy(brightness_buff_, pFrameBuffer->pImgBuf, img_size); 

                        std::thread stop_thread(GXStreamOff,hDevice_);
                        stop_thread.detach();  
                    }
 
                }

                
                status = GXQBuf(hDevice_, pFrameBuffer);

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
                }
                break;
                case 10:
                {     
                    parallel_cuda_unwrap_phase(1);  
                }
                break;
                case 12:
                {    
                    parallel_cuda_compute_phase(2); 
                    parallel_cuda_unwrap_phase(2);    
                }
                break;
                // case 15:
                // {                   
                // }
                // break;
                case 18:
                {    

                    parallel_cuda_compute_merge_phase(repetition_count);
                    cudaDeviceSynchronize();
                    // parallel_cuda_compute_phase(3);  
                    parallel_cuda_unwrap_phase(3);   

  
                    
                    generate_pointcloud_base_table();
                    //  cudaDeviceSynchronize();
                     LOG(INFO) << "generate_pointcloud_base_table";



                    switch (generate_brigntness_model_)
                    { 
                        case 2:
                        {
                            
                            status = GXStreamOff(hDevice_); 
                            lc3010.stop_pattern_sequence();
                            lc3010.init();
                            switchToSingleShotMode();
                            //发光，自定义曝光时间 
                            lc3010.enable_solid_field();
                            bool capture_one_ret = captureSingleExposureImage(generate_brightness_exposure_time_,brightness_buff_);
                            lc3010.disable_solid_field();   
                            switchToScanMode(); 
                             
                        }
                        break;
                        case 3:
                        {
                            
                            status = GXStreamOff(hDevice_); 
                            lc3010.stop_pattern_sequence();
                            lc3010.init();
                            switchToSingleShotMode();
                            //不发光，自定义曝光时间  
                            bool capture_one_ret = captureSingleExposureImage(generate_brightness_exposure_time_,brightness_buff_); 
                            switchToScanMode(); 
                        }
                        break;
                    
                    default:
                        break;
                    }
                }
                break;
  
                break;

                default:
                    break;
                }
 
            }
            else
            {
                status = GXStreamOff(hDevice_); 
                return false;
            }
        }
    }
    else
    {
        return false;
    }

 
    return true;
}

bool CameraDh::captureFrame04ToGpu()
{
    // switchToScanMode();
    // LOG(INFO) << "switchToScanMode";

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
         
                    parallel_cuda_copy_signal_patterns((unsigned char *)pFrameBuffer->pImgBuf,i); 

                    if(18 == i && 1 == generate_brigntness_model_)
                    {
                        memcpy(brightness_buff_, pFrameBuffer->pImgBuf, img_size); 

                        std::thread stop_thread(GXStreamOff,hDevice_);
                        stop_thread.detach();  
                    }
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
                }
                break;
                case 10:
                {     
                    parallel_cuda_unwrap_phase(1);  
                }
                break;
                case 12:
                {    
                    parallel_cuda_compute_phase(2); 
                }
                break;
                case 15:
                {     
                    parallel_cuda_unwrap_phase(2);                  
                }
                break;
                case 18:
                {    
                    parallel_cuda_compute_phase(3);  
                    parallel_cuda_unwrap_phase(3);   

 

                    // if (phase_compensate_value != 0)
                    // {
                    //     LOG(INFO) << "start offset";
                    //     cv::Mat unwrap_map_x(1200, 1920, CV_32FC1, cv::Scalar(0));
                    //     cv::Mat brightness_map(1200, 1920, CV_8UC1, brightness_buff_);
                    //     parallel_cuda_copy_unwrap_phase_from_gpu(0, (float *)unwrap_map_x.data);
                    //     compensatePhaseBaseScharr(unwrap_map_x, brightness_map, phase_compensate_value * 128);
                    //     parallel_cuda_copy_unwrap_phase_to_gpu(0, (float *)unwrap_map_x.data); 
                    //     LOG(INFO) << "finished offset";
                    // }
                    
                    generate_pointcloud_base_table();
                    //  cudaDeviceSynchronize();
                     LOG(INFO) << "generate_pointcloud_base_table";



                    switch (generate_brigntness_model_)
                    { 
                        case 2:
                        {
                            
                            status = GXStreamOff(hDevice_); 
                            lc3010.stop_pattern_sequence();
                            lc3010.init();
                            switchToSingleShotMode();
                            //发光，自定义曝光时间 
                            lc3010.enable_solid_field();
                            bool capture_one_ret = captureSingleExposureImage(generate_brightness_exposure_time_,brightness_buff_);
                            lc3010.disable_solid_field();   
                            switchToScanMode(); 
                             
                        }
                        break;
                        case 3:
                        {
                            
                            status = GXStreamOff(hDevice_); 
                            lc3010.stop_pattern_sequence();
                            lc3010.init();
                            switchToSingleShotMode();
                            //不发光，自定义曝光时间  
                            bool capture_one_ret = captureSingleExposureImage(generate_brightness_exposure_time_,brightness_buff_); 
                            switchToScanMode(); 
                        }
                        break;
                    
                    default:
                        break;
                    }
                }
                break;
  
                break;

                default:
                    break;
                }
 
            }
            else
            {
                status = GXStreamOff(hDevice_); 
                return false;
            }
        }
    }
    else
    {
        return false;
    }

 
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

bool CameraDh::setScanGain(float value)
{
    bool ret = setGain(value);

    if(ret)
    { 
      scan_camera_gain_ = value;
    }

    return ret;
}

bool CameraDh::setScanExposure(float value)
{

    bool ret = setExposure(value);

    if(ret)
    { 
      scan_camera_exposure_ = value;
    }

    return ret;
}

bool CameraDh::setGain(float value)
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    //�� �� �� �� ֵ
    status = GXSetFloat(hDevice_, GX_FLOAT_GAIN, value);


    if(status == GX_STATUS_SUCCESS)
    {
        return true;
    }


    LOG(INFO)<<"Error Status: "<<status;
	return false;
}

bool CameraDh::getGain(float &value)
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    //�� �� �� �� ֵ
    double gain = 0;
    status = GXGetFloat(hDevice_, GX_FLOAT_GAIN, &gain);


    if(status == GX_STATUS_SUCCESS)
    {

        value = gain;
        
        return true;
    }


    LOG(INFO)<<"Error Status: "<<status;
	return false; 
}

bool CameraDh::setExposure(float value)
{

    GX_STATUS status = GX_STATUS_SUCCESS;
    //�� �� �� �� ֵ
    status = GXSetFloat(hDevice_, GX_FLOAT_EXPOSURE_TIME, value);


    if(status == GX_STATUS_SUCCESS)
    {
        return true;
    }


    LOG(INFO)<<"Error Status: "<<status;
	return false;
}

bool CameraDh::getExposure(float &value)
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    //�� �� �� �� ֵ
    double exposure = 0;
    status = GXGetFloat(hDevice_, GX_FLOAT_EXPOSURE_TIME, &exposure);


    if(status == GX_STATUS_SUCCESS)
    {

        value = exposure;
        
        return true;
    }


    LOG(INFO)<<"Error Status: "<<status;
	return false; 
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

    }


    lc3010.init();
    // float temperature = lc3010.get_temperature();
    // printf("temperature=%f deg\n", temperature);
    // LOG(INFO)<<"temperature="<<temperature;
    return true;
}


bool CameraDh::captureSingleExposureImage(float exposure,char* buffer)
{

    bool ret = setExposure(exposure);

    if(!ret)
    {
        LOG(INFO)<<"setExposure Error!";
        return false;
    }

    ret = captureSingleImage(buffer);
    if(!ret)
    {
        LOG(INFO)<<"captureSingleImage Error!";
        return false;
    }


    ret = setExposure(scan_camera_exposure_);
    if(!ret)
    {
        LOG(INFO)<<"setExposure Error!";
        return false;
    }

    return ret;
}


bool CameraDh::copyBrightness(char* buffer)
{
    memcpy(buffer, brightness_buff_, buffer_size_);

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
                // LOG(TRACE)<<"H:" <<img_rows<<" W: "<<img_cols<<std::endl;
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

bool CameraDh::CaptureSelfTest()
{
    bool ret = false;
    switchToScanMode();

    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXSetEnum(hDevice_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
    LOG(INFO) << "SetEnum status 1 = " << status;

    status = GXStreamOn(hDevice_);
    LOG(INFO) << "StreamOn status 2 = " << status;

    if (status == GX_STATUS_SUCCESS)
    {
        lc3010.start_pattern_sequence();

        LOG(INFO) << "self-test start to receive one image";
        
        PGX_FRAME_BUFFER pFrameBuffer;
        status = GXDQBuf(hDevice_, &pFrameBuffer, 1000);
        LOG(INFO) << "DQBuf status 3 = " << status;

        if (status == GX_STATUS_SUCCESS)
        {
            if (pFrameBuffer->nStatus == GX_FRAME_STATUS_SUCCESS)
            {
                ret = true;
            }
            status = GXQBuf(hDevice_, pFrameBuffer);
            LOG(INFO) << "QBuf status 4 = " << status;
        }
    }

    status = GXStreamOff(hDevice_);
    LOG(INFO) << "StreamOff status 5 = " << status;

    return ret;
}

bool CameraDh::captureRawTest(int num,char* buffer)
{
    switchToScanMode();

    PGX_FRAME_BUFFER pFrameBuffer;
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


