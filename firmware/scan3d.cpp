#include "scan3d.h"
#include "easylogging++.h"
#include "encode_cuda.cuh" 
#include "../test/LookupTableFunction.h"  

Scan3D::Scan3D()
{

}

Scan3D::~Scan3D()
{

}

bool Scan3D::init()
{
    //相机初始化
    camera_ = new CameraGalaxy(); 
    if(!camera_->openCamera())
    { 
        LOG(INFO)<<"Open Galaxy Camera Error!";

        delete camera_;
        camera_ = new CameraBasler();
        if (!camera_->openCamera())
        {
            LOG(INFO) << "Open Basler Camera Error!"; 
            return false;
        }
        else
        { 
            LOG(INFO)<<"Open Basler Camera:";
        }
    }
    else
    {
        LOG(INFO)<<"Open Galaxy Camera:";
    }
    camera_->switchToExternalTriggerMode();    
    LOG(INFO)<<"switchToExternalTriggerMode!"; 

    camera_->getImageSize(image_width_,image_height_);

    buff_brightness_ = new unsigned char[image_width_*image_height_];
    buff_depth_ = new float[image_width_*image_height_];
    buff_depth_ = new float[3*image_width_*image_height_];
 
    /*****************************************************************************************/
    
    //光机初始化
    lc3010_.init();  
    lc3010_.SetLedCurrent(1023,1023,1023);

    LOG(INFO)<<"lc3010 init"; 

/**********************************************************************************************/

    //cuda初始化 
    cuda_malloc_memory();

    if(!readCalibParam())
    {
        LOG(INFO)<<"Read Calib Param Error!";   
    }
    else
    {
        cuda_copy_calib_data(calib_param_.camera_intrinsic, 
		         calib_param_.projector_intrinsic, 
			 calib_param_.camera_distortion,
	                 calib_param_.projector_distortion, 
			 calib_param_.rotation_matrix, 
			 calib_param_.translation_matrix); 
    }

   
	LookupTableFunction lookup_table_machine_; 
    MiniLookupTableFunction minilookup_table_machine_;

	lookup_table_machine_.setCalibData(calib_param_);
    minilookup_table_machine_.setCalibData(calib_param_);

    LOG(INFO)<<"start read table:";
    
    cv::Mat xL_rotate_x;
    cv::Mat xL_rotate_y;
    cv::Mat rectify_R1;
    cv::Mat pattern_mapping;
    cv::Mat pattern_minimapping;

    bool read_map_ok = lookup_table_machine_.readTableFloat("./", xL_rotate_x, xL_rotate_y, rectify_R1, pattern_mapping);
    bool read_minimap_ok = minilookup_table_machine_.readTableFloat("./", xL_rotate_x, xL_rotate_y, rectify_R1, pattern_minimapping);
  
    if(read_map_ok)
    {  
        LOG(INFO)<<"read table finished!";
	    cv::Mat R1_t = rectify_R1.t();
        xL_rotate_x.convertTo(xL_rotate_x, CV_32F);
        xL_rotate_y.convertTo(xL_rotate_y, CV_32F);
        R1_t.convertTo(R1_t, CV_32F);
        pattern_mapping.convertTo(pattern_mapping, CV_32F);

        LOG(INFO)<<"start copy table:";
        reconstruct_copy_talbe_to_cuda_memory((float*)pattern_mapping.data,(float*)xL_rotate_x.data,(float*)xL_rotate_y.data,(float*)R1_t.data);
        LOG(INFO)<<"copy finished!";

        float b = sqrt(pow(calib_param_.translation_matrix[0], 2) + pow(calib_param_.translation_matrix[1], 2) + pow(calib_param_.translation_matrix[2], 2));
        reconstruct_set_baseline(b);
    }
    if (read_minimap_ok)
    {
        cv::Mat R1_t = rectify_R1.t();
        xL_rotate_x.convertTo(xL_rotate_x, CV_32F);
        xL_rotate_y.convertTo(xL_rotate_y, CV_32F);
        R1_t.convertTo(R1_t, CV_32F);
        pattern_minimapping.convertTo(pattern_minimapping, CV_32F);

        LOG(INFO) << "start copy minitable:";
        reconstruct_copy_minitalbe_to_cuda_memory((float*)pattern_minimapping.data, (float*)xL_rotate_x.data, (float*)xL_rotate_y.data, (float*)R1_t.data);
        LOG(INFO) << "copy minitable finished!";

        float b = sqrt(pow(calib_param_.translation_matrix[0], 2) + pow(calib_param_.translation_matrix[1], 2) + pow(calib_param_.translation_matrix[2], 2));
        reconstruct_set_baseline(b);
    }
 
}

bool Scan3D::captureFrame04()
{

    lc3010_.pattern_mode04();
    if (!camera_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        return false;
    }

    lc3010_.start_pattern_sequence();

    unsigned char *buf= new unsigned char[image_width_*image_height_];

    for (int i = 0; i < 19; i++)
    {
        LOG(INFO)<<"grap "<<i<<" image:";
        if (!camera_->grap(buf))
        {
            camera_->streamOff();
            return false;
        }
        LOG(INFO)<<"finished!";

        parallel_cuda_copy_signal_patterns(buf, i);

        // copy to gpu
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

            generate_pointcloud_base_table();
            //  cudaDeviceSynchronize();
            LOG(INFO) << "generate_pointcloud_base_table";

        }
        
        default:
            break;
        }
    }

    if (!camera_->streamOff())
    {
        LOG(INFO) << "Stream Off Error";
        return false;
    }
    
    reconstruct_copy_depth_from_cuda_memory(buff_depth_);
    reconstruct_copy_brightness_from_cuda_memory(buff_brightness_);
    // reconstruct_copy_pointcloud_from_cuda_memory(buff_pointcloud_);

    return true;
}

bool Scan3D::readCalibParam()
{
    std::ifstream ifile;

    ifile.open("calib_param.txt");

    if(!ifile.is_open())
    {
        return false;
    }

    int n_params = sizeof(calib_param_)/sizeof(float);
    for(int i=0; i<n_params; i++)
    {
	ifile>>(((float*)(&calib_param_))[i]);
    }
    ifile.close();
    return true;
}

void Scan3D::copyBrightnessData(unsigned char* &ptr)
{ 
	memcpy(ptr, buff_brightness_, sizeof(unsigned char)*image_height_*image_width_); 
}

void Scan3D::copyDepthData(float* &ptr)
{ 
	memcpy(ptr, buff_depth_, sizeof(float)*image_height_*image_width_);
}