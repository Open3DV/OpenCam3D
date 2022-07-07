#include "encode_cuda.cuh"
#include <opencv2/core.hpp> 
#include <opencv2/imgcodecs.hpp>
#include <device_launch_parameters.h>
#include <device_functions.h>
#include <cuda_texture_types.h>
#include <texture_types.h>
#include "cuda_runtime.h" 
#include <cuda_runtime.h>

#include <iostream>
#include <stdint.h>
#include <vector>  
#include "easylogging++.h"
#include "protocol.h"


int patterns_count_ = 36;
int wrap_count_ = 8;
int unwrap_count_ = 2;
int image_width_ = 1920;
int image_height_ = 1200;

__device__ int d_image_width_ = 1920;
__device__ int d_image_height_ = 1200;
__device__ float d_confidence_ = 1200;


unsigned char* d_patterns_list_hdr_0[36];
float* d_confidence_list_hdr_0[8];
float* d_wrap_map_list_hdr_0[8];
float* d_unwrap_map_list_hdr_0[2];

unsigned char* d_patterns_list_hdr_1[36];
float* d_confidence_list_hdr_1[8];
float* d_wrap_map_list_hdr_1[8];
float* d_unwrap_map_list_hdr_1[2];

unsigned char* d_patterns_list_hdr_2[36];
float* d_confidence_list_hdr_2[8];
float* d_wrap_map_list_hdr_2[8];
float* d_unwrap_map_list_hdr_2[2];

#define D_HDR_MAX_NUM 6

float* d_hdr_depth_map_list_[D_HDR_MAX_NUM];
unsigned char* d_hdr_brightness_list_[D_HDR_MAX_NUM];
float* d_hdr_bright_pixel_sum_list_[D_HDR_MAX_NUM];
float* d_hdr_depth_map_;
unsigned char* d_hdr_brightness_;

#define D_REPETITIONB_MAX_NUM 10
unsigned char* d_repetition_patterns_list_[6*D_REPETITIONB_MAX_NUM]; 
unsigned short* d_repetition_merge_patterns_list_[6];  

#define D_REPETITION_02_MAX_NUM 37
__device__ unsigned short* d_repetition_02_merge_patterns_list_[D_REPETITION_02_MAX_NUM];  
/*****************************************************************************/

texture<unsigned char, 1, cudaReadModeElementType> texture_patterns_0;
texture<unsigned char, 1, cudaReadModeElementType> texture_patterns_1;
texture<unsigned char, 1, cudaReadModeElementType> texture_patterns_2;
texture<unsigned char, 1, cudaReadModeElementType> texture_patterns_3;
texture<unsigned char, 1, cudaReadModeElementType> texture_patterns_4;
texture<unsigned char, 1, cudaReadModeElementType> texture_patterns_5;
texture<unsigned char, 1, cudaReadModeElementType> texture_patterns_6;
texture<unsigned char, 1, cudaReadModeElementType> texture_patterns_7;
texture<unsigned char, 1, cudaReadModeElementType> texture_patterns_8;
texture<unsigned char, 1, cudaReadModeElementType> texture_patterns_9;
texture<unsigned char, 1, cudaReadModeElementType> texture_patterns_10;
texture<unsigned char, 1, cudaReadModeElementType> texture_patterns_11;
texture<unsigned char, 1, cudaReadModeElementType> texture_patterns_12;
texture<unsigned char, 1, cudaReadModeElementType> texture_patterns_13;
texture<unsigned char, 1, cudaReadModeElementType> texture_patterns_14;
texture<unsigned char, 1, cudaReadModeElementType> texture_patterns_15;
texture<unsigned char, 1, cudaReadModeElementType> texture_patterns_16;
texture<unsigned char, 1, cudaReadModeElementType> texture_patterns_17;
texture<unsigned char, 1, cudaReadModeElementType> texture_patterns_18;

__device__ unsigned char* d_patterns_list[36];
__device__ unsigned char* d_brightness_;
__device__ float* d_confidence_list[8];
__device__ float* d_wrap_map_list[8];
__device__ float* d_unwrap_map_list[2];
__device__ float* d_point_cloud_map_;
__device__ float* d_depth_map_;
__device__ short* d_depth_map_short_;
__device__ float* d_triangulation_error_map_;

unsigned char* d_mask_;
/*********************************************************************************/
float* d_camera_intrinsic_;
float* d_project_intrinsic_;
float* d_camera_distortion_;
float* d_projector_distortion_;
float* d_rotation_matrix_;
float* d_translation_matrix_;

__device__ int d_dlp_width_ = 1920;
__device__ int d_dlp_height_ = 1080;
__device__ float d_max_phase_ = 2* 3.1415926535;

bool load_calib_data_flag_ = false;

#define DF_PI 3.1415926535
/*********************************************************************************/


__device__ float d_baseline_ = 0;
 
// 因为有多个查找表，在上传查找表前，先释放内存，防止内存泄漏
__device__ float* d_single_pattern_mapping_ = NULL;
__device__ float* d_single_pattern_minimapping_ = NULL;
__device__ float* d_xL_rotate_x_ = NULL;
__device__ float* d_xL_rotate_y_ = NULL; 
__device__ float* d_R_1_ = NULL; 

  
/*********************************************************************************************/


dim3 threadsPerBlock(8, 8);
dim3 blocksPerGrid((image_width_ + threadsPerBlock.x - 1) / threadsPerBlock.x,
(image_height_ + threadsPerBlock.y - 1) / threadsPerBlock.y);


#define CHECK(call)\
{\
  const cudaError_t error=call;\
  if(error!=cudaSuccess)\
  {\
      printf("ERROR: %s:%d,",__FILE__,__LINE__);\
      printf("code:%d,reason:%s\n",error,cudaGetErrorString(error));\
      exit(1);\
  }\
}


void cuda_set_config(struct SystemConfigDataStruct param)
{ 
	cudaMemcpyToSymbol(d_confidence_, &param.Instance().firwmare_param_.confidence, sizeof(float));
}

bool cuda_set_camera_version(int version)
{
    switch (version)
    {
    case DFX_800:
    {
		int dlp_width = 1280;
		int dlp_height = 720;
		cudaMemcpyToSymbol(d_dlp_width_, &dlp_width, sizeof(int));
		cudaMemcpyToSymbol(d_dlp_height_, &dlp_height, sizeof(int));
  
		int camera_width = 1920;
		int camera_height = 1200;
		cudaMemcpyToSymbol(d_image_width_, &camera_width, sizeof(int));
		cudaMemcpyToSymbol(d_image_height_, &camera_height, sizeof(int));

        return true;
    }
    break;

    case DFX_1800:
    {
		int dlp_width = 1920;
		int dlp_height = 1080;
		cudaMemcpyToSymbol(d_dlp_width_, &dlp_width, sizeof(int));
		cudaMemcpyToSymbol(d_dlp_height_, &dlp_height, sizeof(int));

		int camera_width = 1920;
		int camera_height = 1200;
		cudaMemcpyToSymbol(d_image_width_, &camera_width, sizeof(int));
		cudaMemcpyToSymbol(d_image_height_, &camera_height, sizeof(int));
        return true;
    }
    break;

    default:
        break;
    }

	return false;
}


/***********************************************************************************************************************************************/

bool parallel_cuda_copy_signal_patterns(unsigned char* patterns_ptr,int serial_flag)
{
	CHECK(cudaMemcpyAsync(d_patterns_list[serial_flag], patterns_ptr, image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyHostToDevice)); 
}

bool parallel_cuda_copy_repetition_signal_patterns(unsigned char* patterns_ptr,int serial_flag)
{
	CHECK(cudaMemcpyAsync(d_repetition_patterns_list_[serial_flag], patterns_ptr, image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyHostToDevice));
}


bool parallel_cuda_clear_repetition_02_patterns()
{
	for(int i = 0;i< D_REPETITION_02_MAX_NUM;i++)
	{ 
		 cudaMemset(d_repetition_02_merge_patterns_list_[i], 0,image_width_* image_height_*sizeof(ushort));
		// CHECK(cudaMemcpyAsync(d_repetition_02_merge_patterns_list_[i], &val,image_width_* image_height_*sizeof(ushort), cudaMemcpyHostToDevice));
	}
	// cudaDeviceSynchronize();
  
  return true;
}

bool parallel_cuda_merge_repetition_02_patterns(int repetition_serial)
{
	// int merge_serial = repetition_serial%19; 
	cuda_merge_pattern<< <blocksPerGrid, threadsPerBlock >> >(d_patterns_list[repetition_serial],image_height_, image_width_,d_repetition_02_merge_patterns_list_[repetition_serial]);

	return true;
}

bool parallel_cuda_merge_repetition_patterns(int repetition_serial)
{

	int merge_serial = repetition_serial%6; 
	cuda_merge_pattern<< <blocksPerGrid, threadsPerBlock >> >(d_repetition_patterns_list_[repetition_serial],image_height_, image_width_,d_repetition_merge_patterns_list_[merge_serial]);

	return true;
}

__global__ void cuda_merge_pattern(unsigned char * const d_in_pattern,uint32_t img_height, uint32_t img_width,unsigned short * const d_out_merge_pattern)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * img_width + idx;

	if (idx < img_width && idy < img_height)
	{
  
		d_out_merge_pattern[offset] += d_in_pattern[offset];  

	}
}


bool parallel_cuda_compute_model_02_merge_repetition_02_phase(int repetition_count)
{
	int i = 0;
	cuda_merge_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_repetition_02_merge_patterns_list_[i+ 0], d_repetition_02_merge_patterns_list_[i+ 1],
		d_repetition_02_merge_patterns_list_[i+ 2],d_repetition_02_merge_patterns_list_[i+ 3],repetition_count, d_wrap_map_list[0], d_confidence_list[0]);
			
	i = 4;
	cuda_merge_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_repetition_02_merge_patterns_list_[i+ 0], d_repetition_02_merge_patterns_list_[i+ 1],
		d_repetition_02_merge_patterns_list_[i+ 2],d_repetition_02_merge_patterns_list_[i+ 3],repetition_count, d_wrap_map_list[1], d_confidence_list[1]);

	i = 8;
	cuda_merge_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_repetition_02_merge_patterns_list_[i+ 0], d_repetition_02_merge_patterns_list_[i+ 1],
		d_repetition_02_merge_patterns_list_[i+ 2],d_repetition_02_merge_patterns_list_[i+ 3],repetition_count, d_wrap_map_list[2], d_confidence_list[2]);
	
	i = 12;
	cuda_merge_six_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_repetition_02_merge_patterns_list_[i+ 0], d_repetition_02_merge_patterns_list_[i+ 1],
		d_repetition_02_merge_patterns_list_[i+ 2],d_repetition_02_merge_patterns_list_[i+ 3],d_repetition_02_merge_patterns_list_[i+ 4],d_repetition_02_merge_patterns_list_[i+ 5] ,
		repetition_count,image_height_, image_width_, d_wrap_map_list[3], d_confidence_list[3]);

	i = 18;
	cuda_merge_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_repetition_02_merge_patterns_list_[i+ 0], d_repetition_02_merge_patterns_list_[i+ 1],
		d_repetition_02_merge_patterns_list_[i+ 2],d_repetition_02_merge_patterns_list_[i+ 3],repetition_count, d_wrap_map_list[4], d_confidence_list[4]);
			
	i = 22;
	cuda_merge_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_repetition_02_merge_patterns_list_[i+ 0], d_repetition_02_merge_patterns_list_[i+ 1],
		d_repetition_02_merge_patterns_list_[i+ 2],d_repetition_02_merge_patterns_list_[i+ 3],repetition_count, d_wrap_map_list[5], d_confidence_list[5]);

	i = 26;
	cuda_merge_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_repetition_02_merge_patterns_list_[i+ 0], d_repetition_02_merge_patterns_list_[i+ 1],
		d_repetition_02_merge_patterns_list_[i+ 2],d_repetition_02_merge_patterns_list_[i+ 3],repetition_count, d_wrap_map_list[6], d_confidence_list[6]);
	
	i = 30;
	cuda_merge_six_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_repetition_02_merge_patterns_list_[i+ 0], d_repetition_02_merge_patterns_list_[i+ 1],
		d_repetition_02_merge_patterns_list_[i+ 2],d_repetition_02_merge_patterns_list_[i+ 3],d_repetition_02_merge_patterns_list_[i+ 4],d_repetition_02_merge_patterns_list_[i+ 5] ,
		repetition_count,image_height_, image_width_, d_wrap_map_list[7], d_confidence_list[7]);

	cuda_merge_brigntness_map<< <blocksPerGrid, threadsPerBlock >> >(d_repetition_02_merge_patterns_list_[36],repetition_count,d_brightness_);
}

bool parallel_cuda_compute_merge_repetition_02_phase(int repetition_count)
{
	
	cuda_merge_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_repetition_02_merge_patterns_list_[0], d_repetition_02_merge_patterns_list_[1],
		d_repetition_02_merge_patterns_list_[2],d_repetition_02_merge_patterns_list_[3],repetition_count, d_wrap_map_list[0], d_confidence_list[0]);
			
	cuda_merge_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_repetition_02_merge_patterns_list_[4], d_repetition_02_merge_patterns_list_[5],
		d_repetition_02_merge_patterns_list_[6],d_repetition_02_merge_patterns_list_[7],repetition_count, d_wrap_map_list[1], d_confidence_list[1]);

	cuda_merge_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_repetition_02_merge_patterns_list_[8], d_repetition_02_merge_patterns_list_[9],
		d_repetition_02_merge_patterns_list_[10],d_repetition_02_merge_patterns_list_[11],repetition_count, d_wrap_map_list[2], d_confidence_list[2]);
	
	cuda_merge_six_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_repetition_02_merge_patterns_list_[12], d_repetition_02_merge_patterns_list_[13],
		d_repetition_02_merge_patterns_list_[14],d_repetition_02_merge_patterns_list_[15],d_repetition_02_merge_patterns_list_[16],d_repetition_02_merge_patterns_list_[17] ,
		repetition_count,image_height_, image_width_, d_wrap_map_list[3], d_confidence_list[3]);

	return true;
}

bool parallel_cuda_compute_merge_phase(int repetition_count)
{

	cuda_merge_six_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_repetition_merge_patterns_list_[0], d_repetition_merge_patterns_list_[1],
		d_repetition_merge_patterns_list_[2],d_repetition_merge_patterns_list_[3],d_repetition_merge_patterns_list_[4],d_repetition_merge_patterns_list_[5] ,
		repetition_count,image_height_, image_width_, d_wrap_map_list[3], d_confidence_list[3]);

	return true;
}

__global__ void cuda_merge_six_step_phase_shift(unsigned short * const d_in_0, unsigned short * const d_in_1, unsigned short * const d_in_2, 
	unsigned short * const d_in_3,unsigned short* const d_in_4,unsigned short* const d_in_5,int repetition_count,
	uint32_t img_height, uint32_t img_width,float * const d_out, float * const confidence)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * img_width + idx;
	float s_0 =  0;
	float s_1 =  0.866025;
	float s_2 =  0.866025;
	float s_3 =  0;
	float s_4 =  -0.866025;
	float s_5 =  -0.866025;
	float c_0 =  1;
	float c_1 =  0.5;
	float c_2 =  -0.5;
	float c_3 =  -1;
	float c_4 =  -0.5;
	float c_5 =  0.5;
	
	if (idx < img_width && idy < img_height)
	{

		float a = c_0 *d_in_3[offset] + c_1 *d_in_4[offset] + c_2 *d_in_5[offset] + c_3* d_in_0[offset] +c_4*d_in_1[offset] + c_5*d_in_2[offset];
		float b = s_0 *d_in_3[offset] + s_1 *d_in_4[offset] + s_2 *d_in_5[offset] + s_3* d_in_0[offset] +s_4*d_in_1[offset] + s_5*d_in_2[offset];

  
		confidence[offset] = std::sqrt(a*a + b*b);
		d_out[offset] = DF_PI + std::atan2(a, b);
	}

	
}

/***********************************************************************************************************************************************************/

bool parallel_cuda_compute_phase(int serial_flag)
{
	 
	switch(serial_flag)
	{
		case 0:
		{ 
			// int i= 0;
			// cuda_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_patterns_list[i+0], d_patterns_list[i + 1], d_patterns_list[i + 2],
			// 	d_patterns_list[i + 3], image_height_, image_width_, d_wrap_map_list[serial_flag], d_confidence_list[serial_flag]);

				cuda_four_step_phase_shift_texture<< <blocksPerGrid, threadsPerBlock >> >(serial_flag,d_wrap_map_list[serial_flag], d_confidence_list[serial_flag]);
		}
		break;
		case 1:
		{

			// int i= 4;
			// cuda_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_patterns_list[i+0], d_patterns_list[i + 1], d_patterns_list[i + 2],
			// 	d_patterns_list[i + 3], image_height_, image_width_, d_wrap_map_list[serial_flag], d_confidence_list[serial_flag]);
				
				cuda_four_step_phase_shift_texture<< <blocksPerGrid, threadsPerBlock >> >(serial_flag,d_wrap_map_list[serial_flag], d_confidence_list[serial_flag]);
			
		}
		break;
		case 2:
		{ 
			// int i= 8;
			// cuda_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_patterns_list[i+0], d_patterns_list[i + 1], d_patterns_list[i + 2],
			// 	d_patterns_list[i + 3], image_height_, image_width_, d_wrap_map_list[serial_flag], d_confidence_list[serial_flag]);
				
				cuda_four_step_phase_shift_texture<< <blocksPerGrid, threadsPerBlock >> >(serial_flag,d_wrap_map_list[serial_flag], d_confidence_list[serial_flag]);
		}
		break;
		case 3:
		{ 
			// int i= 12; 
			// cuda_six_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_patterns_list[i+0], d_patterns_list[i + 1], d_patterns_list[i + 2],
			// 	d_patterns_list[i + 3],d_patterns_list[i + 4],d_patterns_list[i + 5] ,
			// 	 image_height_, image_width_, d_wrap_map_list[serial_flag], d_confidence_list[serial_flag]);
 
				cuda_six_step_phase_shift_texture<< <blocksPerGrid, threadsPerBlock >> > (d_wrap_map_list[serial_flag], d_confidence_list[serial_flag]);
				// cudaDeviceSynchronize();

				// cv::Mat phase(1200, 1920, CV_32F, cv::Scalar(0));
				// CHECK(cudaMemcpy(phase.data, d_wrap_map_list[serial_flag], 1 * image_height_ * image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
				// cv::imwrite("phase1.tiff",phase);
		}
		break;
		case 4:
		{
			int i= 18;
			cuda_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_patterns_list[i+0], d_patterns_list[i + 1], d_patterns_list[i + 2],
				d_patterns_list[i + 3], image_height_, image_width_, d_wrap_map_list[serial_flag], d_confidence_list[serial_flag]);
		}
		break;
		case 5:
		{
			int i= 22;
			cuda_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_patterns_list[i+0], d_patterns_list[i + 1], d_patterns_list[i + 2],
				d_patterns_list[i + 3], image_height_, image_width_, d_wrap_map_list[serial_flag], d_confidence_list[serial_flag]);
		}
		break;
		case 6:
		{
			int i= 26;
			cuda_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_patterns_list[i+0], d_patterns_list[i + 1], d_patterns_list[i + 2],
				d_patterns_list[i + 3], image_height_, image_width_, d_wrap_map_list[serial_flag], d_confidence_list[serial_flag]);
		}
		break;
  
		default :
			break;
	}

	
	
	return true;
}


bool parallel_cuda_unwrap_phase(int serial_flag)
{

	switch(serial_flag)
	{
		case 0:
		{ 

		}
		break;
		case 1:
		{ 
			// CHECK( cudaFuncSetCacheConfig (cuda_variable_phase_unwrap, cudaFuncCachePreferL1) );
            cuda_variable_phase_unwrap<< <blocksPerGrid, threadsPerBlock >> >(d_wrap_map_list[0], d_wrap_map_list[1], 8.0,
				image_height_, image_width_,CV_PI, d_unwrap_map_list[0]);
            // CHECK ( cudaGetLastError () );

			// cuda_variable_phase_unwrap << <blocksPerGrid, threadsPerBlock >> >(d_wrap_map_list[0], d_wrap_map_list[1], 8.0,
			// 	image_height_, image_width_, d_unwrap_map_list[0]);
				
			
		}
		break;
		case 2:
		{ 
			// CHECK( cudaFuncSetCacheConfig (cuda_variable_phase_unwrap, cudaFuncCachePreferL1) );
			cuda_variable_phase_unwrap << <blocksPerGrid, threadsPerBlock >> >(d_unwrap_map_list[0], d_wrap_map_list[2], 4.0,
				image_height_, image_width_,CV_PI, d_unwrap_map_list[0]); 
			// CHECK ( cudaGetLastError () );
		}
		break;
		case 3:
		{ 
			// CHECK( cudaFuncSetCacheConfig (cuda_variable_phase_unwrap, cudaFuncCachePreferL1) );
			cuda_variable_phase_unwrap << <blocksPerGrid, threadsPerBlock >> >(d_unwrap_map_list[0], d_wrap_map_list[3], 4.0,
				image_height_, image_width_,1.5, d_unwrap_map_list[0]); 

			// CHECK ( cudaGetLastError () );
		}
		break;
		case 4:
		{
 
		}
		break;
		case 5:
		{
			cuda_variable_phase_unwrap << <blocksPerGrid, threadsPerBlock >> >(d_wrap_map_list[4], d_wrap_map_list[5], 8.0,
				image_height_, image_width_,CV_PI, d_unwrap_map_list[1]);
		}
		break;
		case 6:
		{
			cuda_variable_phase_unwrap << <blocksPerGrid, threadsPerBlock >> >(d_unwrap_map_list[1], d_wrap_map_list[6], 4.0,
				image_height_, image_width_,CV_PI, d_unwrap_map_list[1]);
			// cuda_normalize_phase << <blocksPerGrid, threadsPerBlock >> >(d_unwrap_map_list[0],128.0, d_unwrap_map_list[1],18.0,
			// image_height_, image_width_, d_unwrap_map_list[0],d_unwrap_map_list[1]);
			
			LOG(INFO)<<"unwrap 6:  ";

		}
		break;
		case 7:
		{
			cuda_variable_phase_unwrap << <blocksPerGrid, threadsPerBlock >> >(d_unwrap_map_list[1], d_wrap_map_list[7], 4.0,
				image_height_, image_width_,CV_PI, d_unwrap_map_list[1]);
			cuda_normalize_phase << <blocksPerGrid, threadsPerBlock >> >(d_unwrap_map_list[0],128.0, d_unwrap_map_list[1],72.0,
			image_height_, image_width_, d_unwrap_map_list[0],d_unwrap_map_list[1]);
			
			LOG(INFO)<<"unwrap 7:  ";

		}
		break;
 

		default :
			break;
	}


	return true;
}


bool parallel_cuda_reconstruct()
{
	cuda_rebuild << <blocksPerGrid, threadsPerBlock >> >(d_unwrap_map_list[0], d_unwrap_map_list[1],d_camera_intrinsic_,d_camera_distortion_,
		d_project_intrinsic_,d_projector_distortion_,d_rotation_matrix_,d_translation_matrix_, 
		d_point_cloud_map_,d_depth_map_, d_triangulation_error_map_, d_confidence_list[3]);

  
	cudaDeviceSynchronize();

	
    LOG(INFO)<<"rebuild data!";
	 

    // LOG(INFO)<<"unwrap_0";

	// cv::Mat confidence_map(image_height_, image_width_, CV_32F, cv::Scalar(0));
	// cudaMemcpy(confidence_map.data, d_confidence_list[3], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);

	// cv::Mat wrap_1(image_height_, image_width_, CV_32F, cv::Scalar(0));
	// cudaMemcpy(wrap_1.data, d_wrap_map_list[1], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);
	// cv::Mat wrap_2(image_height_, image_width_, CV_32F, cv::Scalar(0));
	// cudaMemcpy(wrap_2.data, d_wrap_map_list[2], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);

	// cv::Mat wrap_3(image_height_, image_width_, CV_32F, cv::Scalar(0));
	// cudaMemcpy(wrap_3.data, d_wrap_map_list[3], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);

	// cv::imwrite("confidence_map.tiff",confidence_map);
	// cv::imwrite("wrap_3.tiff",wrap_3);
	// cv::imwrite("wrap_1.tiff",wrap_1);
	// cv::imwrite("wrap_2.tiff",wrap_2);

	// cv::Mat unwrap_0(image_height_, image_width_, CV_32F, cv::Scalar(0));
	// cudaMemcpy(unwrap_0.data, d_unwrap_map_list[0], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);

	// cv::Mat unwrap_1(image_height_, image_width_, CV_32F, cv::Scalar(0));
	// cudaMemcpy(unwrap_1.data,d_unwrap_map_list[1], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);

	// cv::Mat deep_map(image_height_, image_width_, CV_32F, cv::Scalar(0));
	// cudaMemcpy(deep_map.data, d_depth_map_, image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);

	// cv::Mat err_map(image_height_, image_width_, CV_32F, cv::Scalar(0));
	// cudaMemcpy(err_map.data, d_triangulation_error_map_, image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);

	// cv::Mat points_map(image_height_, image_width_, CV_32FC3, cv::Scalar(0));
	// cudaMemcpy(points_map.data, d_point_cloud_map_, 3*image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);

 
	
    // LOG(INFO)<<"copy data!";

	// cv::imwrite("unwrap_map_0.tiff",unwrap_0);
	// cv::imwrite("unwrap_map_1.tiff",unwrap_1);
	// cv::imwrite("deep_map.tiff",deep_map);
	// cv::imwrite("err_map.tiff",err_map);
	// cv::imwrite("points_map.tiff",points_map);
	
    // LOG(INFO)<<"rebuild data!";


}


bool parallel_cuda_copy_pointcloud_from_gpu(float* pointcloud,unsigned char* brightness)
{
	if(!load_calib_data_flag_)
	{
		return false;
	} 

	LOG(INFO)<<"copy......"; 
	cuda_get_brightness_data(brightness); 
	cuda_get_pointcloud_data(pointcloud); 
	LOG(INFO)<<"copy result";
}

bool parallel_cuda_copy_result_from_gpu(float* depth,unsigned char* brightness)
{
	if(!load_calib_data_flag_)
	{
		return false;
	} 

	LOG(INFO)<<"copy......"; 
	cuda_get_brightness_data(brightness); 
	cuda_get_depth_data(depth); 
	LOG(INFO)<<"copy result";
 
}

bool parallel_cuda_copy_unwrap_phase_from_gpu(int serial_flag,float* unwrap_map)
{

	CHECK(cudaMemcpy(unwrap_map, d_unwrap_map_list[serial_flag], 1 * image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
	return true;
}

bool parallel_cuda_copy_unwrap_phase_to_gpu(int serial_flag,float* unwrap_map)
{

	CHECK(cudaMemcpy(d_unwrap_map_list[serial_flag], unwrap_map,  1 * image_height_*image_width_ * sizeof(float), cudaMemcpyHostToDevice));
	return true;
}

void BubbleSort(float  *p, int length, int * ind_diff)
{
	for (int m = 0; m < length; m++)
	{
		ind_diff[m] = m;
	}
 
	for (int i = 0; i < length; i++)
	{
		for (int j = 0; j < length- i - 1; j++)
		{
			if (p[j] > p[j + 1])
			{
				float temp = p[j];
				p[j] = p[j + 1];
				p[j + 1] = temp;
 
				int ind_temp = ind_diff[j];
				ind_diff[j] = ind_diff[j + 1];
				ind_diff[j + 1] = ind_temp;
			}
		}
	}
}

 

bool parallel_cuda_merge_hdr_data(int hdr_num,float* depth_map, unsigned char* brightness)
{
	
	LOG(INFO)<<"sum pixels ";
	float sum_pixels_list[6];  

    for(int i= 0;i<hdr_num;i++)
    { 
		CHECK(cudaMemcpy(&sum_pixels_list[i], d_hdr_bright_pixel_sum_list_[i], 1* sizeof(float), cudaMemcpyDeviceToHost));
    }
 
 
	std::vector<float> param_list;
	std::vector<int> id; 
	std::vector<bool> flag_list;

	for (int i = 0; i < hdr_num; i++)
	{ 
        param_list.push_back(sum_pixels_list[i]);
		id.push_back(0);
		flag_list.push_back(true);
    } 
   	std::sort(param_list.begin(),param_list.end(),std::greater<float>());
 
 
	for (int i = 0; i < hdr_num; i++)
	{ 
		
		for(int j= 0;j< hdr_num;j++)
		{
			if(param_list[i] == sum_pixels_list[j])
			{
				if(flag_list[j])
				{ 
					id[i] = j;
					flag_list[j] = false; 
					break;
				}
			}
		}
		 
    } 

 
	for (int i = 0; i < hdr_num; i++)
	{ 
        LOG(INFO)<<"sum pixels "<<i<<": "<<sum_pixels_list[i]<<" _ "<<id[i];
    }
 

	switch(hdr_num)
	{
		case 1:
		{

			CHECK(cudaMemcpy(depth_map, d_hdr_depth_map_list_[0], 1 * image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_list_[0], 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));
		} 
		break;
		case 2:
		{
			parallel_cuda_merge_hdr_2 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_depth_map_list_[id[0]],d_hdr_depth_map_list_[id[1]], d_hdr_brightness_list_[id[0]], 
				d_hdr_brightness_list_[id[1]], image_height_, image_width_, d_hdr_depth_map_,d_hdr_brightness_);

				
			CHECK(cudaMemcpy(depth_map, d_hdr_depth_map_, 1 * image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));

		}
		break;
		case 3:
		{
			parallel_cuda_merge_hdr_3 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_depth_map_list_[id[0]],d_hdr_depth_map_list_[id[1]],d_hdr_depth_map_list_[id[2]], d_hdr_brightness_list_[id[0]], 
				d_hdr_brightness_list_[id[1]], d_hdr_brightness_list_[id[2]], image_height_, image_width_, d_hdr_depth_map_,d_hdr_brightness_);
				
			CHECK(cudaMemcpy(depth_map, d_hdr_depth_map_, 1 * image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));

		}
		break;
		case 4:
		{
			parallel_cuda_merge_hdr_4 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_depth_map_list_[id[0]],d_hdr_depth_map_list_[id[1]],d_hdr_depth_map_list_[id[2]],d_hdr_depth_map_list_[id[3]],
				 d_hdr_brightness_list_[id[0]], d_hdr_brightness_list_[id[1]], d_hdr_brightness_list_[id[2]], d_hdr_brightness_list_[id[3]], 
				image_height_, image_width_, d_hdr_depth_map_,d_hdr_brightness_);
				
			CHECK(cudaMemcpy(depth_map, d_hdr_depth_map_, 1 * image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));

		}
		break;
		case 5:
		{
			parallel_cuda_merge_hdr_5 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_depth_map_list_[id[0]],d_hdr_depth_map_list_[id[1]],d_hdr_depth_map_list_[id[2]],
				d_hdr_depth_map_list_[id[3]],d_hdr_depth_map_list_[id[4]],
				 d_hdr_brightness_list_[id[0]], d_hdr_brightness_list_[id[1]], d_hdr_brightness_list_[id[2]], d_hdr_brightness_list_[id[3]], d_hdr_brightness_list_[id[4]], 
				image_height_, image_width_, d_hdr_depth_map_,d_hdr_brightness_);
				
			CHECK(cudaMemcpy(depth_map, d_hdr_depth_map_, 1 * image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));

		}
		break;
		case 6:
		{
			parallel_cuda_merge_hdr_6 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_depth_map_list_[id[0]],d_hdr_depth_map_list_[id[1]],d_hdr_depth_map_list_[id[2]],
				d_hdr_depth_map_list_[id[3]],d_hdr_depth_map_list_[id[4]],d_hdr_depth_map_list_[id[5]],
				 d_hdr_brightness_list_[id[0]], d_hdr_brightness_list_[id[1]], d_hdr_brightness_list_[id[2]], d_hdr_brightness_list_[id[3]], d_hdr_brightness_list_[id[4]], 
				 d_hdr_brightness_list_[id[5]], 
				image_height_, image_width_, d_hdr_depth_map_,d_hdr_brightness_);
				
			CHECK(cudaMemcpy(depth_map, d_hdr_depth_map_, 1 * image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));

		}
		break;

		default:
		 		return false;

	}

 	// CHECK(cudaMemcpy(brightness, d_hdr_brightness_list_[id[0]], 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));
 	CHECK(cudaMemcpy(brightness, d_hdr_brightness_list_[hdr_num-1], 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));
	LOG(INFO)<<"DHR Finished!";

	return true;
}

__global__ void parallel_cuda_merge_hdr_6(const float*  depth_map_0,const float*  depth_map_1,const float*  depth_map_2,
	const float*  depth_map_3,const float*  depth_map_4,const float*  depth_map_5,
	const unsigned char* brightness_0,const unsigned char* brightness_1,const unsigned char* brightness_2,
	const unsigned char* brightness_3,const unsigned char* brightness_4,const unsigned char* brightness_5,
	uint32_t img_height, uint32_t img_width, float* const depth_map,unsigned char * const brightness)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * img_width + idx;

	if (idx < img_width && idy < img_height)
	{
 

		float pixel= 0;
		pixel +=  brightness_0[offset];
		pixel +=  brightness_1[offset];
		pixel +=  brightness_2[offset];
		pixel +=  brightness_3[offset];
		pixel +=  brightness_4[offset];
		pixel +=  brightness_5[offset];

		pixel/= 6.0;


		brightness[offset] = pixel;

		if(brightness_0[offset] < 255)
		{
			// brightness[offset] = brightness_0[offset];
			depth_map[offset] = depth_map_0[offset];
		}

		else if(brightness_1[offset] < 255)
		{
			// brightness[offset] = brightness_1[offset];
			depth_map[offset] = depth_map_1[offset];
		}
		else if(brightness_2[offset] < 255)
		{
			// brightness[offset] = brightness_1[offset];
			depth_map[offset] = depth_map_2[offset];
		}
		else if(brightness_3[offset] < 255)
		{
			// brightness[offset] = brightness_1[offset];
			depth_map[offset] = depth_map_3[offset];
		}
		else if(brightness_4[offset] < 255)
		{
			// brightness[offset] = brightness_1[offset];
			depth_map[offset] = depth_map_4[offset];
		}
		else
		{	
			// brightness[offset] = brightness_2[offset];
			depth_map[offset] = depth_map_5[offset];
		}
		//没有深度则用最亮的深度值
		if (depth_map[offset] <= 0)
		{
			depth_map[offset] = depth_map_0[offset];
		}
	}
}

__global__ void parallel_cuda_merge_hdr_5(const float*  depth_map_0,const float*  depth_map_1,const float*  depth_map_2,
	const float*  depth_map_3,const float*  depth_map_4,
	const unsigned char* brightness_0,const unsigned char* brightness_1,const unsigned char* brightness_2,
	const unsigned char* brightness_3,const unsigned char* brightness_4,
	uint32_t img_height, uint32_t img_width, float* const depth_map,unsigned char * const brightness)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * img_width + idx;

	if (idx < img_width && idy < img_height)
	{



		float pixel= 0;
		pixel +=  brightness_0[offset];
		pixel +=  brightness_1[offset];
		pixel +=  brightness_2[offset];
		pixel +=  brightness_3[offset];
		pixel +=  brightness_4[offset];

		pixel/= 5.0;


		brightness[offset] = pixel;

		if(brightness_0[offset] < 255)
		{
			// brightness[offset] = brightness_0[offset];
			depth_map[offset] = depth_map_0[offset];
		}

		else if(brightness_1[offset] < 255)
		{
			// brightness[offset] = brightness_1[offset];
			depth_map[offset] = depth_map_1[offset];
		}
		else if(brightness_2[offset] < 255)
		{
			// brightness[offset] = brightness_1[offset];
			depth_map[offset] = depth_map_2[offset];
		}
		else if(brightness_3[offset] < 255)
		{
			// brightness[offset] = brightness_1[offset];
			depth_map[offset] = depth_map_3[offset];
		}
		else
		{	
			// brightness[offset] = brightness_2[offset];
			depth_map[offset] = depth_map_4[offset];
		}
		//没有深度则用最亮的深度值
		if (depth_map[offset] <= 0)
		{
			depth_map[offset] = depth_map_0[offset];
		}
	}
}


__global__ void parallel_cuda_merge_hdr_4(const float*  depth_map_0,const float*  depth_map_1,const float*  depth_map_2,const float*  depth_map_3,
	const unsigned char* brightness_0,const unsigned char* brightness_1,const unsigned char* brightness_2,const unsigned char* brightness_3,
	uint32_t img_height, uint32_t img_width, float* const depth_map,unsigned char * const brightness)
	{
		const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
		const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
		const unsigned int offset = idy * img_width + idx;
	
		if (idx < img_width && idy < img_height)
		{
	
	
	
			float pixel= 0;
			pixel +=  brightness_0[offset];
			pixel +=  brightness_1[offset];
			pixel +=  brightness_2[offset];
			pixel +=  brightness_3[offset];
	
			pixel/= 4.0;
	
	
			brightness[offset] = pixel;
	
			if(brightness_0[offset] < 255)
			{
				// brightness[offset] = brightness_0[offset];
				depth_map[offset] = depth_map_0[offset];
			}
	
			else if(brightness_1[offset] < 255)
			{
				// brightness[offset] = brightness_1[offset];
				depth_map[offset] = depth_map_1[offset];
			}
			else if(brightness_2[offset] < 255)
			{
				// brightness[offset] = brightness_1[offset];
				depth_map[offset] = depth_map_2[offset];
			}
			else
			{	
				// brightness[offset] = brightness_2[offset];
				depth_map[offset] = depth_map_3[offset];
			}
			//没有深度则用最亮的深度值
			if (depth_map[offset] <= 0)
			{
				depth_map[offset] = depth_map_0[offset];
			}
		}
	}

__global__ void parallel_cuda_merge_hdr_3(const float*  depth_map_0,const float*  depth_map_1,const float*  depth_map_2,const unsigned char* brightness_0,const unsigned char* brightness_1,
	const unsigned char* brightness_2,uint32_t img_height, uint32_t img_width, float* const depth_map,unsigned char * const brightness)
	{
		const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
		const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
		const unsigned int offset = idy * img_width + idx;
	
		if (idx < img_width && idy < img_height)
		{
	
	
	
			float pixel= 0;
			pixel +=  brightness_0[offset];
			pixel +=  brightness_1[offset];
			pixel +=  brightness_2[offset];
	
			pixel/= 3.0;
	
	
			brightness[offset] = pixel;
	
			if(brightness_0[offset] < 255)
			{
				// brightness[offset] = brightness_0[offset];
				depth_map[offset] = depth_map_0[offset];
			}
	
			else if(brightness_1[offset] < 255)
			{
				// brightness[offset] = brightness_1[offset];
				depth_map[offset] = depth_map_1[offset];
			}
			else
			{	
				// brightness[offset] = brightness_2[offset];
				depth_map[offset] = depth_map_2[offset];
			}
				//没有深度则用最亮的深度值
			if(depth_map[offset]<= 0)
			{
				depth_map[offset] = depth_map_0[offset];
			}
	
		}
	}

__global__ void parallel_cuda_merge_hdr_2(const float*  depth_map_0,const float*  depth_map_1,const unsigned char* brightness_0,const unsigned char* brightness_1,
	uint32_t img_height, uint32_t img_width, float* const depth_map,unsigned char * const brightness)
	{
		const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
		const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
		const unsigned int offset = idy * img_width + idx;
	
		if (idx < img_width && idy < img_height)
		{
	
			float pixel= 0;
			pixel +=  brightness_0[offset];
			pixel +=  brightness_1[offset];
	
			pixel/= 2.0;
	
	
			brightness[offset] = pixel;
	
			if(brightness_0[offset] < 255)
			{
				// brightness[offset] = brightness_0[offset];
				depth_map[offset] = depth_map_0[offset];
			}
			else 
			{
				// brightness[offset] = brightness_1[offset];
				depth_map[offset] = depth_map_1[offset];
			}

			//没有深度则用最亮的深度值
			if(depth_map[offset]<= 0)
			{
				depth_map[offset] = depth_map_0[offset];
			}

		}
	}


__global__ void parallel_cuda_count_sum_pixel(const unsigned char* brightness,uint32_t img_height, uint32_t img_width, float* sum_pixels)
{
			const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
		const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
		const unsigned int offset = idy * img_width + idx;
	
		if (idx < img_width && idy < img_height)
		{ 
			*sum_pixels +=  brightness[offset];  
		}
}
	
bool parallel_cuda_copy_result_to_hdr(int serial_flag,int brigntness_serial)
{
	if(!load_calib_data_flag_)
	{
		return false;
	}
 

	CHECK(cudaMemcpyAsync(d_hdr_depth_map_list_[serial_flag], d_depth_map_, 1 * image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToDevice)); 
	CHECK(cudaMemcpyAsync(d_hdr_brightness_list_[serial_flag], d_patterns_list[brigntness_serial], 1 * image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToDevice));

	float val  = 0;
	CHECK(cudaMemcpyAsync(d_hdr_bright_pixel_sum_list_[serial_flag], &val, sizeof(float), cudaMemcpyHostToDevice)); 
 	parallel_cuda_count_sum_pixel << <blocksPerGrid, threadsPerBlock >> > (d_hdr_brightness_list_[serial_flag],image_height_,image_width_,d_hdr_bright_pixel_sum_list_[serial_flag]);
 
	LOG(INFO)<<"parallel_cuda_copy_result_to_hdr: "<<serial_flag;
	return true;
}



/***********************************************************************************************************************************************/

 bool cuda_merge_hdr_data(std::vector<float*> depth_map_list,std::vector<unsigned char*> brightness_list,float* depth_map, unsigned char* brightness)
{

	if(depth_map_list.size() != brightness_list.size())
	{
		return false;
	}

	LOG(INFO)<<"HDR";

	for(int i= 0;i< 3;i++)
	{ 
		CHECK(cudaMemcpy(d_hdr_depth_map_list_[i], depth_map_list[i], image_height_*image_width_ * sizeof(float), cudaMemcpyHostToDevice));
		CHECK(cudaMemcpy(d_hdr_brightness_list_[i], brightness_list[i], image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyHostToDevice));
	}



	cuda_merge_hdr << <blocksPerGrid, threadsPerBlock >> > (d_hdr_depth_map_list_[0],d_hdr_depth_map_list_[1],d_hdr_depth_map_list_[2], d_hdr_brightness_list_[0], 
		d_hdr_brightness_list_[1], d_hdr_brightness_list_[2], image_height_, image_width_, d_hdr_depth_map_,d_hdr_brightness_);

	CHECK(cudaMemcpy(depth_map, d_hdr_depth_map_, 1 * image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
	CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));


	LOG(INFO)<<"Finished!";


	return true;

}


bool cuda_get_frame_03_hdr(std::vector<unsigned char*> patterns,int group_flag,float* depth,unsigned char* brightness)
{
	if(31 != patterns.size())
	{
		return false;
	}

	if(!load_calib_data_flag_)
	{
		return false;
	}

	LOG(INFO)<<"hdr a: "<<group_flag;
	cuda_copy_patterns_hdr(patterns,group_flag); 

 
	LOG(INFO)<<"hdr b";
	cuda_compute_phase_03_hdr(group_flag); 
	 
	// LOG(INFO)<<"c";
	// cuda_unwrap_phase_03(); 
 

	// LOG(INFO)<<"d";
	// cuda_reconstruct();

	// LOG(INFO)<<"e";

	// cuda_get_brightness_data(brightness);

	// cuda_get_depth_data(depth);
 
 
	LOG(INFO)<<"f";
	return true;


}
	


bool cuda_get_frame_03(std::vector<unsigned char*> patterns,float* depth,unsigned char* brightness)
{
	if(31 != patterns.size())
	{
		return false;
	}

	if(!load_calib_data_flag_)
	{
		return false;
	}

	LOG(INFO)<<"a";
	cuda_copy_patterns(patterns); 

	LOG(INFO)<<"b";
	cuda_compute_phase_03();
//	CHECK(cudaMemcpy(phase_map, d_wrap_map_list[0], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
	 
	LOG(INFO)<<"c";
	cuda_unwrap_phase_03(); 



	



	LOG(INFO)<<"d";
	cuda_reconstruct();

	LOG(INFO)<<"e";

	cuda_get_brightness_data(brightness);

	cuda_get_depth_data(depth);

	//cuda_get_frame_data(depth,brightness);


	LOG(INFO)<<"f";
	return true;


}
	


bool cuda_get_frame_base_24(std::vector<unsigned char*> patterns,float* depth,unsigned char* brightness)
{
	if(24 != patterns.size())
	{
		return false;
	}

	if(!load_calib_data_flag_)
	{
		return false;
	}

	LOG(INFO)<<"a";
	cuda_copy_patterns(patterns); 

	LOG(INFO)<<"b";
	cuda_compute_phase();
//	CHECK(cudaMemcpy(phase_map, d_wrap_map_list[0], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
	 
	LOG(INFO)<<"c";
	cuda_unwrap_phase(); 



	



	LOG(INFO)<<"d";
	cuda_reconstruct();

	LOG(INFO)<<"e";
	
	cuda_get_frame_data(depth,brightness);


	LOG(INFO)<<"f";
	return true;


}
		

bool cuda_reconstruct_base_24(std::vector<unsigned char*> patterns, float* point_cloud)
{
	if(24 != patterns.size())
	{
		return false;
	}

	if(!load_calib_data_flag_)
	{
		return false;
	}

	LOG(INFO)<<"a";
	cuda_copy_patterns(patterns); 

	LOG(INFO)<<"b";
	cuda_compute_phase();
//	CHECK(cudaMemcpy(phase_map, d_wrap_map_list[0], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
	 
	LOG(INFO)<<"c";
	cuda_unwrap_phase(); 
	 
	LOG(INFO)<<"d";
	cuda_reconstruct_pointcloud(point_cloud);

	LOG(INFO)<<"e";
	return true;
}


bool cuda_malloc_memory()
{ 
	
	for(int i= 0;i< patterns_count_;i++)
	{
		cudaMalloc((void**)&d_patterns_list[i], image_height_*image_width_ * sizeof(unsigned char));
		
		// cudaBindTexture(0,texture_patterns_list[i],d_patterns_list[i]);

		cudaMalloc((void**)&d_patterns_list_hdr_0[i], image_height_*image_width_ * sizeof(unsigned char));
		cudaMalloc((void**)&d_patterns_list_hdr_1[i], image_height_*image_width_ * sizeof(unsigned char));
		cudaMalloc((void**)&d_patterns_list_hdr_2[i], image_height_*image_width_ * sizeof(unsigned char));
	}

	cudaBindTexture(0,texture_patterns_0,d_patterns_list[0]);
	cudaBindTexture(0,texture_patterns_1,d_patterns_list[1]);
	cudaBindTexture(0,texture_patterns_2,d_patterns_list[2]);
	cudaBindTexture(0,texture_patterns_3,d_patterns_list[3]);
	cudaBindTexture(0,texture_patterns_4,d_patterns_list[4]);
	cudaBindTexture(0,texture_patterns_5,d_patterns_list[5]);
	cudaBindTexture(0,texture_patterns_6,d_patterns_list[6]);
	cudaBindTexture(0,texture_patterns_7,d_patterns_list[7]);
	cudaBindTexture(0,texture_patterns_8,d_patterns_list[8]);
	cudaBindTexture(0,texture_patterns_9,d_patterns_list[9]);
	cudaBindTexture(0,texture_patterns_10,d_patterns_list[10]);
	cudaBindTexture(0,texture_patterns_11,d_patterns_list[11]);
	cudaBindTexture(0,texture_patterns_12,d_patterns_list[12]);
	cudaBindTexture(0,texture_patterns_13,d_patterns_list[13]);
	cudaBindTexture(0,texture_patterns_14,d_patterns_list[14]);
	cudaBindTexture(0,texture_patterns_15,d_patterns_list[15]);
	cudaBindTexture(0,texture_patterns_16,d_patterns_list[16]);
	cudaBindTexture(0,texture_patterns_17,d_patterns_list[17]);
	cudaBindTexture(0,texture_patterns_18,d_patterns_list[18]);


 

	for (int i = 0; i< wrap_count_; i++)
	{
		cudaMalloc((void**)&d_wrap_map_list[i], image_height_*image_width_ * sizeof(float));
		cudaMalloc((void**)&d_confidence_list[i], image_height_*image_width_ * sizeof(float));

		cudaMalloc((void**)&d_wrap_map_list_hdr_0[i], image_height_*image_width_ * sizeof(float));
		cudaMalloc((void**)&d_confidence_list_hdr_0[i], image_height_*image_width_ * sizeof(float));
		cudaMalloc((void**)&d_wrap_map_list_hdr_1[i], image_height_*image_width_ * sizeof(float));
		cudaMalloc((void**)&d_confidence_list_hdr_1[i], image_height_*image_width_ * sizeof(float));
		cudaMalloc((void**)&d_wrap_map_list_hdr_2[i], image_height_*image_width_ * sizeof(float));
		cudaMalloc((void**)&d_confidence_list_hdr_2[i], image_height_*image_width_ * sizeof(float));
	}

	for (int i = 0; i< unwrap_count_; i++)
	{
		cudaMalloc((void**)&d_unwrap_map_list[i], image_height_*image_width_ * sizeof(float));

		
		cudaMalloc((void**)&d_unwrap_map_list_hdr_0[i], image_height_*image_width_ * sizeof(float)); 
		cudaMalloc((void**)&d_unwrap_map_list_hdr_1[i], image_height_*image_width_ * sizeof(float)); 
		cudaMalloc((void**)&d_unwrap_map_list_hdr_2[i], image_height_*image_width_ * sizeof(float));
	}

	for (int i = 0; i< D_HDR_MAX_NUM; i++)
	{
		cudaMalloc((void**)&d_hdr_depth_map_list_[i], image_height_*image_width_ * sizeof(float));
		cudaMalloc((void**)&d_hdr_brightness_list_[i], image_height_*image_width_ * sizeof(unsigned char)); 
		cudaMalloc((void**)&d_hdr_bright_pixel_sum_list_[i], 1 * sizeof(float)); 
	}
	cudaMalloc((void**)&d_hdr_depth_map_, image_height_*image_width_ * sizeof(float));
	cudaMalloc((void**)&d_hdr_brightness_, image_height_*image_width_ * sizeof(unsigned char));


	cudaMalloc((void**)&d_brightness_, image_height_*image_width_ * sizeof(unsigned char));
	cudaMalloc((void**)&d_mask_, image_height_*image_width_ * sizeof(unsigned char));


	cudaMalloc((void**)&d_camera_intrinsic_, 3*3 * sizeof(float));
	cudaMalloc((void**)&d_project_intrinsic_, 3 * 3 * sizeof(float));

	cudaMalloc((void**)&d_camera_distortion_, 1* 5 * sizeof(float));
	cudaMalloc((void**)&d_projector_distortion_, 1 * 5 * sizeof(float));

	cudaMalloc((void**)&d_rotation_matrix_, 3 * 3 * sizeof(float));
	cudaMalloc((void**)&d_translation_matrix_, 1 * 3 * sizeof(float));


	cudaMalloc((void**)&d_point_cloud_map_, 3*image_height_*image_width_ * sizeof(float));
	cudaMalloc((void**)&d_depth_map_, image_height_*image_width_ * sizeof(float));
	cudaMalloc((void**)&d_triangulation_error_map_, image_height_*image_width_ * sizeof(float));
 
	//分配重复patterns数据
	for(int i= 0;i< D_REPETITIONB_MAX_NUM*6;i++)
	{
		cudaMalloc((void**)&d_repetition_patterns_list_[i], image_height_*image_width_ * sizeof(unsigned char)); 
	}

	for(int i= 0;i< 6;i++)
	{
		cudaMalloc((void**)&d_repetition_merge_patterns_list_[i], image_height_*image_width_ * sizeof(unsigned short)); 
	}
 
 	for(int i= 0;i< D_REPETITION_02_MAX_NUM;i++)
	{
		cudaMalloc((void**)&d_repetition_02_merge_patterns_list_[i], image_height_*image_width_ * sizeof(unsigned short)); 
	}
 
	
	
	reconstruct_cuda_malloc_memory();
	reconstruct_cuda_minimalloc_memory();

	cudaDeviceSynchronize();

	return true;
}


bool cuda_free_memory()
{

	for (int i = 0; i< patterns_count_; i++)
	{ 
		// cudaUnbindTexture(texture_patterns_list[i]);
		cudaFree(d_patterns_list[i]);
		cudaFree(d_patterns_list_hdr_0[i]);
		cudaFree(d_patterns_list_hdr_1[i]);
		cudaFree(d_patterns_list_hdr_2[i]);
	}

	cudaUnbindTexture(texture_patterns_0);
	cudaUnbindTexture(texture_patterns_1);
	cudaUnbindTexture(texture_patterns_2);
	cudaUnbindTexture(texture_patterns_3);
	cudaUnbindTexture(texture_patterns_4);
	cudaUnbindTexture(texture_patterns_5);
	cudaUnbindTexture(texture_patterns_6);
	cudaUnbindTexture(texture_patterns_7);
	cudaUnbindTexture(texture_patterns_8);
	cudaUnbindTexture(texture_patterns_9);
	cudaUnbindTexture(texture_patterns_10);
	cudaUnbindTexture(texture_patterns_11);
	cudaUnbindTexture(texture_patterns_12);
	cudaUnbindTexture(texture_patterns_13);
	cudaUnbindTexture(texture_patterns_14);
	cudaUnbindTexture(texture_patterns_15);
	cudaUnbindTexture(texture_patterns_16);
	cudaUnbindTexture(texture_patterns_17);
	cudaUnbindTexture(texture_patterns_18);

	for (int i = 0; i< wrap_count_; i++)
	{ 
		
		cudaFree(d_wrap_map_list[i]);
		cudaFree(d_confidence_list[i]);

		cudaFree(d_wrap_map_list_hdr_0[i]);
		cudaFree(d_confidence_list_hdr_0[i]);

		cudaFree(d_wrap_map_list_hdr_1[i]);
		cudaFree(d_confidence_list_hdr_1[i]);

		cudaFree(d_wrap_map_list_hdr_2[i]);
		cudaFree(d_confidence_list_hdr_2[i]);

	}

	for (int i = 0; i< unwrap_count_; i++)
	{ 
		cudaFree(d_unwrap_map_list[i]);
		
		cudaFree(d_unwrap_map_list_hdr_0[i]); 
		cudaFree(d_unwrap_map_list_hdr_1[i]);
		cudaFree(d_unwrap_map_list_hdr_2[i]);
	}

	for (int i = 0; i< D_HDR_MAX_NUM; i++)
	{ 
		cudaFree(d_hdr_depth_map_list_[i]);
		cudaFree(d_hdr_brightness_list_[i]);
		cudaFree(d_hdr_bright_pixel_sum_list_[i]);
	}
		cudaFree(d_hdr_depth_map_);
		cudaFree(d_hdr_brightness_);
	


	cudaFree(d_brightness_);
	cudaFree(d_mask_);

	cudaFree(d_camera_intrinsic_);
	cudaFree(d_project_intrinsic_);

	cudaFree(d_camera_distortion_);
	cudaFree(d_projector_distortion_);

	cudaFree(d_rotation_matrix_);
	cudaFree(d_translation_matrix_);


	cudaFree(d_point_cloud_map_);
	cudaFree(d_depth_map_);
	cudaFree(d_triangulation_error_map_);

		//分配重复patterns数据
	for(int i= 0;i< D_REPETITIONB_MAX_NUM*6;i++)
	{
		cudaFree(d_repetition_patterns_list_[i]); 
	}

	for(int i= 0;i< 6;i++)
	{
		cudaFree(d_repetition_merge_patterns_list_[i]);  
	}

	for(int i= 0;i< D_REPETITION_02_MAX_NUM;i++)
	{
		cudaFree(d_repetition_02_merge_patterns_list_[i]);  
	}

	
 
	reconstruct_cuda_free_memory();
	reconstruct_cuda_free_minimemory();

	return true;
}

bool cuda_reconstruct()
{

	cuda_rebuild << <blocksPerGrid, threadsPerBlock >> >(d_unwrap_map_list[0], d_unwrap_map_list[1],d_camera_intrinsic_,d_camera_distortion_,
		d_project_intrinsic_,d_projector_distortion_,d_rotation_matrix_,d_translation_matrix_, 
		d_point_cloud_map_,d_depth_map_, d_triangulation_error_map_, d_confidence_list[2]);


	cudaDeviceSynchronize();
    LOG(INFO)<<"rebuild data!";
	
 

    // LOG(INFO)<<"unwrap_0";

	// cv::Mat unwrap_0(image_height_, image_width_, CV_32F, cv::Scalar(0));
	// cudaMemcpy(unwrap_0.data, d_unwrap_map_list[0], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);

	// cv::Mat unwrap_1(image_height_, image_width_, CV_32F, cv::Scalar(0));
	// cudaMemcpy(unwrap_1.data, d_unwrap_map_list[1], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);

	// cv::Mat deep_map(image_height_, image_width_, CV_32F, cv::Scalar(0));
	// cudaMemcpy(deep_map.data, d_depth_map_, image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);

	// cv::Mat err_map(image_height_, image_width_, CV_32F, cv::Scalar(0));
	// cudaMemcpy(err_map.data, d_triangulation_error_map_, image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);

	// cv::Mat points_map(image_height_, image_width_, CV_32FC3, cv::Scalar(0));
	// cudaMemcpy(points_map.data, d_point_cloud_map_, 3*image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);

 
	
    // LOG(INFO)<<"copy data!";

	// cv::imwrite("unwrap_map_0.tiff",unwrap_0);
	// cv::imwrite("unwrap_map_1.tiff",unwrap_1);
	// cv::imwrite("deep_map.tiff",deep_map);
	// cv::imwrite("err_map.tiff",err_map);
	// cv::imwrite("points_map.tiff",points_map);


}


bool cuda_get_pointcloud_data(float* pointcloud)
{
	
	CHECK(cudaMemcpy(pointcloud, d_point_cloud_map_, 3 * image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
}

bool cuda_get_depth_data(float* depth)
{


	CHECK(cudaMemcpy(depth, d_depth_map_, 1 * image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost));

}

bool cuda_get_brightness_data(unsigned char* brightness)
{


	CHECK(cudaMemcpy(brightness, d_patterns_list[30], 1 * image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));

}



bool cuda_get_frame_data(float* depth,unsigned char* bright)
{


	cuda_merge_brightness << <blocksPerGrid, threadsPerBlock >> > (d_patterns_list[0], d_patterns_list[1], d_patterns_list[2],
			d_patterns_list[3], image_height_, image_width_, d_brightness_);


	cudaDeviceSynchronize();
	CHECK(cudaMemcpy(depth, d_depth_map_, 1 * image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
	CHECK(cudaMemcpy(bright, d_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));

}

bool cuda_reconstruct_pointcloud(float* point_cloud)
{
	cuda_rebuild << <blocksPerGrid, threadsPerBlock >> >(d_unwrap_map_list[0], d_unwrap_map_list[1],d_camera_intrinsic_,d_camera_distortion_,
		d_project_intrinsic_,d_projector_distortion_,d_rotation_matrix_,d_translation_matrix_, 
		d_point_cloud_map_,d_depth_map_, d_triangulation_error_map_, d_confidence_list[2]);


	cudaDeviceSynchronize();

	CHECK(cudaMemcpy(point_cloud, d_point_cloud_map_, 3 * image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost));

	//cv::Mat test_point_cloud(image_height_, image_width_, CV_32FC3, cv::Scalar(0));
	//cudaMemcpy(test_point_cloud.data, d_cuda_point_cloud_map_, 3*image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);

	//point_cloud = test_point_cloud.clone();

	//cv::Mat test_depth(image_height_, image_width_, CV_32F, cv::Scalar(0));
	//cudaMemcpy(test_depth.data, d_cuda_depth_map_, image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);

	//cv::Mat test_error(image_height_, image_width_, CV_32F, cv::Scalar(0));
	//cudaMemcpy(test_error.data, d_cuda_triangulation_error_map_, image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);

	return true;
}



bool cuda_unwrap_phase_03()
{
	
	 
	cuda_mul_phase_unwrap << <blocksPerGrid, threadsPerBlock >> >(d_wrap_map_list[0], d_wrap_map_list[1], d_wrap_map_list[2],image_height_, image_width_, d_unwrap_map_list[0]);

	// cudaDeviceSynchronize();


	
	cuda_variable_phase_unwrap << <blocksPerGrid, threadsPerBlock >> >(d_unwrap_map_list[0], d_wrap_map_list[3], 4.0,
			image_height_, image_width_, 1.5,d_unwrap_map_list[0]);
	

	
	cuda_mul_phase_unwrap << <blocksPerGrid, threadsPerBlock >> >(d_wrap_map_list[4], d_wrap_map_list[5], d_wrap_map_list[6],image_height_, image_width_, d_unwrap_map_list[1]);


	cuda_normalize_phase << <blocksPerGrid, threadsPerBlock >> >(d_unwrap_map_list[0],128.0, d_unwrap_map_list[1],18.0,
			image_height_, image_width_, d_unwrap_map_list[0],d_unwrap_map_list[1]);
	


//	cv::Mat test_unwrap_x(image_height_, image_width_, CV_32F, cv::Scalar(0));
//	cv::Mat test_unwrap_y(image_height_, image_width_, CV_32F, cv::Scalar(0));
//	cudaMemcpy(test_unwrap_x.data, d_unwrap_map_list[0], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);
//	cudaMemcpy(test_unwrap_y.data, d_unwrap_map_list[1], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);
//	std::string path_x = "../../../debug_data/unwrap_img_x.tiff";
//	cv::imwrite(path_x,test_unwrap_x);
//	std::string path_y = "../../../debug_data/unwrap_img_y.tiff";
//	cv::imwrite(path_y,test_unwrap_y);
	

	//cv::Mat test_memory(image_height_, image_width_, CV_32F, cv::Scalar(0));
	//cudaMemcpy(test_memory.data, d_cuda_unwrap_map_list[1], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);

	return true;
}


bool cuda_unwrap_phase()
{
	
	for(int i= 0;i< unwrap_count_;i++)
	{ 
		cuda_mul_phase_unwrap << <blocksPerGrid, threadsPerBlock >> >(d_wrap_map_list[3*i+0], d_wrap_map_list[3 * i + 1], d_wrap_map_list[3 * i + 2],
			image_height_, image_width_, d_unwrap_map_list[i]);
	}
	
	cuda_normalize_phase << <blocksPerGrid, threadsPerBlock >> >(d_unwrap_map_list[0],32.0, d_unwrap_map_list[1],32.0*720/1280,
			image_height_, image_width_, d_unwrap_map_list[0],d_unwrap_map_list[1]);
	

	


//	cv::Mat test_unwrap_x(image_height_, image_width_, CV_32F, cv::Scalar(0));
//	cv::Mat test_unwrap_y(image_height_, image_width_, CV_32F, cv::Scalar(0));
//	cudaMemcpy(test_unwrap_x.data, d_unwrap_map_list[0], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);
//	cudaMemcpy(test_unwrap_y.data, d_unwrap_map_list[1], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);
//	std::string path_x = "../../../debug_data/unwrap_img_x.tiff";
//	cv::imwrite(path_x,test_unwrap_x);
//	std::string path_y = "../../../debug_data/unwrap_img_y.tiff";
//	cv::imwrite(path_y,test_unwrap_y);
	

	//cv::Mat test_memory(image_height_, image_width_, CV_32F, cv::Scalar(0));
	//cudaMemcpy(test_memory.data, d_cuda_unwrap_map_list[1], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);

	return true;
}



bool cuda_compute_phase_03_hdr(int group_flag)
{
	 
	
	switch(group_flag)
	{

		case 0:
		{
			for(int i= 0;i< 3;i++)
			{ 
				cuda_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_patterns_list_hdr_0[4*i+0], d_patterns_list_hdr_0[4 * i + 1], d_patterns_list_hdr_0[4 * i + 2],
					d_patterns_list_hdr_0[4 * i + 3], image_height_, image_width_, d_wrap_map_list_hdr_0[i], d_confidence_list_hdr_0[i]);
			}
		 
			for(int i= 3;i< 4;i++)
			{
		
				cuda_six_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_patterns_list_hdr_0[4*i+0], d_patterns_list_hdr_0[4 * i + 1], d_patterns_list_hdr_0[4 * i + 2],
					d_patterns_list_hdr_0[4 * i + 3],d_patterns_list_hdr_0[4 * i + 4],d_patterns_list_hdr_0[4 * i + 5] , image_height_, image_width_, d_wrap_map_list_hdr_0[i], d_confidence_list_hdr_0[i]);
			}
		
			int offset_i = 2;
		
			for(int i= 4;i< 7;i++)
			{ 
				cuda_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_patterns_list_hdr_0[4*i + offset_i +0], d_patterns_list_hdr_0[4 * i + offset_i + 1], d_patterns_list_hdr_0[4 * i + offset_i + 2],
					d_patterns_list_hdr_0[4 * i + offset_i + 3], image_height_, image_width_, d_wrap_map_list_hdr_0[i], d_confidence_list_hdr_0[i]);
			}
			
		}
		break;

		case 1:
		{

			for(int i= 0;i< 3;i++)
			{ 
				cuda_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_patterns_list_hdr_1[4*i+0], d_patterns_list_hdr_1[4 * i + 1], d_patterns_list_hdr_1[4 * i + 2],
					d_patterns_list_hdr_1[4 * i + 3], image_height_, image_width_, d_wrap_map_list_hdr_1[i], d_confidence_list_hdr_1[i]);
			}
		 
			for(int i= 3;i< 4;i++)
			{
		
				cuda_six_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_patterns_list_hdr_1[4*i+0], d_patterns_list_hdr_1[4 * i + 1], d_patterns_list_hdr_1[4 * i + 2],
					d_patterns_list_hdr_1[4 * i + 3],d_patterns_list_hdr_1[4 * i + 4],d_patterns_list_hdr_1[4 * i + 5] , image_height_, image_width_, d_wrap_map_list_hdr_1[i], d_confidence_list_hdr_1[i]);
			}
		
			int offset_i = 2;
		
			for(int i= 4;i< 7;i++)
			{ 
				cuda_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_patterns_list_hdr_1[4*i + offset_i +0], d_patterns_list_hdr_1[4 * i + offset_i + 1], d_patterns_list_hdr_1[4 * i + offset_i + 2],
					d_patterns_list_hdr_1[4 * i + offset_i + 3], image_height_, image_width_, d_wrap_map_list_hdr_1[i], d_confidence_list_hdr_1[i]);
			}

		}
		break;

		case 2:
		{ 		
			for(int i= 0;i< 3;i++)
			{ 
				cuda_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_patterns_list_hdr_2[4*i+0], d_patterns_list_hdr_2[4 * i + 1], d_patterns_list_hdr_2[4 * i + 2],
					d_patterns_list_hdr_2[4 * i + 3], image_height_, image_width_, d_wrap_map_list_hdr_2[i], d_confidence_list_hdr_2[i]);
			}
		 
			for(int i= 3;i< 4;i++)
			{
		
				cuda_six_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_patterns_list_hdr_2[4*i+0], d_patterns_list_hdr_2[4 * i + 1], d_patterns_list_hdr_2[4 * i + 2],
					d_patterns_list_hdr_2[4 * i + 3],d_patterns_list_hdr_2[4 * i + 4],d_patterns_list_hdr_2[4 * i + 5] , image_height_, image_width_, d_wrap_map_list_hdr_2[i], d_confidence_list_hdr_2[i]);
			}
		
			int offset_i = 2;
		
			for(int i= 4;i< 7;i++)
			{ 
				cuda_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_patterns_list_hdr_2[4*i + offset_i +0], d_patterns_list_hdr_2[4 * i + offset_i + 1], d_patterns_list_hdr_2[4 * i + offset_i + 2],
					d_patterns_list_hdr_2[4 * i + offset_i + 3], image_height_, image_width_, d_wrap_map_list_hdr_2[i], d_confidence_list_hdr_2[i]);
			}

		}
		break;

		default:
		break;


	}










	// cudaDeviceSynchronize();

	//cv::Mat test_patterns(image_height_, image_width_, CV_8U, cv::Scalar(0));
	//cudaMemcpy(test_patterns.data,d_patterns_list[0], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);
//	std::string path = "../../debug_data/unwrap_img_0.tiff";
//	cv::imwrite(path,test_patterns);
		


//	cv::Mat test_memory(image_height_, image_width_, CV_32F, cv::Scalar(0));
//	cudaMemcpy(test_memory.data, d_wrap_map_list[0], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);
//	std::string path = "../../debug_data/wrap_img_0.tiff";
//	cv::imwrite(path,test_memory);
	
	return true;
}



bool cuda_compute_phase_03()
{
	 
	for(int i= 0;i< 3;i++)
	{

		cuda_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_patterns_list[4*i+0], d_patterns_list[4 * i + 1], d_patterns_list[4 * i + 2],
			d_patterns_list[4 * i + 3], image_height_, image_width_, d_wrap_map_list[i], d_confidence_list[i]);
	}


	for(int i= 3;i< 4;i++)
	{

		cuda_six_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_patterns_list[4*i+0], d_patterns_list[4 * i + 1], d_patterns_list[4 * i + 2],
			d_patterns_list[4 * i + 3],d_patterns_list[4 * i + 4],d_patterns_list[4 * i + 5] , image_height_, image_width_, d_wrap_map_list[i], d_confidence_list[i]);
	}

	int offset_i = 2;

	for(int i= 4;i< 7;i++)
	{

		cuda_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_patterns_list[4*i + offset_i +0], d_patterns_list[4 * i + offset_i + 1], d_patterns_list[4 * i + offset_i + 2],
			d_patterns_list[4 * i + offset_i + 3], image_height_, image_width_, d_wrap_map_list[i], d_confidence_list[i]);
	}




	// cudaDeviceSynchronize();

	//cv::Mat test_patterns(image_height_, image_width_, CV_8U, cv::Scalar(0));
	//cudaMemcpy(test_patterns.data,d_patterns_list[0], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);
//	std::string path = "../../debug_data/unwrap_img_0.tiff";
//	cv::imwrite(path,test_patterns);
		


//	cv::Mat test_memory(image_height_, image_width_, CV_32F, cv::Scalar(0));
//	cudaMemcpy(test_memory.data, d_wrap_map_list[0], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);
//	std::string path = "../../debug_data/wrap_img_0.tiff";
//	cv::imwrite(path,test_memory);
	
	return true;
}



bool cuda_compute_phase()
{
	 
	for(int i= 0;i< wrap_count_;i++)
	{

		cuda_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_patterns_list[4*i+0], d_patterns_list[4 * i + 1], d_patterns_list[4 * i + 2],
			d_patterns_list[4 * i + 3], image_height_, image_width_, d_wrap_map_list[i], d_confidence_list[i]);
	}


	cudaDeviceSynchronize();

	//cv::Mat test_patterns(image_height_, image_width_, CV_8U, cv::Scalar(0));
	//cudaMemcpy(test_patterns.data,d_patterns_list[0], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);
//	std::string path = "../../debug_data/unwrap_img_0.tiff";
//	cv::imwrite(path,test_patterns);
		


//	cv::Mat test_memory(image_height_, image_width_, CV_32F, cv::Scalar(0));
//	cudaMemcpy(test_memory.data, d_wrap_map_list[0], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost);
//	std::string path = "../../debug_data/wrap_img_0.tiff";
//	cv::imwrite(path,test_memory);
	
	return true;
}

bool cuda_copy_calib_data(float* camera_intrinsic, float* project_intrinsic, float* camera_distortion,
	float* projector_distortion, float* rotation_matrix, float* translation_matrix)
{
	//if(!camera_intrinsic.data || !project_intrinsic.data || !camera_distortion.data
	//	|| !projector_distortion.data || !rotation_matrix.data || !translation_matrix.data)
	//{
	//	return false;
	//}

	CHECK(cudaMemcpy(d_camera_intrinsic_, camera_intrinsic, 3 * 3 * sizeof(float), cudaMemcpyHostToDevice));
	CHECK(cudaMemcpy(d_project_intrinsic_, project_intrinsic, 3 * 3 * sizeof(float), cudaMemcpyHostToDevice));

	CHECK(cudaMemcpy(d_camera_distortion_, camera_distortion, 1 * 5 * sizeof(float), cudaMemcpyHostToDevice));
	CHECK(cudaMemcpy(d_projector_distortion_, projector_distortion, 1 * 5 * sizeof(float), cudaMemcpyHostToDevice));

	CHECK(cudaMemcpy(d_rotation_matrix_, rotation_matrix, 3 * 3 * sizeof(float), cudaMemcpyHostToDevice));
	CHECK(cudaMemcpy(d_translation_matrix_, translation_matrix, 1* 3 * sizeof(float), cudaMemcpyHostToDevice));

	load_calib_data_flag_ = true;

	//cv::Mat test_memory(3, 3, CV_32F, cv::Scalar(0));
	//cudaMemcpy(test_memory.data, camera_intrinsic_, 3*3 * sizeof(float), cudaMemcpyDeviceToHost);

	return true;
}

bool cuda_copy_patterns_hdr(std::vector<unsigned char*> patterns,int flag)
{
	
	if(patterns.empty())
	{
		return false;
	}


	switch(flag)
	{

		case 0:
		{
			for(int i= 0;i< patterns.size();i++)
			{ 
				CHECK(cudaMemcpy(d_patterns_list_hdr_0[i], patterns[i], image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyHostToDevice));
			}
		}
		break;

		case 1:
		{
			for(int i= 0;i< patterns.size();i++)
			{ 
				CHECK(cudaMemcpy(d_patterns_list_hdr_1[i], patterns[i], image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyHostToDevice));
			}
		}
		break;

		case 2:
		{ 			for(int i= 0;i< patterns.size();i++)
			{ 
				CHECK(cudaMemcpy(d_patterns_list_hdr_2[i], patterns[i], image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyHostToDevice));
			}
		}
		break;

		default:
		break;


	}



	   

	return true;
}

bool cuda_copy_patterns(std::vector<unsigned char*> patterns)
{
	
	if(patterns.empty())
	{
		return false;
	}

	for(int i= 0;i< patterns.size();i++)
	{ 
		CHECK(cudaMemcpy(d_patterns_list[i], patterns[i], image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyHostToDevice));
	}
	  
	// cudaDeviceSynchronize();

	return true;
}

__global__ void cuda_merge_hdr(const float*  depth_map_0,const float*  depth_map_1,const float*  depth_map_2,const unsigned char* brightness_0,const unsigned char* brightness_1,
	const unsigned char* brightness_2,uint32_t img_height, uint32_t img_width, float* const depth_map,unsigned char * const brightness)
{

	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * img_width + idx;

	if (idx < img_width && idy < img_height)
	{



		float pixel= 0;
		pixel +=  brightness_0[offset];
		pixel +=  brightness_1[offset];
		pixel +=  brightness_2[offset];

		pixel/= 3.0;


		brightness[offset] = pixel;

		if(brightness_0[offset] < 255)
		{
			// brightness[offset] = brightness_0[offset];
			depth_map[offset] = depth_map_0[offset];
		}

		else if(brightness_1[offset] < 255)
		{
			// brightness[offset] = brightness_1[offset];
			depth_map[offset] = depth_map_1[offset];
		}
		else
		{	
			// brightness[offset] = brightness_2[offset];
			depth_map[offset] = depth_map_2[offset];
		}


	}



	
}




__global__ void cuda_merge_brightness(unsigned char* const d_in_0,unsigned char* const d_in_1,unsigned char* d_in_2,unsigned char* d_in_3,
	uint32_t img_height, uint32_t img_width,unsigned char * const d_out)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * img_width + idx;

	if (idx < img_width && idy < img_height)
	{

		float a = d_in_3[offset] - d_in_1[offset];
		float b = d_in_0[offset] - d_in_2[offset];

		float ave = (d_in_0[offset] + d_in_1[offset] + d_in_2[offset] +d_in_3[offset])/4.0;
  
		float val = ave + std::sqrt(a*a + b*b);
 

		if(val> 255)
		{
		   val = 255.0;
		}
		unsigned char c = val;	
		d_out[offset] = c;
	}

	

}


__global__ void cuda_six_step_phase_shift_texture(float * const d_out, float * const confidence)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy *  d_image_width_ + idx;
	float s_0 =  0;
	float s_1 =  0.866025;
	float s_2 =  0.866025;
	float s_3 =  0;
	float s_4 =  -0.866025;
	float s_5 =  -0.866025;
	float c_0 =  1;
	float c_1 =  0.5;
	float c_2 =  -0.5;
	float c_3 =  -1;
	float c_4 =  -0.5;
	float c_5 =  0.5;

 

	unsigned char pixel_0 = tex1Dfetch(texture_patterns_12, offset);
	unsigned char pixel_1 = tex1Dfetch(texture_patterns_13, offset);
	unsigned char pixel_2 = tex1Dfetch(texture_patterns_14, offset);
	unsigned char pixel_3 = tex1Dfetch(texture_patterns_15, offset);
	unsigned char pixel_4 = tex1Dfetch(texture_patterns_16, offset);
	unsigned char pixel_5 = tex1Dfetch(texture_patterns_17, offset);

	if (idx < d_image_width_ && idy < d_image_height_)
	{
  
		float a = c_0 *pixel_3 + c_1 *pixel_4 + c_2 *pixel_5 + c_3* pixel_0 +c_4*pixel_1 + c_5*pixel_2;
		float b = s_0 *pixel_3 + s_1 *pixel_4 + s_2 *pixel_5 + s_3* pixel_0 +s_4*pixel_1 + s_5*pixel_2;


		int over_num = 0;
		if(pixel_0 >= 255)
		{
			over_num++;
		}
		if (pixel_1 >= 255)
		{
			over_num++;
		}
		if (pixel_2 >= 255)
		{
			over_num++;
		}
		if (pixel_3 >= 255)
		{
			over_num++;
		}
		if (pixel_4 >= 255)
		{
			over_num++;
		}
		if (pixel_5 >= 255)
		{
			over_num++;
		}

		if(over_num> 3)
		{
			confidence[offset] = 0;
			d_out[offset] = -1;
		}
		else
		{
			confidence[offset] = std::sqrt(a*a + b*b);
			d_out[offset] = DF_PI + std::atan2(a, b);
		}
  
		// confidence[offset] = std::sqrt(a*a + b*b);
		// d_out[offset] = DF_PI + std::atan2(a, b);
	}
}

__global__ void cuda_six_step_phase_shift(unsigned char * const d_in_0, unsigned char * const d_in_1, unsigned char * const d_in_2, unsigned char * const d_in_3,unsigned char* const d_in_4,unsigned char* const d_in_5,
	uint32_t img_height, uint32_t img_width,float * const d_out, float * const confidence)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * img_width + idx;
	float s_0 =  0;
	float s_1 =  0.866025;
	float s_2 =  0.866025;
	float s_3 =  0;
	float s_4 =  -0.866025;
	float s_5 =  -0.866025;
	float c_0 =  1;
	float c_1 =  0.5;
	float c_2 =  -0.5;
	float c_3 =  -1;
	float c_4 =  -0.5;
	float c_5 =  0.5;
	
	if (idx < img_width && idy < img_height)
	{

		float a = c_0 *d_in_3[offset] + c_1 *d_in_4[offset] + c_2 *d_in_5[offset] + c_3* d_in_0[offset] +c_4*d_in_1[offset] + c_5*d_in_2[offset];
		float b = s_0 *d_in_3[offset] + s_1 *d_in_4[offset] + s_2 *d_in_5[offset] + s_3* d_in_0[offset] +s_4*d_in_1[offset] + s_5*d_in_2[offset];


		int over_num = 0;
		if(d_in_0[offset]>= 255)
		{
			over_num++;
		}
		if (d_in_1[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_2[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_3[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_4[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_5[offset] >= 255)
		{
			over_num++;
		}

		if(over_num> 3)
		{
			confidence[offset] = 0;
			d_out[offset] = -1;
		}
		else
		{
			confidence[offset] = std::sqrt(a*a + b*b);
			d_out[offset] = DF_PI + std::atan2(a, b);
		}
  
		// confidence[offset] = std::sqrt(a*a + b*b);
		// d_out[offset] = DF_PI + std::atan2(a, b);
	}
}



__global__ void cuda_four_step_phase_shift_texture(int serial_flag,float * const d_out, float * const confidence)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * d_image_width_ + idx;
	unsigned char pixel_0 = 0;
	unsigned char pixel_1 = 0;
	unsigned char pixel_2 = 0;
	unsigned char pixel_3 = 0;

	if (idx < d_image_width_ && idy < d_image_height_)
	{


		switch (serial_flag)
		{
		case 0:
		{
			pixel_0 = tex1Dfetch(texture_patterns_0, offset);
			pixel_1 = tex1Dfetch(texture_patterns_1, offset);
			pixel_2 = tex1Dfetch(texture_patterns_2, offset);
			pixel_3 = tex1Dfetch(texture_patterns_3, offset);
		}
		break;
		case 1:
		{
			pixel_0 = tex1Dfetch(texture_patterns_4, offset);
			pixel_1 = tex1Dfetch(texture_patterns_5, offset);
			pixel_2 = tex1Dfetch(texture_patterns_6, offset);
			pixel_3 = tex1Dfetch(texture_patterns_7, offset);
		}
		break;
		case 2:
		{
			pixel_0 = tex1Dfetch(texture_patterns_8, offset);
			pixel_1 = tex1Dfetch(texture_patterns_9, offset);
			pixel_2 = tex1Dfetch(texture_patterns_10, offset);
			pixel_3 = tex1Dfetch(texture_patterns_11, offset);
		}
		break;

		default:
			break;
		}



		float a = pixel_3 - pixel_1;
		float b = pixel_0 - pixel_2;

		int over_num = 0;
		if(pixel_0 >= 255)
		{
			over_num++;
		}
		if (pixel_1 >= 255)
		{
			over_num++;
		}
		if (pixel_2 >= 255)
		{
			over_num++;
		}
		if (pixel_3 >= 255)
		{
			over_num++;
		}

		if(over_num> 1)
		{
			confidence[offset] = 0;
			d_out[offset] = -1;
		}
		else
		{
			confidence[offset] = std::sqrt(a*a + b*b);
			d_out[offset] = DF_PI + std::atan2(a, b);
		}





	}
}


__global__ void cuda_merge_brigntness_map(unsigned short * const merge_brightness,int repetition_count, unsigned char* brightness)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * d_image_width_ + idx;
 

	if (idx < d_image_width_ && idy < d_image_height_)
	{ 
		brightness[offset] = 0.5 + (merge_brightness[offset]/repetition_count); 
  	 
	}
}

__global__ void cuda_merge_four_step_phase_shift(unsigned short * const d_in_0, unsigned short * const d_in_1, unsigned short * const d_in_2, 
	unsigned short * const d_in_3,int repetition_count,float * const d_out, float * const confidence)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * d_image_width_ + idx;

	int max_pixel = 255*repetition_count;

	if (idx < d_image_width_ && idy < d_image_height_)
	{

		float a = d_in_3[offset] - d_in_1[offset];
		float b = d_in_0[offset] - d_in_2[offset];

		int over_num = 0;
		if(d_in_0[offset]>= max_pixel)
		{
			over_num++;
		}
		if (d_in_1[offset] >= max_pixel)
		{
			over_num++;
		}
		if (d_in_2[offset] >= max_pixel)
		{
			over_num++;
		}
		if (d_in_3[offset] >= max_pixel)
		{
			over_num++;
		}

		if(over_num> 1)
		{
			confidence[offset] = 0;
			d_out[offset] = -1;
		}
		else
		{
			confidence[offset] = std::sqrt(a*a + b*b);
			d_out[offset] = DF_PI + std::atan2(a, b);
		}
  
	}
}

__global__ void cuda_four_step_phase_shift(unsigned char * const d_in_0, unsigned char * const d_in_1, unsigned char * const d_in_2, unsigned char * const d_in_3,
	uint32_t img_height, uint32_t img_width,float * const d_out, float * const confidence)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * img_width + idx;

	if (idx < img_width && idy < img_height)
	{

		float a = d_in_3[offset] - d_in_1[offset];
		float b = d_in_0[offset] - d_in_2[offset];

		int over_num = 0;
		if(d_in_0[offset]>= 255)
		{
			over_num++;
		}
		if (d_in_1[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_2[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_3[offset] >= 255)
		{
			over_num++;
		}

		if(over_num> 1)
		{
			confidence[offset] = 0;
			d_out[offset] = -1;
		}
		else
		{
			confidence[offset] = std::sqrt(a*a + b*b);
			d_out[offset] = DF_PI + std::atan2(a, b);
		}





	}
}


__global__ void cuda_normalize_phase(float * const d_in_unwrap_x, float rate_x,float * const d_in_unwrap_y,float rate_y,uint32_t img_height, uint32_t img_width, float * const d_out_normal_x,float * d_out_normal_y)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;

	const unsigned int offset = idy*img_width + idx;

	if (idx < img_width && idy < img_height)
	{

		/*****************************************************************************/

		d_out_normal_x[offset] = d_in_unwrap_x[offset] /rate_x;
		d_out_normal_y[offset] = d_in_unwrap_y[offset] /rate_y;
		

		/******************************************************************/
	}
}

__global__ void cuda_variable_phase_unwrap(float * const d_in_wrap_abs, float * const d_in_wrap_high,float const rate,uint32_t img_height, uint32_t img_width,float threshold, float * const d_out)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;

	int offset = idy * img_width + idx;

	if (idx < img_width && idy < img_height)
	{

		/*****************************************************************************/

		float temp = 0.5 + (rate * d_in_wrap_abs[idy * img_width + idx] - d_in_wrap_high[idy * img_width + idx]) / (2*DF_PI);
		int k = temp;
		// d_out[idy * img_width + idx] = DF_PI*k + d_in_wrap_high[idy * img_width + idx];


		float unwrap_value =  2*DF_PI*k + d_in_wrap_high[idy * img_width + idx]; 
		float err = unwrap_value - (rate * d_in_wrap_abs[idy * img_width + idx]);
		if(abs(err)> threshold)
		{
			d_out[idy * img_width + idx] = -10.0; 
		}
		else
		{ 
			d_out[idy * img_width + idx] = unwrap_value;
		}

		/******************************************************************/
	}
}




 
__global__ void cuda_mul_phase_unwrap(float * const d_in_wrap_0, float * const d_in_wrap_1, float * const d_in_wrap_2,
	uint32_t img_height, uint32_t img_width, float * const d_out)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;

	int offset = idy* img_width +idx;

	if (idx < img_width && idy < img_height)
	{

		/*****************************************************************************/

		double temp = 0.5 + (8 * d_in_wrap_0[idy * img_width + idx] - d_in_wrap_1[idy * img_width + idx]) / (DF_PI);
		int k = temp;
		
		
		d_out[idy * img_width + idx] = DF_PI*k + d_in_wrap_1[idy * img_width + idx];

		float err = d_out[offset] - 8 *d_in_wrap_0[offset];
		if(abs(err)> 1.0)
		{
			d_out[offset] = -10;
		}
		
		float old_val = d_out[offset];
		/******************************************************************/
		temp = 0.5 + (4 * d_out[idy * img_width + idx] - d_in_wrap_2[idy * img_width + idx]) / (DF_PI);
		k = temp;

		d_out[idy * img_width + idx] = DF_PI*k + d_in_wrap_2[idy * img_width + idx];

	        err = d_out[offset] - 4 * old_val;
		if(abs(err)> 1.0)
		{
			d_out[offset] = -10;
		}
		
	}
}


__device__ void triangulation(float x_norm_L, float y_norm_L, float x_norm_R, float y_norm_R, float* R, float* T,
	float& X_L, float& Y_L, float& Z_L, float& X_R, float& Y_R, float& Z_R,
	float& error)
{
	float u_x_L = R[0] * x_norm_L + R[1] * y_norm_L + R[2];
	float u_y_L = R[3] * x_norm_L + R[4] * y_norm_L + R[5];
	float u_w_L = R[6] * x_norm_L + R[7] * y_norm_L + R[8];

	float n_x2_L = x_norm_L * x_norm_L + y_norm_L * y_norm_L + 1;
	float n_x2_R = x_norm_R * x_norm_R + y_norm_R * y_norm_R + 1;

	float D = u_x_L * x_norm_R + u_y_L * y_norm_R + u_w_L;
	float DD = n_x2_L * n_x2_R - D * D;

	float dot_uT = u_x_L * T[0] + u_y_L * T[1] + u_w_L * T[2];
	float dot_xttT = x_norm_R * T[0] + y_norm_R * T[1] + T[2];
	float dot_xttu = u_x_L * x_norm_R + u_y_L * y_norm_R + u_w_L;

	float NN1 = dot_xttu*dot_xttT - n_x2_R*dot_uT;
	float NN2 = n_x2_L*dot_xttT - dot_uT*dot_xttu;

	float Zt = NN1 / DD;
	float Ztt = NN2 / DD;

	float X1 = x_norm_L * Zt;
	float Y1 = y_norm_L * Zt;
	float Z1 = Zt;

	float X2_R = x_norm_R * Ztt - T[0];
	float Y2_R = y_norm_R * Ztt - T[1];
	float Z2_R = Ztt - T[2];

	float X2 = R[0] * X2_R + R[3] * Y2_R + R[6] * Z2_R;
	float Y2 = R[1] * X2_R + R[4] * Y2_R + R[7] * Z2_R;
	float Z2 = R[2] * X2_R + R[5] * Y2_R + R[8] * Z2_R;

	X_L = (X1 + X2) / 2.0;
	Y_L = (Y1 + Y2) / 2.0;
	Z_L = (Z1 + Z2) / 2.0;

	//XR = R * XL + T;
	X_R = R[0] * X_L + R[1] * Y_L + R[2] * Z_L + T[0];
	Y_R = R[3] * X_L + R[4] * Y_L + R[5] * Z_L + T[1];
	Z_R = R[6] * X_L + R[7] * Y_L + R[8] * Z_L + T[2];

	error = sqrt((X1 - X2) * (X1 - X2) + (Y1 - Y2) * (Y1 - Y2) + (Z1 - Z2) * (Z1 - Z2));
	 
}

__device__ void  normalizePoint(float x, float y, float fc_x, float fc_y,
	float cc_x, float cc_y, float k1, float k2,  float p1, float p2, float k3,
	float& x_norm, float& y_norm)
{
	float x_distort = (x - cc_x) / fc_x;
	float y_distort = (y - cc_y) / fc_y;

	float x_iter = x_distort;
	float y_iter = y_distort;

	for (int i = 0; i < 20; i++)
	{
		float r_2 = x_iter * x_iter + y_iter * y_iter;
		float r_4 = r_2 * r_2;
		float r_6 = r_4 * r_2;
		float k_radial = 1 + k1 * r_2 + k2 * r_4 + k3 * r_6;
		float delta_x = 2 * p1 * x_iter * y_iter + p2 * (r_2 + 2 * x_iter * x_iter);
		float delta_y = p1 * (r_2 + 2 * y_iter * y_iter) + 2 * p2 * x_iter * y_iter;
		x_iter = (x_distort - delta_x) / k_radial;
		y_iter = (y_distort - delta_y) / k_radial;
	}
	//x_norm = x_iter*fc_x+ cc_x;
	//y_norm = y_iter*fc_y + cc_y;

	x_norm = x_iter;
	y_norm = y_iter;
}

__global__ void cuda_rebuild(float * const d_in_unwrap_x, float * const d_in_unwrap_y, float * const camera_intrinsic, float * const camera_distortion,
	 float * const projector_intrinsic, float * const projector_distortion, float * const rotation_matrix, float * const translation_matrix,
	float * const d_out_point_cloud_map, float * const d_out_depth_map, float * const d_out_error_map, float * const confidence_map)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;


	const unsigned int serial_id = idy * d_image_width_ + idx;

	if (idx < d_image_width_ && idy < d_image_height_)
	{
		/****************************************************************************/
		//phase to position
		float dlp_x = d_in_unwrap_x[idy * d_image_width_ + idx] * d_dlp_width_ / (128.0*2*DF_PI);
		float dlp_y = d_in_unwrap_y[idy * d_image_width_ + idx] * d_dlp_height_ / (18.0*2*DF_PI);

		//if(100 == idx && 100 == idy)
		//{
		//	printf("%f\n", camera_intrinsic[0]); 
		//}

		/*****************************************************************************/
		////undistort
		////fc_x = i[0];fc_y = i[4]; cc_x = i[2]; cc_y = i[5];
		////k1= d[0]; k2= d[1]; p1 = d[2]; p2 = d[3]; k3 = d[4] 
		 
		float x_norm_L = 0;
		float y_norm_L = 0;

		normalizePoint(idx, idy, camera_intrinsic[0], camera_intrinsic[4], camera_intrinsic[2], camera_intrinsic[5],
			camera_distortion[0], camera_distortion[1], camera_distortion[2], camera_distortion[3], camera_distortion[4],
			x_norm_L, y_norm_L);


		float x_norm_R = 0;
		float y_norm_R = 0;

		normalizePoint(dlp_x, dlp_y, projector_intrinsic[0], projector_intrinsic[4], projector_intrinsic[2], projector_intrinsic[5],
			projector_distortion[0], projector_distortion[1], projector_distortion[2], projector_distortion[3], projector_distortion[4],
			x_norm_R, y_norm_R);

 

		/**********************************************************************************************************/
		//reconstruct
		float X_L, Y_L, Z_L, X_R, Y_R, Z_R, error;

		triangulation(x_norm_L, y_norm_L, x_norm_R, y_norm_R, rotation_matrix, translation_matrix,
			X_L, Y_L, Z_L, X_R, Y_R, Z_R, error);
		if(confidence_map[serial_id] > 10 && error< 3.0 && dlp_x > 0 && dlp_y > 0)	
		//if(confidence_map[serial_id] > 10 && error< 0.5 && dlp_x> 0.0 && dlp_y > 0.0)
		{
		    d_out_point_cloud_map[3 * serial_id + 0] = X_L;
		    d_out_point_cloud_map[3 * serial_id + 1] = Y_L;
		    d_out_point_cloud_map[3 * serial_id + 2] = Z_L;
		    d_out_depth_map[serial_id] = Z_L;
		}
		else
		{
		    d_out_point_cloud_map[3 * serial_id + 0] = 0;
		    d_out_point_cloud_map[3 * serial_id + 1] = 0;
		    d_out_point_cloud_map[3 * serial_id + 2] = 0;
		    d_out_depth_map[serial_id] = 0;
		}

		d_out_error_map[serial_id] = error;


		/******************************************************************/
 

	}
}




/***************************************************************************************************/
__device__ float computePointsDistance(float* p0,float* p1)
{
	return std::sqrt(p0[0]*p1[0] + p0[1] *p1[1] + p0[2] * p1[2]);
}
 
//滤波
__global__ void cuda_removal_points_base_mask(uint32_t img_height, uint32_t img_width,float* const point_cloud_map,float* const deep_map,uchar* remove_mask)
{
  	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y; 
  
	const unsigned int serial_id = idy * img_width + idx;

	if (idx < img_width && idy < img_height)
	{
		if(0 == remove_mask[serial_id])
		{
			deep_map[serial_id] = 0;
			point_cloud_map[3 * serial_id + 0] = 0;
			point_cloud_map[3 * serial_id + 1] = 0;
			point_cloud_map[3 * serial_id + 2] = 0;
		}

	}

}

//滤波
__global__ void cuda_filter_radius_outlier_removal(uint32_t img_height, uint32_t img_width,float* const point_cloud_map,uchar* remove_mask,float dot_spacing, float radius,int threshold)
{
 	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y; 
  
	const unsigned int serial_id = idy * img_width + idx;

	if (idx < img_width && idy < img_height)
	{
		/****************************************************************************/
		//定位区域
		if (point_cloud_map[3 * serial_id + 2] > 0)
		{
			remove_mask[serial_id] = 255;

			int w = 1 + radius / dot_spacing;

			int s_r = idy - w;
			int s_c = idx - w;

			int e_r = idy + w;
			int e_c = idx + w;

			if (s_r < 0)
			{
				s_r = 0;
			}
			if (s_c < 0)
			{
				s_c = 0;
			}

			if (e_r >= img_height)
			{
				e_r = img_height - 1;
			}

			if (e_c >= img_width)
			{
				e_c = img_width - 1;
			}

			int num = 0;

			for (int r = s_r; r <= e_r; r++)
			{
				for (int c = s_c; c <= e_c; c++)
				{
					int pos = r * img_width + c;
					if (point_cloud_map[3 * pos + 2] > 0)
					{  
						float dx= point_cloud_map[3 * serial_id + 0] - point_cloud_map[3 * pos + 0];
						float dy= point_cloud_map[3 * serial_id + 1] - point_cloud_map[3 * pos + 1];
						float dz= point_cloud_map[3 * serial_id + 2] - point_cloud_map[3 * pos + 2];

						float dist = std::sqrt(dx * dx + dy * dx + dz * dz); 
 
						if (radius > dist)
						{
							num++;
						}
					}
				}
			} 

			if (num < threshold)
			{ 
				remove_mask[serial_id] = 0;
			} 
		}
		else
		{ 
			remove_mask[serial_id] = 0;
		}

		/******************************************************************/
	}
}


/*****************************************************************************************************************************************************/


bool generate_pointcloud_base_table()
{

	reconstruct_pointcloud_base_table << <blocksPerGrid, threadsPerBlock >> > (d_xL_rotate_x_ , d_xL_rotate_y_, 
                                                d_single_pattern_mapping_, d_R_1_,d_confidence_list[3],d_unwrap_map_list[0],
												image_height_,image_width_,d_baseline_,d_point_cloud_map_,d_depth_map_);


	// LOG(INFO)<<"remove start:";
	// //相机像素为5.4um、焦距12mm。dot_spacing = 5.4*distance/12000 mm，典型值0.54mm（1200） 
	cuda_filter_radius_outlier_removal << <blocksPerGrid, threadsPerBlock >> > (image_height_,image_width_,d_point_cloud_map_,d_mask_,0.5,2.5,6); 
	cuda_removal_points_base_mask << <blocksPerGrid, threadsPerBlock >> > (image_height_,image_width_,d_point_cloud_map_,d_depth_map_,d_mask_); 

    // cudaDeviceSynchronize();
	// cv::Mat point_cloud_map(1200, 1920, CV_32FC3, cv::Scalar(0.0));  
	// CHECK(cudaMemcpy((float*)point_cloud_map.data, d_point_cloud_map_, 3 * image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost));

	// std::vector<cv::Mat> channel;
	// cv::split(point_cloud_map, channel); 
	// cv::imwrite("pointcloud_map_x.tiff",channel[0]);
	// cv::imwrite("pointcloud_map_y.tiff",channel[1]);
	// cv::imwrite("pointcloud_map_z.tiff",channel[2]);
	// LOG(INFO)<<"remove finished!";
}
 
 bool generate_pointcloud_base_minitable()
{

	reconstruct_pointcloud_base_minitable << <blocksPerGrid, threadsPerBlock >> > (d_xL_rotate_x_, d_xL_rotate_y_,
		d_single_pattern_minimapping_, d_R_1_, d_confidence_list[3], d_unwrap_map_list[0],
		image_height_, image_width_, d_baseline_, d_point_cloud_map_, d_depth_map_);


}

__device__ float bilinear_interpolation(float x, float y, int map_width, float *mapping)
{

	int x1 = floor(x);
	int y1 = floor(y);
	int x2 = x1 + 1;
	int y2 = y1 + 1;

	//row-y,col-x

	if (x1 == 1919) {
		float out = mapping[y1 *map_width + x1];
		return out;
	}
	else {
		float fq11 = mapping[y1 *map_width + x1];
		float fq21 = mapping[y1 *map_width + x2];
		float fq12 = mapping[y2 *map_width + x1];
		float fq22 = mapping[y2 *map_width + x2];

		if (-2 == fq11 || -2 == fq21 || -2 == fq12 || -2 == fq22)
		{
			return -2;
		}

		float out = fq11 * (x2 - x) * (y2 - y) + fq21 * (x - x1) * (y2 - y) + fq12 * (x2 - x) * (y - y1) + fq22 * (x - x1) * (y - y1);

		return out;
	}
	 

}


__device__ float mini_bilinear_interpolation(float x, float y, int map_width, float *mapping)
{
	//map_width = 129;

	//先找到这个点所对应的mini中的四个角点
	//然后将这四个点算出来
	//最后双线性插值

	int index_x1 = floor(x / 16);
	int index_y1 = floor((y-1301) / 16);
	int index_x2 = index_x1 + 1;
	int index_y2 = index_y1 + 1;

	int x1 = index_x1 * 16;
	int y1 = index_y1 * 16 + 1301;
	int x2 = x1 + 16;
	int y2 = y1 + 16;

	//因为我生成的表比原来大，所以无需考虑边界条件
	//fq_xy
	float fq11 = mapping[index_y1 *map_width + index_x1];
	float fq21 = mapping[index_y1 *map_width + index_x2];
	float fq12 = mapping[index_y2 *map_width + index_x1];
	float fq22 = mapping[index_y2 *map_width + index_x2];

	float out = (fq11 * (x2 - x) * (y2 - y) + fq21 * (x - x1) * (y2 - y) + fq12 * (x2 - x) * (y - y1) + fq22 * (x - x1) * (y - y1))/256.;

	return out;
}


__global__ void reconstruct_pointcloud_base_table(float * const xL_rotate_x,float * const xL_rotate_y,float * const single_pattern_mapping,float * const R_1,float * const confidence_map,
                                                        float * const phase_x,uint32_t img_height, uint32_t img_width,float b, float * const pointcloud,float * const depth)
{
    const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;

  
	const unsigned int serial_id = idy * img_width + idx;

	if (idx < img_width && idy < img_height)
	{
		/****************************************************************************/
		//phase to position
		float Xp = phase_x[serial_id] * d_dlp_width_ / (128.0*2*DF_PI); 
  
    	float Xcr = bilinear_interpolation(idx, idy,1920, xL_rotate_x);
        float Ycr = bilinear_interpolation(idx, idy, 1920,xL_rotate_y);
        float Xpr = bilinear_interpolation(Xp, (Ycr + 1) * 2000, 2000, single_pattern_mapping);
        float delta_X = std::abs(Xcr - Xpr);
		// float delta_X = Xpr -Xcr;
        float Z = b / delta_X;
	
		float X_L = Z * Xcr * R_1[0] + Z * Ycr * R_1[1] + Z * R_1[2];
		float Y_L = Z * Xcr * R_1[3] + Z * Ycr * R_1[4] + Z * R_1[5];
		float Z_L = Z * Xcr * R_1[6] + Z * Ycr * R_1[7] + Z * R_1[8];
 
  
		if(confidence_map[serial_id] > d_confidence_ && Z_L > 10 && Z_L< 60000 && Xp > 0)
		{
		    pointcloud[3 * serial_id + 0] = X_L;
		    pointcloud[3 * serial_id + 1] = Y_L;
		    pointcloud[3 * serial_id + 2] = Z_L; 
			
		    depth[serial_id] = Z_L; 
		}
		else
		{
		    pointcloud[3 * serial_id + 0] = 0;
		    pointcloud[3 * serial_id + 1] = 0;
		    pointcloud[3 * serial_id + 2] = 0; 
			
		    depth[serial_id] = 0; 
		}

		
		if (-2 == Xcr || -2 == Ycr || -2 == Xpr)
		{
			pointcloud[3 * serial_id + 0] = 0;
		    pointcloud[3 * serial_id + 1] = 0;
		    pointcloud[3 * serial_id + 2] = 0; 
			
		    depth[serial_id] = 0; 
		}
  
		/******************************************************************/


	}
}


__global__ void reconstruct_pointcloud_base_minitable(float* const xL_rotate_x, float* const xL_rotate_y, float* const single_pattern_minimapping, float* const R_1, float* const confidence_map,
	float* const phase_x, uint32_t img_height, uint32_t img_width, float b, float* const pointcloud, float* const depth)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;


	const unsigned int serial_id = idy * img_width + idx;

	if (idx < img_width && idy < img_height)
	{
		/****************************************************************************/
		//phase to position
		float Xp = phase_x[serial_id] * 1280.0 / (128.0 * 2 * DF_PI);

		float Xcr = bilinear_interpolation(idx, idy, 1920, xL_rotate_x);
		float Ycr = bilinear_interpolation(idx, idy, 1920, xL_rotate_y);
		//修改此处即可，需要自己写一个函数去查表
		float Xpr = mini_bilinear_interpolation(Xp, (Ycr + 1) * 2000, 128, single_pattern_minimapping);
		float delta_X = std::abs(Xcr - Xpr);
		float Z = b / delta_X;

		float X_L = Z * Xcr * R_1[0] + Z * Ycr * R_1[1] + Z * R_1[2];
		float Y_L = Z * Xcr * R_1[3] + Z * Ycr * R_1[4] + Z * R_1[5];
		float Z_L = Z * Xcr * R_1[6] + Z * Ycr * R_1[7] + Z * R_1[8];


		if (confidence_map[serial_id] > 10 && Z_L > 100 && Z_L < 2000)
		{
			pointcloud[3 * serial_id + 0] = X_L;
			pointcloud[3 * serial_id + 1] = Y_L;
			pointcloud[3 * serial_id + 2] = Z_L;

			depth[serial_id] = Z_L;
		}
		else
		{
			pointcloud[3 * serial_id + 0] = 0;
			pointcloud[3 * serial_id + 1] = 0;
			pointcloud[3 * serial_id + 2] = 0;

			depth[serial_id] = 0;
		}

		/******************************************************************/


	}
}


void reconstruct_copy_talbe_to_cuda_memory(float* mapping,float* rotate_x,float* rotate_y,float* r_1)
{
   
	CHECK(cudaMemcpyAsync(d_R_1_, r_1, 3*3 * sizeof(float), cudaMemcpyHostToDevice)); 
	CHECK(cudaMemcpyAsync(d_single_pattern_mapping_, mapping, 4000*2000 * sizeof(float), cudaMemcpyHostToDevice));
	CHECK(cudaMemcpyAsync(d_xL_rotate_x_, rotate_x, image_height_*image_width_ * sizeof(float), cudaMemcpyHostToDevice));
	CHECK(cudaMemcpyAsync(d_xL_rotate_y_, rotate_y, image_height_*image_width_ * sizeof(float), cudaMemcpyHostToDevice));

}


void reconstruct_copy_minitalbe_to_cuda_memory(float* minimapping, float* rotate_x, float* rotate_y, float* r_1)
{

	CHECK(cudaMemcpyAsync(d_R_1_, r_1, 3 * 3 * sizeof(float), cudaMemcpyHostToDevice));
	CHECK(cudaMemcpyAsync(d_single_pattern_minimapping_, minimapping, 128 * 128 * sizeof(float), cudaMemcpyHostToDevice));
	CHECK(cudaMemcpyAsync(d_xL_rotate_x_, rotate_x, image_height_ * image_width_ * sizeof(float), cudaMemcpyHostToDevice));
	CHECK(cudaMemcpyAsync(d_xL_rotate_y_, rotate_y, image_height_ * image_width_ * sizeof(float), cudaMemcpyHostToDevice));

}


void reconstruct_set_baseline(float b)
{
    d_baseline_ = b;  
}

void reconstruct_copy_pointcloud_from_cuda_memory(float* pointcloud)
{ 
	CHECK(cudaMemcpy(pointcloud, d_point_cloud_map_, 3 * image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
}


void reconstruct_copy_confidence_from_cuda_memory(float* confidence)
{ 
	CHECK(cudaMemcpy(confidence, d_confidence_list[3], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost)); 
}

void reconstruct_copy_depth_from_cuda_memory(float* depth)
{
	CHECK(cudaMemcpy(depth, d_depth_map_, image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost)); 
} 

void reconstruct_copy_brightness_from_cuda_memory(unsigned char* brightness)
{
	CHECK(cudaMemcpy(brightness, d_patterns_list[18], image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost)); 
}

  
void copy_phase_from_cuda_memory(float* phase_x,float* phase_y)
{
	CHECK(cudaMemcpy(phase_x, d_unwrap_map_list[0], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost)); 
	CHECK(cudaMemcpy(phase_y, d_unwrap_map_list[1], image_height_*image_width_ * sizeof(float), cudaMemcpyDeviceToHost)); 
 
}
 
void copy_merge_brightness_from_cuda_memory(unsigned char* brightness)
{ 
	CHECK(cudaMemcpy(brightness, d_brightness_, image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost)); 
}

void reconstruct_cuda_malloc_memory()
{


	//为了防止重复的开辟内存，应先释放指针
	cudaFree(d_single_pattern_mapping_);
    cudaFree(d_xL_rotate_x_);
    cudaFree(d_xL_rotate_y_);
    cudaFree(d_R_1_);
	cudaMalloc((void**)&d_single_pattern_mapping_, 4000*2000 * sizeof(float)); 
	cudaMalloc((void**)&d_xL_rotate_x_, image_height_*image_width_ * sizeof(float)); 
	cudaMalloc((void**)&d_xL_rotate_y_, image_height_*image_width_ * sizeof(float)); 
	cudaMalloc((void**)&d_R_1_, 3*3 * sizeof(float)); 

  
}


void reconstruct_cuda_minimalloc_memory()
{
	
	//为了防止重复的开辟内存，应先释放指针
	cudaFree(d_single_pattern_minimapping_);
    cudaFree(d_xL_rotate_x_);
    cudaFree(d_xL_rotate_y_);
    cudaFree(d_R_1_);
	cudaMalloc((void**)&d_single_pattern_minimapping_, 128*128 * sizeof(float)); 
	cudaMalloc((void**)&d_xL_rotate_x_, image_height_*image_width_ * sizeof(float)); 
	cudaMalloc((void**)&d_xL_rotate_y_, image_height_*image_width_ * sizeof(float)); 
	cudaMalloc((void**)&d_R_1_, 3*3 * sizeof(float)); 

  
}

void reconstruct_cuda_free_memory()
{
    cudaFree(d_single_pattern_mapping_);
    cudaFree(d_xL_rotate_x_);
    cudaFree(d_xL_rotate_y_);
    cudaFree(d_R_1_);
 
}

void reconstruct_cuda_free_minimemory()
{
    cudaFree(d_single_pattern_minimapping_);
    cudaFree(d_xL_rotate_x_);
    cudaFree(d_xL_rotate_y_);
    cudaFree(d_R_1_);
 
}


 


/****************************************************************************************************/