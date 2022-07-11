#ifndef DF8_ENCODE_CUDA_CUH
#define DF8_ENCODE_CUDA_CUH

//#include <opencv2/core.hpp> 
#include <device_launch_parameters.h>
//#include <device_functions.h>
#include <cuda_runtime.h>
#include <iostream>
#include <stdint.h>
#include <vector>
#include "system_config_settings.h"

//滤波
// __global__ void cuda_filter_radius_outlier_removal(uint32_t img_height, uint32_t img_width,float* const point_cloud_map,uchar* remove_mask,float radius,int threshold);

__device__ float computePointsDistance(float* p0,float* p1);

bool cuda_set_confidence(float val);

void cuda_set_config(struct SystemConfigDataStruct param);

bool cuda_set_camera_version(int version);

void BubbleSort(float  *p, int length, int * ind_diff);
/***************************************************************************************/
//用于查找表
bool generate_pointcloud_base_table();
bool generate_pointcloud_base_minitable();

void reconstruct_cuda_malloc_memory();
void reconstruct_cuda_minimalloc_memory();
void reconstruct_cuda_free_memory();
void reconstruct_cuda_free_minimemory();
void reconstruct_set_baseline(float b);
void reconstruct_copy_talbe_to_cuda_memory(float* mapping,float* rotate_x,float* rotate_y,float* r_1);
void reconstruct_copy_minitalbe_to_cuda_memory(float* mapping, float* rotate_x, float* rotate_y, float* r_1);
void reconstruct_copy_pointcloud_from_cuda_memory(float* pointcloud);
void reconstruct_copy_depth_from_cuda_memory(float* depth);
void reconstruct_copy_confidence_from_cuda_memory(float* confidence);
void reconstruct_copy_brightness_from_cuda_memory(unsigned char* brightness);

void copy_phase_from_cuda_memory(float* phase_x,float* phase_y);
void copy_merge_brightness_from_cuda_memory(unsigned char* brightness);

__global__ void reconstruct_pointcloud_base_table(float * const xL_rotate_x,float * const xL_rotate_y,float * const single_pattern_mapping,float * const R_1,float * const confidence_map,
                                                        float * const phase_x,uint32_t img_height, uint32_t img_width,float b, float * const pointcloud,float * const depth);
__global__ void reconstruct_pointcloud_base_minitable(float * const xL_rotate_x,float * const xL_rotate_y,float * const single_pattern_mapping,float * const R_1,float * const confidence_map,
                                                        float * const phase_x,uint32_t img_height, uint32_t img_width,float b, float * const pointcloud,float * const depth);

__device__ float bilinear_interpolation(float x, float y, int map_width, float *mapping);
__device__ float mini_bilinear_interpolation(float x, float y, int map_width, float *mapping);

/**********************************************************************************************/
//并行于图像采集

bool transform_pointcloud(float* rotate,float* translation,float * const transform_depth);
/**********************************************************************************************/
//重复模式
bool parallel_cuda_copy_repetition_signal_patterns(unsigned char* patterns_ptr,int serial_flag);

bool parallel_cuda_merge_repetition_patterns(int repetition_serial);
bool parallel_cuda_merge_repetition_02_patterns(int repetition_serial);
bool parallel_cuda_clear_repetition_02_patterns();


bool parallel_cuda_compute_merge_phase(int repetition_count); 
bool parallel_cuda_compute_merge_repetition_02_phase(int repetition_count); 
bool parallel_cuda_compute_model_02_merge_repetition_02_phase(int repetition_count); 

__global__ void cuda_merge_pattern(unsigned char * const d_in_pattern,uint32_t img_height, uint32_t img_width,unsigned short * const d_out_merge_pattern);

__global__ void cuda_merge_brigntness_map(unsigned short * const merge_brightness,int repetition_count, unsigned char* brightness);

__global__ void cuda_merge_six_step_phase_shift(unsigned short * const d_in_0, unsigned short * const d_in_1, unsigned short * const d_in_2, 
	unsigned short * const d_in_3,unsigned short* const d_in_4,unsigned short* const d_in_5,int repetition_count,
	uint32_t img_height, uint32_t img_width,float * const d_out, float * const confidence);

__global__ void cuda_merge_four_step_phase_shift(unsigned short * const d_in_0, unsigned short * const d_in_1, unsigned short * const d_in_2, 
	unsigned short * const d_in_3,int repetition_count,float * const d_out, float * const confidence);
/**********************************************************************************************/
bool parallel_cuda_copy_signal_patterns(unsigned char* patterns_ptr,int serial_flag);

bool parallel_cuda_compute_phase(int serial_flag);

bool parallel_cuda_unwrap_phase(int serial_flag);

bool parallel_cuda_reconstruct();

bool parallel_cuda_copy_unwrap_phase_from_gpu(int serial_flag,float* unwrap_map);

bool parallel_cuda_copy_unwrap_phase_to_gpu(int serial_flag,float* unwrap_map);

bool parallel_cuda_copy_result_from_gpu(float* depth,unsigned char* brightness);

bool parallel_cuda_copy_pointcloud_from_gpu(float* pointcloud,unsigned char* brightness);

bool parallel_cuda_merge_hdr_data(int hdr_num,float* depth_map, unsigned char* brightness); 

bool parallel_cuda_copy_result_to_hdr(int serial_flag,int brigntness_serial);

__global__ void parallel_cuda_count_sum_pixel(const unsigned char* brightness,uint32_t img_height, uint32_t img_width, float* sum_pixels);

__global__ void parallel_cuda_merge_hdr_6(const float*  depth_map_0,const float*  depth_map_1,const float*  depth_map_2,
	const float*  depth_map_3,const float*  depth_map_4,const float*  depth_map_5,
	const unsigned char* brightness_0,const unsigned char* brightness_1,const unsigned char* brightness_2,
	const unsigned char* brightness_3,const unsigned char* brightness_4,const unsigned char* brightness_5,
	uint32_t img_height, uint32_t img_width, float* const depth_map,unsigned char * const brightness);

__global__ void parallel_cuda_merge_hdr_5(const float*  depth_map_0,const float*  depth_map_1,const float*  depth_map_2,
	const float*  depth_map_3,const float*  depth_map_4,
	const unsigned char* brightness_0,const unsigned char* brightness_1,const unsigned char* brightness_2,
	const unsigned char* brightness_3,const unsigned char* brightness_4,
	uint32_t img_height, uint32_t img_width, float* const depth_map,unsigned char * const brightness);

__global__ void parallel_cuda_merge_hdr_4(const float*  depth_map_0,const float*  depth_map_1,const float*  depth_map_2,const float*  depth_map_3,
	const unsigned char* brightness_0,const unsigned char* brightness_1,const unsigned char* brightness_2,const unsigned char* brightness_3,
	uint32_t img_height, uint32_t img_width, float* const depth_map,unsigned char * const brightness);

__global__ void parallel_cuda_merge_hdr_3(const float*  depth_map_0,const float*  depth_map_1,const float*  depth_map_2,const unsigned char* brightness_0,const unsigned char* brightness_1,
	const unsigned char* brightness_2,uint32_t img_height, uint32_t img_width, float* const depth_map,unsigned char * const brightness);

__global__ void parallel_cuda_merge_hdr_2(const float*  depth_map_0,const float*  depth_map_1,const unsigned char* brightness_0,const unsigned char* brightness_1,
	uint32_t img_height, uint32_t img_width, float* const depth_map,unsigned char * const brightness);

/*******************************************************************************************/
//����device�ڴ�
bool cuda_malloc_memory();
//�����궨����
bool cuda_copy_calib_data(float* camera_intrinsic, float* project_intrinsic, float* camera_distortion,
	float* projector_distortion, float* rotation_matrix, float* translation_matrix);
//��ά�ؽ�
bool cuda_reconstruct_base_24(std::vector<unsigned char*> patterns,float* point_cloud); 

bool cuda_get_frame_base_24(std::vector<unsigned char*> patterns,float* depth, unsigned char* brightness); 

bool cuda_get_frame_03(std::vector<unsigned char*> patterns,float* depth, unsigned char* brightness); 

bool cuda_merge_hdr_data(std::vector<float*> depth_map_list,std::vector<unsigned char*> brightness_list,float* depth_map, unsigned char* brightness); 


bool cuda_compute_phase_03_hdr(int group_flag);

bool cuda_copy_patterns_hdr(std::vector<unsigned char*> patterns,int flag);

bool cuda_get_frame_03_hdr(std::vector<unsigned char*> patterns,int group_flag,float* depth,unsigned char* brightness);

bool cuda_reconstruct();

bool cuda_get_frame_data(float* depth,unsigned char* brightness);

bool cuda_get_depth_data(float* depth);

bool cuda_get_pointcloud_data(float* pointcloud);

bool cuda_get_brightness_data(unsigned char* brightness);


//�ͷ�����device�ڴ�
bool cuda_free_memory();

/*****************************************************************************************/
 
bool cuda_copy_patterns(std::vector<unsigned char*> patterns);

bool cuda_compute_phase();

bool cuda_unwrap_phase();

bool cuda_compute_phase_03();

bool cuda_unwrap_phase_03();


bool cuda_reconstruct_pointcloud(float* point_cloud);
/******************************************************************************************/

 
__device__ void  normalizePoint(float x, float y, float fc_x, float fc_y,
	float cc_x, float cc_y, float k1, float k2, float p1, float p2, float k3,
	float& x_norm, float& y_norm);

__device__ void triangulation(float x_norm_L, float y_norm_L,float x_norm_R, float y_norm_R,float* R, float* T,
	float& X_L, float& Y_L, float& Z_L,float& X_R, float& Y_R, float& Z_R,
	float& error);


/*********************************************************************************************/

__global__ void cuda_merge_hdr(const float*  depth_map_0,const float*  depth_map_1,const float*  depth_map_2,const unsigned char* brightness_0,const unsigned char* brightness_1,
	const unsigned char* brightness_2,uint32_t img_height, uint32_t img_width, float* const depth_map,unsigned char * const brightness);

__global__ void cuda_six_step_phase_shift(unsigned char * const d_in_0, unsigned char * const d_in_1, unsigned char * const d_in_2, unsigned char * const d_in_3,unsigned char* const d_in_4,unsigned char* const d_in_5,
	uint32_t img_height, uint32_t img_width,float * const d_out, float * const confidence);

__global__ void cuda_six_step_phase_shift_texture(float * const d_out, float * const confidence);

__global__ void cuda_four_step_phase_shift_texture(int serial_flag,float * const d_out, float * const confidence);

//kernel
__global__ void cuda_four_step_phase_shift(unsigned char * const d_in_0, unsigned char * const d_in_1, unsigned char * const d_in_2, unsigned char * const d_in_3,
	uint32_t img_height, uint32_t img_width,float * const d_out, float * const confidence);

__global__ void cuda_merge_brightness(unsigned char * const d_in_0, unsigned char * const d_in_1, unsigned char * const d_in_2, unsigned char * const d_in_3,
	uint32_t img_height, uint32_t img_width,unsigned char * const d_out);

__global__ void cuda_normalize_phase(float * const d_in_unwrap_x,float rate_x, float * const d_in_unwrap_y, float rate_y, uint32_t img_height, uint32_t img_width, float * const d_out_normal_x,float * const d_out_normal_y);
 

__global__ void cuda_variable_phase_unwrap(float * const d_in_wrap_abs, float * const d_in_wrap_high, float const rate, uint32_t img_height, uint32_t img_width, float threshold,float * const d_out);
 
__global__ void cuda_mul_phase_unwrap(float * const d_in_wrap_0, float * const d_in_wrap_1, float * const d_in_wrap_2,
	uint32_t img_height, uint32_t img_width, float * const d_out);
 
__global__ void cuda_rebuild(float * const d_in_unwrap_x, float * const d_in_unwrap_y, float * const camera_intrinsic, float * const camera_distortion,
	float * const projector_intrinsic, float * const projector_distortion, float * const rotation_matrix, float * const translation_matrix,
	float * const d_out_point_cloud_map, float * const d_out_depth_map, float * const d_out_error_map, float * const confidence_map);

#endif

