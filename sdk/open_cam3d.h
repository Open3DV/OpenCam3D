#pragma once

#ifdef _WIN32 
#define DF_SDK_API __declspec(dllexport)

#elif __linux
#define DF_SDK_API 
#endif
 
#include "../Firmware/protocol.h" 
#include "../firmware/system_config_settings.h"

/***************************************************************************************/

extern "C"
{

	//返回码
	//0: 成功; -1:失败; -2:未获取相机分辨率分配内存

	//相机标定参数结构体
	struct CalibrationParam
	{
		//相机内参
		float intrinsic[3 * 3];
		//相机外参
		float extrinsic[4 * 4];
		//相机畸变
		float distortion[1 * 12];//<k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4>

	};


	//函数名： DfConnect
	//功能： 连接相机
	//输入参数： camera_id（相机id）
	//输出参数： 无
	//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
	DF_SDK_API int DfConnect(const char* camera_id);

	//函数名： DfGetCameraResolution
	//功能： 获取相机分辨率
	//输入参数： 无
	//输出参数： width(图像宽)、height(图像高)
	//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败.
	DF_SDK_API int  DfGetCameraResolution(int* width, int* height);

	//函数名： DfCaptureData
	//功能： 采集一帧数据并阻塞至返回状态
	//输入参数： exposure_num（曝光次数）：可设置值为1、2、3.
	//输出参数： timestamp(时间戳)
	//返回值： 类型（int）:返回0表示获取采集数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfCaptureData(int exposure_num, char* timestamp);

	//函数名： DfGetDepthData
	//功能： 采集点云数据并阻塞至返回结果
	//输入参数：无
	//输出参数： depth(深度图)
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfGetDepthData(unsigned short* depth);


	//函数名： DfGetBrightnessData
	//功能： 采集点云数据并阻塞至返回结果
	//输入参数：无
	//输出参数： brightness(亮度图)
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfGetBrightnessData(unsigned char* brightness);

	//函数名： DfGetPointcloudData
	//功能： 采集点云数据并阻塞至返回结果
	//输入参数：无
	//输出参数： point_cloud(点云)
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfGetPointcloudData(float* point_cloud);

	//函数名： DfConnect
	//功能： 断开相机连接
	//输入参数： camera_id（相机id）
	//输出参数： 无
	//返回值： 类型（int）:返回0表示断开成功;返回-1表示断开失败.
	DF_SDK_API int DfDisconnect(const char* camera_id);

	//函数名： DfGetCalibrationParam
	//功能： 获取相机标定参数
	//输入参数： 无
	//输出参数： calibration_param（相机标定参数结构体）
	//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
	DF_SDK_API int DfGetCalibrationParam(struct CalibrationParam* calibration_param);


	/***************************************************************************************************************************************************************/



}

/**************************************************************************************/

DF_SDK_API int DfConnectNet(const char* ip);

DF_SDK_API int DfDisconnectNet();

DF_SDK_API int DfGetCameraResolution(int* width, int* height);

DF_SDK_API int DfGetCameraData(
	short* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size,
	short* point_cloud, int point_cloud_buf_size,
	unsigned char* confidence, int confidence_buf_size);

DF_SDK_API int GetBrightness(unsigned char* brightness, int brightness_buf_size);

DF_SDK_API int DfGetCameraRawData01(unsigned char* raw, int raw_buf_size);

DF_SDK_API int DfGetCameraRawData02(unsigned char* raw, int raw_buf_size);

DF_SDK_API int DfGetCameraRawDataTest(unsigned char* raw, int raw_buf_size);

DF_SDK_API int DfGetCameraRawData03(unsigned char* raw, int raw_buf_size);

DF_SDK_API bool depthTransformPointcloud(float* depth_map, float* point_cloud_map);

DF_SDK_API int DfGetPointCloud(float* point_cloud, int point_cloud_buf_size);

DF_SDK_API int DfGetFrame01(float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size);

DF_SDK_API int DfGetFrameHdr(float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size);

DF_SDK_API int DfGetFrame03(float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size);

DF_SDK_API int DfGetRepetitionFrame03(int count,float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size);

DF_SDK_API int DfGetCalibrationParam(struct CameraCalibParam& calibration_param);

DF_SDK_API int DfSetCalibrationParam(const struct CameraCalibParam& calibration_param);

DF_SDK_API int DfGetDeviceTemperature(float& temperature);

DF_SDK_API int DfRegisterOnDropped(int (*p_function)(void*));

DF_SDK_API int DfGetSystemConfigParam(struct SystemConfigParam& config_param);

DF_SDK_API int DfSetSystemConfigParam(const struct SystemConfigParam& config_param);

DF_SDK_API int DfEnableCheckerboard(float& temperature);

DF_SDK_API int DfDisableCheckerboard(float& temperature);

DF_SDK_API int DfLoadPatternData(int buildDataSize, char* LoadBuffer);

DF_SDK_API int DfProgramPatternData(char* org_buffer, char* back_buffer, unsigned int pattern_size);

DF_SDK_API int DfGetNetworkBandwidth(int &speed);