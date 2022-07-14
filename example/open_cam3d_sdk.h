﻿#pragma once

#ifdef _WIN32 
#define DF_SDK_API __declspec(dllexport)

#elif __linux
#define DF_SDK_API 
#endif
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
		//相机畸变，只用前5个
		float distortion[1 * 12];//<k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4>

	};


	//函数名： DfRegisterOnDropped
	//功能：注册断连回调函数
	//输入参数：无
	//输出参数：p_function（回调函数）
	//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
	DF_SDK_API int DfRegisterOnDropped(int (*p_function)(void*));

	//函数名： DfConnect
	//功能： 连接相机
	//输入参数： camera_id（相机ip地址）
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
	//输入参数： exposure_num（曝光次数）：设置值为1为单曝光，大于1为多曝光模式（具体参数在相机gui中设置）.
	//输出参数： timestamp(时间戳)
	//返回值： 类型（int）:返回0表示获取采集数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfCaptureData(int exposure_num, char* timestamp);

	//函数名： DfGetDepthData
	//功能： 获取深度图
	//输入参数：无
	//输出参数： depth(深度图)
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfGetDepthData(unsigned short* depth);


	//函数名： DfGetBrightnessData
	//功能： 获取亮度图
	//输入参数：无
	//输出参数： brightness(亮度图)
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfGetBrightnessData(unsigned char* brightness);

	//函数名： DfGetHeightMapData
	//功能： 获取校正到基准平面的高度映射图
	//输入参数：无
	//输出参数： height_map(高度映射图)
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfGetHeightMapData(float* height_map);

	//函数名： DfGetStandardPlaneParam
	//功能： 获取基准平面参数
	//输入参数：无
	//输出参数： R(旋转矩阵：3*3)、T(平移矩阵：3*1)
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfGetStandardPlaneParam(float* R, float* T);

	//函数名： DfGetHeightMapDataBaseParam
	//功能： 获取校正到基准平面的高度映射图
	//输入参数：R(旋转矩阵)、T(平移矩阵)
	//输出参数： height_map(高度映射图)
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfGetHeightMapDataBaseParam(float* R, float* T, float* height_map);

	//函数名： DfGetPointcloudData
	//功能： 获取点云
	//输入参数：无
	//输出参数： point_cloud(点云)
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfGetPointcloudData(float* point_cloud);

	//函数名： DfConnect
	//功能： 断开相机连接
	//输入参数： camera_id（相机ip地址）
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
	//参数设置


	//函数名： DfSetParamLedCurrent
	//功能： 设置LED电流
	//输入参数： led（电流值）
	//输出参数： 无
	//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
	DF_SDK_API int DfSetParamLedCurrent(int led);


	//函数名： DfGetParamLedCurrent
	//功能： 设置LED电流
	//输入参数： 无
	//输出参数： led（电流值）
	//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
	DF_SDK_API int DfGetParamLedCurrent(int& led);

	//函数名： DfSetParamHdr
	//功能： 设置多曝光参数（最大曝光次数为6次）
	//输入参数： num（曝光次数）、exposure_param[6]（6个曝光参数、前num个有效）
	//输出参数： 无
	//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
	DF_SDK_API int DfSetParamHdr(int num, int exposure_param[6]);


	//函数名： DfGetParamHdr
	//功能： 设置多曝光参数（最大曝光次数为6次）
	//输入参数： 无
	//输出参数： num（曝光次数）、exposure_param[6]（6个曝光参数、前num个有效）
	//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
	DF_SDK_API int DfGetParamHdr(int& num, int exposure_param[6]);

	//函数名： DfSetParamStandardPlaneExternal
	//功能： 设置基准平面的外参
	//输入参数：R(旋转矩阵：3*3)、T(平移矩阵：3*1)
	//输出参数： 无
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfSetParamStandardPlaneExternal(float* R, float* T);

	//函数名： DfGetParamStandardPlaneExternal
	//功能： 获取基准平面的外参
	//输入参数：无
	//输出参数： R(旋转矩阵：3*3)、T(平移矩阵：3*1)
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfGetParamStandardPlaneExternal(float* R, float* T);

	//函数名： DfSetParamGenerateBrightness
	//功能： 设置生成亮度图参数
	//输入参数：model(1:与条纹图同步连续曝光、2：单独发光曝光、3：不发光单独曝光)、exposure(亮度图曝光时间)
	//输出参数： 无
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfSetParamGenerateBrightness(int model, float exposure);

	//函数名： DfGetParamGenerateBrightness
	//功能： 获取生成亮度图参数
	//输入参数： 无
	//输出参数：model(1:与条纹图同步连续曝光、2：单独发光曝光、3：不发光单独曝光)、exposure(亮度图曝光时间)
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfGetParamGenerateBrightness(int& model, float& exposure);

	//函数名： DfSetParamCameraExposure
	//功能： 设置相机曝光时间
	//输入参数：exposure(相机曝光时间)
	//输出参数： 无
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfSetParamCameraExposure(float exposure);

	//函数名： DfGetParamCameraExposure
	//功能： 获取相机曝光时间
	//输入参数： 无
	//输出参数：exposure(相机曝光时间)
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfGetParamCameraExposure(float& exposure);

	//函数名： DfSetParamMixedHdr
	//功能： 设置混合多曝光参数（最大曝光次数为6次）
	//输入参数： num（曝光次数）、exposure_param[6]（6个曝光参数、前num个有效）、led_param[6]（6个led亮度参数、前num个有效）
	//输出参数： 无
	//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
	DF_SDK_API int DfSetParamMixedHdr(int num, int exposure_param[6], int led_param[6]);

	//函数名： DfGetParamMixedHdr
	//功能： 获取混合多曝光参数（最大曝光次数为6次）
	//输入参数： 无
	//输出参数： num（曝光次数）、exposure_param[6]（6个曝光参数、前num个有效）、led_param[6]（6个led亮度参数、前num个有效）
	//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
	DF_SDK_API int DfGetParamMixedHdr(int& num, int exposure_param[6], int led_param[6]);

	//函数名： DfSetParamCameraConfidence
	//功能： 设置相机曝光时间
	//输入参数：confidence(相机置信度)
	//输出参数： 无
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfSetParamCameraConfidence(float confidence);

	//函数名： DfGetParamCameraConfidence
	//功能： 获取相机曝光时间
	//输入参数： 无
	//输出参数：confidence(相机置信度)
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfGetParamCameraConfidence(float& confidence);


	//函数名： DfSetParamCameraGain
	//功能： 设置相机增益
	//输入参数：gain(相机增益)
	//输出参数： 无
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfSetParamCameraGain(float gain);

	//函数名： DfGetParamCameraGain
	//功能： 获取相机增益
	//输入参数： 无
	//输出参数：gain(相机增益)
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfGetParamCameraGain(float& gain);

	//函数名： DfSetParamSmoothing
	//功能： 设置点云平滑参数
	//输入参数：smoothing(0:关、1-5:平滑程度由低到高)
	//输出参数： 无
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfSetParamSmoothing(int smoothing);

	//函数名： DfGetParamSmoothing
	//功能： 设置点云平滑参数
	//输入参数：无
	//输出参数：smoothing(0:关、1-5:平滑程度由低到高)
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfGetParamSmoothing(int& smoothing);
}
