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

	//������
	//0: �ɹ�; -1:ʧ��; -2:δ��ȡ����ֱ��ʷ����ڴ�

	//����궨�����ṹ��
	struct CalibrationParam
	{
		//����ڲ�
		float intrinsic[3 * 3];
		//������
		float extrinsic[4 * 4];
		//�������
		float distortion[1 * 12];//<k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4>

	};


	//�������� DfConnect
	//���ܣ� �������
	//��������� camera_id�����id��
	//��������� ��
	//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
	DF_SDK_API int DfConnect(const char* camera_id);

	//�������� DfGetCameraResolution
	//���ܣ� ��ȡ����ֱ���
	//��������� ��
	//��������� width(ͼ���)��height(ͼ���)
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�����ɹ�;����-1��ʾ��ȡ����ʧ��.
	DF_SDK_API int  DfGetCameraResolution(int* width, int* height);

	//�������� DfCaptureData
	//���ܣ� �ɼ�һ֡���ݲ�����������״̬
	//��������� exposure_num���ع��������������ֵΪ1��2��3.
	//��������� timestamp(ʱ���)
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�ɼ����ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
	DF_SDK_API int DfCaptureData(int exposure_num, char* timestamp);

	//�������� DfGetDepthData
	//���ܣ� �ɼ��������ݲ����������ؽ��
	//�����������
	//��������� depth(���ͼ)
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
	DF_SDK_API int DfGetDepthData(unsigned short* depth);


	//�������� DfGetBrightnessData
	//���ܣ� �ɼ��������ݲ����������ؽ��
	//�����������
	//��������� brightness(����ͼ)
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
	DF_SDK_API int DfGetBrightnessData(unsigned char* brightness);

	//�������� DfGetPointcloudData
	//���ܣ� �ɼ��������ݲ����������ؽ��
	//�����������
	//��������� point_cloud(����)
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
	DF_SDK_API int DfGetPointcloudData(float* point_cloud);

	//�������� DfConnect
	//���ܣ� �Ͽ��������
	//��������� camera_id�����id��
	//��������� ��
	//����ֵ�� ���ͣ�int��:����0��ʾ�Ͽ��ɹ�;����-1��ʾ�Ͽ�ʧ��.
	DF_SDK_API int DfDisconnect(const char* camera_id);

	//�������� DfGetCalibrationParam
	//���ܣ� ��ȡ����궨����
	//��������� ��
	//��������� calibration_param������궨�����ṹ�壩
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�궨�����ɹ�;����-1��ʾ��ȡ�궨����ʧ��.
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