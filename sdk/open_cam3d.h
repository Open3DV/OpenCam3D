#pragma once

#ifdef _WIN32 
#define DF_SDK_API __declspec(dllexport)

#elif __linux
#define DF_SDK_API 
#endif
 


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
		//������䣬ֻ��ǰ5��
		float distortion[1 * 12];//<k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4>

	};


	//�������� DfConnect
	//���ܣ� �������
	//��������� camera_id�����ip��ַ��
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
	//��������� exposure_num���ع������������ֵΪ1Ϊ���ع⣬����1Ϊ���ع�ģʽ��������������gui�����ã�.
	//��������� timestamp(ʱ���)
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�ɼ����ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
	DF_SDK_API int DfCaptureData(int exposure_num, char* timestamp);

	//�������� DfGetDepthData
	//���ܣ� ��ȡ���ͼ
	//�����������
	//��������� depth(���ͼ)
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
	DF_SDK_API int DfGetDepthData(unsigned short* depth);


	//�������� DfGetBrightnessData
	//���ܣ� ��ȡ����ͼ
	//�����������
	//��������� brightness(����ͼ)
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
	DF_SDK_API int DfGetBrightnessData(unsigned char* brightness);

	//�������� DfGetHeightMapData
	//���ܣ� ��ȡУ������׼ƽ��ĸ߶�ӳ��ͼ
	//�����������
	//��������� height_map(�߶�ӳ��ͼ)
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
	DF_SDK_API int DfGetHeightMapData(float* height_map);

	//�������� DfGetStandardPlaneParam
	//���ܣ� ��ȡ��׼ƽ�����
	//�����������
	//��������� R(��ת����3*3)��T(ƽ�ƾ���3*1)
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
	DF_SDK_API int DfGetStandardPlaneParam(float* R,float* T);

	//�������� DfGetHeightMapDataBaseParam
	//���ܣ� ��ȡУ������׼ƽ��ĸ߶�ӳ��ͼ
	//���������R(��ת����)��T(ƽ�ƾ���)
	//��������� height_map(�߶�ӳ��ͼ)
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
	DF_SDK_API int DfGetHeightMapDataBaseParam(float* R, float* T, float* height_map);

	//�������� DfGetPointcloudData
	//���ܣ� ��ȡ����
	//�����������
	//��������� point_cloud(����)
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
	DF_SDK_API int DfGetPointcloudData(float* point_cloud);

	//�������� DfConnect
	//���ܣ� �Ͽ��������
	//��������� camera_id�����ip��ַ��
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
	//��������


	//�������� DfSetParamLedCurrent
	//���ܣ� ����LED����
	//��������� led������ֵ��
	//��������� ��
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�궨�����ɹ�;����-1��ʾ��ȡ�궨����ʧ��.
	DF_SDK_API int DfSetParamLedCurrent(int led);


	//�������� DfGetParamLedCurrent
	//���ܣ� ����LED����
	//��������� ��
	//��������� led������ֵ��
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�궨�����ɹ�;����-1��ʾ��ȡ�궨����ʧ��.
	DF_SDK_API int DfGetParamLedCurrent(int& led);

	//�������� DfSetParamHdr
	//���ܣ� ���ö��ع����������ع����Ϊ6�Σ�
	//��������� num���ع��������exposure_param[6]��6���ع������ǰnum����Ч��
	//��������� ��
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�궨�����ɹ�;����-1��ʾ��ȡ�궨����ʧ��.
	DF_SDK_API int DfSetParamHdr(int num,int exposure_param[6]);


	//�������� DfGetParamHdr
	//���ܣ� ���ö��ع����������ع����Ϊ6�Σ�
	//��������� ��
	//��������� num���ع��������exposure_param[6]��6���ع������ǰnum����Ч��
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�궨�����ɹ�;����-1��ʾ��ȡ�궨����ʧ��.
	DF_SDK_API int DfGetParamHdr(int& num, int exposure_param[6]);

	//�������� DfSetParamStandardPlaneExternal
	//���ܣ� ���û�׼ƽ������
	//���������R(��ת����3*3)��T(ƽ�ƾ���3*1)
	//��������� ��
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
	DF_SDK_API int DfSetParamStandardPlaneExternal(float* R, float* T);

	//�������� DfGetParamStandardPlaneExternal
	//���ܣ� ��ȡ��׼ƽ������
	//�����������
	//��������� R(��ת����3*3)��T(ƽ�ƾ���3*1)
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
	DF_SDK_API int DfGetParamStandardPlaneExternal(float* R, float* T);

	//�������� DfSetParamGenerateBrightness
	//���ܣ� ������������ͼ����
	//���������model(1:������ͼͬ�������ع⡢2�����������ع⡢3�������ⵥ���ع�)��exposure(����ͼ�ع�ʱ��)
	//��������� ��
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
	DF_SDK_API int DfSetParamGenerateBrightness(int model,float exposure);

	//�������� DfGetParamGenerateBrightness
	//���ܣ� ��ȡ��������ͼ����
	//��������� ��
	//���������model(1:������ͼͬ�������ع⡢2�����������ع⡢3�������ⵥ���ع�)��exposure(����ͼ�ع�ʱ��)
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
	DF_SDK_API int DfGetParamGenerateBrightness(int& model, float& exposure);

	//�������� DfSetParamCameraExposure
	//���ܣ� ��������ع�ʱ��
	//���������exposure(����ع�ʱ��)
	//��������� ��
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
	DF_SDK_API int DfSetParamCameraExposure(float exposure);

	//�������� DfGetParamCameraExposure
	//���ܣ� ��ȡ����ع�ʱ��
	//��������� ��
	//���������exposure(����ع�ʱ��)
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
	DF_SDK_API int DfGetParamCameraExposure(float &exposure);

	//�������� DfSetParamOffset
	//���ܣ� ���ò�������
	//���������offset(����ֵ)
	//��������� ��
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
	DF_SDK_API int DfSetParamOffset(float offset);

	//�������� DfGetParamOffset
	//���ܣ� ��ȡ��������
	//�����������
	//���������offset(����ֵ)
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
	DF_SDK_API int DfGetParamOffset(float& offset);

	//�������� DfSetParamMixedHdr
	//���ܣ� ���û�϶��ع����������ع����Ϊ6�Σ�
	//��������� num���ع��������exposure_param[6]��6���ع������ǰnum����Ч����led_param[6]��6��led���Ȳ�����ǰnum����Ч��
	//��������� ��
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�궨�����ɹ�;����-1��ʾ��ȡ�궨����ʧ��.
	DF_SDK_API int DfSetParamMixedHdr(int num, int exposure_param[6], int led_param[6]);

	//�������� DfGetParamMixedHdr
	//���ܣ� ��ȡ��϶��ع����������ع����Ϊ6�Σ�
	//��������� ��
	//��������� num���ع��������exposure_param[6]��6���ع������ǰnum����Ч����led_param[6]��6��led���Ȳ�����ǰnum����Ч��
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�궨�����ɹ�;����-1��ʾ��ȡ�궨����ʧ��.
	DF_SDK_API int DfGetParamMixedHdr(int &num, int exposure_param[6], int led_param[6]);
}


/**************************************************************************************/
//�������� DfConnectNet��������DfConnect:��ע������ӣ�DfConnectNet������ע��ֱ�����ӣ�
//���ܣ� �������
//��������� ip
//��������� ��
//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfConnectNet(const char* ip);

//�������� DfDisconnectNet
//���ܣ� �Ͽ����
//�����������
//��������� ��
//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfDisconnectNet();

//DF_SDK_API int DfGetCameraResolution(int* width, int* height);

//�������� DfGetCameraData
//���ܣ� ��ȡһ֡�������
//�����������
//��������� ��
//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfGetCameraData(
	short* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size,
	short* point_cloud, int point_cloud_buf_size,
	unsigned char* confidence, int confidence_buf_size);

//�������� GetBrightness
//���ܣ� ��ȡһ������ͼ����
//���������brightness_buf_size������ͼ�ߴ�sizeof(unsigned char) * width * height��
//���������brightness
//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int GetBrightness(unsigned char* brightness, int brightness_buf_size);

//�������� DfGetCameraRawData01
//���ܣ� �ɼ�һ������ͼ��һ��24���Ĳ���������ͼ
//���������raw_buf_size��24��8λͼ�ĳߴ磩
//���������raw
//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfGetCameraRawData01(unsigned char* raw, int raw_buf_size);

//�������� DfGetCameraRawData02
//���ܣ� �ɼ�һ������ͼ��һ��37����24���Ĳ���������ͼ+12��������������ͼ+һ������ͼ
//���������raw_buf_size��37��8λͼ�ĳߴ磩
//���������raw
//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfGetCameraRawData02(unsigned char* raw, int raw_buf_size);

//�������� DfGetCameraRawDataTest
//���ܣ� �ɼ�һ������ͼ 
//���������raw_buf_size��һ��ͼ�ĳߴ磩
//���������raw�ڴ�ָ��
//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfGetCameraRawDataTest(unsigned char* raw, int raw_buf_size);

//�������� DfGetCameraRawData03
//���ܣ� �ɼ�һ������ͼ��һ��31����24���Ĳ���������ͼ+6����ֱ�����������������ͼ+һ������ͼ
//���������raw_buf_size��31��8λͼ�ĳߴ磩
//���������raw
//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfGetCameraRawData03(unsigned char* raw, int raw_buf_size);

//�������� depthTransformPointcloud
//���ܣ� ���ͼת���ƽӿ�
//���������depth_map�����ͼ��
//���������point_cloud_map�����ƣ�
//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API bool depthTransformPointcloud(float* depth_map, float* point_cloud_map);

//�������� transformPointcloud
//���ܣ� ��������ϵת���ӿ�
//���������rotate����ת���󣩡�translation��ƽ�ƾ���
//���������point_cloud_map�����ƣ�
//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API bool transformPointcloud(float* org_point_cloud_map, float* transform_point_cloud_map, float* rotate, float* translation);

//�������� transformPointcloudInv
//���ܣ� ��������ϵ���任�����Խӿڣ�
DF_SDK_API bool transformPointcloudInv(float* point_cloud_map, float* rotate, float* translation);

//�������� DfGetPointCloud
//���ܣ� ��ȡ���ƽӿڣ�����24���Ĳ�����ͼ
//���������point_cloud_buf_size�������ڴ��С��
//���������point_cloud_map�����ƣ�
//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfGetPointCloud(float* point_cloud, int point_cloud_buf_size);

//�������� DfGetFrame01
//���ܣ� ��ȡһ֡���ݣ�����ͼ+���ͼ��������Raw01������ͼ
//���������depth_buf_size�����ͼ�ߴ磩��brightness_buf_size������ͼ�ߴ磩
//���������depth�����ͼ����brightness������ͼ��
//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfGetFrame01(float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size);

//�������� DfGetFrameHdr
//���ܣ� ��ȡһ֡���ݣ�����ͼ+���ͼ��������Raw03��λͼ��HDRģʽ
//���������depth_buf_size�����ͼ�ߴ磩��brightness_buf_size������ͼ�ߴ磩
//���������depth�����ͼ����brightness������ͼ��
//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfGetFrameHdr(float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size);

//�������� DfGetFrame03
//���ܣ� ��ȡһ֡���ݣ�����ͼ+���ͼ��������Raw03��λͼ
//���������depth_buf_size�����ͼ�ߴ磩��brightness_buf_size������ͼ�ߴ磩
//���������depth�����ͼ����brightness������ͼ��
//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfGetFrame03(float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size);

//�������� DfGetFrame04
//���ܣ� ��ȡһ֡���ݣ�����ͼ+���ͼ��������Raw04��λͼ
//���������depth_buf_size�����ͼ�ߴ磩��brightness_buf_size������ͼ�ߴ磩
//���������depth�����ͼ����brightness������ͼ��
//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfGetFrame04(float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size);

//�������� DfGetRepetitionFrame03
//���ܣ� ��ȡһ֡���ݣ�����ͼ+���ͼ��������Raw03��λͼ��6�����Ƶ�ͼ�ظ�count��
//���������count���ظ���������depth_buf_size�����ͼ�ߴ磩��brightness_buf_size������ͼ�ߴ磩
//���������depth�����ͼ����brightness������ͼ��
//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfGetRepetitionFrame03(int count,float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size);

//�������� DfGetCalibrationParam
//���ܣ���ȡ�궨�����ӿ�
//�����������
//���������calibration_param���궨������
//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfGetCalibrationParam(struct CameraCalibParam& calibration_param);

//�������� DfSetCalibrationParam
//���ܣ����ñ궨�����ӿ�
//���������calibration_param���궨������
//�����������
//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfSetCalibrationParam(const struct CameraCalibParam& calibration_param);

//�������� DfSetCalibrationLookTable
//���ܣ����ñ궨�����ӿ�
//���������calibration_param���궨������,rotate_x��rotate_y, rectify_r1, mapping
//�����������
//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfSetCalibrationLookTable(const struct CameraCalibParam& calibration_param,float* rotate_x,
	float* rotate_y,float* rectify_r1,float* mapping);

//�������� DfGetDeviceTemperature
//���ܣ���ȡ�豸�¶�
//�����������
//���������temperature�����϶ȣ�
//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfGetDeviceTemperature(float& temperature);

//�������� DfRegisterOnDropped
//���ܣ�ע������ص�����
//�����������
//���������p_function���ص�������
//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfRegisterOnDropped(int (*p_function)(void*));

//�������� DfGetCalibrationParam
//���ܣ���ȡ������ò����ӿ�
//���������config_param�����ò�����
//�����������
//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfGetSystemConfigParam(struct SystemConfigParam& config_param);

//�������� DfGetCalibrationParam
//���ܣ�����������ò����ӿ�
//���������config_param�����ò�����
//�����������
//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfSetSystemConfigParam(const struct SystemConfigParam& config_param);

//��������  DfEnableCheckerboard
//���ܣ�    �򿪹����ͶӰ���̸�
//�����������
//���������nano�¶�1
//����ֵ��  ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfEnableCheckerboard(float& temperature);

//��������  DfDisableCheckerboard
//���ܣ�    �رչ��ͶӰ
//�����������
//���������nano�¶�2
//����ֵ��  ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfDisableCheckerboard(float& temperature);

//��������  DfLoadPatternData
//���ܣ�    �ӹ�����ư��ȡԤ�����õ�42��ͶӰͼƬ����--Pattern Data
//�����������ȡPattern Data�Ļ�������С����ȡPattern Data�Ļ�������ַ
//���������Pattern Data����
//����ֵ��  ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfLoadPatternData(int buildDataSize, char* LoadBuffer);

//��������  DfProgramPatternData
//���ܣ�    ��������ư�DLP��ͶӰͼƬ����--Pattern Data����PC��д�������ư�FLASH
//���������Ҫд������ݰ���ַ���ض����������ݰ���ַ�����ݰ���С
//����������ض�����������
//����ֵ��  ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfProgramPatternData(char* org_buffer, char* back_buffer, unsigned int pattern_size);

//��������  DfGetNetworkBandwidth
//���ܣ�    ��ȡ��·���������
//�����������
//�����������������С
//����ֵ��  ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfGetNetworkBandwidth(int &speed);

//��������  DfGetFirmwareVersion
//���ܣ�    ��ȡ�̼��汾
//����������汾�Ż�������ַ������������
//����������汾��
//����ֵ��  ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfGetFirmwareVersion(char* pVersion, int length);

//��������  DfGetCameraVersion
//���ܣ�    ��ȡ����ͺ�
//�����������
//����������ͺţ�800��1800��
//����ֵ��  ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfGetCameraVersion(int& version);