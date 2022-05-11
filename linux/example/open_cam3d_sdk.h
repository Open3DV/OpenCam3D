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
	DF_SDK_API int DfGetStandardPlaneParam(float* R, float* T);

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
	DF_SDK_API int DfSetParamHdr(int num, int exposure_param[6]);


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
	DF_SDK_API int DfSetParamGenerateBrightness(int model, float exposure);

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
	DF_SDK_API int DfGetParamCameraExposure(float& exposure);

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
	DF_SDK_API int DfGetParamMixedHdr(int& num, int exposure_param[6], int led_param[6]);
}

