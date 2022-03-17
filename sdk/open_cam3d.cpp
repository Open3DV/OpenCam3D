#include "open_cam3d.h"
#include "socket_tcp.h"
#include <assert.h>
#include <iostream>
#include <thread>
#include "utils.h"
#include "../firmware/easylogging++.h"
#include<chrono>
#include<ctime>
#include <time.h>
#include <stddef.h> 
#include "../test/triangulation.h"
#include "../firmware/protocol.h" 
#include "../firmware/system_config_settings.h"

using namespace std;
using namespace std::chrono;

/**********************************************************************************************************************/
//socket
INITIALIZE_EASYLOGGINGPP

const int image_width = 1920;
const int image_height = 1200;
const int image_size = image_width * image_height;
bool connected = false;
long long token = 0;
//const char* camera_id_;
std::string camera_id_;
std::thread heartbeat_thread;

extern SOCKET g_sock_heartbeat;
extern SOCKET g_sock;

int (*p_OnDropped)(void*) = 0;

/**************************************************************************************************************/


struct CameraCalibParam calibration_param_;
bool connected_flag_ = false;

int camera_width_ = 1920;
int camera_height_ = 1200;

const char* camera_ip_ = "";


int depth_buf_size_ = 0;
int pointcloud_buf_size_ = 0;
int brightness_bug_size_ = 0;
float* point_cloud_buf_ = NULL;
float* trans_point_cloud_buf_ = NULL;
bool transform_pointcloud_flag_ = false;
float* depth_buf_ = NULL;
unsigned char* brightness_buf_ = NULL;


/**************************************************************************************************************************/

std::time_t getTimeStamp(long long& msec)
{
	std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
	auto tmp = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
	seconds sec = duration_cast<seconds>(tp.time_since_epoch());


	std::time_t timestamp = tmp.count();

	msec = tmp.count() - sec.count() * 1000;
	//std::time_t timestamp = std::chrono::system_clock::to_time_t(tp);
	return timestamp;
}

std::tm* gettm(long long timestamp)
{
	auto milli = timestamp + (long long)8 * 60 * 60 * 1000; //�˴�ת��Ϊ����������ʱ�䣬���������ʱ����Ҫ�������޸�
	auto mTime = std::chrono::milliseconds(milli);
	auto tp = std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>(mTime);
	auto tt = std::chrono::system_clock::to_time_t(tp);
	std::tm* now = std::gmtime(&tt);
	//printf("%4d��%02d��%02d�� %02d:%02d:%02d\n", now->tm_year + 1900, now->tm_mon + 1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec);
	return now;
}


std::string get_timestamp()
{

	long long msec = 0;
	char time_str[7][16];
	auto t = getTimeStamp(msec);
	//std::cout << "Millisecond timestamp is: " << t << std::endl;
	auto time_ptr = gettm(t);
	sprintf(time_str[0], "%02d", time_ptr->tm_year + 1900); //�·�Ҫ��1
	sprintf(time_str[1], "%02d", time_ptr->tm_mon + 1); //�·�Ҫ��1
	sprintf(time_str[2], "%02d", time_ptr->tm_mday);//��
	sprintf(time_str[3], "%02d", time_ptr->tm_hour);//ʱ
	sprintf(time_str[4], "%02d", time_ptr->tm_min);// ��
	sprintf(time_str[5], "%02d", time_ptr->tm_sec);//ʱ
	sprintf(time_str[6], "%02lld", msec);// ��
	//for (int i = 0; i < 7; i++)
	//{
	//	std::cout << "time_str[" << i << "] is: " << time_str[i] << std::endl;
	//}

	std::string timestamp = "";

	timestamp += time_str[0];
	timestamp += "-";
	timestamp += time_str[1];
	timestamp += "-";
	timestamp += time_str[2];
	timestamp += " ";
	timestamp += time_str[3];
	timestamp += ":";
	timestamp += time_str[4];
	timestamp += ":";
	timestamp += time_str[5];
	timestamp += ",";
	timestamp += time_str[6];

	//std::cout << timestamp << std::endl;

	return timestamp;
}


/***************************************************************************************************************************/
//�������
int on_dropped(void* param)
{
	std::cout << "Network dropped!" << std::endl;
	return 0;
}


bool transformPointcloudInv(float* point_cloud_map, float* rotate, float* translation)
{

	int point_num = camera_height_ * camera_width_;

	int nr = camera_height_;
	int nc = camera_width_;

#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{

		for (int c = 0; c < nc; c++)
		{

			int offset = r * camera_width_ + c;

			float x = point_cloud_map[3 * offset + 0] + translation[0];
			float y = point_cloud_map[3 * offset + 1] + translation[1];
			float z = point_cloud_map[3 * offset + 2] + translation[2];
			 
			point_cloud_map[3 * offset + 0] = rotate[0] * x + rotate[1] * y + rotate[2] * z;
			point_cloud_map[3 * offset + 1] = rotate[3] * x + rotate[4] * y + rotate[5] * z;
			point_cloud_map[3 * offset + 2] = rotate[6] * x + rotate[7] * y + rotate[8] * z;
  
		}

	}


	return true;
}

 bool transformPointcloud(float* org_point_cloud_map, float* transform_point_cloud_map, float* rotate, float* translation)
{


	 int point_num = camera_height_ * camera_width_;

	 int nr = camera_height_;
	 int nc = camera_width_;

	#pragma omp parallel for
	 for (int r = 0; r < nr; r++)
	 {

		 for (int c = 0; c < nc; c++)
		 {
 
			 int offset = r * camera_width_ + c;

			 float x = org_point_cloud_map[3 * offset + 0];
			 float y = org_point_cloud_map[3 * offset + 1];
			 float z = org_point_cloud_map[3 * offset + 2];
			  
			 //if (z > 0)
			 //{
				 transform_point_cloud_map[3 * offset + 0] = rotate[0] * x + rotate[1] * y + rotate[2] * z + translation[0];
				 transform_point_cloud_map[3 * offset + 1] = rotate[3] * x + rotate[4] * y + rotate[5] * z + translation[1];
				 transform_point_cloud_map[3 * offset + 2] = rotate[6] * x + rotate[7] * y + rotate[8] * z + translation[2];
				  
			 //}
			 //else
			 //{
				// point_cloud_map[3 * offset + 0] = 0;
				// point_cloud_map[3 * offset + 1] = 0;
				// point_cloud_map[3 * offset + 2] = 0;
			 //}


		 }

	 }


	 return true;
}

bool depthTransformPointcloud(float* depth_map, float* point_cloud_map)
{


	//double camera_fx = camera_intrinsic_.at<double>(0, 0);
	//double camera_fy = camera_intrinsic_.at<double>(1, 1); 
	//double camera_cx = camera_intrinsic_.at<double>(0, 2);
	//double camera_cy = camera_intrinsic_.at<double>(1, 2);


	float camera_fx = calibration_param_.camera_intrinsic[0];
	float camera_fy = calibration_param_.camera_intrinsic[4];

	float camera_cx = calibration_param_.camera_intrinsic[2];
	float camera_cy = calibration_param_.camera_intrinsic[5];


	float k1 = calibration_param_.camera_distortion[0];
	float k2 = calibration_param_.camera_distortion[1];
	float p1 = calibration_param_.camera_distortion[2];
	float p2 = calibration_param_.camera_distortion[3];
	float k3 = calibration_param_.camera_distortion[4];

	//LOG(INFO) << "camera_fx: " << camera_fx << std::endl;
	//LOG(INFO) << "camera_fy: " << camera_fy << std::endl;
	//LOG(INFO) << "camera_cx: " << camera_cx << std::endl;
	//LOG(INFO) << "camera_cy: " << camera_cy << std::endl;
	//LOG(INFO) << "camera_width_: " << camera_width_ << std::endl;
	//LOG(INFO) << "camera_height_: " << camera_height_ << std::endl;



	int point_num = camera_height_ * camera_width_;

	int nr = camera_height_;
	int nc = camera_width_;

#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{

		for (int c = 0; c < nc; c++)
		{
			double undistort_x = c;
			double undistort_y = r;

			//undistortPoint(c, r, camera_fx, camera_fy,
			//	camera_cx, camera_cy, k1, k2, k3, p1, p2, undistort_x, undistort_y);

			int offset = r * camera_width_ + c;
			if (depth_map[offset] > 0)
			{
				point_cloud_map[3 * offset + 0] = (undistort_x - camera_cx) * depth_map[offset] / camera_fx;
				point_cloud_map[3 * offset + 1] = (undistort_y - camera_cy) * depth_map[offset] / camera_fy;
				point_cloud_map[3 * offset + 2] = depth_map[offset];


			}
			else
			{
				point_cloud_map[3 * offset + 0] = 0;
				point_cloud_map[3 * offset + 1] = 0;
				point_cloud_map[3 * offset + 2] = 0;
			}


		}

	}


	return true;
}



/**************************************************************************************************************************/


//�������� DfConnect
//���ܣ� �������
//��������� camera_id�����id��
//��������� ��
//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
DF_SDK_API int DfConnect(const char* camera_id)
{
	 
	DfRegisterOnDropped(on_dropped);

	
	int ret = DfConnectNet(camera_id);
	if (ret == DF_FAILED)
	{
		return -1;
	}

	ret = DfGetCalibrationParam(calibration_param_);

	if (ret == DF_FAILED)
	{
		return -1;
	}

	//LOG(INFO) << "fx: " << calibration_param_.camera_intrinsic[0];
	//LOG(INFO) << "fy: " << calibration_param_.camera_intrinsic[4];


	int width, height;
	ret = DfGetCameraResolution(&width, &height);


	if (ret == DF_FAILED)
	{
		return -1;
	}

	camera_width_ = width;
	camera_height_ = height;



	//��ʼ��

	camera_ip_ = camera_id;
	connected_flag_ = true;

	int image_size = camera_width_ * camera_height_;

	depth_buf_size_ = image_size * 1 * 4;
	depth_buf_ = (float*)(new char[depth_buf_size_]);

	pointcloud_buf_size_ = depth_buf_size_ * 3;
	point_cloud_buf_ = (float*)(new char[pointcloud_buf_size_]);

	trans_point_cloud_buf_ = (float*)(new char[pointcloud_buf_size_]);

	brightness_bug_size_ = image_size;
	brightness_buf_ = new unsigned char[brightness_bug_size_];


	LOG(INFO) << "Connect Camera: " << camera_ip_;

	return 0;
}

//�������� DfGetCameraResolution
//���ܣ� ��ȡ����ֱ���
//��������� ��
//��������� width(ͼ���)��height(ͼ���)
//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�����ɹ�;����-1��ʾ��ȡ����ʧ��.
DF_SDK_API int DfGetCameraResolution(int* width, int* height)
{
	if (!connected)
	{
		return -1;
	}

	*width = camera_width_;
	*height = camera_height_;


	return 0;
}

//�������� DfCaptureData
//���ܣ� �ɼ�һ֡���ݲ�����������״̬
//��������� exposure_num���ع������������1��Ϊ���ع�ģʽ
//��������� timestamp(ʱ���)
//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�ɼ����ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
DF_SDK_API int DfCaptureData(int exposure_num, char* timestamp)
{

	//LOG(INFO) << "Debug Connect:";

	//DfnRegisterOnDropped(on_dropped);

	//int ret = DfnConnect(camera_ip_);
	//if (ret == DF_FAILED)
	//{
	//	return 0;
	//}

	bool ret = -1;

	if (exposure_num > 1)
	{
		LOG(TRACE) << " Get Frame HDR:";
		ret = DfGetFrameHdr(depth_buf_, depth_buf_size_, brightness_buf_, brightness_bug_size_);
	}
	else
	{

		LOG(TRACE) << " Get Frame03:";
		ret = DfGetFrame03(depth_buf_, depth_buf_size_, brightness_buf_, brightness_bug_size_);
	}


	//LOG(TRACE) << "Debug Get Temperature:"; 
	//float temperature_value = 0; 
	//ret = DfnGetDeviceTemperature(temperature_value); 
	//LOG(TRACE) << "Temperature: "<< temperature_value;

	//LOG(TRACE) << "Debug Disconnect:";
	//DfnDisconnect();


	//LOG(INFO) << "Debug Disconnect Finished";

	std::string time = get_timestamp();
	for (int i = 0; i < time.length(); i++)
	{
		timestamp[i] = time[i];
	}

	transform_pointcloud_flag_ = false;


	return 0;
}

//�������� DfGetDepthData
//���ܣ� �ɼ��������ݲ����������ؽ��
//�����������
//��������� depth(���ͼ)
//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
DF_SDK_API int DfGetDepthData(unsigned short* depth)
{
	if (!connected_flag_)
	{
		return -1;
	}


	LOG(INFO) << "Trans Depth:";
	int point_num = camera_height_ * camera_width_;

	int nr = camera_height_;
	int nc = camera_width_;

#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{

		for (int c = 0; c < nc; c++)
		{
			int offset = r * camera_width_ + c;

			if (depth_buf_[offset] > 0)
			{
				depth[offset] = depth_buf_[offset] * 10 + 0.5;
			}
			else
			{
				depth[offset] = 0;
			}

		}


	}

	LOG(INFO) << "Get Depth!";

	return 0;
}


//�������� DfGetBrightnessData
//���ܣ� �ɼ��������ݲ����������ؽ��
//�����������
//��������� brightness(����ͼ)
//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
DF_SDK_API int DfGetBrightnessData(unsigned char* brightness)
{
	if (!connected_flag_)
	{
		return -1;
	}


	LOG(INFO) << "Trans Brightness:";

	memcpy(brightness, brightness_buf_, brightness_bug_size_);

	//brightness = brightness_buf_;

	LOG(INFO) << "Get Brightness!";

	return 0;
}

//�������� DfGetStandardPlaneParam
//���ܣ� ��ȡ��׼ƽ�����
//�����������
//��������� R(��ת����3*3)��T(ƽ�ƾ���3*1)
//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
DF_SDK_API int DfGetStandardPlaneParam(float* R, float* T)
{

	LOG(INFO) << "DfGetStandardPlaneParam";
 
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}

	int param_buf_size = 12 * 4;
	float* plane_param = new float[param_buf_size];

	ret = send_command(DF_CMD_GET_STANDARD_PLANE_PARAM, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "token checked ok";
		LOG(INFO) << "receiving buffer, param_buf_size=" << param_buf_size;
		ret = recv_buffer((char*)plane_param, param_buf_size, g_sock);
		LOG(INFO) << "plane param received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		} 
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get frame rejected";
		close_socket(g_sock);
		return DF_FAILED;
	}

	LOG(INFO) << "Get plane param success";
	close_socket(g_sock);


	memcpy(R, plane_param, 9*4);
	memcpy(T, plane_param+9, 3 * 4);

	delete [] plane_param;

	return DF_SUCCESS;
	  
}

//�������� DfGetHeightMapDataBaseParam
//���ܣ� ��ȡУ������׼ƽ��ĸ߶�ӳ��ͼ
//���������R(��ת����)��T(ƽ�ƾ���)
//��������� height_map(�߶�ӳ��ͼ)
//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
DF_SDK_API int DfGetHeightMapDataBaseParam(float* R, float* T , float* height_map)
{
	if (!connected_flag_)
	{
		return -1;
	} 
	//struct SystemConfigParam system_config_param;
	//int ret_code = DfGetSystemConfigParam(system_config_param);
	//if (0 != ret_code)
	//{
	//	std::cout << "Get Param Error;";
	//	return -1;
	//}

	LOG(INFO) << "Transform Pointcloud:";

	if (!transform_pointcloud_flag_)
	{
		depthTransformPointcloud((float*)depth_buf_, (float*)point_cloud_buf_);
		transform_pointcloud_flag_ = true;
	}

	//memcpy(trans_point_cloud_buf_, point_cloud_buf_, pointcloud_buf_size_);
	transformPointcloud((float*)point_cloud_buf_,(float*)trans_point_cloud_buf_, R, T);


	int nr = camera_height_;
	int nc = camera_width_;
	#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{
		for (int c = 0; c < nc; c++)
		{
			int offset = r * camera_width_ + c;
			if (depth_buf_[offset] > 0)
			{
				height_map[offset] = trans_point_cloud_buf_[offset * 3 + 2];
			}
			else
			{
				height_map[offset] = NULL;
			}

		}


	}


	LOG(INFO) << "Get Height Map!";

	return 0;
}

//�������� DfGetHeightMapData
//���ܣ� �ɼ��������ݲ����������ؽ��
//�����������
//��������� height_map(�߶�ӳ��ͼ)
//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
DF_SDK_API int DfGetHeightMapData(float* height_map)
{
	if (!connected_flag_)
	{
		return -1;
	}


	struct SystemConfigParam system_config_param;
	int ret_code = DfGetSystemConfigParam(system_config_param);
	if (0 != ret_code)
	{
		std::cout << "Get Param Error;";
		return -1;
	}

	LOG(INFO) << "Transform Pointcloud:"; 

	if (!transform_pointcloud_flag_)
	{
		depthTransformPointcloud((float*)depth_buf_, (float*)point_cloud_buf_);
		transform_pointcloud_flag_ = true;
	}

	//memcpy(trans_point_cloud_buf_, point_cloud_buf_, pointcloud_buf_size_);
	transformPointcloud((float*)point_cloud_buf_,(float*)trans_point_cloud_buf_, system_config_param.standard_plane_external_param, &system_config_param.standard_plane_external_param[9]);
	 

	int nr = camera_height_;
	int nc = camera_width_; 
	#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{ 
		for (int c = 0; c < nc; c++)
		{
			int offset = r * camera_width_ + c; 
			if (depth_buf_[offset] > 0)
			{
				height_map[offset] = trans_point_cloud_buf_[offset*3+2];
			}
			else
			{
				height_map[offset] = NULL;
			}

		}


	}


	LOG(INFO) << "Get Height Map!";

	return 0;
}

//�������� DfGetPointcloudData
//���ܣ� �ɼ��������ݲ����������ؽ��
//�����������
//��������� point_cloud(����)
//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
DF_SDK_API int DfGetPointcloudData(float* point_cloud)
{
	if (!connected_flag_)
	{
		return -1;
	}

	LOG(INFO) << "Transform Pointcloud:";

	if (!transform_pointcloud_flag_)
	{
		depthTransformPointcloud(depth_buf_, point_cloud_buf_);
		transform_pointcloud_flag_ = true;
	} 

	memcpy(point_cloud, point_cloud_buf_, pointcloud_buf_size_);
	 

	LOG(INFO) << "Get Pointcloud!";

	return 0;
}

//�������� DfConnect
//���ܣ� �Ͽ��������
//��������� camera_id�����id��
//��������� ��
//����ֵ�� ���ͣ�int��:����0��ʾ�Ͽ��ɹ�;����-1��ʾ�Ͽ�ʧ��.
DF_SDK_API int DfDisconnect(const char* camera_id)
{
	if (!connected_flag_)
	{
		return -1;
	}


	DfDisconnectNet();

	delete[] depth_buf_;
	delete[] brightness_buf_;
	delete[] point_cloud_buf_;
	delete[] trans_point_cloud_buf_;

	connected_flag_ = false;

	return 0;
}

//�������� DfGetCalibrationParam
//���ܣ� ��ȡ����궨����
//��������� ��
//��������� calibration_param������궨�����ṹ�壩
//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�궨�����ɹ�;����-1��ʾ��ȡ�궨����ʧ��.
DF_SDK_API int DfGetCalibrationParam(struct CalibrationParam* calibration_param)
{
	if (!connected_flag_)
	{
		return -1;
	}

	//calibration_param = &calibration_param_;

	for (int i = 0; i < 9; i++)
	{
		calibration_param->intrinsic[i] = calibration_param_.camera_intrinsic[i];
	}

	for (int i = 0; i < 5; i++)
	{
		calibration_param->distortion[i] = calibration_param_.camera_distortion[i];
	}

	for (int i = 5; i < 12; i++)
	{
		calibration_param->distortion[i] = 0;
	}

	float extrinsic[4 * 4] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

	for (int i = 0; i < 16; i++)
	{
		calibration_param->extrinsic[i] = extrinsic[i];
	}


	return 0;
}


/***************************************************************************************************************************************************************/





/*******************************************************************************************************************/

int HeartBeat()
{
	LOG(TRACE) << "heart beat: ";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock_heartbeat);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock_heartbeat);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_HEARTBEAT, g_sock_heartbeat);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to send disconnection cmd";
		close_socket(g_sock_heartbeat);
		return DF_FAILED;
	}
	ret = send_buffer((char*)&token, sizeof(token), g_sock_heartbeat);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to send token";
		close_socket(g_sock_heartbeat);
		return DF_FAILED;
	}
	int command;
	ret = recv_command(&command, g_sock_heartbeat);
	if (command == DF_CMD_OK)
	{
		ret = DF_SUCCESS;
	}
	else if (command == DF_CMD_REJECT)
	{
		ret = DF_FAILED;
	}
	else
	{
		assert(0);
	}
	close_socket(g_sock_heartbeat);
	return ret;
}

int HeartBeat_loop()
{
	while (connected)
	{
		int ret = HeartBeat();
		if (ret == DF_FAILED)
		{
			connected = false;
			p_OnDropped(0);
		}
		for (int i = 0; i < 100; i++)
		{
			if (!connected)
			{
				break;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
	return 0;
}


DF_SDK_API int DfConnectNet(const char* ip)
{
	camera_id_ = ip;
	LOG(INFO) << "start connection: " <<ip;
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	 

	LOG(INFO) << "sending connection cmd";
	ret = send_command(DF_CMD_CONNECT, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(INFO) << "Failed to send connection cmd";
		close_socket(g_sock);
		return DF_FAILED;
	}
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_SUCCESS)
	{
		if (command == DF_CMD_OK)
		{
			LOG(INFO) << "Recieved connection ok" ;
			ret = recv_buffer((char*)&token, sizeof(token), g_sock);
			if (ret == DF_SUCCESS)
			{
				connected = true;
				LOG(INFO) << "token: " << token;
				close_socket(g_sock);
				if (heartbeat_thread.joinable())
				{
					heartbeat_thread.join();
				}
				heartbeat_thread = std::thread(HeartBeat_loop);
				return DF_SUCCESS;
			}
		}
		else if(command == DF_CMD_REJECT)
		{
			LOG(INFO) << "connection rejected";
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	return DF_FAILED;
}

DF_SDK_API int DfDisconnectNet()
{
	LOG(INFO) <<"token "<<token<< " try to disconnection";

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_DISCONNECT, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(INFO) << "Failed to send disconnection cmd";
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	if (ret == DF_FAILED)
	{
		LOG(INFO) << "Failed to send token";
		close_socket(g_sock);
		return DF_FAILED;
	}
	int command;
	ret = recv_command(&command, g_sock);

	connected = false;
	token = 0;

	if (heartbeat_thread.joinable())
	{
		heartbeat_thread.join();
	}

	LOG(INFO) << "Camera disconnected";
	return close_socket(g_sock);
}

  

DF_SDK_API int GetBrightness(unsigned char* brightness, int brightness_buf_size)
{
	LOG(INFO) << "GetBrightness";
	assert(brightness_buf_size >= image_size * sizeof(unsigned char));
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	//std::cout << "1" << std::endl;
	ret = send_command(DF_CMD_GET_BRIGHTNESS, g_sock);
	//std::cout << "send token " << token<< std::endl;
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "token checked ok" ;
		ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get brightness rejected";
		close_socket(g_sock);
		return DF_FAILED;
	}

	LOG(INFO) << "Get brightness success";
	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfGetCameraData(
	short* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size,
	short* point_cloud, int point_cloud_buf_size,
	unsigned char* confidence, int confidence_buf_size)
{
	int ret = DF_SUCCESS;
	if (depth)
	{
		assert(depth_buf_size >= image_size * sizeof(short));
		send_command(DF_CMD_GET_DEPTH, g_sock);
		ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
		if (ret == DF_FAILED)
		{
			return DF_FAILED;
		}
	}
	if (brightness)
	{
		GetBrightness(brightness, brightness_buf_size);
	}

	if (point_cloud)
	{
		assert(point_cloud_buf_size >= image_size * sizeof(short)*3);
		send_command(DF_CMD_GET_POINTCLOUD, g_sock);
		ret = recv_buffer((char*)point_cloud, point_cloud_buf_size, g_sock);
		if (ret == DF_FAILED)
		{
			return DF_FAILED;
		}
	}
	if (confidence)
	{
		assert(confidence_buf_size >= image_size * sizeof(unsigned char));
		send_command(DF_CMD_GET_CONFIDENCE, g_sock);
		ret = recv_buffer((char*)confidence, confidence_buf_size, g_sock);
		if (ret == DF_FAILED)
		{
			return DF_FAILED;
		}
	}
	return DF_SUCCESS;
}

DF_SDK_API int DfGetFrameHdr(float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size)
{
	LOG(INFO) << "GetFrameHdr";
	assert(depth_buf_size == image_size * sizeof(float) * 1);
	assert(brightness_buf_size == image_size * sizeof(char) * 1);
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_FRAME_HDR, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "token checked ok";
		LOG(INFO) << "receiving buffer, depth_buf_size=" << depth_buf_size;
		ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
		LOG(INFO) << "depth received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer, brightness_buf_size=" << brightness_buf_size;
		ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
		LOG(INFO) << "brightness received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}



		//brightness = (unsigned char*)depth + depth_buf_size;
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get frame rejected";
		close_socket(g_sock);
		return DF_FAILED;
	}

	LOG(INFO) << "Get frame success";
	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfGetRepetitionFrame03(int count, float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size)
{
	LOG(INFO) << "GetRepetitionFrame03";
	assert(depth_buf_size == image_size * sizeof(float) * 1);
	assert(brightness_buf_size == image_size * sizeof(char) * 1);
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}


	ret = send_command(DF_CMD_GET_REPETITION_FRAME_03, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		ret = send_buffer((char*)(&count), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "token checked ok";
		LOG(INFO) << "receiving buffer, depth_buf_size=" << depth_buf_size;
		ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
		LOG(INFO) << "depth received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer, brightness_buf_size=" << brightness_buf_size;
		ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
		LOG(INFO) << "brightness received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		//brightness = (unsigned char*)depth + depth_buf_size;
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get frame rejected";
		close_socket(g_sock);
		return DF_FAILED;
	}

	LOG(INFO) << "Get frame success";
	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfGetFrame03(float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size)
{
	LOG(INFO) << "GetFrame03";
	assert(depth_buf_size == image_size * sizeof(float) * 1);
	assert(brightness_buf_size == image_size * sizeof(char) * 1);
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	 

	ret = send_command(DF_CMD_GET_FRAME_03, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "token checked ok";
		LOG(INFO) << "receiving buffer, depth_buf_size=" << depth_buf_size;
		ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
		LOG(INFO) << "depth received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer, brightness_buf_size=" << brightness_buf_size;
		ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
		LOG(INFO) << "brightness received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		//brightness = (unsigned char*)depth + depth_buf_size;
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get frame rejected";
		close_socket(g_sock);
		return DF_FAILED;
	}

	LOG(INFO) << "Get frame success";
	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfGetFrame01(float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size)
{
	LOG(INFO) << "GetFrame01";
	assert(depth_buf_size == image_size * sizeof(float) * 1);
	assert(brightness_buf_size == image_size * sizeof(char) * 1);
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_FRAME_01, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "token checked ok";
		LOG(INFO) << "receiving buffer, depth_buf_size=" << depth_buf_size;
		ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
		LOG(INFO) << "depth received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer, brightness_buf_size=" << brightness_buf_size;
		ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
		LOG(INFO) << "brightness received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}



		//brightness = (unsigned char*)depth + depth_buf_size;
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get frame rejected";
		close_socket(g_sock);
		return DF_FAILED;
	}

	LOG(INFO) << "Get frame success";
	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfGetPointCloud(float* point_cloud, int point_cloud_buf_size)
{
	LOG(INFO) << "GetPointCloud";
	assert(point_cloud_buf_size == image_size * sizeof(float) * 3);
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_POINTCLOUD, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "token checked ok";
		LOG(INFO) << "receiving buffer, point_cloud_buf_size=" << point_cloud_buf_size;
		ret = recv_buffer((char*)point_cloud, point_cloud_buf_size, g_sock);
		LOG(INFO) << "point_cloud received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get point_cloud rejected";
		close_socket(g_sock);
		return DF_FAILED;
	}

	LOG(INFO) << "Get point_cloud success";
	close_socket(g_sock);
	return DF_SUCCESS;
}


DF_SDK_API int DfGetCameraRawData03(unsigned char* raw, int raw_buf_size)
{
	if (raw)
	{
		LOG(INFO) << "GetRaw03";
		assert(raw_buf_size >= image_size * sizeof(unsigned char) * 36);
		int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_RAW_03, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (command == DF_CMD_OK)
		{
			LOG(INFO) << "token checked ok";
			LOG(INFO) << "receiving buffer, raw_buf_size=" << raw_buf_size;
			ret = recv_buffer((char*)raw, raw_buf_size, g_sock);
			LOG(INFO) << "images received";
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			LOG(INFO) << "Get raw rejected";
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "Get raw success";
		close_socket(g_sock);
		return DF_SUCCESS;
	}
	return DF_FAILED;
}

DF_SDK_API int DfGetCameraRawData02(unsigned char* raw, int raw_buf_size)
{
	if (raw)
	{
		LOG(INFO) << "GetRawTest";
		assert(raw_buf_size >= image_size * sizeof(unsigned char) * 37);
		int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_RAW_TEST, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (command == DF_CMD_OK)
		{
			LOG(INFO) << "token checked ok";
			LOG(INFO) << "receiving buffer, raw_buf_size=" << raw_buf_size;
			ret = recv_buffer((char*)raw, raw_buf_size, g_sock);
			LOG(INFO) << "images received";
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			LOG(INFO) << "Get raw rejected";
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "Get raw success";
		close_socket(g_sock);
		return DF_SUCCESS;
	}
	return DF_FAILED;
}

DF_SDK_API int DfGetCameraRawDataTest(unsigned char* raw, int raw_buf_size)
{
	if (raw)
	{
		LOG(INFO) << "GetRawTest";
		assert(raw_buf_size >= image_size * sizeof(unsigned char) * 37);
		int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_RAW_TEST, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (command == DF_CMD_OK)
		{
			LOG(INFO) << "token checked ok";
			LOG(INFO) << "receiving buffer, raw_buf_size=" << raw_buf_size;
			ret = recv_buffer((char*)raw, raw_buf_size, g_sock);
			LOG(INFO) << "images received";
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			LOG(INFO) << "Get raw rejected";
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "Get raw success";
		close_socket(g_sock);
		return DF_SUCCESS;
	}
	return DF_FAILED;
}



DF_SDK_API int DfGetCameraRawData01(unsigned char* raw, int raw_buf_size)
{
	if (raw)
	{
		LOG(INFO) << "Get Raw 01" ;
		assert(raw_buf_size >= image_size * sizeof(unsigned char) * 72);
		int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_RAW, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (command == DF_CMD_OK)
		{
			LOG(INFO) << "token checked ok";
			LOG(INFO) << "receiving buffer, raw_buf_size=" << raw_buf_size;
			ret = recv_buffer((char*)raw, raw_buf_size, g_sock);
			LOG(INFO) << "images received";
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			LOG(INFO) << "Get raw rejected";
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "Get raw success";
		close_socket(g_sock);
		return DF_SUCCESS;
	}
	return DF_FAILED;
}

DF_SDK_API int DfGetDeviceTemperature(float& temperature)
{
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_TEMPERATURE, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&temperature), sizeof(temperature), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock); 
		return DF_FAILED;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

// --------------------------------------------------------------
// -- Enable and disable checkerboard, by wantong, 2022-01-27
DF_SDK_API int DfEnableCheckerboard(float& temperature)
{
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_ENABLE_CHECKER_BOARD, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&temperature), sizeof(temperature), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfDisableCheckerboard(float& temperature)
{
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_DISABLE_CHECKER_BOARD, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&temperature), sizeof(temperature), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}
// --------------------------------------------------------------
DF_SDK_API int DfLoadPatternData(int buildDataSize, char *LoadBuffer)
{
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_LOAD_PATTERN_DATA, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		ret = send_buffer((char*)(&buildDataSize), sizeof(buildDataSize), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = recv_buffer(LoadBuffer, buildDataSize, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfProgramPatternData(char *org_buffer, char *back_buffer, unsigned int pattern_size)
{
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_PROGRAM_PATTERN_DATA, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		ret = send_buffer((char*)(&pattern_size), sizeof(pattern_size), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = send_buffer(org_buffer, pattern_size, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = recv_buffer(back_buffer, pattern_size, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}
// --------------------------------------------------------------
DF_SDK_API int DfGetNetworkBandwidth(int& speed)
{
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_NETWORK_BANDWIDTH, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&speed), sizeof(speed), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}
// --------------------------------------------------------------
DF_SDK_API int DfGetFirmwareVersion(char* pVersion, int length)
{
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_FIRMWARE_VERSION, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		ret = recv_buffer(pVersion, length, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}
// --------------------------------------------------------------
DF_SDK_API int DfGetSystemConfigParam(struct SystemConfigParam& config_param)
{
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_SYSTEM_CONFIG_PARAMETERS, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&config_param), sizeof(config_param), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfSetSystemConfigParam(const struct SystemConfigParam& config_param)
{
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_SYSTEM_CONFIG_PARAMETERS, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		ret = send_buffer((char*)(&config_param), sizeof(config_param), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfGetCalibrationParam(struct CameraCalibParam& calibration_param)
{
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_CAMERA_PARAMETERS, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&calibration_param), sizeof(calibration_param), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfSetCalibrationParam(const struct CameraCalibParam& calibration_param)
{
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_CAMERA_PARAMETERS, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		ret = send_buffer((char*)(&calibration_param), sizeof(calibration_param), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfRegisterOnDropped(int (*p_function)(void*))
{
	p_OnDropped = p_function;
	return 0;
}

/*****************************************************************************************************/

	//�������� DfSetParamStandardPlaneExternal
	//���ܣ� ���û�׼ƽ������
	//���������R(��ת����3*3)��T(ƽ�ƾ���3*1)
	//��������� ��
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
DF_SDK_API int DfSetParamStandardPlaneExternal(float* R, float* T)
{


	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_STANDARD_PLANE_EXTERNAL_PARAM, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
   
		float plane_param[12];
		 
		memcpy(plane_param, R, 9 * sizeof(float));
		memcpy(plane_param + 9,T, 3 * sizeof(float));

		ret = send_buffer((char*)(plane_param), sizeof(float)*12, g_sock);
	 

		if (ret == DF_FAILED)
		{
			close_socket(g_sock); 
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;


}

//�������� DfGetParamStandardPlaneExternal
//���ܣ� ��ȡ��׼ƽ������
//�����������
//��������� R(��ת����3*3)��T(ƽ�ƾ���3*1)
//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
DF_SDK_API int DfGetParamStandardPlaneExternal(float* R, float* T)
{
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_STANDARD_PLANE_EXTERNAL_PARAM, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{

		//int param_buf_size = 12 * sizeof(float);
		//float* plane_param = new float[param_buf_size];
		float plane_param[12];

		ret = recv_buffer((char*)(plane_param), sizeof(float) * 12, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);  
			return DF_FAILED;
		}

		memcpy(R, plane_param, 9 * sizeof(float));
		memcpy(T, plane_param + 9, 3 * sizeof(float));
		   
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//�������� DfSetParamHdr
//���ܣ� ���ö��ع����������ع����Ϊ6�Σ�
//��������� num���ع��������exposure_param[6]��6���ع������ǰnum����Ч��
//��������� ��
//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�궨�����ɹ�;����-1��ʾ��ȡ�궨����ʧ��.
DF_SDK_API int DfSetParamHdr(int num, int exposure_param[6])
{
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_HDR, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		int param[7];
		param[0] = num;

		memcpy(param+1, exposure_param, sizeof(int)*6);

		ret = send_buffer((char*)(param), sizeof(int) * 7, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}


//�������� DfGetParamHdr
//���ܣ� ���ö��ع����������ع����Ϊ6�Σ�
//��������� ��
//��������� num���ع��������exposure_param[6]��6���ع������ǰnum����Ч��
//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�궨�����ɹ�;����-1��ʾ��ȡ�궨����ʧ��.
DF_SDK_API int DfGetParamHdr(int& num, int exposure_param[6])
{
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_HDR, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		int param[7];
		param[0] = num;
		 
		ret = recv_buffer((char*)(param), sizeof(int) * 7, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}


		memcpy(exposure_param, param+1, sizeof(int) * 6);
		num = param[0];

	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}


	//�������� DfSetParamLedCurrent
	//���ܣ� ����LED����
	//��������� led������ֵ��
	//��������� ��
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�궨�����ɹ�;����-1��ʾ��ȡ�궨����ʧ��.
DF_SDK_API int DfSetParamLedCurrent(int led)
{
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_LED_CURRENT, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		ret = send_buffer((char*)(&led), sizeof(led), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}


	//�������� DfGetParamLedCurrent
	//���ܣ� ����LED����
	//��������� ��
	//��������� led������ֵ��
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�궨�����ɹ�;����-1��ʾ��ȡ�궨����ʧ��.
DF_SDK_API int DfGetParamLedCurrent(int& led)
{
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_CAMERA_PARAMETERS, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&led), sizeof(led), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}


/*********************************************************************************************************/