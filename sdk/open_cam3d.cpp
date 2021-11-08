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
#include<stddef.h> 
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
int brightness_bug_size_ = 0;
float* point_cloud_buf_ = NULL;
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
	auto milli = timestamp + (long long)8 * 60 * 60 * 1000; //此处转化为东八区北京时间，如果是其它时区需要按需求修改
	auto mTime = std::chrono::milliseconds(milli);
	auto tp = std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>(mTime);
	auto tt = std::chrono::system_clock::to_time_t(tp);
	std::tm* now = std::gmtime(&tt);
	//printf("%4d年%02d月%02d日 %02d:%02d:%02d\n", now->tm_year + 1900, now->tm_mon + 1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec);
	return now;
}


std::string get_timestamp()
{

	long long msec = 0;
	char time_str[7][16];
	auto t = getTimeStamp(msec);
	//std::cout << "Millisecond timestamp is: " << t << std::endl;
	auto time_ptr = gettm(t);
	sprintf(time_str[0], "%02d", time_ptr->tm_year + 1900); //月份要加1
	sprintf(time_str[1], "%02d", time_ptr->tm_mon + 1); //月份要加1
	sprintf(time_str[2], "%02d", time_ptr->tm_mday);//天
	sprintf(time_str[3], "%02d", time_ptr->tm_hour);//时
	sprintf(time_str[4], "%02d", time_ptr->tm_min);// 分
	sprintf(time_str[5], "%02d", time_ptr->tm_sec);//时
	sprintf(time_str[6], "%02d", msec);// 分
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
//网格掉线
int on_dropped(void* param)
{
	std::cout << "Network dropped!" << std::endl;
	return 0;
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
			int offset = r * camera_width_ + c;
			if (depth_map[offset] > 0)
			{
				point_cloud_map[3 * offset + 0] = (c - camera_cx) * depth_map[offset] / camera_fx;
				point_cloud_map[3 * offset + 1] = (r - camera_cy) * depth_map[offset] / camera_fy;
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


//函数名： DfConnect
//功能： 连接相机
//输入参数： camera_id（相机id）
//输出参数： 无
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
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



	//初始化

	camera_ip_ = camera_id;
	connected_flag_ = true;

	int image_size = camera_width_ * camera_height_;

	depth_buf_size_ = image_size * 1 * 4;
	depth_buf_ = (float*)(new char[depth_buf_size_]);

	point_cloud_buf_ = (float*)(new char[depth_buf_size_ * 3]);

	brightness_bug_size_ = image_size;
	brightness_buf_ = new unsigned char[brightness_bug_size_];


	LOG(INFO) << "Connect Camera: " << camera_ip_;

	return 0;
}

//函数名： DfGetCameraResolution
//功能： 获取相机分辨率
//输入参数： 无
//输出参数： width(图像宽)、height(图像高)
//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败.
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

//函数名： DfCaptureData
//功能： 采集一帧数据并阻塞至返回状态
//输入参数： exposure_num（曝光次数）：可设置值为1、2、3.
//输出参数： timestamp(时间戳)
//返回值： 类型（int）:返回0表示获取采集数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfCaptureData(int exposure_num, char* timestamp)
{

	//LOG(INFO) << "Debug Connect:";

	//DfnRegisterOnDropped(on_dropped);

	//int ret = DfnConnect(camera_ip_);
	//if (ret == DF_FAILED)
	//{
	//	return 0;
	//}


	LOG(TRACE) << "Debug Get Frame03:";
	bool ret = DfGetFrame03(depth_buf_, depth_buf_size_, brightness_buf_, brightness_bug_size_);

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



	return 0;
}

//函数名： DfGetDepthData
//功能： 采集点云数据并阻塞至返回结果
//输入参数：无
//输出参数： depth(深度图)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
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


//函数名： DfGetBrightnessData
//功能： 采集点云数据并阻塞至返回结果
//输入参数：无
//输出参数： brightness(亮度图)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
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

//函数名： DfGetPointcloudData
//功能： 采集点云数据并阻塞至返回结果
//输入参数：无
//输出参数： point_cloud(点云)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetPointcloudData(float* point_cloud)
{
	if (!connected_flag_)
	{
		return -1;
	}

	LOG(INFO) << "Transform Pointcloud:";

	depthTransformPointcloud(depth_buf_, point_cloud);

	LOG(INFO) << "Get Pointcloud!";

	return 0;
}

//函数名： DfConnect
//功能： 断开相机连接
//输入参数： camera_id（相机id）
//输出参数： 无
//返回值： 类型（int）:返回0表示断开成功;返回-1表示断开失败.
DF_SDK_API int DfDisconnect(const char* camera_id)
{
	if (!connected_flag_)
	{
		return -1;
	}


	DfDisconnectNet();

	delete[] depth_buf_;
	delete[] brightness_buf_;

	connected_flag_ = false;

	return 0;
}

//函数名： DfGetCalibrationParam
//功能： 获取相机标定参数
//输入参数： 无
//输出参数： calibration_param（相机标定参数结构体）
//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
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