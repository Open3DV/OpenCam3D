#include "../sdk/open_cam3d.h"
#include "opencv2/opencv.hpp"
#include <windows.h>
#include <assert.h>
#include <fstream>
#include "getopt.h" 


const char* help_info =
"Examples:\n\
\n\
1.Get temperature:\n\
open_cam3d.exe --get-temperature --ip 192.168.x.x\n\
\n\
2.Get brightness image:\n\
open_cam3d.exe --get-brightness --ip 192.168.x.x --path ./brightness.bmp\n\
\n\
3.Get point cloud:\n\
open_cam3d.exe --get-pointcloud --ip 192.168.x.x --path ./point_cloud.xyz\n\
\n\
4.Get Frame 01:\n\
open_cam3d.exe --get-frame-01 --ip 192.168.x.x --path ./frame_01\n\
\n\
5.Get Frame 03:\n\
open_cam3d.exe --get-frame-03 --ip 192.168.x.x --path ./frame_03\n\
\n\
6.Get Frame Hdr:\n\
open_cam3d.exe --get-frame-hdr --ip 192.168.x.x --path ./frame_hdr\n\
\n\
7.Get calibration parameters: \n\
open_cam3d.exe --get-calib-param --ip 192.168.x.x --path ./param.txt\n\
\n\
8.Set calibration parameters: \n\
open_cam3d.exe--set-calib-param --ip 192.168.x.x --path ./param.txt\n\
\n\
9.Get raw images (Mode 01): \n\
open_cam3d.exe --get-raw-01 --ip 192.168.x.x --path ./raw01_image_dir\n\
\n\
10.Get raw images (Mode 02): \n\
open_cam3d.exe --get-raw-02 --ip 192.168.x.x --path ./raw02_image_dir\n\
\n\
11.Get raw images (Mode 03): \n\
open_cam3d.exe --get-raw-03 --ip 192.168.x.x --path ./raw03_image_dir\n\
";


bool depthTransformPointcloud(cv::Mat depth_map, cv::Mat& point_cloud_map);
int get_frame_01(const char* ip, const char* frame_path);
int get_frame_03(const char* ip, const char* frame_path);
int get_repetition_frame_03(const char* ip,int count, const char* frame_path);
int get_frame_hdr(const char* ip, const char* frame_path);
void save_frame(float* depth_buffer, unsigned char* bright_buffer, const char* frame_path);
void save_images(const char* raw_image_dir, unsigned char* buffer, int image_size, int image_num);
void save_point_cloud(float* point_cloud_buffer, const char* pointcloud_path);
void save_color_point_cloud(float* point_cloud_buffer, unsigned char* brightness_buffer, const char* pointcloud_path);
int on_dropped(void* param);
int get_brightness(const char* ip, const char* image_path);
int get_pointcloud(const char* ip, const char* pointcloud_path);
int get_raw_01(const char* ip, const char* raw_image_dir);
int get_raw_02(const char* ip, const char* raw_image_dir);
int get_raw_03(const char* ip, const char* raw_image_dir);
int get_calib_param(const char* ip, const char* calib_param_path);
int set_calib_param(const char* ip, const char* calib_param_path);
int get_temperature(const char* ip);

extern int optind, opterr, optopt;
extern char* optarg;

enum opt_set
{
	IP,
	PATH,
	COUNT,
	GET_TEMPERATURE,
	GET_CALIB_PARAM,
	SET_CALIB_PARAM,
	GET_RAW_01,
	GET_RAW_02,
	GET_RAW_03,
	GET_POINTCLOUD,
	GET_FRAME_01,
	GET_FRAME_03,
	GET_REPETITION_FRAME_03,
	GET_FRAME_HDR,
	GET_BRIGHTNESS,
	HELP
};

static struct option long_options[] =
{
	{"ip",required_argument,NULL,IP},
	{"path", required_argument, NULL, PATH},
	{"count", required_argument, NULL, COUNT},
	{"get-temperature",no_argument,NULL,GET_TEMPERATURE},
	{"get-calib-param",no_argument,NULL,GET_CALIB_PARAM},
	{"set-calib-param",no_argument,NULL,SET_CALIB_PARAM},
	{"get-raw-01",no_argument,NULL,GET_RAW_01},
	{"get-raw-02",no_argument,NULL,GET_RAW_02},
	{"get-raw-03",no_argument,NULL,GET_RAW_03},
	{"get-pointcloud",no_argument,NULL,GET_POINTCLOUD},
	{"get-frame-01",no_argument,NULL,GET_FRAME_01},
	{"get-frame-03",no_argument,NULL,GET_FRAME_03},
	{"get-repetition-frame-03",no_argument,NULL,GET_REPETITION_FRAME_03},
	{"get-frame-hdr",no_argument,NULL,GET_FRAME_HDR},
	{"get-brightness",no_argument,NULL,GET_BRIGHTNESS},
	{"help",no_argument,NULL,HELP},
};


const char* camera_id;
const char* path;
const char* repetition_count;
int command = HELP;

struct CameraCalibParam calibration_param_;


int main(int argc, char* argv[])
{
	int c = 0;

	while (EOF != (c = getopt_long(argc, argv, "i:h", long_options, NULL)))
	{
		switch (c)
		{
		case IP:
			camera_id = optarg;
			break;
		case PATH:
			path = optarg;
			break;
		case COUNT:
			repetition_count = optarg;
			break;
		case '?':
			printf("unknow option:%c\n", optopt);
			break;
		default:
			command = c;
			break;
		}
	}

	switch (command)
	{
	case HELP:
		printf(help_info);
		break;
	case GET_TEMPERATURE:
		get_temperature(camera_id);
		break;
	case GET_CALIB_PARAM:
		get_calib_param(camera_id, path);
		break;
	case SET_CALIB_PARAM:
		set_calib_param(camera_id, path);
		break;
	case GET_RAW_01:
		get_raw_01(camera_id, path);
		break;
	case GET_RAW_02:
		get_raw_02(camera_id, path);
		break;
	case GET_RAW_03:
		get_raw_03(camera_id, path);
		break;
	case GET_FRAME_01:
		get_frame_01(camera_id, path);
		break;
	case GET_FRAME_03:
		get_frame_03(camera_id, path);
		break;
	case GET_REPETITION_FRAME_03:
	{ 
		int num = std::atoi(repetition_count);
		get_repetition_frame_03(camera_id, num, path);
	}

		break;
	case GET_FRAME_HDR:
		get_frame_hdr(camera_id, path);
		break;
	case GET_POINTCLOUD:
		get_pointcloud(camera_id, path);
		break;
	case GET_BRIGHTNESS:
		get_brightness(camera_id, path);
		break;
	default:
		break;
	}

	return 0;
}

int on_dropped(void* param)
{
	std::cout << "Network dropped!" << std::endl;
	return 0;
}


bool depthTransformPointcloud(cv::Mat depth_map, cv::Mat& point_cloud_map)
{
	if (depth_map.empty())
	{
		return false;
	}
	 

	double camera_fx = calibration_param_.camera_intrinsic[0];
	double camera_fy = calibration_param_.camera_intrinsic[4];

	double camera_cx = calibration_param_.camera_intrinsic[2];
	double camera_cy = calibration_param_.camera_intrinsic[5];
  
	int nr = depth_map.rows;
	int nc = depth_map.cols;


	cv::Mat points_map(nr, nc, CV_32FC3, cv::Scalar(0, 0, 0));


	for (int r = 0; r < nr; r++)
	{

		float* ptr_d = depth_map.ptr<float>(r);
		cv::Vec3f* ptr_p = points_map.ptr<cv::Vec3f>(r);

		for (int c = 0; c < nc; c++)
		{

			if (ptr_d[c] > 0)
			{

				cv::Point3f p;
				p.z = ptr_d[c];

				p.x = (c - camera_cx) * p.z / camera_fx;
				p.y = (r - camera_cy) * p.z / camera_fy;


				ptr_p[c][0] = p.x;
				ptr_p[c][1] = p.y;
				ptr_p[c][2] = p.z;
			}


		}


	}


	point_cloud_map = points_map.clone();


	return true;
}

void save_frame(float* depth_buffer, unsigned char* bright_buffer, const char* frame_path)
{
	std::string folderPath = frame_path;

	cv::Mat depth_map(1200, 1920, CV_32F, depth_buffer);
	cv::Mat bright_map(1200, 1920, CV_8U, bright_buffer);


	std::string depth_path = folderPath + ".tiff";
	cv::imwrite(depth_path, depth_map);
	std::cout << "save depth: " << depth_path << "\n";

	std::string bright_path = folderPath + ".bmp";
	cv::imwrite(bright_path, bright_map);
	std::cout << "save brightness: " << bright_path << "\n";

	cv::Mat point_cloud_map;
	depthTransformPointcloud(depth_map, point_cloud_map);


	std::string pointcloud_path = folderPath + ".xyz";

	save_color_point_cloud((float*)point_cloud_map.data, (unsigned char*)bright_map.data, pointcloud_path.c_str());

	std::cout << "save point cloud: " << pointcloud_path << "\n";

	//struct CameraCalibParam calibration_param;
	//DfGetCalibrationParam(calibration_param);

}

void save_images(const char* raw_image_dir, unsigned char* buffer, int image_size, int image_num)
{
	std::string folderPath = raw_image_dir;
	std::string mkdir_cmd = std::string("mkdir ") + folderPath;
	system(mkdir_cmd.c_str());

	for (int i = 0; i < image_num; i++)
	{
		std::stringstream ss;
		cv::Mat image(1200, 1920, CV_8UC1, buffer + (long)(image_size * i));
		ss << std::setw(2) << std::setfill('0') << i;
		std::string filename = folderPath + "/phase" + ss.str() + ".bmp";
		cv::imwrite(filename, image);
	}
}

void save_color_point_cloud(float* point_cloud_buffer, unsigned char* brightness_buffer, const char* pointcloud_path)
{
	std::ofstream ofile;
	ofile.open(pointcloud_path);
	for (int i = 0; i < 1920 * 1200; i++)
	{
		if (point_cloud_buffer[i * 3 + 2] > 0.01)
			ofile << point_cloud_buffer[i * 3] << " " << point_cloud_buffer[i * 3 + 1] << " " << point_cloud_buffer[i * 3 + 2] << " "
			<< (int)brightness_buffer[i] << " " << (int)brightness_buffer[i] << " " << (int)brightness_buffer[i] << std::endl;
	}
	ofile.close();
}


void save_point_cloud(float* point_cloud_buffer, const char* pointcloud_path)
{
	std::ofstream ofile;
	ofile.open(pointcloud_path);
	for (int i = 0; i < 1920 * 1200; i++)
	{
		if (point_cloud_buffer[i * 3 + 2] > 0.01)
			ofile << point_cloud_buffer[i * 3] << " " << point_cloud_buffer[i * 3 + 1] << " " << point_cloud_buffer[i * 3 + 2] << std::endl;
	}
	ofile.close();
}

int get_brightness(const char* ip, const char* image_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);

	int image_size = width * height;

	cv::Mat brightness(cv::Size(width, height), CV_8U);
	unsigned char* brightness_buf = (unsigned char*)brightness.data;
	ret = DfGetCameraData(0, 0,
		brightness_buf, image_size,
		0, 0,
		0, 0);
	cv::imwrite(image_path, brightness);

	DfDisconnectNet();
	return 1;
}


int get_frame_hdr(const char* ip, const char* frame_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);


	ret = DfGetCalibrationParam(calibration_param_);

	int image_size = width * height;

	int depth_buf_size = image_size * 1 * 4;
	float* depth_buf = (float*)(new char[depth_buf_size]);

	int brightness_bug_size = image_size;
	unsigned char* brightness_buf = new unsigned char[brightness_bug_size];

	ret = DfGetFrameHdr(depth_buf, depth_buf_size, brightness_buf, brightness_bug_size);



	save_frame(depth_buf, brightness_buf, frame_path);



	delete[] depth_buf;
	delete[] brightness_buf;

	DfDisconnectNet();



	return 1;
}


int get_repetition_frame_03(const char* ip, int count, const char* frame_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);


	ret = DfGetCalibrationParam(calibration_param_);

	int image_size = width * height;

	int depth_buf_size = image_size * 1 * 4;
	float* depth_buf = (float*)(new char[depth_buf_size]);

	int brightness_bug_size = image_size;
	unsigned char* brightness_buf = new unsigned char[brightness_bug_size];

	ret = DfGetRepetitionFrame03(count,depth_buf, depth_buf_size, brightness_buf, brightness_bug_size);

	DfDisconnectNet();

	save_frame(depth_buf, brightness_buf, frame_path);



	delete[] depth_buf;
	delete[] brightness_buf;

}

int get_frame_03(const char* ip, const char* frame_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);


	ret = DfGetCalibrationParam(calibration_param_);

	int image_size = width * height;

	int depth_buf_size = image_size * 1 * 4;
	float* depth_buf = (float*)(new char[depth_buf_size]);

	int brightness_bug_size = image_size;
	unsigned char* brightness_buf = new unsigned char[brightness_bug_size];

	ret = DfGetFrame03(depth_buf, depth_buf_size, brightness_buf, brightness_bug_size);

	DfDisconnectNet();

	save_frame(depth_buf, brightness_buf, frame_path);



	delete[] depth_buf;
	delete[] brightness_buf;



	return 1;
}

int get_frame_01(const char* ip, const char* frame_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);


	ret = DfGetCalibrationParam(calibration_param_);

	int image_size = width * height;

	int depth_buf_size = image_size * 1 * 4;
	float* depth_buf = (float*)(new char[depth_buf_size]);

	int brightness_bug_size = image_size;
	unsigned char* brightness_buf = new unsigned char[brightness_bug_size];

	ret = DfGetFrame01(depth_buf, depth_buf_size, brightness_buf, brightness_bug_size);



	save_frame(depth_buf, brightness_buf, frame_path);



	delete[] depth_buf;
	delete[] brightness_buf;

	DfDisconnectNet();

	return 1;
}

int get_pointcloud(const char* ip, const char* pointcloud_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);

	int image_size = width * height;

	int point_cloud_buf_size = image_size * 3 * 4;
	float* point_cloud_buf = (float*)(new char[point_cloud_buf_size]);

	ret = DfGetPointCloud(point_cloud_buf, point_cloud_buf_size);

	save_point_cloud(point_cloud_buf, pointcloud_path);

	delete[] point_cloud_buf;

	DfDisconnectNet();
	return 1;
}

int get_raw_01(const char* ip, const char* raw_image_dir)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int capture_num = 24;

	int width, height;
	DfGetCameraResolution(&width, &height);

	int image_size = width * height;

	unsigned char* raw_buf = new unsigned char[(long)(image_size * capture_num)];

	ret = DfGetCameraRawData01(raw_buf, image_size * capture_num);

	save_images(raw_image_dir, raw_buf, image_size, capture_num);

	delete[] raw_buf;

	DfDisconnectNet();
	return 1;

}


int get_raw_03(const char* ip, const char* raw_image_dir)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);

	int image_size = width * height;

	unsigned char* raw_buf = new unsigned char[(long)(image_size * 31)];

	ret = DfGetCameraRawData03(raw_buf, image_size * 31);

	save_images(raw_image_dir, raw_buf, image_size, 31);

	delete[] raw_buf;

	DfDisconnectNet();
	return 1;
}

int get_raw_02(const char* ip, const char* raw_image_dir)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);

	int capture_num = 37;

	int image_size = width * height;

	unsigned char* raw_buf = new unsigned char[(long)(image_size * capture_num)];

	ret = DfGetCameraRawDataTest(raw_buf, image_size * capture_num);

	save_images(raw_image_dir, raw_buf, image_size, capture_num);

	delete[] raw_buf;

	DfDisconnectNet();
	return 1;
}

int get_calib_param(const char* ip, const char* calib_param_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	struct CameraCalibParam calibration_param;
	DfGetCalibrationParam(calibration_param);
	std::ofstream ofile;
	ofile.open(calib_param_path);
	for (int i = 0; i < sizeof(calibration_param) / sizeof(float); i++)
	{
		ofile << ((float*)(&calibration_param))[i] << std::endl;
	}
	ofile.close();

	DfDisconnectNet();
	return 1;
}

int set_calib_param(const char* ip, const char* calib_param_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	struct CameraCalibParam calibration_param;
	std::ifstream ifile;
	ifile.open(calib_param_path);
	for (int i = 0; i < sizeof(calibration_param) / sizeof(float); i++)
	{
		ifile >> ((float*)(&calibration_param))[i];
	}
	ifile.close();

	DfSetCalibrationParam(calibration_param);

	DfDisconnectNet();
	return 1;
}

int get_temperature(const char* ip)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	float temperature = 0;
	DfGetDeviceTemperature(temperature);
	std::cout << "Device temperature: " << temperature << std::endl;

	DfDisconnectNet();
	return 1;
}
