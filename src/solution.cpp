#ifdef _WIN32  
#include <windows.h>
#include <io.h>
#elif __linux 
#include <cstring>
#include "iostream" 
#include <fstream> 
#include <sys/types.h>
#include <dirent.h>
#include <sys/io.h>
#endif 

#include "solution.h"  
#include <iostream>  
#include <assert.h>
#include <fstream>
#include <iomanip>
#include "../sdk/open_cam3d.h"
#include "../firmware/protocol.h"  
#include "../src/reconstruct.h"
/**************************************************************************/

int on_dropped_solution(void* param)
{
	std::cout << "Solution Network dropped!" << std::endl;
	return 0;
}

Solution::Solution()
{
	camera_version_ = 800;
}

Solution::~Solution()
{

}




bool Solution::getCameraCalibData(std::string ip, struct CameraCalibParam& param)
{
	DfRegisterOnDropped(on_dropped_solution);

	int ret = DfConnectNet(ip.c_str());
	if (ret == DF_FAILED)
	{
		return 0;
	}

	//struct CameraCalibParam calibration_param;
	//DfGetCalibrationParam(calibration_param);

	ret = DfGetCalibrationParam(param);


	if (ret == DF_FAILED)
	{
		return 0;
	}


	DfDisconnectNet();

	calib_param_ = param;

	return true;
}


bool Solution::readCameraCalibData(std::string path, struct CameraCalibParam& param)
{
	std::ifstream myfile(path);

	if (!myfile.is_open())
	{
		std::cout << "can not open this file" << std::endl;
		return 0;
	}

	float I[40] = { 0 };

	//��data1�ļ��ж���int����
	for (int i = 0; i < 40; i++)
	{

		myfile >> I[i];
		//std::cout << I[i] << std::endl;

	}
	myfile.close();


	param.camera_intrinsic[0] = I[0];
	param.camera_intrinsic[1] = I[1];
	param.camera_intrinsic[2] = I[2];
	param.camera_intrinsic[3] = I[3];
	param.camera_intrinsic[4] = I[4];
	param.camera_intrinsic[5] = I[5];
	param.camera_intrinsic[6] = I[6];
	param.camera_intrinsic[7] = I[7];
	param.camera_intrinsic[8] = I[8];

	param.camera_distortion[0] = I[9];
	param.camera_distortion[1] = I[10];
	param.camera_distortion[2] = I[11];
	param.camera_distortion[3] = I[12];
	param.camera_distortion[4] = I[13];


	param.projector_intrinsic[0] = I[14];
	param.projector_intrinsic[1] = I[15];
	param.projector_intrinsic[2] = I[16];
	param.projector_intrinsic[3] = I[17];
	param.projector_intrinsic[4] = I[18];
	param.projector_intrinsic[5] = I[19];
	param.projector_intrinsic[6] = I[20];
	param.projector_intrinsic[7] = I[21];
	param.projector_intrinsic[8] = I[22];


	param.projector_distortion[0] = I[23];
	param.projector_distortion[1] = I[24];
	param.projector_distortion[2] = I[25];
	param.projector_distortion[3] = I[26];
	param.projector_distortion[4] = I[27];


	param.rotation_matrix[0] = I[28];
	param.rotation_matrix[1] = I[29];
	param.rotation_matrix[2] = I[30];
	param.rotation_matrix[3] = I[31];
	param.rotation_matrix[4] = I[32];
	param.rotation_matrix[5] = I[33];
	param.rotation_matrix[6] = I[34];
	param.rotation_matrix[7] = I[35];
	param.rotation_matrix[8] = I[36];


	param.translation_matrix[0] = I[37];
	param.translation_matrix[1] = I[38];
	param.translation_matrix[2] = I[39];



	return true;
}



bool Solution::saveCameraCalibData(std::string path, struct CameraCalibParam& param)
{
	std::ofstream ofile;
	ofile.open(path);
	for (int i = 0; i < sizeof(param) / sizeof(float); i++)
	{
		ofile << ((float*)(&param))[i] << std::endl;
	}
	ofile.close();


	calib_param_ = param;
	return true;
}


bool Solution::getCameraVersion(const char* ip, int& version)
{
	int ret = DfConnectNet(ip);
	if (ret != DF_SUCCESS)
	{
		return false;
	}

	//获取相机型号参数
	ret = DfGetCameraVersion(version);
	if (ret != DF_SUCCESS)
	{
		std::cout << "Get Get Camera Version Failed!";
		return false;
	}

	ret = DfDisconnectNet();
	if (ret != DF_SUCCESS)
	{
		std::cout << "Disconnect Failed!";
	}

	return true;
}

bool Solution::setCameraVersion(int version)
{
	camera_version_ = version;

	switch (version)
	{
	case DFX_800:
	{
		return true;
	}
	break;

	case DFX_1800:
	{
		return true;
	}
	break;

	default:
		break;
	}

	return false;
}

bool Solution::captureRaw01(const char* ip, std::vector<cv::Mat>& patterns)
{

	DfRegisterOnDropped(on_dropped_solution);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return false;
	}

	int capture_num = 24;

	int width, height;
	DfGetCameraResolution(&width, &height);

	int image_size = width * height;

	unsigned char* raw_buf = new unsigned char[(long)(image_size * capture_num)];

	ret = DfGetCameraRawData01(raw_buf, image_size * capture_num);
	if (ret == DF_FAILED)
	{
		delete[] raw_buf;
		return false;
	}


	ret = DfDisconnectNet();
	if (ret == DF_FAILED)
	{
		std::cout << "Disconnect Failed!";
	}

	for (int i = 0; i < capture_num; i++)
	{
		cv::Mat pattern(height, width, CV_8U, raw_buf + image_size * i);
		patterns.push_back(pattern.clone());
	}

	delete[] raw_buf;
	return true;
}


bool Solution::reconstructFrame01(std::vector<cv::Mat> patterns, cv::Mat& depth, cv::Mat& brightness)
{
	if (24 != patterns.size())
	{
		std::cout << "wrong patterns size: " << patterns.size() << std::endl;
		return false;
	}

	bool ret = true;
	int ver_pstterns_num = 12;
	int hor_patterns_num = 12;

	int nr = patterns[0].rows;
	int nc = patterns[0].cols;

	std::vector<cv::Mat> ver_patterns_img(patterns.begin(), patterns.begin() + ver_pstterns_num);
	std::vector<cv::Mat> hor_patterns_img(patterns.begin() + ver_pstterns_num, patterns.end());

	std::vector<cv::Mat> ver_wrap_img_4;
	std::vector<cv::Mat> hor_wrap_img_4;
	std::vector<cv::Mat> ver_mask_map_4;
	std::vector<cv::Mat> hor_mask_map_4;
	std::vector<cv::Mat> ver_confidence_map_4;
	std::vector<cv::Mat> hor_confidence_map_4;
	std::vector<cv::Mat> ver_brightness_map_4;
	std::vector<cv::Mat> hor_brightness_map_4;


	std::vector<cv::Mat> ver_patterns_img_4(ver_patterns_img.begin(), ver_patterns_img.begin() + ver_pstterns_num);
	std::vector<cv::Mat> hor_patterns_img_4(hor_patterns_img.begin(), hor_patterns_img.begin() + hor_patterns_num);


	Encode encode;
	ret = encode.computePhaseBaseFourStep(ver_patterns_img_4, ver_wrap_img_4, ver_mask_map_4, ver_confidence_map_4, ver_brightness_map_4);
	ret = encode.computePhaseBaseFourStep(hor_patterns_img_4, hor_wrap_img_4, hor_mask_map_4, hor_confidence_map_4, hor_brightness_map_4);




	cv::Mat wrap_0 = ver_wrap_img_4[0].clone();
	cv::Mat wrap_1 = ver_wrap_img_4[1].clone();
	cv::Mat wrap_2 = ver_wrap_img_4[2].clone();


	std::vector<float> variable_wrap_rate;
	variable_wrap_rate.push_back(8);
	variable_wrap_rate.push_back(4);
	//variable_wrap_rate.push_back(4);

	std::vector<cv::Mat> select_ver_wrap_img = ver_wrap_img_4;
	std::vector<cv::Mat> select_hor_wrap_img = hor_wrap_img_4;


	cv::Mat unwrap_mask_ver(nr, nc, CV_8U, cv::Scalar(255));
	cv::Mat unwrap_mask_hor(nr, nc, CV_8U, cv::Scalar(255));

	cv::Mat unwrap_hor, unwrap_ver;

	float ver_period_num = 1;
	for (int r_i = 0; r_i < variable_wrap_rate.size(); r_i++)
	{
		ver_period_num *= variable_wrap_rate[r_i];
	}


	ret = encode.unwrapVariableWavelengthPatterns(select_ver_wrap_img, variable_wrap_rate, unwrap_ver, unwrap_mask_ver);
	if (!ret)
	{
		std::cout << "unwrap Error!";
		return false;
	}


	float hor_period_num = 1;
	for (int r_i = 0; r_i < variable_wrap_rate.size(); r_i++)
	{
		hor_period_num *= variable_wrap_rate[r_i];
	}


	ret = encode.unwrapVariableWavelengthPatterns(select_hor_wrap_img, variable_wrap_rate, unwrap_hor, unwrap_mask_hor);
	if (!ret)
	{
		std::cout << "unwrap Error!";
		return false;
	}


	float confidence_val = 10;
	float ver_period = ver_period_num;
	float hor_period = hor_period_num * 720.0 / 1280.0;

	ret = encode.maskBaseConfidence(ver_confidence_map_4[2], confidence_val, unwrap_mask_ver);
	if (!ret)
	{
		std::cout << "mask Base Confidence Failed!";
		return false;
	}

	ret = encode.maskBaseConfidence(ver_confidence_map_4[2], confidence_val, unwrap_mask_hor);
	if (!ret)
	{
		std::cout << "mask Base Confidence Failed!";
		return false;
	}

	unwrap_ver /= ver_period;
	unwrap_hor /= hor_period;

	encode.maskMap(unwrap_mask_ver, unwrap_ver);
	encode.maskMap(unwrap_mask_hor, unwrap_hor);

	cv::Mat deep_map;

	Reconstruct reconstruct;
	reconstruct.setCalibData(calib_param_);
	reconstruct.setCameraVersion(camera_version_);

	cv::Mat err_map;

	ret = reconstruct.rebuildData(unwrap_ver, unwrap_hor, 1, deep_map, err_map);
	if (!ret)
	{
		std::cout << "Rebuild Error!";

		return false;

	}

	std::vector<cv::Mat> channels;
	cv::split(deep_map, channels);

	depth = channels[2].clone();

	//cv::Mat merge_brightness(nr, nc, CV_8U, cv::Scalar(0));

	//for (int r = 0; r < nr; r++)
	//{
	//	uchar* ptr_0 = patterns[0].ptr<uchar>(r);
	//	uchar* ptr_1 = patterns[1].ptr<uchar>(r);
	//	uchar* ptr_2 = patterns[2].ptr<uchar>(r);
	//	uchar* ptr_3 = patterns[3].ptr<uchar>(r);
	//	uchar* ptr_b = merge_brightness.ptr<uchar>(r);

	//	for (int c = 0; c < nc; c++)
	//	{
	//		float val = (ptr_0[c] + ptr_1[c] + ptr_2[c] + ptr_3[c]) / 4;
	//		ptr_b[c] = val;
	//	}
	//}

	brightness = ver_brightness_map_4[0].clone();

	return true;
}


bool Solution::reconstructFrame01BaseFirmware(const char* ip, std::vector<cv::Mat>& patterns, cv::Mat& depth, cv::Mat& brightness)
{
	if (24 != patterns.size())
	{
		std::cout << "wrong patterns size!" << std::endl;
	}

	DfRegisterOnDropped(on_dropped_solution);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return false;
	}

	int capture_num = 24;

	int width, height;
	DfGetCameraResolution(&width, &height);

	int image_size = width * height;
	unsigned char* raw_buf = new unsigned char[(long)(image_size * capture_num)];

	for (int i = 0; i < capture_num; i++)
	{
		memcpy(raw_buf + i * image_size, patterns[i].data, image_size);
	}

	int depth_buf_size = image_size * sizeof(float);
	float* depth_buf = new float[(long)(image_size)];

	int brightness_buf_size = image_size * sizeof(unsigned char);
	unsigned char* brightness_buf = new unsigned char[(long)(image_size)];

	ret = DfGetTestFrame01(raw_buf, image_size * capture_num, depth_buf, depth_buf_size, brightness_buf, brightness_buf_size);
	if (ret == DF_FAILED)
	{
		delete[] raw_buf;
		delete[] depth_buf;
		delete[] brightness_buf;
		return false;
	}


	ret = DfDisconnectNet();
	if (ret == DF_FAILED)
	{
		std::cout << "Disconnect Failed!";
	}

	cv::Mat depth_img(height, width, CV_32F, depth_buf);
	cv::Mat brightness_img(height, width, CV_8U, brightness_buf);

	depth = depth_img.clone();
	brightness = brightness_img.clone();


	delete[] raw_buf;
	delete[] depth_buf;
	delete[] brightness_buf;
	return true;
}


bool Solution::readPatterns(std::string dir, std::vector<cv::Mat>& patterns)
{

	std::vector<std::string> files;
	getFiles(dir, files);
	patterns.clear();

	for (int i = 0; i < files.size(); i++)
	{
		std::string path = files[i];

		cv::Mat img = cv::imread(path, 0);
		if (img.empty())
		{
			return false;
		}
		std::cout << path << std::endl;
		patterns.push_back(img.clone());
	}

	return true;
}

bool Solution::savePatterns(std::string dir, std::vector<cv::Mat> patterns)
{
	if (patterns.empty())
	{
		return false;
	}

	std::string folderPath = dir;
	std::string mkdir_cmd = std::string("mkdir ") + folderPath;
	system(mkdir_cmd.c_str());

	for (int i = 0; i < patterns.size(); i++)
	{
		std::stringstream ss;
		//cv::Mat image = patterns[i];
		ss << std::setw(2) << std::setfill('0') << i;
		std::string filename = folderPath + "\\pattern_" + ss.str() + ".bmp";
		cv::imwrite(filename, patterns[i]);
	}

	return true;
}


void Solution::getFiles(std::string path, std::vector<std::string>& files)
{

#ifdef _WIN32 
	//�ļ����  
	intptr_t    hFile = 0;
	//�ļ���Ϣ������һ���洢�ļ���Ϣ�Ľṹ��  
	struct _finddata_t fileinfo;
	std::string p;//�ַ��������·��
	if ((hFile = _findfirst(p.assign(path).append("/*.bmp").c_str(), &fileinfo)) != -1)//�����ҳɹ��������
	{
		do
		{
			//�����Ŀ¼,����֮�����ļ����ڻ����ļ��У�  
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				//�ļ���������"."&&�ļ���������".."
					//.��ʾ��ǰĿ¼
					//..��ʾ��ǰĿ¼�ĸ�Ŀ¼
					//�ж�ʱ�����߶�Ҫ���ԣ���Ȼ�����޵ݹ�������ȥ�ˣ�
				//if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				//	getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
			}
			//�������,�����б�  
			else
			{
				files.push_back(p.assign(path).append("/").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		//_findclose������������
		_findclose(hFile);
	}


#elif __linux
	DIR* pDir;
	struct dirent* ptr;
	if (!(pDir = opendir(path.c_str())))
		return;
	while ((ptr = readdir(pDir)) != 0) {
		if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
			files.push_back(path + "/" + ptr->d_name);
	}
	closedir(pDir);

#endif 


}