#include "solution.h" 
#include "Support_Function.h"
#include <io.h>
#include <iostream>  
#include "../sdk/open_cam3d.h"
#include <windows.h>
#include <assert.h>
#include <fstream>
#include <iomanip>
#include "../Firmware/protocol.h"
//#include "../cmd/getopt.h" 
/**************************************************************************/
int on_dropped(void* param)
{
	std::cout << "Network dropped!" << std::endl;
	return 0;
}

std::string& DfSolution::replaceAll(std::string& str, const   std::string& old_value, const   std::string& new_value)
{
	while (true) {
		std::string::size_type   pos(0);
		if ((pos = str.find(old_value)) != std::string::npos)
			str.replace(pos, old_value.length(), new_value);
		else   break;
	}
	return   str;
}

bool DfSolution::savePatterns(std::string dir, std::vector<cv::Mat> patterns)
{
	std::string new_dir = replaceAll(dir, "/", "\\");

	std::string folderPath = new_dir;
	std::string mkdir_cmd = std::string("mkdir ") + folderPath;
	system(mkdir_cmd.c_str());

  
	//std::cout << mkdir_cmd << std::endl;

	for (int i = 0; i < patterns.size(); i++)
	{
		std::stringstream ss;
		cv::Mat image = patterns[i];
		ss << std::setw(2) << std::setfill('0') << i;
		std::string filename = folderPath + "\\pattern_" + ss.str() + ".bmp";
		bool ret = cv::imwrite(filename, image);
		std::cout << "save: " << filename <<" "<< ret << std::endl;
	}

	return true;
}


bool DfSolution::saveCameraCalibData(std::string path, struct CameraCalibParam& param)
{
	std::ofstream ofile;
	ofile.open(path);
	for (int i = 0; i < sizeof(param) / sizeof(float); i++)
	{
		ofile << ((float*)(&param))[i] << std::endl;
	}
	ofile.close();

	return true;
}

bool DfSolution::readCameraCalibData(std::string path, struct CameraCalibParam& param)
{
	std::ifstream myfile(path);

	if (!myfile.is_open())
	{
		std::cout << "can not open this file" << std::endl;
		return 0;
	}

	float I[40] = { 0 };

	//从data1文件中读入int数据
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

bool DfSolution::getCameraCalibData(std::string ip, struct CameraCalibParam& param)
{
	DfRegisterOnDropped(on_dropped);

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

	return true;
}

bool DfSolution::captureMixedVariableWavelengthPatterns(std::string ip, std::vector<cv::Mat>& patterns)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip.c_str());
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);

	int image_size = width * height;

	int capture_num = 31;

	unsigned char* raw_buf = new unsigned char[(long)(image_size * capture_num)];

	ret = DfGetCameraRawData03(raw_buf, image_size * capture_num);
	 

	patterns.clear();

	for (int i = 0; i < capture_num; i++)
	{
		std::stringstream ss;
		cv::Mat image(1200, 1920, CV_8UC1, raw_buf + (long)(image_size * i));
 
		patterns.push_back(image.clone());
	}

	delete[] raw_buf;

	DfDisconnectNet();

	return true;
}

/**************************************************************************/
 

void  DfSolution::getFiles(std::string path, std::vector<std::string>& files)
{
	//文件句柄  
	intptr_t    hFile = 0;
	//文件信息，声明一个存储文件信息的结构体  
	struct _finddata_t fileinfo;
	string p;//字符串，存放路径
	if ((hFile = _findfirst(p.assign(path).append("/*.bmp").c_str(), &fileinfo)) != -1)//若查找成功，则进入
	{
		do
		{
			//如果是目录,迭代之（即文件夹内还有文件夹）  
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				//文件名不等于"."&&文件名不等于".."
					//.表示当前目录
					//..表示当前目录的父目录
					//判断时，两者都要忽略，不然就无限递归跳不出去了！
				//if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				//	getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
			}
			//如果不是,加入列表  
			else
			{
				files.push_back(p.assign(path).append("/").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		//_findclose函数结束查找
		_findclose(hFile);
	}

}
 

bool DfSolution::readImages(std::string dir, std::vector<cv::Mat>& patterns)
{
	//if (dir.empty())
	//{
	//	return false;
	//}

	std::vector<std::string> files;

	getFiles(dir, files);

	//for (int i = 0; i < files.size(); i++)
	//{
	//	std::cout<< files[i].c_str() << std::endl;
	//}

	patterns.clear();

	for (int i = 0; i < files.size(); i++)
	{
		//std::string path = dir + "/phase";
		std::string path = files[i];
		//if (i < 10)
		//{
		//	path += "0";
		//}

		//path +=  std::to_string(i) + "_p.bmp";
 

	 
		cv::Mat img = cv::imread(path, 0);
		if (img.empty())
		{
			return false;
		}
		std::cout << path << std::endl;

		patterns.push_back(img.clone());

		//patterns[i] = img.clone();
	}



	return true;
}


bool DfSolution::reconstructMixedVariableWavelengthPatternsBaseXYSR(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param,std::string pointcloud_path)
{

	if (31 != patterns.size())
	{
		return false;
	}

	bool ret = true;

	int ver_pstterns_num = 18;
	int hor_patterns_num = 12;  

	int nr = patterns[0].rows;
	int nc = patterns[0].cols;

	std::vector<cv::Mat> ver_patterns_img(patterns.begin(), patterns.begin() + ver_pstterns_num);
	std::vector<cv::Mat> hor_patterns_img(patterns.begin() + ver_pstterns_num, patterns.end() - 1);

	std::vector<cv::Mat> ver_wrap_img_4;
	std::vector<cv::Mat> hor_wrap_img_4;
	cv::Mat ver_confidence_map_4;
	cv::Mat hor_confidence_map_4;

	std::vector<cv::Mat> ver_wrap_img_6;
	std::vector<cv::Mat> hor_wrap_img_6;
	cv::Mat ver_confidence_map_6;
	cv::Mat hor_confidence_map_6;

	cv::Mat org_mask_(nr, nc, CV_8U, cv::Scalar(255));
	cv::rectangle(org_mask_, cv::Point(0, 0), cv::Point(nc - 1, nr - 1), cv::Scalar(0), 3);


	cv::Mat test_mask_ = org_mask_.clone();


	std::vector<cv::Mat> ver_patterns_img_4(ver_patterns_img.begin(), ver_patterns_img.begin() + ver_pstterns_num - 6);
	std::vector<cv::Mat> hor_patterns_img_4(hor_patterns_img.begin(), hor_patterns_img.begin() + hor_patterns_num);

	std::vector<cv::Mat> ver_patterns_img_6(ver_patterns_img.begin() + ver_pstterns_num - 6, ver_patterns_img.begin() + ver_pstterns_num);
	//std::vector<cv::Mat> hor_patterns_img_6(hor_patterns_img.begin() + patterns.size() / 2 - 6, hor_patterns_img.begin() + patterns.size() / 2);



	DF_Encode encode_machine_;

	ret = encode_machine_.computePhaseBaseFourStep(ver_patterns_img_4, ver_wrap_img_4, test_mask_, ver_confidence_map_4);
	ret = encode_machine_.computePhaseBaseFourStep(hor_patterns_img_4, hor_wrap_img_4, test_mask_, hor_confidence_map_4);


	ret = encode_machine_.computePhaseBaseSixStep(ver_patterns_img_6, ver_wrap_img_6, test_mask_, ver_confidence_map_6);
	//ret = encode_machine_.computePhaseBaseSixStep(hor_patterns_img_6, hor_wrap_img_6, test_mask_, hor_confidence_map_6);

	std::vector<double> variable_wrap_rate;
	variable_wrap_rate.push_back(8);
	variable_wrap_rate.push_back(4);
	variable_wrap_rate.push_back(4);


	cv::Mat unwrap_mask = test_mask_.clone();


	int discard_num = 0;

	std::vector<cv::Mat> select_ver_wrap_img = ver_wrap_img_4;
	std::vector<cv::Mat> select_hor_wrap_img = hor_wrap_img_4;

	select_ver_wrap_img.push_back(ver_wrap_img_6[0]);
	//select_hor_wrap_img.push_back(hor_wrap_img_6[0]);


	cv::Mat unwrap_hor, unwrap_ver;

	float ver_period_num = 1;

	for (int r_i = 0; r_i < variable_wrap_rate.size(); r_i++)
	{
		ver_period_num *= variable_wrap_rate[r_i];
	}

	ret = encode_machine_.unwrapVariableWavelengthPatterns(select_ver_wrap_img, variable_wrap_rate, unwrap_ver, unwrap_mask);
	if (!ret)
	{
		std::cout << "unwrap Error!";
		return false;
	}

	variable_wrap_rate.pop_back();

	float hor_period_num = 1;

	for (int r_i = 0; r_i < variable_wrap_rate.size(); r_i++)
	{
		hor_period_num *= variable_wrap_rate[r_i];
	}

	ret = encode_machine_.unwrapVariableWavelengthPatterns(select_hor_wrap_img, variable_wrap_rate, unwrap_hor, unwrap_mask);
	if (!ret)
	{
		std::cout << "unwrap Error!";
		return false;
	}



	float confidence_val = 10;

	float ver_period = ver_period_num;
	float hor_period = hor_period_num * 720.0 / 1280.0;


	unwrap_ver /= ver_period;
	unwrap_hor /= hor_period;

	encode_machine_.selectMaskBaseConfidence(ver_confidence_map_6, confidence_val, unwrap_mask);
	//encode_machine_.selectMaskBaseConfidence(hor_confidence_map_4, confidence_val, unwrap_mask);


	encode_machine_.maskMap(unwrap_mask, unwrap_ver);
	encode_machine_.maskMap(unwrap_mask, unwrap_hor);



	cv::Mat deep_map;


	DF_Reconstruct reconstruct_machine_;
	reconstruct_machine_.setCalibData(calib_param);
	 

	//Read Unwrap Map

	//cv::Mat read_unwrap_ver = cv::imread("../unwrap_map_0.tiff", cv::IMREAD_UNCHANGED);
	//cv::Mat read_unwrap_hor = cv::imread("../unwrap_map_1.tiff", cv::IMREAD_UNCHANGED);

	//read_unwrap_ver.convertTo(read_unwrap_ver, CV_64F);
	//read_unwrap_hor.convertTo(read_unwrap_hor, CV_64F);

	//encode_machine_.maskMap(unwrap_mask, read_unwrap_ver);
	//encode_machine_.maskMap(unwrap_mask, read_unwrap_hor);

	//ret = reconstruct_machine_.rebuildData(read_unwrap_ver, read_unwrap_hor, 1, deep_map);

	ret = reconstruct_machine_.rebuildData(unwrap_ver, unwrap_hor, 1, deep_map);
	if (!ret)
	{
		std::cout << "Rebuild Error!";

		return false;

	}

	cv::Mat texture_map = patterns[30];  

	cv::Mat undistort_img;
	reconstruct_machine_.undistortedImage(texture_map, undistort_img);
	texture_map = undistort_img.clone();


	/*********************************************************************************/
	 

	std::string work_path_ =  pointcloud_path + "/test";

	 

	std::vector<cv::Mat> deep_channels;

	cv::split(deep_map, deep_channels);

	cv::Mat depth_map;

	deep_channels[2].convertTo(depth_map, CV_32F);

	std::string save_depth_tiff = work_path_ + "_depth.tiff";
	std::string save_points_dir = work_path_ + "_points.xyz";
	std::string save_depth_txt_dir = work_path_ + "_depth.txt";
	std::string save_confidence_dir = work_path_ +  "_confidence.bmp";
	std::string save_depth_dir = work_path_ +  "_depth.bmp";
	std::string save_brightness_dir = work_path_  + "_brightness.bmp";
	std::string save_points_z_dir = work_path_   + "point_z.tiff";

	cv::Mat color_map, grey_map;
	MapToColor(deep_map, color_map, grey_map, 400, 800);
	MaskZMap(color_map, unwrap_mask);

	cv::imwrite(save_depth_tiff, depth_map);
	cv::imwrite(save_brightness_dir, texture_map); 
	cv::imwrite(save_depth_dir, color_map);
	SavePointToTxt(deep_map, save_points_dir, texture_map);
	//file_io_machine.saveDepthMapToTxt(deep_map, save_depth_txt_dir);

	std::cout << "pointcloud: " << save_points_dir;
 
	return true;


}
 

 
 