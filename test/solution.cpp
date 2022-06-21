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
#include "support_function.h" 
#include <iostream>  
#include "../sdk/open_cam3d.h"
#include <assert.h>
#include <fstream>
#include <iomanip>
#include "../firmware/protocol.h"
#include "AnalyseError.h"
#include "../calibration/calibrate_function.h" 
#include "../gui/PrecisionTest.h"
#include "LookupTableFunction.h"
//#include "../cmd/getopt.h" 
#include "FilterModule.h"
/**************************************************************************/

DfSolution::DfSolution()
{
	camera_version_ = 800;
}

DfSolution::~DfSolution()
{

}


bool DfSolution::setCameraVersion(int version)
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

int on_dropped_solution(void* param)
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
		std::cout << "save: " << filename << " " << ret << std::endl;
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




bool DfSolution::getCameraCalibData(std::string ip, struct CameraCalibParam& param)
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

	return true;
}


bool DfSolution::captureModel04Patterns(std::string ip, std::vector<cv::Mat>& patterns)
{
	DfRegisterOnDropped(on_dropped_solution);

	int ret = DfConnectNet(ip.c_str());
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);

	int image_size = width * height;

	int capture_num = 19;

	unsigned char* raw_buf = new unsigned char[(long)(image_size * capture_num)];

	ret = DfGetCameraRawData04(raw_buf, image_size * capture_num);


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

bool DfSolution::captureMixedVariableWavelengthPatterns(std::string ip, std::vector<cv::Mat>& patterns)
{
	DfRegisterOnDropped(on_dropped_solution);

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

#ifdef _WIN32 
	//�ļ����  
	intptr_t    hFile = 0;
	//�ļ���Ϣ������һ���洢�ļ���Ϣ�Ľṹ��  
	struct _finddata_t fileinfo;
	string p;//�ַ��������·��
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


bool DfSolution::readFolderImages(std::string dir, std::vector<std::vector<cv::Mat>>& patterns_list)
{
	//读取多组标定条纹图案图像
	std::vector<std::vector<std::string>> files_list;
	getFilesList(dir, files_list);

	for (int l_i = 0; l_i < files_list.size(); l_i++)
	{
		std::vector<std::string> files = files_list[l_i];
		std::vector<cv::Mat> patterns;

		for (int p_i = 0; p_i < files.size(); p_i++)
		{
			cv::Mat img = cv::imread(files[p_i], 0);

			patterns.push_back(img.clone());
			std::cout << "read: " << files[p_i] << std::endl;
		}

		patterns_list.push_back(patterns);
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


		//cv::GaussianBlur(img, img, cv::Size(5, 5), 0, 1);

		patterns.push_back(img.clone());

		//patterns[i] = img.clone();
	}



	return true;
}

bool DfSolution::testCalibrationParamBaseBoard(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string err_map_path)
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


	DF_Encode encode_machine_;
	ret = encode_machine_.computePhaseBaseFourStep(ver_patterns_img_4, ver_wrap_img_4, test_mask_, ver_confidence_map_4);
	ret = encode_machine_.computePhaseBaseFourStep(hor_patterns_img_4, hor_wrap_img_4, test_mask_, hor_confidence_map_4);


	ret = encode_machine_.computePhaseBaseSixStep(ver_patterns_img_6, ver_wrap_img_6, test_mask_, ver_confidence_map_6);

	std::vector<double> variable_wrap_rate;
	variable_wrap_rate.push_back(8);
	variable_wrap_rate.push_back(4);
	variable_wrap_rate.push_back(4);


	cv::Mat unwrap_mask = test_mask_.clone();


	int discard_num = 0;

	std::vector<cv::Mat> select_ver_wrap_img = ver_wrap_img_4;
	std::vector<cv::Mat> select_hor_wrap_img = hor_wrap_img_4;

	select_ver_wrap_img.push_back(ver_wrap_img_6[0]);


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


	encode_machine_.maskMap(unwrap_mask, unwrap_ver);
	encode_machine_.maskMap(unwrap_mask, unwrap_hor);



	cv::Mat deep_map;
	DF_Reconstruct reconstruct_machine_;
	reconstruct_machine_.setCalibData(calib_param);

	cv::Mat err_map;

	ret = reconstruct_machine_.rebuildData(unwrap_ver, unwrap_hor, 1, deep_map, err_map);
	if (!ret)
	{
		std::cout << "Rebuild Error!";

		return false;

	}

	cv::Mat texture_map = patterns[30];
	cv::Mat undistort_img;
	reconstruct_machine_.undistortedImage(texture_map, undistort_img);
	texture_map = undistort_img.clone();


	/**************************************************************************************************/
		//ICP 
	Calibrate_Function calib_function;
	std::vector<cv::Point2f> undist_circle_points;
	bool found = calib_function.findCircleBoardFeature(undistort_img, undist_circle_points);

	if (!found)
	{
		return false;
	}



	std::vector<cv::Mat> deep_channels;
	cv::split(deep_map, deep_channels);
	cv::Mat depth_map;
	deep_channels[2].convertTo(depth_map, CV_32F);
	cv::medianBlur(depth_map, depth_map, 3);


	cv::Mat points_map(depth_map.size(), CV_32FC3, cv::Scalar(0., 0., 0.));
	reconstruct_machine_.depthTransformPointcloud(depth_map, points_map);

	/*******************************************************************************************/
	std::vector<cv::Point3f> point_3d;
	calib_function.bilinearInterpolationFeaturePoints(undist_circle_points, point_3d, points_map);

	PrecisionTest precision_machine;
	cv::Mat pc1(point_3d.size(), 3, CV_64F, cv::Scalar(0));
	cv::Mat pc2(point_3d.size(), 3, CV_64F, cv::Scalar(0));

	std::vector<cv::Point3f> world_points = calib_function.generateAsymmetricWorldFeature(20, 10);

	for (int i = 0; i < point_3d.size(); i++)
	{
		pc2.at<double>(i, 0) = point_3d[i].x;
		pc2.at<double>(i, 1) = point_3d[i].y;
		pc2.at<double>(i, 2) = point_3d[i].z;
	}
	for (int i = 0; i < world_points.size(); i++)
	{
		pc1.at<double>(i, 0) = world_points[i].x;
		pc1.at<double>(i, 1) = world_points[i].y;
		pc1.at<double>(i, 2) = world_points[i].z;
	}

	cv::Mat r(3, 3, CV_64F, cv::Scalar(0));
	cv::Mat t(3, 3, CV_64F, cv::Scalar(0));

	precision_machine.svdIcp(pc1, pc2, r, t);

	std::vector<cv::Point3f> transform_points;

	precision_machine.transformPoints(point_3d, transform_points, r, t);

	double diff = precision_machine.computeTwoPointSetDistance(world_points, transform_points);

	std::cout << "�������: " << diff << " mm" << std::endl;

	if (diff > 0.1)
	{
		std::cout << "����ڲξ��Ȳ�����" << std::endl;

		if (0 == calib_function.testOverExposure(undistort_img, undist_circle_points))
		{
			std::cout << "�궨������ˣ�" << std::endl;
			std::cout << "�����ͶӰ���ȣ�" << std::endl;
		}
	}
	else
	{
		std::cout << "����ڲκϸ�" << std::endl;
	}
	/****************************************************************************************************/

		//��ʾ

	cv::Mat draw_color_img;
	cv::Size board_size = calib_function.getBoardSize();
	cv::cvtColor(undistort_img, draw_color_img, cv::COLOR_GRAY2BGR);
	cv::drawChessboardCorners(draw_color_img, board_size, undist_circle_points, found);


	cv::Mat render_brightness;
	renderBrightnessImage(texture_map, render_brightness);
	err_map.convertTo(err_map, CV_32F);
	/*********************************************************************************/


	std::string work_path_ = err_map_path + "/test_calibration_param";


	std::string save_draw_board_dir = work_path_ + "_draw_board.bmp";
	std::string save_brightness_dir = work_path_ + "_brightness.bmp";


	cv::imwrite(save_brightness_dir, render_brightness);
	cv::imwrite(save_draw_board_dir, draw_color_img);

	std::cout << "save image: " << save_brightness_dir << std::endl;
	std::cout << "save image: " << save_draw_board_dir << std::endl;

	return true;

}


bool DfSolution::testCalibrationParamBasePlane(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string err_map_path)
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


	DF_Encode encode_machine_;
	ret = encode_machine_.computePhaseBaseFourStep(ver_patterns_img_4, ver_wrap_img_4, test_mask_, ver_confidence_map_4);
	ret = encode_machine_.computePhaseBaseFourStep(hor_patterns_img_4, hor_wrap_img_4, test_mask_, hor_confidence_map_4);


	ret = encode_machine_.computePhaseBaseSixStep(ver_patterns_img_6, ver_wrap_img_6, test_mask_, ver_confidence_map_6);

	std::vector<double> variable_wrap_rate;
	variable_wrap_rate.push_back(8);
	variable_wrap_rate.push_back(4);
	variable_wrap_rate.push_back(4);


	cv::Mat unwrap_mask = test_mask_.clone();


	int discard_num = 0;

	std::vector<cv::Mat> select_ver_wrap_img = ver_wrap_img_4;
	std::vector<cv::Mat> select_hor_wrap_img = hor_wrap_img_4;

	select_ver_wrap_img.push_back(ver_wrap_img_6[0]);


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


	encode_machine_.maskMap(unwrap_mask, unwrap_ver);
	encode_machine_.maskMap(unwrap_mask, unwrap_hor);



	cv::Mat deep_map;
	DF_Reconstruct reconstruct_machine_;
	reconstruct_machine_.setCalibData(calib_param);

	cv::Mat err_map;

	ret = reconstruct_machine_.rebuildData(unwrap_ver, unwrap_hor, 1, deep_map, err_map);
	if (!ret)
	{
		std::cout << "Rebuild Error!";

		return false;

	}

	cv::Mat color_err_map;
	cv::Mat gray_err_map;
	renderErrorMap(err_map, color_err_map, gray_err_map, 0., 0.1);

	AnalyseError analyse_err_machine;

	double err_value = analyse_err_machine.computeError(err_map);
	//std::cout << "calibrate err: " << err_value << std::endl;

	std::cout << "�������: " << err_value << " mm" << std::endl;

	if (err_value > 0.1)
	{
		std::cout << "����ڲξ��Ȳ�����" << std::endl;
	}
	else
	{
		std::cout << "����ڲκϸ�" << std::endl;

	}


	cv::Mat texture_map = patterns[30];

	cv::Mat undistort_img;
	reconstruct_machine_.undistortedImage(texture_map, undistort_img);
	texture_map = undistort_img.clone();

	cv::Mat render_brightness;
	renderBrightnessImage(texture_map, render_brightness);

	err_map.convertTo(err_map, CV_32F);
	/*********************************************************************************/


	std::string work_path_ = err_map_path + "/test_calibration_param";


	std::string save_err_tiff = work_path_ + "_err.tiff";
	std::string save_color_err_tiff = work_path_ + "_color_err.tiff";
	std::string save_brightness_dir = work_path_ + "_brightness.bmp";



	cv::imwrite(save_err_tiff, err_map);
	cv::imwrite(save_brightness_dir, render_brightness);
	cv::imwrite(save_color_err_tiff, gray_err_map);

	std::cout << "save image: " << save_err_tiff << std::endl;
	std::cout << "save image: " << save_brightness_dir << std::endl;
	std::cout << "save image: " << save_color_err_tiff << std::endl;

	return true;
}


bool DfSolution::findMaskBaseConfidenceLocalGrads(cv::Mat confidence_map, float threshold, cv::Mat& mask)
{
	if (confidence_map.empty())
	{
		return true;
	}

	int nr = confidence_map.rows;
	int nc = confidence_map.cols;

	cv::Mat sobel_map;

	cv::Sobel(confidence_map, sobel_map, -1, 1, 1, 3);


	cv::Mat bin_map;

	cv::threshold(confidence_map, bin_map, threshold, 255, cv::THRESH_BINARY);
	//bin_map.convertTo(bin_map, CV_8UC1);
}

bool  DfSolution::findMaskBaseConfidence(cv::Mat confidence_map, int threshold, cv::Mat& mask)
{
	if (confidence_map.empty())
	{
		return true;
	}

	int nr = confidence_map.rows;
	int nc = confidence_map.cols;


	cv::Mat bin_map;

	cv::threshold(confidence_map, bin_map, threshold, 255, cv::THRESH_BINARY);
	bin_map.convertTo(bin_map, CV_8UC1);

	std::vector<std::vector<cv::Point>> contours;

	cv::findContours(
		bin_map,
		contours,
		cv::noArray(),
		cv::RETR_EXTERNAL,
		cv::CHAIN_APPROX_SIMPLE
	);

	std::vector<cv::Point> max_contours;
	int max_contours_size = 0;

	for (int i = 0; i < contours.size(); i++)
	{
		if (contours[i].size() > max_contours_size)
		{
			max_contours_size = contours[i].size();
			max_contours = contours[i];
		}

	}

	contours.clear();
	contours.push_back(max_contours);

	cv::Mat show_contours(nr, nc, CV_8U, cv::Scalar(0));
	cv::drawContours(show_contours, contours, -1, cv::Scalar(255), -1);

	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::Mat result;
	cv::erode(show_contours, result, element);

	mask = result.clone();


	return true;
}


bool DfSolution::reconstructPatterns04RepetitionBaseTable(std::vector<std::vector<cv::Mat>> patterns_list, struct CameraCalibParam calib_param, std::string pointcloud_path)
{
	/***********************************************************************************/

	clock_t startTime, endTime;
	startTime = clock();//��ʱ��ʼ


	LookupTableFunction lookup_table_machine_;
	lookup_table_machine_.setCalibData(calib_param);
	lookup_table_machine_.setCameraVersion(camera_version_);

	cv::Mat xL_rotate_x;
	cv::Mat xL_rotate_y;
	cv::Mat R1;
	cv::Mat pattern_mapping;
	lookup_table_machine_.generateLookTable(xL_rotate_x, xL_rotate_y, R1, pattern_mapping);

	//lookup_table_machine_.readTable("../", 1200, 1920);


	endTime = clock();//��ʱ����
	std::cout << "The run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

	/************************************************************************************/

	std::vector<cv::Mat> patterns;

	DF_Encode encode_machine_;
	encode_machine_.mergePatterns(patterns_list, patterns);

	if (19 != patterns.size())
	{
		return false;
	}

	bool ret = true;

	int ver_pstterns_num = 18;

	int nr = patterns[0].rows;
	int nc = patterns[0].cols;

	std::vector<cv::Mat> ver_patterns_img(patterns.begin(), patterns.begin() + ver_pstterns_num);

	std::vector<cv::Mat> ver_wrap_img_4;
	cv::Mat ver_confidence_map_4;

	std::vector<cv::Mat> ver_wrap_img_6;
	cv::Mat ver_confidence_map_6;

	cv::Mat org_mask_(nr, nc, CV_8U, cv::Scalar(255));
	cv::rectangle(org_mask_, cv::Point(0, 0), cv::Point(nc - 1, nr - 1), cv::Scalar(0), 3);


	cv::Mat test_mask_ = org_mask_.clone();


	std::vector<cv::Mat> ver_patterns_img_4(ver_patterns_img.begin(), ver_patterns_img.begin() + ver_pstterns_num - 6);
	std::vector<cv::Mat> ver_patterns_img_6(ver_patterns_img.begin() + ver_pstterns_num - 6, ver_patterns_img.begin() + ver_pstterns_num);




	ret = encode_machine_.computePhaseBaseFourStep(ver_patterns_img_4, ver_wrap_img_4, test_mask_, ver_confidence_map_4);
	ret = encode_machine_.computePhaseBaseSixStep(ver_patterns_img_6, ver_wrap_img_6, test_mask_, ver_confidence_map_6);

	std::vector<double> variable_wrap_rate;
	variable_wrap_rate.push_back(8);
	variable_wrap_rate.push_back(4);
	variable_wrap_rate.push_back(4);


	cv::Mat unwrap_mask = test_mask_.clone();

	std::vector<cv::Mat> select_ver_wrap_img = ver_wrap_img_4;
	select_ver_wrap_img.push_back(ver_wrap_img_6[0]);


	cv::Mat unwrap_ver;
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
	cv::Mat wrap_0 = ver_wrap_img_4[0].clone();
	cv::Mat wrap_1 = ver_wrap_img_4[1].clone();
	cv::Mat wrap_2 = ver_wrap_img_4[2].clone();
	cv::Mat wrap_3 = ver_wrap_img_6[0].clone();

	float confidence_val = 5;

	float ver_period = ver_period_num;
	unwrap_ver /= ver_period;

	encode_machine_.selectMaskBaseConfidence(ver_confidence_map_6, confidence_val, unwrap_mask);
	encode_machine_.maskMap(unwrap_mask, unwrap_ver);

	cv::Mat confidence_mask;
	//findMaskBaseConfidence(ver_confidence_map_6, 3, confidence_mask);
	////findMaskBaseConfidenceLocalGrads(ver_confidence_map_6, 3.0, confidence_mask);
	//encode_machine_.maskMap(confidence_mask, unwrap_ver);

	cv::Mat texture_map = patterns[18];
	texture_map /= patterns_list.size();
	texture_map.convertTo(texture_map, CV_8U);
	cv::Mat undistort_img;
	lookup_table_machine_.undistortedImage(texture_map, undistort_img);
	texture_map = undistort_img.clone();

	cv::Mat z_map_table;
	//����ؽ���deep_map ��ͨ��Ϊx y z��ͨ����double ����
	lookup_table_machine_.rebuildData(unwrap_ver, 1, z_map_table, unwrap_mask);

	cv::Mat deep_map_table;

	std::vector<cv::Point3f> points_cloud;
	ret = lookup_table_machine_.generate_pointcloud(z_map_table, unwrap_mask, deep_map_table);


	//startTime = clock();//��ʱ��ʼ   
	//FilterModule filter_machine;
	////相机像素为5.4um、焦距12mm。dot_spacing = 5.4*distance/12000 mm，典型值0.54mm（1200） 
	//filter_machine.RadiusOutlierRemoval(deep_map_table, unwrap_mask, 0.8, 3, 4);
	////filter_machine.statisticOutlierRemoval(deep_map_table, 6, 1);
	//endTime = clock();//��ʱ����
	//std::cout << "statisticOutlierRemoval run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

	/*********************************************************************************/

	std::string work_path_ = pointcloud_path + "/test";


	std::vector<cv::Mat> deep_channels;
	cv::split(deep_map_table, deep_channels);
	cv::Mat z_map;
	deep_channels[2].convertTo(z_map, CV_32F);


	std::string save_err_tiff = work_path_ + "_err_table.tiff";
	std::string save_depth_tiff = work_path_ + "_depth_table.tiff";
	std::string save_points_dir = work_path_ + "_points_table.xyz";
	std::string save_depth_txt_dir = work_path_ + "_depth_table.txt";
	std::string save_confidence_dir = work_path_ + "_confidence_table.bmp";
	std::string save_depth_dir = work_path_ + "_depth_table.bmp";
	std::string save_brightness_dir = work_path_ + "_brightness_table.bmp";
	std::string save_points_z_dir = work_path_ + "point_z_table.tiff";

	cv::Mat color_map, grey_map;
	MapToColor(deep_map_table, color_map, grey_map, 300, 2000);
	MaskZMap(color_map, unwrap_mask);


	//cv::imwrite(save_err_tiff, err_map);
	cv::imwrite(save_depth_tiff, z_map);
	cv::imwrite(save_brightness_dir, texture_map);
	cv::imwrite(save_depth_dir, color_map);
	SavePointToTxt(deep_map_table, save_points_dir, texture_map);

	std::cout << "pointcloud: " << save_points_dir;

	return true;
}

bool DfSolution::reconstructMixedVariableWavelengthXPatternsBaseTable(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string pointcloud_path)
{
	/***********************************************************************************/

	clock_t startTime, endTime;
	startTime = clock();//��ʱ��ʼ


	LookupTableFunction lookup_table_machine_;
	//LookupTableFunction lookup_table_machine;
	lookup_table_machine_.setCalibData(calib_param);
	lookup_table_machine_.setCameraVersion(camera_version_);

	cv::Mat xL_rotate_x;
	cv::Mat xL_rotate_y;
	cv::Mat R1;
	cv::Mat pattern_mapping;
	lookup_table_machine_.generateLookTable(xL_rotate_x, xL_rotate_y, R1, pattern_mapping);

	//lookup_table_machine_.readTable("../", 1200, 1920);


	endTime = clock();//��ʱ����
	std::cout << "The run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

	/************************************************************************************/



	if (19 != patterns.size())
	{
		return false;
	}

	bool ret = true;

	int ver_pstterns_num = 18;

	int nr = patterns[0].rows;
	int nc = patterns[0].cols;

	std::vector<cv::Mat> ver_patterns_img(patterns.begin(), patterns.begin() + ver_pstterns_num);

	std::vector<cv::Mat> ver_wrap_img_4;
	cv::Mat ver_confidence_map_4;

	std::vector<cv::Mat> ver_wrap_img_6;
	cv::Mat ver_confidence_map_6;

	cv::Mat org_mask_(nr, nc, CV_8U, cv::Scalar(255));
	cv::rectangle(org_mask_, cv::Point(0, 0), cv::Point(nc - 1, nr - 1), cv::Scalar(0), 3);


	cv::Mat test_mask_ = org_mask_.clone();


	std::vector<cv::Mat> ver_patterns_img_4(ver_patterns_img.begin(), ver_patterns_img.begin() + ver_pstterns_num - 6);
	std::vector<cv::Mat> ver_patterns_img_6(ver_patterns_img.begin() + ver_pstterns_num - 6, ver_patterns_img.begin() + ver_pstterns_num);



	DF_Encode encode_machine_;

	ret = encode_machine_.computePhaseBaseFourStep(ver_patterns_img_4, ver_wrap_img_4, test_mask_, ver_confidence_map_4);
	ret = encode_machine_.computePhaseBaseSixStep(ver_patterns_img_6, ver_wrap_img_6, test_mask_, ver_confidence_map_6);

	std::vector<double> variable_wrap_rate;
	variable_wrap_rate.push_back(8);
	variable_wrap_rate.push_back(4);
	variable_wrap_rate.push_back(4);


	cv::Mat unwrap_mask = test_mask_.clone();

	std::vector<cv::Mat> select_ver_wrap_img = ver_wrap_img_4;
	select_ver_wrap_img.push_back(ver_wrap_img_6[0]);


	cv::Mat unwrap_ver;
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
	cv::Mat wrap_0 = ver_wrap_img_4[0].clone();
	cv::Mat wrap_1 = ver_wrap_img_4[1].clone();
	cv::Mat wrap_2 = ver_wrap_img_4[2].clone();
	cv::Mat wrap_3 = ver_wrap_img_6[0].clone();

	float confidence_val = 10;

	float ver_period = ver_period_num;
	unwrap_ver /= ver_period;

	encode_machine_.selectMaskBaseConfidence(ver_confidence_map_6, confidence_val, unwrap_mask);
	encode_machine_.maskMap(unwrap_mask, unwrap_ver);

	cv::Mat confidence_mask;
	//findMaskBaseConfidence(ver_confidence_map_6, 3, confidence_mask);
	////findMaskBaseConfidenceLocalGrads(ver_confidence_map_6, 3.0, confidence_mask);
	//encode_machine_.maskMap(confidence_mask, unwrap_ver);

	cv::Mat texture_map = patterns[18];
	cv::Mat undistort_img;
	lookup_table_machine_.undistortedImage(texture_map, undistort_img);
	texture_map = undistort_img.clone();

	cv::Mat z_map_table;
	//����ؽ���deep_map ��ͨ��Ϊx y z��ͨ����double ����
	lookup_table_machine_.rebuildData(unwrap_ver, 1, z_map_table, unwrap_mask);

	cv::Mat deep_map_table;

	std::vector<cv::Point3f> points_cloud;
	ret = lookup_table_machine_.generate_pointcloud(z_map_table, unwrap_mask, deep_map_table);


	//startTime = clock();//��ʱ��ʼ   
	//FilterModule filter_machine;
	////相机像素为5.4um、焦距12mm。dot_spacing = 5.4*distance/12000 mm，典型值0.54mm（1200） 
	//filter_machine.RadiusOutlierRemoval(deep_map_table, unwrap_mask, 0.8, 3, 4);
	////filter_machine.statisticOutlierRemoval(deep_map_table, 6, 1);
	//endTime = clock();//��ʱ����
	//std::cout << "statisticOutlierRemoval run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

	/*********************************************************************************/

	std::string work_path_ = pointcloud_path + "/test";


	std::vector<cv::Mat> deep_channels;
	cv::split(deep_map_table, deep_channels);
	cv::Mat z_map;
	deep_channels[2].convertTo(z_map, CV_32F);


	std::string save_err_tiff = work_path_ + "_err_table.tiff";
	std::string save_depth_tiff = work_path_ + "_depth_table.tiff";
	std::string save_points_dir = work_path_ + "_points_table.xyz";
	std::string save_depth_txt_dir = work_path_ + "_depth_table.txt";
	std::string save_confidence_dir = work_path_ + "_confidence_table.bmp";
	std::string save_depth_dir = work_path_ + "_depth_table.bmp";
	std::string save_brightness_dir = work_path_ + "_brightness_table.bmp";
	std::string save_points_z_dir = work_path_ + "point_z_table.tiff";

	cv::Mat color_map, grey_map;
	MapToColor(deep_map_table, color_map, grey_map, 300, 2000);
	MaskZMap(color_map, unwrap_mask);


	//cv::imwrite(save_err_tiff, err_map);
	cv::imwrite(save_depth_tiff, z_map);
	cv::imwrite(save_brightness_dir, texture_map);
	cv::imwrite(save_depth_dir, color_map);
	SavePointToTxt(deep_map_table, save_points_dir, texture_map);

	std::cout << "pointcloud: " << save_points_dir;

	return true;
}


bool DfSolution::reconstructMixedVariableWavelengthPatternsBaseXYSR(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string pointcloud_path)
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

	cv::Mat wrap_0 = ver_wrap_img_4[0].clone();
	cv::Mat wrap_1 = ver_wrap_img_4[1].clone();
	cv::Mat wrap_2 = ver_wrap_img_4[2].clone();
	cv::Mat wrap_3 = ver_wrap_img_6[0].clone();


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
	reconstruct_machine_.setCameraVersion(camera_version_);

	cv::Mat err_map;

	ret = reconstruct_machine_.rebuildData(unwrap_ver, unwrap_hor, 1, deep_map, err_map);
	if (!ret)
	{
		std::cout << "Rebuild Error!";

		return false;

	}


	clock_t startTime, endTime;
	startTime = clock();//��ʱ��ʼ   
	FilterModule filter_machine;
	filter_machine.RadiusOutlierRemoval(deep_map, unwrap_mask, 0.5, 2, 4);
	endTime = clock();//��ʱ����
	std::cout << "RadiusOutlierRemoval run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

	cv::Mat color_err_map;
	cv::Mat gray_err_map;
	renderErrorMap(err_map, color_err_map, gray_err_map, 0., 0.1);

	AnalyseError analyse_err_machine;

	double err_value = analyse_err_machine.computeError(err_map);
	std::cout << "calibrate err: " << err_value << std::endl;

	cv::Mat texture_map = patterns[30];

	cv::Mat undistort_img;
	reconstruct_machine_.undistortedImage(texture_map, undistort_img);
	texture_map = undistort_img.clone();

	err_map.convertTo(err_map, CV_32F);
	/*********************************************************************************/


	std::string work_path_ = pointcloud_path + "/test";



	std::vector<cv::Mat> deep_channels;

	cv::split(deep_map, deep_channels);

	cv::Mat depth_map;

	deep_channels[2].convertTo(depth_map, CV_32F);


	std::string save_err_tiff = work_path_ + "_err.tiff";
	std::string save_depth_tiff = work_path_ + "_depth.tiff";
	std::string save_points_dir = work_path_ + "_points.xyz";
	std::string save_depth_txt_dir = work_path_ + "_depth.txt";
	std::string save_confidence_dir = work_path_ + "_confidence.bmp";
	std::string save_depth_dir = work_path_ + "_depth.bmp";
	std::string save_brightness_dir = work_path_ + "_brightness.bmp";
	std::string save_points_z_dir = work_path_ + "point_z.tiff";

	cv::Mat color_map, grey_map;
	MapToColor(deep_map, color_map, grey_map, 400, 800);
	MaskZMap(color_map, unwrap_mask);



	cv::imwrite(save_err_tiff, err_map);
	cv::imwrite(save_depth_tiff, depth_map);
	cv::imwrite(save_brightness_dir, texture_map);
	cv::imwrite(save_depth_dir, color_map);
	SavePointToTxt(deep_map, save_points_dir, texture_map);
	//file_io_machine.saveDepthMapToTxt(deep_map, save_depth_txt_dir);

	std::cout << "pointcloud: " << save_points_dir;

	return true;


}




