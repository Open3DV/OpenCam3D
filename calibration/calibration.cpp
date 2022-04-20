// open_cam3d_calibration.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>  
#include <string.h>
#include "../test/encode.h"   
#include "../test/support_function.h"
#include "Calibrate_Function.h"
#include "../cmd/getopt.h"
#include "../firmware/version.h"

const char* help_info =
"Examples:\n\
\n\
1.Calibrate:\n\
calibration.exe --calibrate --patterns ./calib --calib ./param.txt \n\
\n\
";

 

extern int optind, opterr, optopt;
extern char* optarg;

enum opt_set
{
	CALIBRATE,
	PATTERNS,
	CALIB,
	HELP
};

static struct option long_options[] =
{
	{"calibrate",no_argument,NULL,CALIBRATE},
	{"patterns", required_argument, NULL, PATTERNS},
	{"calib", required_argument, NULL, CALIB},
	{"help",no_argument,NULL,HELP},
};
 
const char* patterns_path;
const char* calib_path; 
int command = HELP;

void project_version();
bool calibrate_stereo(std::string patterns_path, std::string calib_path);

int main(int argc, char* argv[])
{

	int c = 0; 

	while (EOF != (c = getopt_long(argc, argv, "i:h", long_options, NULL)))
	{
		switch (c)
		{ 
		case PATTERNS:
			patterns_path = optarg;
			break;
		case CALIB:
			calib_path = optarg;
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
		project_version();
		break;
	case  CALIBRATE:
	{
		std::string patterns_str(patterns_path);
		std::string calib_str(calib_path);

		bool ok= calibrate_stereo(patterns_str, calib_str);

		if (!ok)
		{
			printf(help_info);
		 }
		
	}
		break; 
	default:
		break;
	}

	return 1;
}

void project_version()
{
	char info[100 * 1024] = { '\0' };
	char version[] = _VERSION_;
	char enter[] = "\n";

	strcpy_s(info, sizeof(enter), enter);
	strcat_s(info, sizeof(info), version);
	strcat_s(info, sizeof(info), enter);
	strcat_s(info, sizeof(info), enter);

	printf_s(info);
}

bool calibrate_stereo(std::string patterns_path, std::string calib_path)
{
	std::string path = patterns_path;
	Calibrate_Function calib_function;

	//读取多组标定条纹图案图像
	std::vector<std::vector<std::string>> files_list;
	getFilesList(path, files_list);

	std::vector<std::string> current_folder_list;
	getJustCurrentDir(path, current_folder_list);

	if (files_list.empty())
	{
		std::cout << "Read Images error!" << std::endl;
		return false;
	}

	std::cout << "Start Read Board Images...... " << std::endl;

	//文件夹路径
	std::vector<std::string> folder_list;

	//读取标定板图像
	std::vector<cv::Mat> board_images_list;
	for (int b_i = 0; b_i < files_list.size(); b_i++)
	{
		std::vector<std::string> g_image_list = files_list[b_i];

		cv::Mat board_img = cv::imread(g_image_list.back(), 0);
		board_images_list.push_back(board_img);
		folder_list.push_back(current_folder_list[b_i]);
	}

	std::cout << "Board Images Number: "<< board_images_list.size() << std::endl;
	std::cout << "Start Find Board...... " << std::endl;

	//查找标定板初步筛选图片组
	std::vector<std::vector<std::string>> select_images_path_list_base_board;
	std::vector<std::vector<cv::Point2f>> board_points_list;

	for (int g_i = 0; g_i < files_list.size(); g_i++)
	{
		std::vector<std::string> g_image_list = files_list[g_i];
		cv::Mat img = board_images_list[g_i];

		std::vector<cv::Point2f> circle_points;
		bool found = calib_function.findCircleBoardFeature(img, circle_points);


		if (found)
		{
			cv::Mat color_img;
			cv::Size board_size = calib_function.getBoardSize();
			cv::cvtColor(img, color_img, cv::COLOR_GRAY2BGR);
			cv::drawChessboardCorners(color_img, board_size, circle_points, found);

			std::vector<std::string> str_list = vStringSplit(folder_list[g_i], "/"); 

			std::string board_path = path + "/" + std::to_string(g_i) + "_draw.bmp";
			//std::string img_path = path + "/" + std::to_string(g_i) + "_board.bmp";
			cv::imwrite(board_path, color_img);
			//cv::imwrite(img_path, img);
			 

			if (0 == calib_function.testOverExposure(img, circle_points))
			{ 
				std::cout << "over exposure: " << folder_list[g_i] << std::endl;
			}

			select_images_path_list_base_board.push_back(g_image_list);
			board_points_list.push_back(circle_points);

		}


	}

	if (select_images_path_list_base_board.size() < 6)
	{
		std::cout << "Valid Board Number: "<< select_images_path_list_base_board.size() << std::endl;
		return false;
	}

	std::cout << "Valid Board Number: " << select_images_path_list_base_board.size() << std::endl;
	std::cout << "Start Calibrate Camera...... "<< std::endl;

	//标定相机并根据重投影误差筛选图片组
	std::map<int, bool> select_group;
	double err = calib_function.calibrateCamera(board_points_list, select_group);


	std::cout << "Calibrate Camera Error: "<< err << std::endl;
	std::cout << "Camera Select Group Number: " << select_group.size() << std::endl;

	std::vector<std::vector<std::string>> select_images_path_list_base_camera;
	std::vector<std::vector<cv::Point2f>> select_board_points_list;
	std::vector<std::vector<cv::Point2f>> dlp_points_list;
	std::vector<std::vector<cv::Point3f>> world_points_list;
	std::vector<std::string> select_folder_list;

	for (int g_i = 0; g_i < select_group.size(); g_i++)
	{
		if (select_group[g_i])
		{
			select_images_path_list_base_camera.push_back(select_images_path_list_base_board[g_i]);
			select_board_points_list.push_back(board_points_list[g_i]);
			select_folder_list.push_back(folder_list[g_i]);
		}
	}

	/*****************************************************************************************************/

	std::cout << "Start Read Pattern Image...... " << std::endl;

	//读取筛选出来的组图像
	std::vector<std::vector<cv::Mat>> group_image_list;
	for (int g_i = 0; g_i < select_images_path_list_base_camera.size(); g_i++)
	{
		std::vector<std::string> path_list = select_images_path_list_base_camera[g_i];

		std::vector<cv::Mat> img_list;

		for (int n_i = 0; n_i < path_list.size() - 1; n_i++)
		{
			cv::Mat img = cv::imread(path_list[n_i], 0);
			img_list.push_back(img);
		}

		group_image_list.push_back(img_list);
	}


	std::cout << "Read Pattern Finished." << std::endl;
	std::cout << "Start Compute Phase ......" << std::endl; 
	/******************************************************************************************************************/
	//依据相位值选择
	std::vector<bool> select_group_flag_base_phase;
	DF_Encode encode_machine_;


	//相位计算、相位展开、坐标转换
	for (int g_i = 0; g_i < group_image_list.size(); g_i++)
	{
		std::vector<cv::Mat> wrap_img_list;
		std::vector<cv::Mat> img_list = group_image_list[g_i];
		cv::Mat board_img = board_images_list[g_i];

		/*****************************************************************************************************************************************/
		//4+4+4+6
		int vertical_four_num = 12;
		int vertical_six_num = 6;
		int horizontal_four_num = 12;
		int horizontal_six_num = 6;

		std::vector<cv::Mat>::const_iterator list_ptr = img_list.begin();

		std::vector<cv::Mat> ver_patterns_img_4(list_ptr, list_ptr + vertical_four_num);
		std::vector<cv::Mat> ver_patterns_img_6(list_ptr + vertical_four_num, list_ptr + vertical_four_num + vertical_six_num);


		list_ptr = img_list.begin() + +img_list.size() / 2;

		std::vector<cv::Mat> hor_patterns_img_4(list_ptr, list_ptr + horizontal_four_num);
		std::vector<cv::Mat> hor_patterns_img_6(list_ptr + horizontal_four_num, list_ptr + horizontal_four_num + vertical_six_num);


		std::vector<cv::Mat> ver_wrap_img_4;
		std::vector<cv::Mat> hor_wrap_img_4;
		cv::Mat ver_confidence_map_4;
		cv::Mat hor_confidence_map_4;

		std::vector<cv::Mat> ver_wrap_img_6;
		std::vector<cv::Mat> hor_wrap_img_6;
		cv::Mat ver_confidence_map_6;
		cv::Mat hor_confidence_map_6;

		bool ret = true;

		cv::Mat test_mask_ = cv::Mat();

		DF_Encode encode_machine_;


		ret = encode_machine_.computePhaseBaseFourStep(ver_patterns_img_4, ver_wrap_img_4, test_mask_, ver_confidence_map_4);
		ret = encode_machine_.computePhaseBaseFourStep(hor_patterns_img_4, hor_wrap_img_4, test_mask_, hor_confidence_map_4);


		ret = encode_machine_.computePhaseBaseSixStep(ver_patterns_img_6, ver_wrap_img_6, test_mask_, ver_confidence_map_6);
		ret = encode_machine_.computePhaseBaseSixStep(hor_patterns_img_6, hor_wrap_img_6, test_mask_, hor_confidence_map_6);

		std::vector<double> variable_wrap_rate;
		variable_wrap_rate.push_back(8);
		variable_wrap_rate.push_back(4);
		variable_wrap_rate.push_back(4);


		cv::Mat unwrap_mask = test_mask_.clone();

		int discard_num = 0;

		std::vector<cv::Mat> select_ver_wrap_img = ver_wrap_img_4;
		std::vector<cv::Mat> select_hor_wrap_img = hor_wrap_img_4;

		select_ver_wrap_img.push_back(ver_wrap_img_6[0]);
		select_hor_wrap_img.push_back(hor_wrap_img_6[0]);


		cv::Mat unwrap_hor, unwrap_ver;

		float ver_period_num = 1;

		for (int r_i = 0; r_i < variable_wrap_rate.size(); r_i++)
		{
			ver_period_num *= variable_wrap_rate[r_i];
		}

		ret = encode_machine_.unwrapVariableWavelengthPatterns(select_ver_wrap_img, variable_wrap_rate, unwrap_ver, unwrap_mask);
		if (!ret)
		{
			std::cout << "unwrap Error." << std::endl;
			return false;
		}


		float hor_period_num = 1;
		for (int r_i = 0; r_i < variable_wrap_rate.size(); r_i++)
		{
			hor_period_num *= variable_wrap_rate[r_i];
		}

		ret = encode_machine_.unwrapVariableWavelengthPatterns(select_hor_wrap_img, variable_wrap_rate, unwrap_hor, unwrap_mask);
		if (!ret)
		{
			std::cout << "unwrap Error!" << std::endl;
			return false;
		}

		/*******************************************************************************************************************************************/

		float ver_period = ver_period_num;
		float hor_period = hor_period_num * 720.0 / 1280.0;


		unwrap_ver /= ver_period;
		unwrap_hor /= hor_period;
		/************************************************************************************************/

		//相机转换至投影仪

		std::vector<cv::Point2f> dlp_points;

		ret = calib_function.cameraPointsToDlp(select_board_points_list[g_i], unwrap_hor, unwrap_ver, 1, 1280, 720, dlp_points);

		if (!ret)
		{
			std::cout << "Bad Patterns: " << select_folder_list[g_i] << std::endl; 
			select_group_flag_base_phase.push_back(false);
		}
		else
		{
			select_group_flag_base_phase.push_back(true);
		}
		dlp_points_list.push_back(dlp_points);

	}
	/**********************************************************************************/
	//选择出对应图

	if (select_group_flag_base_phase.size() != select_board_points_list.size())
	{
		std::cout << "group num error!" << std::endl;
		return -1;
	}

	std::vector<std::vector<cv::Point2f>> select_board_points_list_base_phase;
	std::vector<std::vector<cv::Point2f>> select_dlp_points_list_base_phase;

	for (int g_i = 0; g_i < select_group_flag_base_phase.size(); g_i++)
	{
		if (select_group_flag_base_phase[g_i])
		{
			select_board_points_list_base_phase.push_back(select_board_points_list[g_i]);
			select_dlp_points_list_base_phase.push_back(dlp_points_list[g_i]);
		}
	}

	 
	std::cout << "Vaild Phase Group Number: " << select_dlp_points_list_base_phase.size() << std::endl;

	std::cout << "Start Calibrate Projector...... " << std::endl;
	//投影仪标定并筛选图像组
	std::map<int, bool> select_group_dlp;
	err = calib_function.calibrateProjector(select_dlp_points_list_base_phase, select_group_dlp);


	std::cout << "Calibrate Projector Error: " << err << std::endl;


	std::vector<std::vector<cv::Point2f>> select_dlp_points_list;
	std::vector<std::vector<cv::Point2f>> select_camera_points_list;

	for (int g_i = 0; g_i < select_group_dlp.size(); g_i++)
	{
		if (select_group_dlp[g_i])
		{
			select_dlp_points_list.push_back(select_dlp_points_list_base_phase[g_i]);
			select_camera_points_list.push_back(select_board_points_list_base_phase[g_i]);
		}
	}



	std::cout << "Projector Select Group Number: " << select_dlp_points_list.size() << std::endl;
	std::cout << "Start Calibrate Stereo...... " << std::endl;
	//对筛选出来的图像进行立体标定
	double stereo_err = calib_function.calibrateStereo(select_camera_points_list, select_dlp_points_list,calib_path);

	  
	std::cout << "Reprojection Error: " << stereo_err << std::endl;
	std::cout << "Reprojection Error should be less than 0.1......" << std::endl;

	//cv::destroyAllWindows();


	return true;
}

