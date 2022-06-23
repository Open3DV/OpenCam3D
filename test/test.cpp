// Dexforce_DF_Source.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include "solution.h"
#include "../firmware/camera_param.h"
#include "../cmd/getopt.h"


const char* help_info =
"Examples:\n\
\n\
1.Capture:\n\
test.exe --capture --ip 192.168.x.x --patterns ./patterns_data --calib ./param.txt --version DFX800 --pointcloud ./pointcloud_data\n\
\n\
2.Read:\n\
test.exe --read --patterns ./patterns_data  --calib ./param.txt --version DFX800  --pointcloud ./pointcloud_data\n\
\n\
3.Read:\n\
test.exe --reconstruct --use look-table --patterns ./patterns_data  --calib ./param.txt --version DFX800 --pointcloud ./pointcloud_data\n\
\n\
4.Read:\n\
test.exe --reconstruct --use minilook-table --patterns ./patterns_data  --calib ./param.txt --version DFX800 --pointcloud ./pointcloud_data\n\
\n\
";

extern int optind, opterr, optopt;
extern char* optarg;

enum opt_set
{
	IP,
	CAPTURE,
	READ,
	RECONSTRUCT,
	USE,
	PATTERNS,
	CALIB,
	POINTCLOUD,
	VERSION,
	HELP
};

static struct option long_options[] =
{
	{"ip",required_argument,NULL,IP},
	{"capture",no_argument,NULL,CAPTURE},
	{"read",no_argument,NULL,READ},
	{"reconstruct",no_argument,NULL,RECONSTRUCT},
	{"use", required_argument, NULL, USE},
	{"patterns", required_argument, NULL, PATTERNS},
	{"calib", required_argument, NULL, CALIB},
	{"pointcloud", required_argument, NULL, POINTCLOUD},
	{"version", required_argument, NULL, VERSION},
	{"help",no_argument,NULL,HELP},
};

const char* camera_ip = "";
const char* patterns_path = "";
const char* calib_path = "";
const char* pointcloud_path = "";
const char* use_type = "";
const char* char_version = "";
int command = HELP;


void capture();
void read();
void reconstruct_base_looktable();
void reconstruct_base_minilooktable();

const char* camera_id;
const char* path;

int version_number = 0;

int main(int argc, char* argv[])
{
	int c = 0;


	while (EOF != (c = getopt_long(argc, argv, "i:h", long_options, NULL)))
	{
		switch (c)
		{
		case IP:
			camera_ip = optarg;
			break;
		case PATTERNS:
			patterns_path = optarg;
			break;
		case CALIB:
			calib_path = optarg;
			break;
		case POINTCLOUD:
			pointcloud_path = optarg;
			break;
		case USE:
			use_type = optarg;
			break;
		case VERSION:
		{
			char_version = optarg;

			std::string version_str(char_version);

			if ("DFX800" == version_str || "dfx800" == version_str || "800" == version_str)
			{
				version_number = 800;
			}
			else if ("DFX1800" == version_str || "dfx1800" == version_str || "1800" == version_str)
			{
				version_number = 1800;
			}
		}
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
	case  CAPTURE:
		capture();
		break;
	case READ:
		read();
		break;
	case RECONSTRUCT:
	{
		std::string cmd(use_type);
		if ("look-table" == cmd)
		{
			reconstruct_base_looktable();
		}
		else if ("minilook-table" == cmd)
		{
			reconstruct_base_minilooktable();
		}

	}
	break;
	default:
		break;
	}

	/*************************************************************************************************/

}


void capture()
{

	struct CameraCalibParam calibration_param_;
	DfSolution solution_machine_;
	std::vector<cv::Mat> patterns_;

	bool ret = solution_machine_.captureMixedVariableWavelengthPatterns(camera_ip, patterns_);

	if (ret)
	{
		solution_machine_.savePatterns(patterns_path, patterns_);
	}

	ret = solution_machine_.getCameraCalibData(camera_ip, calibration_param_);

	if (ret)
	{
		solution_machine_.saveCameraCalibData(calib_path, calibration_param_);
	}
	else
	{
		std::cout << "Get Camera Calib Data Failure!";
	}
	 
	ret = solution_machine_.setCameraVersion(version_number);
	if (!ret)
	{  
		std::cout << "Set Camera Version Error!" << std::endl;
		return;
	}

	solution_machine_.reconstructMixedVariableWavelengthPatternsBaseXYSR(patterns_, calibration_param_, pointcloud_path);
}

void reconstruct_base_looktable()
{
	struct CameraCalibParam calibration_param_;
	DfSolution solution_machine_;
	std::vector<cv::Mat> patterns_;

	bool ret = solution_machine_.readImages(patterns_path, patterns_);

	if (!ret)
	{
		std::cout << "Read Image Error!";
	}

	ret = solution_machine_.readCameraCalibData(calib_path, calibration_param_);

	if (!ret)
	{
		std::cout << "Read Calib Param Error!" << std::endl;
	}
	 
	ret = solution_machine_.setCameraVersion(version_number);
	if (!ret)
	{ 
		std::cout << "Set Camera Version Error!" << std::endl;
		return;
	}

	
	solution_machine_.reconstructMixedVariableWavelengthXPatternsBaseTable(patterns_, calibration_param_, pointcloud_path);
}


void reconstruct_base_minilooktable()
{
	struct CameraCalibParam calibration_param_;
	DfSolution solution_machine_;
	std::vector<cv::Mat> patterns_;

	bool ret = solution_machine_.readImages(patterns_path, patterns_);

	if (!ret)
	{
		std::cout << "Read Image Error!";
	}

	ret = solution_machine_.readCameraCalibData(calib_path, calibration_param_);

	if (!ret)
	{
		std::cout << "Read Calib Param Error!" << std::endl;
	}




	solution_machine_.reconstructMixedVariableWavelengthXPatternsBaseMiniTable(patterns_, calibration_param_, pointcloud_path);
}


void read()
{
	struct CameraCalibParam calibration_param_;
	DfSolution solution_machine_;
	std::vector<cv::Mat> patterns_;

	bool ret = solution_machine_.readImages(patterns_path, patterns_);

	if (!ret)
	{
		std::cout << "Read Image Error!";
	}

	ret = solution_machine_.readCameraCalibData(calib_path, calibration_param_);

	if (!ret)
	{
		std::cout << "Read Calib Param Error!" << std::endl;
	}
	 
	ret = solution_machine_.setCameraVersion(version_number);
	if (!ret)
	{ 
		std::cout << "Set Camera Version Error!" << std::endl;
		return;
	}

	solution_machine_.reconstructMixedVariableWavelengthPatternsBaseXYSR(patterns_, calibration_param_, pointcloud_path);
}
