// Dexforce_DF_Source.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include "solution.h"
#include "../Firmware/camera_param.h"
#include "../cmd/getopt.h"

const char* help_info =
"Examples:\n\
\n\
1.Capture:\n\
test.exe --capture --ip 192.168.x.x --patterns ./patterns_data --calib ./param.txt --pointcloud ./pointcloud_data\n\
\n\
2.Read:\n\
test.exe --read --patterns ./patterns_data  --calib ./param.txt --pointcloud ./pointcloud_data\n\
\n\
";

extern int optind, opterr, optopt;
extern char* optarg;

enum opt_set
{
	IP,
	CAPTURE,
	READ,
	PATTERNS,
	CALIB,
	POINTCLOUD,
	HELP
};

static struct option long_options[] =
{
	{"ip",required_argument,NULL,IP},
	{"capture",no_argument,NULL,CAPTURE},
	{"read",no_argument,NULL,READ},
	{"patterns", required_argument, NULL, PATTERNS},
	{"calib", required_argument, NULL, CALIB},
	{"pointcloud", required_argument, NULL, POINTCLOUD}, 
	{"help",no_argument,NULL,HELP},
};

const char* camera_ip;
const char* patterns_path; 
const char* calib_path;
const char* pointcloud_path;
int command = HELP;


void capture(); 
void read();

const char* camera_id;
const char* path;

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


	solution_machine_.reconstructMixedVariableWavelengthPatternsBaseXYSR(patterns_, calibration_param_, pointcloud_path);
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


	solution_machine_.reconstructMixedVariableWavelengthPatternsBaseXYSR(patterns_, calibration_param_,pointcloud_path);
}