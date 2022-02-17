#include <iostream>
#include <string.h>
#include "open_cam3d_sdk.h"

int main()
{
	/*****************************************************************************************************/


  	int ret_code = DfConnect("192.168.88.109");

	int width = 0, height = 0; 

	if (0 == ret_code)
	{
		//必须连接相机成功后，才可获取相机分辨率
		ret_code = DfGetCameraResolution(&width, &height);
		std::cout << "Width: " << width << "    Height: " << height << std::endl;
	}
	else
	{
		std::cout << "Connect Camera Error!";
		return -1;
	}


	CalibrationParam calib_param;
	ret_code = DfGetCalibrationParam(&calib_param);

	if (0 == ret_code)
	{
		std::cout << "intrinsic: " << std::endl;
		for (int r = 0; r < 3; r++)
		{
			for (int c = 0; c < 3; c++)
			{
				std::cout << calib_param.intrinsic[3 * r + c] << "\t";
			}
			std::cout << std::endl;
		}

		std::cout << "extrinsic: " << std::endl;
		for (int r = 0; r < 4; r++)
		{
			for (int c = 0; c < 4; c++)
			{
				std::cout << calib_param.extrinsic[4 * r + c] << "\t";
			}
			std::cout << std::endl;
		}

		std::cout << "distortion: " << std::endl;
		for (int r = 0; r < 1; r++)
		{
			for (int c = 0; c < 12; c++)
			{
				std::cout << calib_param.distortion[1 * r + c] << "\t";
			}
			std::cout << std::endl;
		}
	}
	else
	{
		std::cout << "Get Calibration Data Error!"<< std::endl;
		return -1;
	}

	//分配内存保存采集结果
	float* point_cloud_data = (float*)malloc(sizeof(float) * width * height * 3);
	memset(point_cloud_data, 0, sizeof(float) * width * height * 3);

	unsigned short* depth_data = (unsigned short*)malloc(sizeof(unsigned short) * width * height);
	memset(depth_data, 0, sizeof(unsigned short) * width * height);

	char* timestamp_data = (char*)malloc(sizeof(char) * 30);
	memset(timestamp_data, 0, sizeof(char) * 30);

	unsigned char* brightness_data = (unsigned char*)malloc(sizeof(unsigned char) * width * height);
	memset(brightness_data, 0, sizeof(unsigned char) * width * height);
	
	int capture_num = 0;
	
	if(0 == ret_code)
	{
	
		ret_code = DfCaptureData(3, timestamp_data);

	std::cout << "timestamp: " << timestamp_data << std::endl;

	if (0 == ret_code)
	{
		ret_code = DfGetBrightnessData(brightness_data);
		if (0 == ret_code)
		{
			std::cout << "Get Brightness!"<< std::endl;
		}

		ret_code = DfGetDepthData(depth_data);
		
		if (0 == ret_code)
		{
			std::cout << "Get Depth!"<< std::endl;
		}


		ret_code = DfGetPointcloudData(point_cloud_data); 
		if (0 == ret_code)
		{
			std::cout << "Get Pointcloud!"<< std::endl;
		}
		
		capture_num++;
		std::cout << "Capture num: "<<capture_num<< std::endl;

	}
	else
	{
	
		std::cout << "Capture Data Error!"<< std::endl;
	
	}
	
	}


	 

    DfDisconnect("192.168.88.109");
}

 
