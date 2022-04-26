#ifndef SYSTEM_SETTING_PARAM_H
#define SYSTEM_SETTING_PARAM_H

#include <iostream>
#include <string>
#include "istream"
#include <fstream> 
#include <vector>
#include <sstream>

std::string &StringTrim(std::string &str);
std::vector<std::string> vStringSplit(const std::string &s, const std::string &delim);

struct SystemConfigParam
{
	//投影亮度
	int led_current;
	//曝光次数
	int exposure_num;
	//曝光参数
	int exposure_param[6];
	//相机曝光时间（us）
	float camera_exposure_time;
	//相机增益DB
	float camera_gain;
	//外参标识:0-相机、1-光机、2-外部标定板。
	int external_param_flag;
	//相机外参:前9个为R、后3个为T。
	float standard_plane_external_param[12];
	//基准平面
	float standard_plane[4];

};

struct FirmwareConfigParam
{
	//生成亮度模式
	int generate_brightness_model;
	//生成亮度曝光
	float generate_brightness_exposure;
	//混合曝光参数
	int mixed_exposure_param_list[6];
	int mixed_led_param_list[6];
	int mixed_exposure_num;
	//1:光强融合多曝光模式、2：曝光时间-光强融合多曝光模式
	int hdr_model;
 

};


struct SystemConfigDataStruct
{
	SystemConfigDataStruct();

	SystemConfigParam config_param_;
	FirmwareConfigParam firwmare_param_;

	bool loadFromSettings(const std::string& f);
	bool saveToSettings(const std::string& f);
 
	static SystemConfigDataStruct& Instance()
	{
		return instance_;
	}

private:
	static SystemConfigDataStruct instance_;
};

#endif