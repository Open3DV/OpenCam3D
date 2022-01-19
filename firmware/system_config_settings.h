
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
};

struct SystemConfigDataStruct
{
	SystemConfigDataStruct();

	SystemConfigParam config_param_;
  
	bool loadFromSettings(const std::string& f);
	bool saveToSettings(const std::string& f);
 
	static SystemConfigDataStruct& Instance()
	{
		return instance_;
	}

private:
	static SystemConfigDataStruct instance_;
};

