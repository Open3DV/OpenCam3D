
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
	int led_current;
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

