#include "system_config_settings.h" 
#include <string>
#include "istream"
#include <fstream> 
#include <vector>
#include <sstream>

SystemConfigDataStruct SystemConfigDataStruct::instance_;
SystemConfigDataStruct::SystemConfigDataStruct()
{
    instance_.config_param_.led_current = 255;
}

bool SystemConfigDataStruct::loadFromSettings(const std::string& f)
{

    std::ifstream ifile;
    ifile.open(f, std::ios::in);
    if (!ifile)
    {
        std::cerr << "Open Settings File Fail." << std::endl;
        return false;
    }

    std::vector<std::string> str_list;
    std::string line;

    while (getline(ifile, line)) // line中不包括每行的换行符
    {
        std::cout << line << std::endl;
        str_list.push_back(line);
    } 
    ifile.close();

        for (int i = 0; i < str_list.size(); i++)
    {
        // std::cout << str_list[i] << std::endl;

        std::vector<std::string> param_list = vStringSplit(str_list[i], ":");

        if (2 == param_list.size())
        { 

            param_list[1] = StringTrim(param_list[1]);

     
            if( "led_current" == param_list[0])
            {
                    instance_.config_param_.led_current = atoi(param_list[1].c_str());
            }
       

             
        }
    }


    return true;

}

bool SystemConfigDataStruct::saveToSettings(const std::string& f)
{

    std::ofstream ofile;
    ofile.open(f, std::ios::out);
    if (!ofile)
    {
        std::cerr << "Open File Fail." << std::endl;
        return false;
    }
  
    ofile << "led_current: " << instance_.config_param_.led_current << std::endl; 
  
    ofile.close();
   
	return true;
}


/***************************************************************************************************************************/
 

std::string& StringTrim(std::string &str)
{
    if (str.empty()){
        return str;
    }
    str.erase(0, str.find_first_not_of(" "));
    str.erase(str.find_last_not_of(" ") + 1);
    return str;
}

std::vector<std::string> vStringSplit(const std::string &s, const std::string &delim)
{
    std::vector<std::string> elems;
    size_t pos = 0;
    size_t len = s.length();
    size_t delim_len = delim.length();
    if (delim_len == 0)
        return elems;
    while (pos < len)
    {
        int find_pos = s.find(delim, pos);
        if (find_pos < 0)
        {
            elems.push_back(s.substr(pos, len - pos));
            break;
        }
        elems.push_back(s.substr(pos, find_pos - pos));
        pos = find_pos + delim_len;
    }
    return elems;
}
