#include "Support_Function.h"
#include "iostream" 
#include <fstream>


/**************************************************************************************************************************/

std::time_t getTimeStamp(int& msec)
{
	std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
	auto tmp = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
	seconds sec = duration_cast<seconds>(tp.time_since_epoch());


	std::time_t timestamp = tmp.count();

	msec = tmp.count() - sec.count() * 1000;
	//std::time_t timestamp = std::chrono::system_clock::to_time_t(tp);
	return timestamp;
}

std::tm* gettm(long long timestamp)
{
	auto milli = timestamp + (long long)8 * 60 * 60 * 1000; //此处转化为东八区北京时间，如果是其它时区需要按需求修改
	auto mTime = std::chrono::milliseconds(milli);
	auto tp = std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>(mTime);
	auto tt = std::chrono::system_clock::to_time_t(tp);
	std::tm* now = std::gmtime(&tt);
	//printf("%4d年%02d月%02d日 %02d:%02d:%02d\n", now->tm_year + 1900, now->tm_mon + 1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec);
	return now;
}


std::string GetTimeStamp()
{

	int msec = 0;
	char time_str[7][16];
	auto t = getTimeStamp(msec);
	//std::cout << "Millisecond timestamp is: " << t << std::endl;
	auto time_ptr = gettm(t);
	sprintf(time_str[0], "%02d", time_ptr->tm_year + 1900); //月份要加1
	sprintf(time_str[1], "%02d", time_ptr->tm_mon + 1); //月份要加1
	sprintf(time_str[2], "%02d", time_ptr->tm_mday);//天
	sprintf(time_str[3], "%02d", time_ptr->tm_hour);//时
	sprintf(time_str[4], "%02d", time_ptr->tm_min);// 分
	sprintf(time_str[5], "%02d", time_ptr->tm_sec);//时
	sprintf(time_str[6], "%02d", msec);// 分
	//for (int i = 0; i < 7; i++)
	//{
	//	std::cout << "time_str[" << i << "] is: " << time_str[i] << std::endl;
	//}

	std::string timestamp = "";

	timestamp += time_str[0];
	timestamp += "-";
	timestamp += time_str[1];
	timestamp += "-";
	timestamp += time_str[2];
	timestamp += " ";
	timestamp += time_str[3];
	timestamp += ":";
	timestamp += time_str[4];
	timestamp += ":";
	timestamp += time_str[5];
	timestamp += ",";
	timestamp += time_str[6];

	//std::cout << timestamp << std::endl;

	return timestamp;
}

/**************************************************************************************************************************/
//path 
std::vector<std::string> vStringSplit(const std::string& s, const std::string& delim)
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

bool getFilesList(std::string dirs, std::vector<std::vector<std::string>>& files_list)
{
	std::vector<std::string> dir_list; 
	getJustCurrentDir(dirs, dir_list);

	if (dir_list.empty())
	{
		return false;
	}

	files_list.clear();

	for (int i = 0; i < dir_list.size(); i++)
	{
		std::string dir = dir_list[i];

		std::vector<std::string> files; 
		getFiles(dir, files); 

		files_list.push_back(files); 
	}

	return true;
}

void getJustCurrentDir(std::string path, std::vector<std::string>& dirs) {
	//文件句柄
	intptr_t hFile = 0;
	//文件信息 
	struct _finddata_t fileinfo;
	std::string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1) {
		do {
			if ((fileinfo.attrib & _A_SUBDIR)) {
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0) {
					dirs.push_back(path /*+ "/" */+ fileinfo.name);
					//files.push_back(p.assign(path).append("\\").append(fileinfo.name));

				} 
			}

		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);

	}

}

void  getFiles(std::string path, std::vector<std::string>& files)
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



/**************************************************************************************************************************/
bool SavePointToTxt(cv::Mat deep_map, std::string path, cv::Mat texture_map)
{
	if (deep_map.empty())
		return false;


	int nr = deep_map.rows;
	int nc = deep_map.cols;

	std::ofstream ofile;
	ofile.open(path);

	double* point_cloud_buffer = deep_map.ptr<double>(0);
	uchar* brightness_buffer = texture_map.ptr<uchar>(0);

	if (texture_map.empty())
	{
		//无颜色
		for (int i = 0; i < nc * nr; i++)
		{
			if (point_cloud_buffer[i * 3 + 2] > 0.01)
				ofile << point_cloud_buffer[i * 3] << " " << point_cloud_buffer[i * 3 + 1] << " " << point_cloud_buffer[i * 3 + 2] <<std::endl;
		}

	}
	else
	{
		//有颜色 
		for (int i = 0; i < nc * nr; i++)
		{
			if (point_cloud_buffer[i * 3 + 2] > 0.01)
				ofile << point_cloud_buffer[i * 3] << " " << point_cloud_buffer[i * 3 + 1] << " " << point_cloud_buffer[i * 3 + 2] << " "
				<< (int)brightness_buffer[i] << " " << (int)brightness_buffer[i] << " " << (int)brightness_buffer[i] << std::endl;
		}
	}



	ofile.close();


	return true;
}

//截取z-map Roi图
bool MaskZMap(cv::Mat& z_map, cv::Mat mask)
{
	if (!z_map.data)
	{
		return false;
	}
	if (!mask.data)
	{
		return false;
	}

	if (3 == z_map.channels())
	{
		for (int r = 0; r < z_map.rows; r++)
		{
			uchar* ptr_m = mask.ptr<uchar>(r);
			cv::Vec3b* ptr_dr = z_map.ptr<cv::Vec3b>(r);

			for (int c = 0; c < z_map.cols; c++)
			{
				if (0 == ptr_m[c])
				{
					ptr_dr[c][0] = 0;
					ptr_dr[c][1] = 0;
					ptr_dr[c][2] = 0;
				}
			}
		}
	}

	if (1 == z_map.channels())
	{
		for (int r = 0; r < z_map.rows; r++)
		{
			uchar* ptr_m = mask.ptr<uchar>(r);
			uchar* ptr_dr = z_map.ptr<uchar>(r);

			for (int c = 0; c < z_map.cols; c++)
			{
				if (0 == ptr_m[c])
				{
					ptr_dr[c] = 0;
				}
			}
		}
	}



	return true;
}


bool MapToColor(cv::Mat deep_map, cv::Mat& color_map, cv::Mat& grey_map, int low_z, int high_z)
{

	if (!deep_map.data)
	{
		return false;
	}

	cv::Mat handle_map(deep_map.size(), CV_8U, cv::Scalar(0));

	int range = high_z - low_z;

	for (int r = 0; r < handle_map.rows; r++)
	{
		uchar* ptr_h = handle_map.ptr<uchar>(r);
		cv::Vec3d* ptr_dr = deep_map.ptr<cv::Vec3d>(r);

		for (int c = 0; c < handle_map.cols; c++)
		{
			if (0 != ptr_dr[c][0] && 0 != ptr_dr[c][1] && 0 != ptr_dr[c][2])
			{
				ptr_h[c] = 0.5 + 255.0 * (ptr_dr[c][2] - low_z) / range;
			}
		}
	}

	grey_map = handle_map.clone();

	cv::applyColorMap(handle_map, color_map, cv::COLORMAP_JET);

	return true;

}

bool MergeTextureMap(std::vector<cv::Mat> patterns, cv::Mat& texture_map)
{
	if (patterns.empty())
	{
		texture_map = cv::Mat();
		return false;
	}


	int nr = patterns[0].rows;
	int nc = patterns[0].cols;

	int num = patterns.size();

	cv::Mat map(nr, nc, CV_8U, cv::Scalar(0));


	switch (num)
	{
	case 2:
	{
		for (int r = 0; r < nr; r++)
		{
			uchar* ptr_p0 = patterns[0].ptr<uchar>(r);
			uchar* ptr_p1 = patterns[1].ptr<uchar>(r);

			uchar* ptr_m = map.ptr<uchar>(r);


			for (int c = 0; c < nc; c++)
			{
				double val = ptr_p0[c] + ptr_p1[c];

				if (val > 255)
				{
					val = 255;
				}

				ptr_m[c] = val;
			}
		}
	}
	break;

	case 4:
	{
		for (int r = 0; r < nr; r++)
		{
			uchar* ptr_p0 = patterns[0].ptr<uchar>(r);
			uchar* ptr_p1 = patterns[1].ptr<uchar>(r);
			uchar* ptr_p2 = patterns[2].ptr<uchar>(r);
			uchar* ptr_p3 = patterns[3].ptr<uchar>(r);

			uchar* ptr_m = map.ptr<uchar>(r);


			for (int c = 0; c < nc; c++)
			{
				double val = (ptr_p0[c] + ptr_p1[c] + ptr_p2[c] + ptr_p3[c]) / 4.0;
				if (val > 255)
				{
					val = 255;
				}

				//double a = ptr_p3[c] - ptr_p1[c];
				//double b = ptr_p0[c] - ptr_p2[c];

				//double val = std::sqrtf(a * a + b * b) + 0.5;

				//val += 127;

				//if (val > 255)
				//{
				//	val = 255;
				//}


				ptr_m[c] = val;
			}
		}
	}
	break;

	default:
	{
		texture_map = cv::Mat();
		return false;
	}
	}


	texture_map = map.clone();


	return true;

}
