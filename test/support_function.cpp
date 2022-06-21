#ifdef _WIN32  
#include <windows.h>
#elif __linux 
#include <sys/types.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#endif 
#include "support_function.h"
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
	auto milli = timestamp + (long long)8 * 60 * 60 * 1000; //�˴�ת��Ϊ����������ʱ�䣬���������ʱ����Ҫ�������޸�
	auto mTime = std::chrono::milliseconds(milli);
	auto tp = std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>(mTime);
	auto tt = std::chrono::system_clock::to_time_t(tp);
	std::tm* now = std::gmtime(&tt);
	//printf("%4d��%02d��%02d�� %02d:%02d:%02d\n", now->tm_year + 1900, now->tm_mon + 1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec);
	return now;
}


std::string GetTimeStamp()
{

	int msec = 0;
	char time_str[7][16];
	auto t = getTimeStamp(msec);
	//std::cout << "Millisecond timestamp is: " << t << std::endl;
	auto time_ptr = gettm(t);
	sprintf(time_str[0], "%02d", time_ptr->tm_year + 1900); //�·�Ҫ��1
	sprintf(time_str[1], "%02d", time_ptr->tm_mon + 1); //�·�Ҫ��1
	sprintf(time_str[2], "%02d", time_ptr->tm_mday);//��
	sprintf(time_str[3], "%02d", time_ptr->tm_hour);//ʱ
	sprintf(time_str[4], "%02d", time_ptr->tm_min);// ��
	sprintf(time_str[5], "%02d", time_ptr->tm_sec);//ʱ
	sprintf(time_str[6], "%02d", msec);// ��
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

bool compareNat(const std::string& a, const std::string& b)
{
	if (a.empty())
		return true;
	if (b.empty())
		return false;
	if (std::isdigit(a[0]) && !std::isdigit(b[0]))
		return true;
	if (!std::isdigit(a[0]) && std::isdigit(b[0]))
		return false;
	if (!std::isdigit(a[0]) && !std::isdigit(b[0]))
	{
		if (std::toupper(a[0]) == std::toupper(b[0]))
			return compareNat(a.substr(1), b.substr(1));
		return (std::toupper(a[0]) < std::toupper(b[0]));
	}

	// Both strings begin with digit --> parse both numbers
	std::istringstream issa(a);
	std::istringstream issb(b);
	int ia, ib;
	issa >> ia;
	issb >> ib;
	if (ia != ib)
		return ia < ib;

	// Numbers are the same --> remove numbers and recurse
	std::string anew, bnew;
	std::getline(issa, anew);
	std::getline(issb, bnew);
	return (compareNat(anew, bnew));
}
void getJustCurrentDir(std::string path, std::vector<std::string>& dirs)
{

#ifdef _WIN32 

	//文件句柄
	intptr_t hFile = 0;
	//文件信息 
	struct _finddata_t fileinfo;
	std::string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1) {
		do {
			if ((fileinfo.attrib & _A_SUBDIR)) {
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0) {
					dirs.push_back(path + "\\" + fileinfo.name);
					//files.push_back(p.assign(path).append("\\").append(fileinfo.name));

				}
			}

		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);

	}


#elif __linux

	DIR* dir = opendir(path.c_str());
	struct dirent* entry;

	std::vector<std::string> name_list;

	//然后通过while循环不断readdir，获取目录中的内容
	while ((entry = readdir(dir)) != 0)
	{

		if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0)
		{

			//
			if (entry->d_type == 4) //如果是子目录，继续递归搜索
			{
				//获取该结构体变量的成员函数d_name就得到了待扫描的文件，然后在使用sprintf函数加入文件绝对路径

				std::string name(entry->d_name);
				//  bool exist_flag = false;

				//  auto iter = name_list.begin();
				//  while (iter != name_list.end())
				//  {
				// 	 if (*iter == name)
				// 	 { // 这命令可以作为查找vetor元素的方法
				// 		 // vct_name_.erase(iter);	 // 删除
				// 		 // iter=vct_name_.erase(iter); //也可以这么写
				// 		 exist_flag = true;
				// 	 }
				// 	 iter++;
				//  }

				//  if (!exist_flag)
				//  {
				name_list.push_back(name);
				//  }


			}
		}


	}

	//最后关闭目录句柄closedir
	closedir(dir);
	std::sort(name_list.begin(), name_list.end(), compareNat);

	for (int i = 0; i < name_list.size(); i++)
	{
		std::string dir = path + "/" + name_list[i];
		dirs.push_back(dir);
	}

#endif 

}

void  getFiles(std::string path, std::vector<std::string>& files)
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
	struct dirent* entry;
	if (!(pDir = opendir(path.c_str())))
		return;
	while ((entry = readdir(pDir)) != 0)
	{
		if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0)
		{

			//
			if (entry->d_type == 8) //如果是子目录，继续递归搜索
			{
				//获取该结构体变量的成员函数d_name就得到了待扫描的文件，然后在使用sprintf函数加入文件绝对路径

				std::string name(entry->d_name);

				if (name.find(".bmp"))
				{
					files.push_back(path + "/" + name);
				}

			}
		}
	}
	closedir(pDir);
	std::sort(files.begin(), files.end(), compareNat);
#endif 


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
		//����ɫ
		for (int i = 0; i < nc * nr; i++)
		{
			if (point_cloud_buffer[i * 3 + 2] > 0.01)
				ofile << point_cloud_buffer[i * 3] << " " << point_cloud_buffer[i * 3 + 1] << " " << point_cloud_buffer[i * 3 + 2] << std::endl;
		}

	}
	else
	{
		if (1 == texture_map.channels())
		{
			//����ɫ 
			for (int i = 0; i < nc * nr; i++)
			{
				if (point_cloud_buffer[i * 3 + 2] > 0.01)
					ofile << point_cloud_buffer[i * 3] << " " << point_cloud_buffer[i * 3 + 1] << " " << point_cloud_buffer[i * 3 + 2] << " "
					<< (int)brightness_buffer[i] << " " << (int)brightness_buffer[i] << " " << (int)brightness_buffer[i] << std::endl;
			}
		}
		else if (3 == texture_map.channels())
		{
			//����ɫ 
			for (int i = 0; i < nc * nr; i++)
			{
				if (point_cloud_buffer[i * 3 + 2] > 0.01)
					ofile << point_cloud_buffer[i * 3] << " " << point_cloud_buffer[i * 3 + 1] << " " << point_cloud_buffer[i * 3 + 2] << " "
					<< (int)brightness_buffer[i * 3] << " " << (int)brightness_buffer[i * 3 + 1] << " " << (int)brightness_buffer[i * 3 + 2] << std::endl;
			}
		}

	}



	ofile.close();


	return true;
}

//��ȡz-map Roiͼ
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


bool renderBrightnessImage(cv::Mat brightness, cv::Mat& render_brightness)
{

	cv::Mat color_map(brightness.size(), CV_8UC3, cv::Scalar(0, 0, 0));

	int nr = color_map.rows;
	int nc = color_map.cols;

	for (int r = 0; r < nr; r++)
	{
		uchar* ptr_b = brightness.ptr<uchar>(r);
		cv::Vec3b* ptr_cb = color_map.ptr<cv::Vec3b>(r);
		for (int c = 0; c < nc; c++)
		{
			if (ptr_b[c] == 255)
			{
				ptr_cb[c][0] = 0;
				ptr_cb[c][1] = 0;
				ptr_cb[c][2] = 255;
			}
			else
			{
				ptr_cb[c][0] = ptr_b[c];
				ptr_cb[c][1] = ptr_b[c];
				ptr_cb[c][2] = ptr_b[c];
			}
		}

	}

	render_brightness = color_map.clone();
	return true;
}


bool renderErrorMap(cv::Mat err_map, cv::Mat& color_map, cv::Mat& gray_map, float low_v, float high_v)
{
	if (!err_map.data)
	{
		return false;
	}

	cv::Mat handle_map(err_map.size(), CV_8U, cv::Scalar(0));

	float range = high_v - low_v;

	for (int r = 0; r < handle_map.rows; r++)
	{
		uchar* ptr_h = handle_map.ptr<uchar>(r);
		double* ptr_dr = err_map.ptr<double>(r);

		for (int c = 0; c < handle_map.cols; c++)
		{

			ptr_h[c] = 0.5 + 255.0 * (ptr_dr[c] - low_v) / range;

		}
	}

	gray_map = handle_map.clone();

	cv::applyColorMap(handle_map, color_map, cv::COLORMAP_JET);

	for (int r = 0; r < color_map.rows; r++)
	{
		cv::Vec3b* ptr_h = color_map.ptr<cv::Vec3b>(r);
		double* ptr_dr = err_map.ptr<double>(r);
		uchar* ptr_g = gray_map.ptr<uchar>(r);

		for (int c = 0; c < color_map.cols; c++)
		{

			if (0 == ptr_dr[c])
			{
				ptr_h[c][0] = 0;
				ptr_h[c][1] = 0;
				ptr_h[c][2] = 0;
			}
			else if (high_v < ptr_dr[c])
			{
				ptr_h[c][0] = 255;
				ptr_h[c][1] = 255;
				ptr_h[c][2] = 255;

				ptr_g[c] = 255;
			}
		}
	}

	renderBrightnessImage(gray_map, gray_map);

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


bool compensatePhaseBaseScharr(cv::Mat& normal_phase, cv::Mat brightness, int offset_value)
{
	if (normal_phase.empty() || brightness.empty())
		return false;

	int nr = brightness.rows;
	int nc = brightness.cols;

	cv::Mat sobel_brightness = brightness.clone();
	cv::Mat sobel_grad_x, scharr_x;

	//cv::Sobel(sobel_brightness, sobel_grad_x, CV_64F, 1, 0, 1);

	Scharr(sobel_brightness, scharr_x, CV_64F, 1, 0, 1, 0, cv::BORDER_DEFAULT);
	cv::GaussianBlur(scharr_x, scharr_x, cv::Size(5, 5), 3, 3);


	for (int r = 0; r < nr; r++)
	{
		double* ptr_sobel = scharr_x.ptr<double>(r);
		double* ptr_phase_map = normal_phase.ptr<double>(r);

		for (int c = 0; c < nc; c++)
		{
			if (std::abs(ptr_sobel[c]) < 300)
			{
				ptr_sobel[c] = 0;
			}
			else
			{
				if (ptr_phase_map[c] > 0)
				{
					ptr_phase_map[c] -= ptr_sobel[c] * 0.0000001 * offset_value;
				}
			}
		}

	}


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
