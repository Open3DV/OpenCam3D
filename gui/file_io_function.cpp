#include "file_io_function.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>


FileIoFunction::FileIoFunction()
{
	calib_path_ = "../config_files/calib.xml";
}


FileIoFunction::~FileIoFunction()
{
}

/****************************************************************************************************************/
//DF8


bool FileIoFunction::dirConvertToImagePathList(QString dir_path, std::vector<QStringList>& images_path_list)
{
	//QString path = QFileDialog::getExistingDirectory(this, "Select Image Folder:", "G:/Code/StructureLight/Images");

	QStringList dir_path_list;

	QStringList mImgNames;
	QDir dir(dir_path);
	dir_path = dir.fromNativeSeparators(dir_path);//  "\\"转为"/" 
	if (!dir.exists())
		mImgNames = QStringList("");
	dir.setFilter(QDir::Dirs | QDir::NoDotAndDotDot);
	dir.setSorting(QDir::Name);
	mImgNames = dir.entryList();
	for (int i = 0; i < mImgNames.size(); ++i)
	{
		//qDebug() << "entryList: " << i << "-" << dir_path + "/" + mImgNames[i];
		dir_path_list.push_back(dir_path + "/" + mImgNames[i]);
	}


	std::vector<QStringList> images_list;


	for (int d_i = 0; d_i < dir_path_list.size(); d_i++)
	{
		std::vector<QString> images_paths;

		QDir images_dir(dir_path_list[d_i]);
		if (!images_dir.exists())
		{
			return false;
		}
		//获取所选文件类型过滤器
		QStringList filters;
		filters << QString("*.bmp");

		//定义迭代器并设置过滤器
		QDirIterator dir_iterator(dir_path_list[d_i], filters,
			QDir::Files | QDir::NoSymLinks,
			QDirIterator::Subdirectories);
		QStringList path_list;
		while (dir_iterator.hasNext())
		{
			dir_iterator.next();
			QFileInfo file_info = dir_iterator.fileInfo();
			QString absolute_file_path = file_info.absoluteFilePath();
			path_list.append(absolute_file_path);
			//qDebug() << absolute_file_path;

		}

		images_list.push_back(path_list);
	}

	images_path_list = images_list;

	return true;

}


bool FileIoFunction::mat_to_float(cv::Mat &mat)
{
	if(!mat.data)
	{
		return false;
	}


	mat.convertTo(mat, CV_32F);
	return true;
}

bool FileIoFunction::readImages(std::vector<cv::Mat> &patterns, QString path)
{

	QDir dir_child(path);
	if (!dir_child.exists())   //判断目录是否存在
	{
		return false;
	}

	QString save_name = dir_child.dirName();

	patterns.clear();

	//获取所选文件类型过滤器
	QStringList filters;
	filters << QString("*.bmp");

	//定义迭代器并设置过滤器
	QDirIterator dir_iterator(path, filters,
		QDir::Files | QDir::NoSymLinks,
		QDirIterator::Subdirectories);
	QStringList path_list;
	while (dir_iterator.hasNext())
	{
		dir_iterator.next();
		QFileInfo file_info = dir_iterator.fileInfo();
		QString absolute_file_path = file_info.absoluteFilePath();
		path_list.append(absolute_file_path);
	}


	for (int i = 0; i < path_list.size(); i++)
	{
		cv::Mat img = cv::imread(path_list[i].toLocal8Bit().toStdString(), 0);

		//cv::blur(img, img, cv::Size(15, 15));
		//cv::GaussianBlur(img, img, cv::Size(15, 15), 5, 5);

		patterns.push_back(img);
		qDebug() << "read: " << path_list[i];
	}

	return true; 
}



/****************************************************************************************************************/
 
//保存点云和z-map图
bool FileIoFunction::savePointsToFolder(QString dir_path, cv::Mat deep_map, cv::Mat mask, cv::Mat texture_map)
{

	if (!deep_map.data || 3!= deep_map.channels())
	{
		return false;
	}


	FileIoFunction file_io_machine;

	cv::Mat color_map, grey_map;
	file_io_machine.mapToColor(deep_map, color_map, grey_map,500, 1500);
	file_io_machine.maskZMap(color_map, mask);


	FileIoFunction file_io_m;
	//int image_num = 0;
	//file_io_m.getFileNum(dir_path, "*.txt", image_num); 
	//image_num++;

	//QString points_path = dir_path + "/res_" + QString::number(image_num) + "_points.txt";
	//QString deep_map_path = dir_path + "/res_" + QString::number(image_num) + "_z_color_map.bmp";
	//QString deep_grey_map_path = dir_path + "/res_" + QString::number(image_num) + "_z_grey_map.bmp";

	//QString points_path = dir_path +  "/points.txt";
	//QString deep_map_path = dir_path + "/color_z_map.bmp";
	//QString deep_grey_map_path = dir_path + "/grey_z_map.bmp";


	QDateTime time = QDateTime::currentDateTime();
	QString time_str = time.toString("yyyy-MM-dd_hh-mm-ss");
	QString save_points_dir = dir_path + "/" + time_str + "_points.txt";
	QString save_depth_txt_dir = dir_path + "/" + time_str + "_depth.txt";
	QString save_confidence_dir = dir_path + "/" + time_str + "_confidence.bmp";
	QString save_depth_dir = dir_path + "/" + time_str + "_depth.bmp";
	QString save_brightness_dir = dir_path + "/" + time_str + "_brightness.bmp";


	//cv::imwrite(save_brightness_dir.toStdString(), texture_map);
	//cv::imwrite(save_confidence_dir.toStdString(), confidence);
	cv::imwrite(save_depth_dir.toStdString(), color_map);
	file_io_machine.SavePointToTxt(deep_map, save_points_dir, texture_map);
	file_io_machine.saveDepthMapToTxt(deep_map, save_depth_txt_dir);

	//file_io_machine.SavePointToTxt(deep_map, points_path, texture_map);
	//cv::imwrite(deep_map_path.toStdString(), color_map);
	//cv::imwrite(deep_grey_map_path.toStdString(), grey_map);

	if (texture_map.data)
	{
		//QString texture_map_path = dir_path + "/res_" + QString::number(image_num) + "_brightness.bmp";
		//QString texture_map_path = dir_path +  "/brightness.bmp";
		cv::imwrite(save_brightness_dir.toStdString(), texture_map);
	}


	return true;

}

bool FileIoFunction::saveMoreExposurePatternsToFolder(QString dir_path, std::vector<std::vector<cv::Mat>> patters_list)
{
	if(patters_list.empty())
	{
		return false;
	}

	QDateTime time = QDateTime::currentDateTime();
	QString time_str = time.toString("yyyy-MM-dd_hh-mm-ss");

	QString save_dir = dir_path + "/" + time_str + "/";


	for(int i= 0;i< patters_list.size();i++)
	{
		QString path = save_dir + "0" + QString::number(i + 1);

		savePatterns(path, patters_list[i]);
	}


	return true;
}


bool FileIoFunction::savePatterns(QString dir_path, std::vector<cv::Mat> patters)
{

	if(patters.empty())
	{
		return false;
	}

	QDir dir(dir_path);

	QString path = dir.absolutePath();
	//QDir dir;
	if (!dir.exists(path))
	{
		bool res = dir.mkpath(path);

	}

	qDebug() << "Save " << patters.size() << " images in " << path;

	path += "/";
	for (int i = 0; i < patters.size(); i++)
	{

		QString save_path = path;
		if (i < 9)
		{
			save_path += "0" + QString::number(i + 1) + ".bmp";
		}
		else
		{
			save_path += QString::number(i + 1) + ".bmp";
		}

		cv::imwrite(save_path.toStdString(), patters[i]);

	}

	return true;
}

//保存图案至文件夹，文件名自定义
bool FileIoFunction::savePatternsToFolder(QString dir_path, std::vector<cv::Mat> patters)
{

	if(patters.empty())
	{
		return false;
	}


	FileIoFunction file_io_machine;

	QDir dir(dir_path); 

	QString path = dir.absolutePath();
	int num = 0;
	bool ret = file_io_machine.getFoldersNum(path, num);

	QString folder_str = "";
	if (num < 10)
	{
		folder_str = "0" + QString::number(num + 1);
	}
	else
	{
		folder_str = QString::number(num + 1);
	}

	path += "/";
	path += folder_str;

	//QDir dir;
	if (!dir.exists(path))
	{
		bool res = dir.mkpath(path); 

	}

	qDebug() << "Save " << patters.size() << " images in " << path;

	path += "/";
	for (int i = 0; i < patters.size(); i++)
	{

		QString save_path = path;
		if (i < 9)
		{
			save_path += "0" + QString::number(i + 1) + ".bmp";
		}
		else
		{
			save_path += QString::number(i + 1) + ".bmp";
		}

		cv::imwrite(save_path.toStdString(), patters[i]);

	}

	return true;
	 
}


bool FileIoFunction::mergeTextureMap(std::vector<cv::Mat> patterns, cv::Mat &texture_map)
{
	if(patterns.empty())
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
		for (int r = 0; r< nr; r++)
		{
			uchar* ptr_p0 = patterns[0].ptr<uchar>(r);
			uchar* ptr_p1 = patterns[1].ptr<uchar>(r); 

			uchar* ptr_m = map.ptr<uchar>(r);


			for(int c= 0;c< nc;c++)
			{
				double val = ptr_p0[c] + ptr_p1[c];

				if(val> 255)
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
		for (int r = 0; r< nr; r++)
		{
			uchar* ptr_p0 = patterns[0].ptr<uchar>(r);
			uchar* ptr_p1 = patterns[1].ptr<uchar>(r);
			uchar* ptr_p2 = patterns[2].ptr<uchar>(r);
			uchar* ptr_p3 = patterns[3].ptr<uchar>(r);

			uchar* ptr_m = map.ptr<uchar>(r);


			for (int c = 0; c< nc; c++)
			{
				double val = (ptr_p0[c] + ptr_p1[c] + ptr_p2[c] + ptr_p3[c]) / 4.0;
				if(val> 255)
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

bool FileIoFunction::sortPatternsBaseInvertProject(std::vector<cv::Mat> invert_patterns, std::vector<cv::Mat> &sort_patterns)
{
	if(invert_patterns.empty())
	{
		return false;
	}

	sort_patterns.clear();

	for (int i = 0; i < invert_patterns.size(); i += 4)
	{
		sort_patterns.push_back(invert_patterns[i + 0]);
		sort_patterns.push_back(invert_patterns[i + 2]);
		sort_patterns.push_back(invert_patterns[i + 1]);
		sort_patterns.push_back(invert_patterns[i + 3]);
	}

	return true;
}

//相位图转灰度图用于显示
bool FileIoFunction::mapToGrey(cv::Mat phase_map, int period_num, cv::Mat &show_map)
{
	if(!phase_map.data)
	{
		return false;
	}


	double phase_max = 2 * CV_PI*std::pow(2.0, period_num - 1);

	cv::Mat grey(phase_map.size(), CV_8U, cv::Scalar(0));

	for (int r = 0; r< phase_map.rows; r++)
	{
		uchar* ptr_g = grey.ptr<uchar>(r);
		double * ptr_dr = phase_map.ptr<double>(r);

		for (int c = 0; c< phase_map.cols; c++)
		{
			ptr_g[c] = 255.0*ptr_dr[c] / phase_max;
		}
	}

	show_map = grey.clone();

	return true;
}


//截取z-map Roi图
bool FileIoFunction::maskZMap(cv::Mat &z_map, cv::Mat mask)
{
	if(!z_map.data)
	{
		return false;
	}
	if (!mask.data)
	{
		return false;
	}

	if(3 == z_map.channels())
	{ 
		for (int r = 0; r< z_map.rows; r++)
		{
			uchar* ptr_m = mask.ptr<uchar>(r);
			cv::Vec3b * ptr_dr = z_map.ptr<cv::Vec3b>(r);

			for (int c = 0; c< z_map.cols; c++)
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
		for (int r = 0; r< z_map.rows; r++)
		{
			uchar* ptr_m = mask.ptr<uchar>(r);
			uchar * ptr_dr = z_map.ptr<uchar>(r);

			for (int c = 0; c< z_map.cols; c++)
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


bool  FileIoFunction::cutDeepMapBaseZ(cv::Mat &deep_map, cv::Mat &mask, int low_z, int high_z)
{

	for (int r = 0; r< deep_map.rows; r++)
	{
		uchar* ptr_m = mask.ptr<uchar>(r);
		cv::Vec3d * ptr_dr = deep_map.ptr<cv::Vec3d>(r);

		for (int c = 0; c< deep_map.cols; c++)
		{

			if (ptr_m[c] > 0)
			{
				if (high_z < ptr_dr[c][2] || low_z > ptr_dr[c][2])
				{
					ptr_dr[c][0] = 0;
					ptr_dr[c][1] = 0;
					ptr_dr[c][2] = 0;
					ptr_m[c] = 0;
				}
			}
			else
			{
				ptr_dr[c][0] = 0;
				ptr_dr[c][1] = 0;
				ptr_dr[c][2] = 0;
			}


		}
	}

	return true;
}

//深度图转z-map图
bool  FileIoFunction::depthToColor(cv::Mat depth_map, cv::Mat& color_map, cv::Mat& grey_map, int low_z, int high_z)
{

	if (!depth_map.data)
	{
		return false;
	}

	cv::Mat handle_map(depth_map.size(), CV_8U, cv::Scalar(0));
	cv::Mat mask_map(depth_map.size(), CV_8U, cv::Scalar(0));

	int range = high_z - low_z;

	if (depth_map.type() != CV_32FC1)
	{
		depth_map.convertTo(depth_map, CV_32FC1);
	}



	for (int r = 0; r < handle_map.rows; r++)
	{
		uchar* ptr_h = handle_map.ptr<uchar>(r);
		uchar* ptr_m = mask_map.ptr<uchar>(r);
		float* ptr_dr = depth_map.ptr<float>(r);

		for (int c = 0; c < handle_map.cols; c++)
		{
			if (low_z<= ptr_dr[c] && ptr_dr[c] <= high_z)
			{
				ptr_h[c] = 0.45 + 255.0 * (ptr_dr[c] - low_z) / range;
				ptr_m[c] = 255;
			}
		}
	}

	grey_map = handle_map.clone();

	cv::applyColorMap(handle_map, color_map, cv::COLORMAP_JET);

	maskZMap(color_map, mask_map);

	return true;
}

bool FileIoFunction::mapToColor(cv::Mat deep_map, cv::Mat &color_map, cv::Mat &grey_map , int low_z, int high_z)
{

	if (!deep_map.data)
	{
		return false;
	}

	cv::Mat handle_map(deep_map.size(), CV_8U, cv::Scalar(0));

	int range = high_z - low_z;

	for (int r = 0; r< handle_map.rows; r++)
	{
		uchar* ptr_h = handle_map.ptr<uchar>(r);
		cv::Vec3d * ptr_dr = deep_map.ptr<cv::Vec3d>(r);

		for (int c = 0; c< handle_map.cols; c++)
		{
			if (0 != ptr_dr[c][0] && 0 != ptr_dr[c][1] && 0 != ptr_dr[c][2])
			{
				ptr_h[c] = 0.5 + 255.0*(ptr_dr[c][2] - low_z) / range;
			}
		}
	}

	grey_map = handle_map.clone();

	cv::applyColorMap(handle_map, color_map, cv::COLORMAP_JET);

	return true;

}

bool FileIoFunction::getFileNum(QString path, QString suffix, int &num)
{
	if (path.isEmpty())
	{
		return false;
	}

	QDir *dir = new QDir(path);
	QStringList filter;
	filter << suffix;
	dir->setNameFilters(filter); //过滤文件类型
	QList<QFileInfo> *fileInfo = new QList<QFileInfo>(dir->entryInfoList(filter));
	num = fileInfo->count();  //文件个数 
	return true;
}


bool FileIoFunction::getFoldersNum(QString path, int &num)
{
	if (path.isEmpty())
	{
		return false;
	}

	QStringList mFolderNames;
	QDir dir(path);
	path = dir.fromNativeSeparators(path);//  "\\"תΪ"/" 
	if (!dir.exists())
		mFolderNames = QStringList("");
	dir.setFilter(QDir::Dirs | QDir::NoDotAndDotDot);
	dir.setSorting(QDir::Name);
	mFolderNames = dir.entryList();

	num = mFolderNames.size();

	return true;
}

bool FileIoFunction::stringToMat(QString str, cv::Mat& out_mat)
{
	QString mid_str = str.mid(1, str.size() - 2);

	QStringList str_list = mid_str.split(";");

	if (str_list.isEmpty())
	{
		return false;
	}

	std::vector<std::vector<double>> val_list;


	for (int r = 0; r< str_list.size() - 1; r++)
	{
		QString str = str_list[r];

		std::vector<double> vals;

		QStringList val_str_list = str.split(",");

		for (int c = 0; c< val_str_list.size(); c++)
		{
			vals.push_back(val_str_list[c].toDouble());
		}
		val_list.push_back(vals);
	}


	cv::Mat data(val_list.size(), val_list[0].size(), CV_64F, cv::Scalar(0));

	for (int r = 0; r< data.rows; r++)
	{
		double* ptr_d = data.ptr<double>(r);

		for (int c = 0; c< data.cols; c++)
		{
			ptr_d[c] = val_list[r][c];
		}

	}

	out_mat = data.clone();

	return true;

}

bool FileIoFunction::matToString(cv::Mat in_mat, QString& str)
{
	if (!in_mat.data)
	{
		return false;
	}


	str = "[";
	for (int r = 0; r< in_mat.rows; r++)
	{
		double* ptr_i = in_mat.ptr<double>(r);

		for (int c = 0; c< in_mat.cols; c++)
		{
			str += QString::number(ptr_i[c]);

			if (c == in_mat.cols - 1)
			{
				str += ";";
			}
			else
			{
				str += ",";
			}
		}
	}
	str += "]";


	return true;
}

bool FileIoFunction::readCalibXml(cv::Mat& camera_intrinsic, cv::Mat& project_intrinsic, cv::Mat& camera_distortion,
	cv::Mat& projector_distortion, cv::Mat& rotation_matrix, cv::Mat& translation_matrix,cv::Mat &M_1,cv::Mat &M_2)
{
	//打开或创建文件
	QFile file(calib_path_); //相对路径、绝对路径、资源路径都行
	if (!file.open(QFile::ReadOnly))
		return false;

	QDomDocument doc;
	if (!doc.setContent(&file))
	{
		file.close();
		return false;
	}
	file.close();

	QDomElement root = doc.documentElement(); //返回根节点
											  //qDebug() << root.nodeName();
	QDomNode node = root.firstChild(); //获得第一个子节点
	while (!node.isNull())  //如果节点不空
	{
		if (node.isElement()) //如果节点是元素
		{
			QDomElement e = node.toElement(); //转换为元素，注意元素和节点是两个数据结构，其实差不多
											  //qDebug() << e.tagName() << " " << e.attribute("id") << " " << e.attribute("time"); //打印键值对，tagName和nodeName是一个东西

			QDomNodeList list = e.childNodes();
			for (int i = 0; i<list.count(); i++) //遍历子元素，count和size都可以用,可用于标签数计数
			{
				QDomNode n = list.at(i);
				if (node.isElement())
				{

					//qDebug() << n.nodeName() << ":" << n.toElement().text();
					if ("camera_intrinsic" == n.nodeName())
					{
						stringToMat(n.toElement().text(), camera_intrinsic);
					}
					else if ("camera_distortion" == n.nodeName())
					{
						stringToMat(n.toElement().text(), camera_distortion);
					}
					else if ("projector_instrinsic" == n.nodeName())
					{
						stringToMat(n.toElement().text(), project_intrinsic);
					}
					else if ("projector_distortion" == n.nodeName())
					{
						stringToMat(n.toElement().text(), projector_distortion);
					}
					else if ("rotation_matrix" == n.nodeName())
					{
						stringToMat(n.toElement().text(), rotation_matrix);
					}
					else if ("translation_matrix" == n.nodeName())
					{
						stringToMat(n.toElement().text(), translation_matrix);
					}
					else
					{
						qDebug() << "bad: " << n.nodeName() << ":" << n.toElement().text();

					}
				}
			}
		}
		node = node.nextSibling(); //下一个兄弟节点,nextSiblingElement()是下一个兄弟元素，都差不多
	}



	/**************************************************************************************************************/
	if (!camera_intrinsic.data || !camera_distortion.data || !project_intrinsic.data ||
		!projector_distortion.data || !rotation_matrix.data || !translation_matrix.data)
	{
		return false;
	}



	/******************************************************************************************************************/

	cv::Mat E_1 = cv::Mat::eye(3, 4, CV_64FC1);
	cv::Mat E_2 = cv::Mat::zeros(3, 4, CV_64FC1);

	M_1 = cv::Mat::zeros(3, 4, CV_64FC1);
	M_2 = cv::Mat::zeros(3, 4, CV_64FC1);

	for (int i = 0; i< rotation_matrix.rows; i++)
	{
		for (int j = 0; j<rotation_matrix.cols; j++)
		{
			E_2.at<double>(i, j) = rotation_matrix.at<double>(i, j);
		}
	}

	for (int i = 0; i<3; i++)
	{
		E_2.at<double>(i, 3) = translation_matrix.at<double>(i);
	}


	M_1 = camera_intrinsic*E_1;
	M_2 = project_intrinsic*E_2;

	//double value_b_ = sqrt(pow(translation_matrix.at<double>(0, 0), 2) + pow(translation_matrix.at<double>(1, 0), 2) + pow(translation_matrix.at<double>(2, 0), 2));

	/********************************************************************************************************************/


	return true;
}

bool FileIoFunction::readCalibXml(QString path, cv::Mat& camera_intrinsic, cv::Mat& project_intrinsic, cv::Mat& camera_distortion,
	cv::Mat& projector_distortion, cv::Mat& rotation_matrix, cv::Mat& translation_matrix)
{
	//打开或创建文件
	QFile file(path); //相对路径、绝对路径、资源路径都行
	if (!file.open(QFile::ReadOnly))
		return false;

	QDomDocument doc;
	if (!doc.setContent(&file))
	{
		file.close();
		return false;
	}
	file.close();

	QDomElement root = doc.documentElement(); //返回根节点
											  //qDebug() << root.nodeName();
	QDomNode node = root.firstChild(); //获得第一个子节点
	while (!node.isNull())  //如果节点不空
	{
		if (node.isElement()) //如果节点是元素
		{
			QDomElement e = node.toElement(); //转换为元素，注意元素和节点是两个数据结构，其实差不多
											  //qDebug() << e.tagName() << " " << e.attribute("id") << " " << e.attribute("time"); //打印键值对，tagName和nodeName是一个东西

			QDomNodeList list = e.childNodes();
			for (int i = 0; i<list.count(); i++) //遍历子元素，count和size都可以用,可用于标签数计数
			{
				QDomNode n = list.at(i);
				if (node.isElement())
				{

					//qDebug() << n.nodeName() << ":" << n.toElement().text();
					if ("camera_intrinsic" == n.nodeName())
					{
						stringToMat(n.toElement().text(), camera_intrinsic);
					}
					else if ("camera_distortion" == n.nodeName())
					{
						stringToMat(n.toElement().text(), camera_distortion);
					}
					else if ("projector_instrinsic" == n.nodeName())
					{
						stringToMat(n.toElement().text(), project_intrinsic);
					}
					else if ("projector_distortion" == n.nodeName())
					{
						stringToMat(n.toElement().text(), projector_distortion);
					}
					else if ("rotation_matrix" == n.nodeName())
					{
						stringToMat(n.toElement().text(), rotation_matrix);
					}
					else if ("translation_matrix" == n.nodeName())
					{
						stringToMat(n.toElement().text(), translation_matrix);
					}
					else
					{
						qDebug() << "bad: " << n.nodeName() << ":" << n.toElement().text();

					}
				}
			}
		}
		node = node.nextSibling(); //下一个兄弟节点,nextSiblingElement()是下一个兄弟元素，都差不多
	}



	/**************************************************************************************************************/
	if (!camera_intrinsic.data || !camera_distortion.data || !project_intrinsic.data ||
		!projector_distortion.data || !rotation_matrix.data || !translation_matrix.data)
	{
		return false;
	}



	/******************************************************************************************************************/




	return true;
}

//保存深度图到txt文件
bool FileIoFunction::saveDepthMapToTxt(cv::Mat points_cloud_mat, QString path)
{
	if (points_cloud_mat.empty())
	{
		return false;
	}

	if (path.isEmpty())
	{
		return false;
	}

	QDir dir(path);
	path = dir.absolutePath();

	QFile file(path);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
	{
		qDebug() << "Save points Error";
		return false;
	}
	else
	{

			QTextStream textStream(&file);
			QString str = "";

			//deep_mat = deep_mat / 1000.0;

			for (int r = 0; r < points_cloud_mat.rows; r++)
			{
				cv::Vec3d * ptr_dr = points_cloud_mat.ptr<cv::Vec3d>(r);
				for (int c = 0; c < points_cloud_mat.cols; c++)
				{
					//if (0 != ptr_dr[c][0] && 0 != ptr_dr[c][1] && 0 != ptr_dr[c][2])
					//{
						str += QString::number(ptr_dr[c][2]);
						str += " ";

					//}

				}
			}

			textStream << str;
			file.close();
		

		qDebug() << "Save depth map: " << path;
	}

	return true;
}

bool FileIoFunction::SavePointToTxt(cv::Mat deep_mat, QString path, cv::Mat texture_map)
{
	if(deep_mat.empty())
	{
		return false;
	}

	if(path.isEmpty())
	{
		return false;
	}

	QDir dir(path);
	path = dir.absolutePath();

	QFile file(path);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
	{
		qDebug() << "Save points Error";
		return false;
	}
	else
	{

		if(texture_map.data)
		{

			if(1 == texture_map.channels())
			{

				/****************************************************************************************************/

				QTextStream textStream(&file);
				QString str = "";

				//deep_mat = deep_mat / 1000.0;

				if (CV_32FC3 == deep_mat.type())
				{
					for (int r = 0; r < deep_mat.rows; r++)
					{
						cv::Vec3f* ptr_dr = deep_mat.ptr<cv::Vec3f>(r);
						uchar* ptr_color = texture_map.ptr<uchar>(r);

						for (int c = 0; c < deep_mat.cols; c++)
						{
							if (0 != ptr_dr[c][0] && 0 != ptr_dr[c][1] && 0 != ptr_dr[c][2])
							{


								str += QString::number(ptr_dr[c][0]);
								str += " ";
								str += QString::number(ptr_dr[c][1]);
								str += " ";
								str += QString::number(ptr_dr[c][2]);
								str += " ";
								str += QString::number(ptr_color[c]);
								str += " ";
								str += QString::number(ptr_color[c]);
								str += " ";
								str += QString::number(ptr_color[c]);
								str += "\n";

							}

						}
					}
				}
				else if (CV_64FC3 == deep_mat.type())
				{
					for (int r = 0; r < deep_mat.rows; r++)
					{
						cv::Vec3d* ptr_dr = deep_mat.ptr<cv::Vec3d>(r);
						uchar* ptr_color = texture_map.ptr<uchar>(r);

						for (int c = 0; c < deep_mat.cols; c++)
						{
							if (0 != ptr_dr[c][0] && 0 != ptr_dr[c][1] && 0 != ptr_dr[c][2])
							{


								str += QString::number(ptr_dr[c][0]);
								str += " ";
								str += QString::number(ptr_dr[c][1]);
								str += " ";
								str += QString::number(ptr_dr[c][2]);
								str += " ";
								str += QString::number(ptr_color[c]);
								str += " ";
								str += QString::number(ptr_color[c]);
								str += " ";
								str += QString::number(ptr_color[c]);
								str += "\n";

							}

						}
					}
				}



				textStream << str;
				file.close();

				/*****************************************************************************************************/
			}
			else if(3 == texture_map.channels())
			{
				/****************************************************************************************************/

				QTextStream textStream(&file);
				QString str = "";

				if (CV_32FC3 == deep_mat.type())
				{
					for (int r = 0; r < deep_mat.rows; r++)
					{
						cv::Vec3f* ptr_dr = deep_mat.ptr<cv::Vec3f>(r);
						cv::Vec3b* ptr_color = texture_map.ptr<cv::Vec3b>(r);

						for (int c = 0; c < deep_mat.cols; c++)
						{
							if (0 != ptr_dr[c][0] && 0 != ptr_dr[c][1] && 0 != ptr_dr[c][2])
							{


								str += QString::number(ptr_dr[c][0]);
								str += " ";
								str += QString::number(ptr_dr[c][1]);
								str += " ";
								str += QString::number(ptr_dr[c][2]);
								str += " ";
								str += QString::number(ptr_color[c][2]);
								str += " ";
								str += QString::number(ptr_color[c][1]);
								str += " ";
								str += QString::number(ptr_color[c][0]);
								str += "\n";

							}

						}
					}
				}
				else if (CV_64FC3 == deep_mat.type())
				{
					for (int r = 0; r < deep_mat.rows; r++)
					{
						cv::Vec3d* ptr_dr = deep_mat.ptr<cv::Vec3d>(r);
						cv::Vec3b* ptr_color = texture_map.ptr<cv::Vec3b>(r);

						for (int c = 0; c < deep_mat.cols; c++)
						{
							if (0 != ptr_dr[c][0] && 0 != ptr_dr[c][1] && 0 != ptr_dr[c][2])
							{


								str += QString::number(ptr_dr[c][0]);
								str += " ";
								str += QString::number(ptr_dr[c][1]);
								str += " ";
								str += QString::number(ptr_dr[c][2]);
								str += " ";
								str += QString::number(ptr_color[c][2]);
								str += " ";
								str += QString::number(ptr_color[c][1]);
								str += " ";
								str += QString::number(ptr_color[c][0]);
								str += "\n";

							}

						}
					}
				}





				textStream << str;
				file.close();

				/*****************************************************************************************************/
			}


		}
		else
		{
			QTextStream textStream(&file);
			QString str = "";

			//deep_mat = deep_mat / 1000.0;

			if (CV_32FC3 == deep_mat.type())
			{
				for (int r = 0; r < deep_mat.rows; r++)
				{
					cv::Vec3f* ptr_dr = deep_mat.ptr<cv::Vec3f>(r);
					for (int c = 0; c < deep_mat.cols; c++)
					{
						if (0 != ptr_dr[c][0] && 0 != ptr_dr[c][1] && 0 != ptr_dr[c][2])
						{


							str += QString::number(ptr_dr[c][0]);
							str += " ";
							str += QString::number(ptr_dr[c][1]);
							str += " ";
							str += QString::number(ptr_dr[c][2]);
							str += "\n";

						}

					}
				}
			}
			else if (CV_64FC3 == deep_mat.type())
			{
				for (int r = 0; r < deep_mat.rows; r++)
				{
					cv::Vec3d* ptr_dr = deep_mat.ptr<cv::Vec3d>(r);
					for (int c = 0; c < deep_mat.cols; c++)
					{
						if (0 != ptr_dr[c][0] && 0 != ptr_dr[c][1] && 0 != ptr_dr[c][2])
						{


							str += QString::number(ptr_dr[c][0]);
							str += " ";
							str += QString::number(ptr_dr[c][1]);
							str += " ";
							str += QString::number(ptr_dr[c][2]);
							str += "\n";

						}

					}
				}
			}



			textStream << str;
			file.close();
		}



		qDebug() << "Save points" << path;
	}

	return true;
}


//写xml
bool FileIoFunction::writeCalibXml(QString path, cv::Mat camera_intrinsic, cv::Mat camera_distortion, cv::Mat projector_instrinsic, cv::Mat projector_distortion, cv::Mat s_r, cv::Mat s_t)
{
	if (!camera_intrinsic.data || !camera_distortion.data || !projector_instrinsic.data || !projector_distortion.data || !s_r.data || !s_t.data)
	{
		return false;
	}


	//增加一个一级子节点以及元素
	QDomDocument doc;
	QFile file(path); //相对路径、绝对路径、资源路径都可以

														   //if (!file.exists())
														   //{
	if (!file.open(QFile::WriteOnly | QFile::Truncate)) //可以用QIODevice，Truncate表示清空原来的内容
		return false;

	//写入xml头部
	QDomProcessingInstruction instruction; //添加处理命令
	instruction = doc.createProcessingInstruction("xml", "version=\"1.0\" encoding=\"UTF-8\"");
	doc.appendChild(instruction);
	//添加根节点
	QDomElement root = doc.createElement("CalibData");
	doc.appendChild(root);

	file.close();
	//}
	//else
	//{
	//	if (file.open(QFile::ReadOnly))
	//	{
	//		if (!doc.setContent(&file))
	//		{
	//			file.close();
	//			return false;
	//		}
	//		file.close();
	//	}

	//}
	/*******************************************************************************************************************************/
	QString camera_intrinsic_str = "";

	matToString(camera_intrinsic, camera_intrinsic_str);

	QString camera_distortion_str = "";
	matToString(camera_distortion, camera_distortion_str);

	QString projector_instrinsic_str = "";
	matToString(projector_instrinsic, projector_instrinsic_str);

	QString projector_distortion_str = "";
	matToString(projector_distortion, projector_distortion_str);


	QString rotation_matrix_str = "";
	matToString(s_r, rotation_matrix_str);


	QString translation_matrix_str = "";
	matToString(s_t, translation_matrix_str);

	/********************************************************************************************************************************/

	root = doc.documentElement();

	QDomElement book = doc.createElement("SystemCalib");
	book.setAttribute("id", 1); //方式一：创建属性  其中键值对的值可以是各种类型

	QDomElement title = doc.createElement("camera_intrinsic"); //创建子元素
	QDomText text; //设置括号标签中间的值
	text = doc.createTextNode(camera_intrinsic_str);
	book.appendChild(title);
	title.appendChild(text);
	//root.appendChild(title);

	title = doc.createElement("camera_distortion"); //创建子元素
	text = doc.createTextNode(camera_distortion_str);
	book.appendChild(title);
	title.appendChild(text);
	//root.appendChild(title);

	title = doc.createElement("projector_instrinsic"); //创建子元素
	text = doc.createTextNode(projector_instrinsic_str);
	title.appendChild(text);
	book.appendChild(title);

	title = doc.createElement("projector_distortion"); //创建子元素
	text = doc.createTextNode(projector_distortion_str);
	title.appendChild(text);
	book.appendChild(title);

	title = doc.createElement("rotation_matrix"); //创建子元素
	text = doc.createTextNode(rotation_matrix_str);
	title.appendChild(text);
	book.appendChild(title);

	title = doc.createElement("translation_matrix"); //创建子元素
	text = doc.createTextNode(translation_matrix_str);
	title.appendChild(text);
	book.appendChild(title);

	root.appendChild(book);

	/*******************************************************************************************************************************/


	//输出到文件
	if (!file.open(QFile::WriteOnly | QFile::Truncate)) //先读进来，再重写，如果不用truncate就是在后面追加内容，就无效了
		return false;
	QTextStream out_stream(&file);
	doc.save(out_stream, 4); //缩进4格
	file.close();

	return true;

}
