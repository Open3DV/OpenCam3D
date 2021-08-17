#include "CalibrateMachine.h"
#include <QtCore/QDebug>
#include <iostream>
#include <complex.h>
#include<QtCore/QFile>
#include<QtCore/QXmlStreamReader>
#include<QtCore/QXmlStreamWriter> 
#include <QtXml/QtXml>
#include <QtCore/QTime>
 

CalibrateMachine::CalibrateMachine()
{

	board_size_.width = 11;
	board_size_.height = 9;

	dlp_width_ = 1280;
	dlp_height_ = 720;

	
}

CalibrateMachine::~CalibrateMachine()
{
}

double CalibrateMachine::Bilinear_interpolation(double x, double y, cv::Mat &mapping)
{

	int x1 = floor(x);
	int y1 = floor(y);
	int x2 = x1 + 1;
	int y2 = y1 + 1;

	//row-y,col-x

	//if (x1 == 1919) {
	//	double out = mapping.at<cv::Vec3d>(y1, x1)[2];
	//	return out;
	//}
	//else {
	double fq11 = mapping.at<double>(y1, x1);
	double fq21 = mapping.at<double>(y1, x2);
	double fq12 = mapping.at<double>(y2, x1);
	double fq22 = mapping.at<double>(y2, x2);



	double out = 0;

	if(fq11> 0.5 && fq21> 0.5 || fq12> 0.5 || fq22> 0.5)
	{
		out = fq11 * (x2 - x) * (y2 - y) + fq21 * (x - x1) * (y2 - y) + fq12 * (x2 - x) * (y - y1) + fq22 * (x - x1) * (y - y1);
	}
	 

	return out;
	//}


}


double CalibrateMachine::repairedPoints(cv::Point pos, cv::Mat map)
{
	if(!map.data)
	{
		return -1;
	}

	if (pos.x);



}

bool CalibrateMachine::cameraPointsToDlp(std::vector<cv::Point2f> camera_points, cv::Mat unwrap_map_hor, cv::Mat unwrap_map_ver, int group_num, int dlp_width, int dlp_height, std::vector<cv::Point2f> &dlp_points)
{
	if (camera_points.empty() || !unwrap_map_hor.data || !unwrap_map_ver.data)
		return false;

	bool ret = true;

	dlp_points.clear();

	double phase_max = 2 * CV_PI*std::pow(2.0, group_num - 1);

	for(int p_i= 0;p_i< camera_points.size();p_i++)
	{
		cv::Point2f pos = camera_points[p_i];

		cv::Point d_p;
		d_p.x = pos.x + 0.5;
		d_p.y = pos.y + 0.5;
		//可以插值优化


		//double hor_val = Bilinear_interpolation(pos.x, pos.y, unwrap_map_hor);
		//double ver_val = Bilinear_interpolation(pos.x, pos.y, unwrap_map_ver);

		//if(hor_val< 0.5 || ver_val< 0.5)
		//{
		//	ret = false;
		//}

		int x = pos.x;
		int y = pos.y;

		double hor_val_0 = unwrap_map_hor.at<double>(y, x);
		double hor_val_1 = unwrap_map_hor.at<double>(y, x+1);

		double ver_val_0 = unwrap_map_ver.at<double>(y, x);
		double ver_val_1 = unwrap_map_ver.at<double>(y+1, x);

		double hor_val = hor_val_0 + (pos.x - x)*(hor_val_1 - hor_val_0);
		double ver_val = ver_val_0 + (pos.y - y)*(ver_val_1 - ver_val_0);


		if(hor_val_0< 0.5 || hor_val_1< 0.5 || ver_val_0< 0.5 || ver_val_1< 0.5)
		{
			ret = false;
		}
		 


		cv::Point2f dlp_p;

		dlp_p.x =dlp_width*  ver_val / phase_max;
		dlp_p.y = dlp_height* hor_val / phase_max;

		dlp_points.push_back(dlp_p);
	
	}

	return ret;

}


bool CalibrateMachine::stringToMat(QString str, cv::Mat& out_mat)
{
	QString mid_str = str.mid(1, str.size() - 2);

	QStringList str_list = mid_str.split(";");

	if(str_list.isEmpty())
	{
		return false;
	}

	std::vector<std::vector<double>> val_list;


	for(int r= 0;r< str_list.size()-1;r++)
	{
		QString str = str_list[r];

		std::vector<double> vals;

		QStringList val_str_list = str.split(",");

		for(int c= 0;c< val_str_list.size();c++)
		{
			vals.push_back(val_str_list[c].toDouble());
		}
		val_list.push_back(vals);
	}


	cv::Mat data(val_list.size(), val_list[0].size(), CV_64F, cv::Scalar(0));

	for(int r= 0;r< data.rows;r++)
	{
		double* ptr_d = data.ptr<double>(r);

		for(int c= 0;c< data.cols;c++)
		{
			ptr_d[c] = val_list[r][c];
		}

	}

	out_mat = data.clone();

	return true;

}

bool CalibrateMachine::matToString(cv::Mat in_mat, QString& str)
{
	if(!in_mat.data)
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


double CalibrateMachine::getRebuildValueB()
{
	return value_b_;
}

bool CalibrateMachine::readCalibXml()
{
	//打开或创建文件
	QFile file("G:/Code/StructureLight/Images/20201127/Calibrate/calib.xml"); //相对路径、绝对路径、资源路径都行
	if (!file.open(QFile::ReadOnly))
	{
		qDebug() << "Read Calib Xml Error";
		return false; 
	}

	QDomDocument doc;
	if (!doc.setContent(&file))
	{
		file.close();
		return false;
	}
	file.close();

	QDomElement root = doc.documentElement(); //返回根节点
	qDebug() << root.nodeName();
	QDomNode node = root.firstChild(); //获得第一个子节点
	while (!node.isNull())  //如果节点不空
	{
		if (node.isElement()) //如果节点是元素
		{
			QDomElement e = node.toElement(); //转换为元素，注意元素和节点是两个数据结构，其实差不多
			qDebug() << e.tagName() << " " << e.attribute("id") << " " << e.attribute("time"); //打印键值对，tagName和nodeName是一个东西



			QDomNodeList list = e.childNodes();
			for (int i = 0; i<list.count(); i++) //遍历子元素，count和size都可以用,可用于标签数计数
			{
				QDomNode n = list.at(i);
				if (node.isElement())
				{

					qDebug() << n.nodeName() << ":" << n.toElement().text();
					if ("camera_intrinsic" == n.nodeName())
					{ 
						stringToMat(n.toElement().text(), camera_intrinsic_);
					}
					else if ("camera_distortion" == n.nodeName())
					{
						stringToMat(n.toElement().text(), camera_distortion_);
					}
					else if ("projector_instrinsic" == n.nodeName())
					{
						stringToMat(n.toElement().text(), project_intrinsic_);
					}
					else if ("projector_distortion" == n.nodeName())
					{
						stringToMat(n.toElement().text(), projector_distortion_);
					}
					else if ("rotation_matrix" == n.nodeName())
					{
						stringToMat(n.toElement().text(), rotation_matrix_);
					}
					else if ("translation_matrix" == n.nodeName())
					{
						stringToMat(n.toElement().text(), translation_matrix_);
					}
					else
					{
						qDebug()<<"bad: " << n.nodeName() << ":" << n.toElement().text();
						
					}
				}
			}
		}
		node = node.nextSibling(); //下一个兄弟节点,nextSiblingElement()是下一个兄弟元素，都差不多
	}



	/**************************************************************************************************************/
	if (!camera_intrinsic_.data || !camera_distortion_.data || !project_intrinsic_.data ||
		!projector_distortion_.data || !rotation_matrix_.data || !translation_matrix_.data)
	{
		return false; 
	}

	cv::Mat E_1 = cv::Mat::eye(3, 4, CV_64FC1);
	cv::Mat E_2 = cv::Mat::zeros(3, 4, CV_64FC1);

	M_1_ = cv::Mat::zeros(3, 4, CV_64FC1);
	M_2_ = cv::Mat::zeros(3, 4, CV_64FC1);

	for (int i = 0; i< rotation_matrix_.rows; i++)
	{
		for (int j = 0; j<rotation_matrix_.cols; j++)
		{
			E_2.at<double>(i, j) = rotation_matrix_.at<double>(i, j);
		}
	}

	for (int i = 0; i<3; i++)
	{
		E_2.at<double>(i, 3) = translation_matrix_.at<double>(i);
	}


	M_1_ = camera_intrinsic_*E_1;
	M_2_ = project_intrinsic_*E_2;

	value_b_ = sqrt(pow(translation_matrix_.at<double>(0, 0), 2) + pow(translation_matrix_.at<double>(1, 0), 2) + pow(translation_matrix_.at<double>(2, 0), 2));


	//M_1 = cameraA*cameraM;
	//M_2 = dlpA*dlpM;


	/******************************************************************************************************************/




	return true;
}

//写xml
bool CalibrateMachine::writeCalibXml(cv::Mat camera_intrinsic, cv::Mat camera_distortion, cv::Mat projector_instrinsic, cv::Mat projector_distortion, cv::Mat s_r, cv::Mat s_t)
{
	if (!camera_intrinsic.data || !camera_distortion.data || !projector_instrinsic.data || !projector_distortion.data || !s_r.data || !s_t.data)
	{
		return false;
	}

	/*************************************************************************************************************************/
	//保存txt
	QFile file_txt("../param.txt");

	file_txt.open(QIODevice::WriteOnly);
	QTextStream stream(&file_txt);

	stream << camera_intrinsic.at<double>(0, 0) << "\n";
	stream << camera_intrinsic.at<double>(0, 1) << "\n";
	stream << camera_intrinsic.at<double>(0, 2) << "\n";
	stream << camera_intrinsic.at<double>(1, 0) << "\n";
	stream << camera_intrinsic.at<double>(1, 1) << "\n";
	stream << camera_intrinsic.at<double>(1, 2) << "\n";
	stream << camera_intrinsic.at<double>(2, 0) << "\n";
	stream << camera_intrinsic.at<double>(2, 1) << "\n";
	stream << camera_intrinsic.at<double>(2, 2) << "\n";


	stream << camera_distortion.at<double>(0, 0) << "\n";
	stream << camera_distortion.at<double>(0, 1) << "\n";
	stream << camera_distortion.at<double>(0, 2) << "\n";
	stream << camera_distortion.at<double>(0, 3) << "\n";
	stream << camera_distortion.at<double>(0, 4) << "\n";



	stream << projector_instrinsic.at<double>(0, 0) << "\n";
	stream << projector_instrinsic.at<double>(0, 1) << "\n";
	stream << projector_instrinsic.at<double>(0, 2) << "\n";
	stream << projector_instrinsic.at<double>(1, 0) << "\n";
	stream << projector_instrinsic.at<double>(1, 1) << "\n";
	stream << projector_instrinsic.at<double>(1, 2) << "\n";
	stream << projector_instrinsic.at<double>(2, 0) << "\n";
	stream << projector_instrinsic.at<double>(2, 1) << "\n";
	stream << projector_instrinsic.at<double>(2, 2) << "\n";


	stream << projector_distortion.at<double>(0, 0) << "\n";
	stream << projector_distortion.at<double>(0, 1) << "\n";
	stream << projector_distortion.at<double>(0, 2) << "\n";
	stream << projector_distortion.at<double>(0, 3) << "\n";
	stream << projector_distortion.at<double>(0, 4) << "\n";

	stream << s_r.at<double>(0, 0) << "\n";
	stream << s_r.at<double>(0, 1) << "\n";
	stream << s_r.at<double>(0, 2) << "\n";
	stream << s_r.at<double>(1, 0) << "\n";
	stream << s_r.at<double>(1, 1) << "\n";
	stream << s_r.at<double>(1, 2) << "\n";
	stream << s_r.at<double>(2, 0) << "\n";
	stream << s_r.at<double>(2, 1) << "\n";
	stream << s_r.at<double>(2, 2) << "\n";

	stream << s_t.at<double>(0, 0) << "\n";
	stream << s_t.at<double>(1, 0) << "\n";
	stream << s_t.at<double>(2, 0) << "\n";

	//for(int r= 0;r< s_r.rows;r++)
	//{
	//	
	//	for(int c= 0;c< s_r.cols;c++)
	//	{
	//		stream << QString::number(s_r.at<double>(r, c));

	//		if(c!= s_r.cols -1)
	//		{
	//			stream << " ";
	//		}
	//	}

	//	stream << "\n";
	//}

	file_txt.close();

	/*************************************************************************************************************************/



	//增加一个一级子节点以及元素
	QDomDocument doc;
	QFile file("../calib.xml"); //相对路径、绝对路径、资源路径都可以

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
	 
	QString camera_distortion_str= "";
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



void CalibrateMachine::createCorrectPoints(cv::Mat unwrap_map_x, cv::Mat unwrap_map_y, std::vector<cv::Point2f> &l_points, std::vector<cv::Point2f> &r_points)
{
	l_points.clear();
	r_points.clear();

	int nr = unwrap_map_x.rows;
	int nc = unwrap_map_x.cols;


	for (int r = 0; r< nr; r++)
	{

		double* x_ptr = unwrap_map_x.ptr<double>(r);
		double* y_ptr = unwrap_map_y.ptr<double>(r);

		for (int c = 0; c< nc; c++)
		{

			double xValue = x_ptr[c];
			double yValue = y_ptr[c];

			//                float xValue= phaseX.at<float>(r,c);
			//                float yValue= phaseY.at<float>(r,c);


			if (abs(xValue) > 0.001 && abs(yValue)   > 0.001)
			{
				double temp_x = 912 * xValue;
				double temp_y = 570 * yValue;

				double dlp_x_value = temp_x / CV_PI;
				double dlp_y_value = temp_y / CV_PI;


				l_points.push_back(cv::Point2f(c, r));
				r_points.push_back(cv::Point2f(dlp_x_value, dlp_y_value));
			}


		}
	}

	qDebug() << "num: " << r_points.size();
}


bool CalibrateMachine::rebuildPointsBaseSingleSide(std::vector<cv::Point2f> camera_points, std::vector<cv::Point2f> dlp_points, std::vector<cv::Point3f> &rebuild_points)
{
	if (!M_1_.data || !M_2_.data)
	{
		return false;
	}

	if (camera_points.size() != dlp_points.size())
	{
		return false;
	}

	rebuild_points.clear();
	rebuild_points.resize(camera_points.size());

	//#pragma omp parallel for
	for (int p_i = 0; p_i< camera_points.size(); p_i++)
	{
		cv::Point2f camera_p = camera_points[p_i];
		cv::Point2f dlp_p = dlp_points[p_i];
		cv::Point3f result_p;

		cv::Mat leftM = cv::Mat::zeros(3, 3, CV_64FC1);
		cv::Mat rightM = cv::Mat::zeros(3, 1, CV_64FC1);
		cv::Mat point = cv::Mat::zeros(3, 1, CV_64FC1);


		leftM.at<double>(0, 0) = camera_p.x*M_1_.at<double>(2, 0) - M_1_.at<double>(0, 0);
		leftM.at<double>(0, 1) = camera_p.x*M_1_.at<double>(2, 1) - M_1_.at<double>(0, 1);
		leftM.at<double>(0, 2) = camera_p.x*M_1_.at<double>(2, 2) - M_1_.at<double>(0, 2);

		leftM.at<double>(1, 0) = camera_p.y*M_1_.at<double>(2, 0) - M_1_.at<double>(1, 0);
		leftM.at<double>(1, 1) = camera_p.y*M_1_.at<double>(2, 1) - M_1_.at<double>(1, 1);
		leftM.at<double>(1, 2) = camera_p.y*M_1_.at<double>(2, 2) - M_1_.at<double>(1, 2);


		leftM.at<double>(2, 0) = dlp_p.x*M_2_.at<double>(2, 0) - M_2_.at<double>(0, 0);
		leftM.at<double>(2, 1) = dlp_p.x*M_2_.at<double>(2, 1) - M_2_.at<double>(0, 1);
		leftM.at<double>(2, 2) = dlp_p.x*M_2_.at<double>(2, 2) - M_2_.at<double>(0, 2);

		//leftM.at<double>(2, 0) = dlp_p.y*M_2_.at<double>(2, 0) - M_2_.at<double>(1, 0);
		//leftM.at<double>(2, 1) = dlp_p.y*M_2_.at<double>(2, 1) - M_2_.at<double>(1, 1);
		//leftM.at<double>(2, 2) = dlp_p.y*M_2_.at<double>(2, 2) - M_2_.at<double>(1, 2);



		rightM.at<double>(0, 0) = M_1_.at<double>(0, 3) - camera_p.x*M_1_.at<double>(2, 3);
		rightM.at<double>(1, 0) = M_1_.at<double>(1, 3) - camera_p.y*M_1_.at<double>(2, 3);

		rightM.at<double>(2, 0) = M_2_.at<double>(0, 3) - dlp_p.x*M_2_.at<double>(2, 3);
		//rightM.at<double>(2, 0) = M_2_.at<double>(1, 3) - dlp_p.y*M_2_.at<double>(2, 3);

		cv::solve(leftM, rightM, point, cv::DECOMP_SVD);

		result_p.x = point.at<double>(0, 0);
		result_p.y = point.at<double>(1, 0);
		result_p.z = point.at<double>(2, 0);

		rebuild_points[p_i] = result_p;

	}





	return true;
}

bool CalibrateMachine::rebuildPoints(std::vector<cv::Point2f> camera_points, std::vector<cv::Point2f> dlp_points, std::vector<cv::Point3f> &rebuild_points)
{
	if (!M_1_.data || !M_2_.data)
	{
		return false;
	}

	if(camera_points.size()!= dlp_points.size())
	{
		return false;
	}

	rebuild_points.clear();
	rebuild_points.resize(camera_points.size());

	#pragma omp parallel for
	for(int p_i= 0;p_i< camera_points.size();p_i++)
	{
		cv::Point2f camera_p = camera_points[p_i];
		cv::Point2f dlp_p = dlp_points[p_i];
		cv::Point3f result_p;

		cv::Mat leftM = cv::Mat::zeros(4, 3, CV_64FC1);
		cv::Mat rightM = cv::Mat::zeros(4, 1, CV_64FC1);
		cv::Mat point = cv::Mat::zeros(3, 1, CV_64FC1);


		leftM.at<double>(0, 0) = camera_p.x*M_1_.at<double>(2, 0) - M_1_.at<double>(0, 0);
		leftM.at<double>(0, 1) = camera_p.x*M_1_.at<double>(2, 1) - M_1_.at<double>(0, 1);
		leftM.at<double>(0, 2) = camera_p.x*M_1_.at<double>(2, 2) - M_1_.at<double>(0, 2);

		leftM.at<double>(1, 0) = camera_p.y*M_1_.at<double>(2, 0) - M_1_.at<double>(1, 0);
		leftM.at<double>(1, 1) = camera_p.y*M_1_.at<double>(2, 1) - M_1_.at<double>(1, 1);
		leftM.at<double>(1, 2) = camera_p.y*M_1_.at<double>(2, 2) - M_1_.at<double>(1, 2);


		leftM.at<double>(2, 0) = dlp_p.x*M_2_.at<double>(2, 0) - M_2_.at<double>(0, 0);
		leftM.at<double>(2, 1) = dlp_p.x*M_2_.at<double>(2, 1) - M_2_.at<double>(0, 1);
		leftM.at<double>(2, 2) = dlp_p.x*M_2_.at<double>(2, 2) - M_2_.at<double>(0, 2);

		leftM.at<double>(3, 0) = dlp_p.y*M_2_.at<double>(2, 0) - M_2_.at<double>(1, 0);
		leftM.at<double>(3, 1) = dlp_p.y*M_2_.at<double>(2, 1) - M_2_.at<double>(1, 1);
		leftM.at<double>(3, 2) = dlp_p.y*M_2_.at<double>(2, 2) - M_2_.at<double>(1, 2);
		 


		rightM.at<double>(0, 0) = M_1_.at<double>(0, 3) - camera_p.x*M_1_.at<double>(2, 3);
		rightM.at<double>(1, 0) = M_1_.at<double>(1, 3) - camera_p.y*M_1_.at<double>(2, 3);

		rightM.at<double>(2, 0) = M_2_.at<double>(0, 3) - dlp_p.x*M_2_.at<double>(2, 3);
		rightM.at<double>(3, 0) = M_2_.at<double>(1, 3) - dlp_p.y*M_2_.at<double>(2, 3);

		cv::solve(leftM, rightM, point, cv::DECOMP_SVD);

		result_p.x = point.at<double>(0, 0);
		result_p.y = point.at<double>(1, 0);
		result_p.z = point.at<double>(2, 0);
		  
		rebuild_points[p_i] = result_p;

	}





	return true;
}

bool CalibrateMachine::rebuildPoint(cv::Point2f camera_p, cv::Point2f dlp_p, cv::Point3f &result_p)
{
	if (!M_1_.data || !M_2_.data)
	{
		return false;
	}

	cv::Mat leftM = cv::Mat::zeros(4, 3, CV_64FC1);
	cv::Mat rightM = cv::Mat::zeros(4, 1, CV_64FC1);
	cv::Mat point = cv::Mat::zeros(3, 1, CV_64FC1);

	leftM.at<double>(0, 0) = camera_p.x*M_1_.at<double>(2, 0) - M_1_.at<double>(0, 0);
	leftM.at<double>(0, 1) = camera_p.x*M_1_.at<double>(2, 1) - M_1_.at<double>(0, 1);
	leftM.at<double>(0, 2) = camera_p.x*M_1_.at<double>(2, 2) - M_1_.at<double>(0, 2);

	leftM.at<double>(1, 0) = camera_p.y*M_1_.at<double>(2, 0) - M_1_.at<double>(1, 0);
	leftM.at<double>(1, 1) = camera_p.y*M_1_.at<double>(2, 1) - M_1_.at<double>(1, 1);
	leftM.at<double>(1, 2) = camera_p.y*M_1_.at<double>(2, 2) - M_1_.at<double>(1, 2);


	leftM.at<double>(2, 0) = dlp_p.x*M_2_.at<double>(2, 0) - M_2_.at<double>(0, 0);
	leftM.at<double>(2, 1) = dlp_p.x*M_2_.at<double>(2, 1) - M_2_.at<double>(0, 1);
	leftM.at<double>(2, 2) = dlp_p.x*M_2_.at<double>(2, 2) - M_2_.at<double>(0, 2);

	leftM.at<double>(3, 0) = dlp_p.y*M_2_.at<double>(2, 0) - M_2_.at<double>(1, 0);
	leftM.at<double>(3, 1) = dlp_p.y*M_2_.at<double>(2, 1) - M_2_.at<double>(1, 1);
	leftM.at<double>(3, 2) = dlp_p.y*M_2_.at<double>(2, 2) - M_2_.at<double>(1, 2);


	rightM.at<double>(0, 0) = M_1_.at<double>(0, 3) - camera_p.x*M_1_.at<double>(2, 3);
	rightM.at<double>(1, 0) = M_1_.at<double>(1, 3) - camera_p.y*M_1_.at<double>(2, 3);

	rightM.at<double>(2, 0) = M_2_.at<double>(0, 3) - dlp_p.x*M_2_.at<double>(2, 3);
	rightM.at<double>(3, 0) = M_2_.at<double>(1, 3) - dlp_p.y*M_2_.at<double>(2, 3);

	cv::solve(leftM, rightM, point, cv::DECOMP_SVD);
	 
	result_p.x = point.at<double>(0, 0);
	result_p.y = point.at<double>(1, 0);
	result_p.z = point.at<double>(2, 0);

	return true;
}


bool CalibrateMachine::mapToColor(cv::Mat deep_map, cv::Mat &color_map, int low_z, int high_z)
{
	if(!deep_map.data)
	{
		return false;
	}

	cv::Mat handle_map(deep_map.size(),CV_8U,cv::Scalar(0));

	int range = high_z - low_z;

	for (int r = 0; r< handle_map.rows; r++)
	{  
		uchar* ptr_h = handle_map.ptr<uchar>(r); 
		cv::Vec3d * ptr_dr = deep_map.ptr<cv::Vec3d>(r);

		for (int c = 0; c< handle_map.cols; c++)
		{
			if (0 != ptr_dr[c][0] && 0 != ptr_dr[c][1] && 0 != ptr_dr[c][2])
			{
				ptr_h[c] = 0.5+ 255.0*(ptr_dr[c][2] - low_z) / range;
			}
		}
	}

	cv::applyColorMap(handle_map, color_map, cv::COLORMAP_JET);

	return true;
}


bool CalibrateMachine::rebuildDataBaseOpenmp(cv::Mat unwrap_map_x, cv::Mat unwrap_map_y, int group_num, cv::Mat &deep_map, cv::Mat texture_map)
{
	if (!M_1_.data || !M_2_.data)
	{
		return false;
	}

	if (!unwrap_map_x.data || !unwrap_map_y.data)
	{
		return false;
	}

	QTime timer;
	timer.start();
	qDebug() << "Start Work";

	double phase_max = 2 * CV_PI*std::pow(2.0, group_num - 1);


	int nr = unwrap_map_x.rows;
	int nc = unwrap_map_x.cols;


	cv::Mat all_deep_map = cv::Mat(nr, nc, CV_64F, cv::Scalar(NULL));
	deep_map = all_deep_map.clone();



	cv::Mat undist_unwrap_map_x, undist_unwrap_map_y;

	cv::undistort(unwrap_map_x, undist_unwrap_map_x, camera_intrinsic_, camera_distortion_, cv::noArray());
	cv::undistort(unwrap_map_y, undist_unwrap_map_y, camera_intrinsic_, camera_distortion_, cv::noArray());

	qDebug() << "undistort Points Time: " << timer.elapsed();
}


bool CalibrateMachine::rebuildBaseSingleSideData(cv::Mat unwrap_map_y, int group_num, cv::Mat &deep_map, cv::Mat texture_map)
{
	if (!M_1_.data || !M_2_.data)
	{
		return false;
	}
	if ( !unwrap_map_y.data)
	{
		return false;
	}

	QTime timer;
	timer.start();
	qDebug() << "Start Work";

	double phase_max = 2 * CV_PI*std::pow(2.0, group_num - 1);


	int nr = unwrap_map_y.rows;
	int nc = unwrap_map_y.cols;


	cv::Mat all_deep_map = cv::Mat(nr, nc, CV_64F, cv::Scalar(NULL));
	deep_map = all_deep_map.clone();

	std::vector<cv::Point2f> camera_points;
	std::vector<cv::Point2f> dlp_points;

	//camera_points.resize(nr*nc);
	//dlp_points.resize(nr*nc);

	//#pragma omp parallel for
	for (int r = 0; r< nr; r++)
	{
		 
		double* ptr_y = unwrap_map_y.ptr<double>(r);
		double* ptr_d = all_deep_map.ptr<double>(r);
		double* ptr_m = deep_map.ptr<double>(r);
		for (int c = 0; c< nc; c++)
		{
			if (-1 != ptr_y[c])
			{
				cv::Point2f dlp_p;
				dlp_p.y = 0;
				dlp_p.x = dlp_height_* ptr_y[c] / phase_max;


				cv::Point2f camera_p(c, r);
				camera_points.push_back(camera_p);
				dlp_points.push_back(dlp_p); 
			}
			//else
			//{
			// camera_points[r*nc + c] = cv::Point2f(-1, -1);
			// dlp_points[r*nc + c] = cv::Point2f(-1, -1);
			//}
		}
	}


	/*********************************************************************************/
	 

	//畸变校正
	//std::vector<cv::Point2f> correct_camera_points;
	//std::vector<cv::Point2f> correct_dlp_points;

	qDebug() << "Select Points Time: " << timer.elapsed();
	timer.restart();


	//cv::undistortPoints(camera_points, correct_camera_points, camera_intrinsic_, camera_distortion_, cv::noArray(), camera_intrinsic_);
	//cv::undistortPoints(dlp_points, correct_dlp_points, project_intrinsic_, projector_distortion_, cv::noArray(), project_intrinsic_);

	qDebug() << "undistort Points Time: " << timer.elapsed();
	timer.restart();

	std::vector<cv::Point3f> rebuild_points;
	rebuildPointsBaseSingleSide(camera_points, dlp_points, rebuild_points);


	qDebug() << "finished time: " << timer.elapsed();







	int points_num = 0;


	std::vector<cv::Point3f> select_rebuild_points;
	cv::Mat deep_map_real_data(deep_map.rows, deep_map.cols, CV_64FC3, cv::Scalar(NULL, NULL, NULL));

	for (int r = 0; r < nr; r++)
	{

		double* ptr_y = unwrap_map_y.ptr<double>(r);
		double* ptr_d = all_deep_map.ptr<double>(r);
		double* ptr_m = deep_map.ptr<double>(r);
		cv::Vec3d * ptr_dr = deep_map_real_data.ptr<cv::Vec3d>(r);
		for (int c = 0; c < nc; c++)
		{
			if (-1 != ptr_y[c])
			{
				if (points_num > rebuild_points.size() - 1)
				{
					return false;
				}

				ptr_d[c] = rebuild_points[points_num].z;
				ptr_m[c] = rebuild_points[points_num].z;

				if (rebuild_points[points_num].z > 600 && rebuild_points[points_num].z < 1600)
				{
					select_rebuild_points.push_back(rebuild_points[points_num]);

					ptr_dr[c][0] = rebuild_points[points_num].x;
					ptr_dr[c][1] = rebuild_points[points_num].y;
					ptr_dr[c][2] = rebuild_points[points_num].z;

				}
				points_num++;
			}

		}

	}


	deep_map = deep_map_real_data.clone();

	/***************************************************************************************************************/

	cv::Mat color_map;

	mapToColor(deep_map, color_map, 600, 1600);

	QString ply_path = "";


	cv::imwrite("G:/Code/StructureLight/Images/20201127/reconstruct/deep_map.bmp", deep_map);

	qDebug() << "Save deep map";

	QString point_save_path = "G:/Code/StructureLight/Images/20201127/reconstruct/points_variable_x.txt";

	QFile file(point_save_path);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
	{
	qDebug() << "Save points Error";
	return false;
	}
	else
	{


	QTextStream textStream(&file);
	QString str = "";

	for (int p_i = 0; p_i< select_rebuild_points.size(); p_i++)
	{

	str += QString::number(select_rebuild_points[p_i].x);
	str += ",";
	str += QString::number(select_rebuild_points[p_i].y);
	str += ",";
	str += QString::number(select_rebuild_points[p_i].z);
	str += "\n";
	}

	textStream << str;
	file.close();


	qDebug() << "Save points"<< point_save_path;
	}

	return true;
}

bool CalibrateMachine::rebuildData(cv::Mat unwrap_map_x, cv::Mat unwrap_map_y, int group_num, cv::Mat &deep_map, cv::Mat texture_map)
{
	if(!M_1_.data || !M_2_.data)
	{
		return false;
	}

	if (!unwrap_map_x.data || !unwrap_map_y.data)
	{
		return false;
	}

	QTime timer;
	timer.start();
	qDebug() << "Start Work";

	double phase_max = 2 * CV_PI*std::pow(2.0, group_num - 1);


	int nr = unwrap_map_x.rows;
	int nc = unwrap_map_x.cols;


	cv::Mat all_deep_map = cv::Mat(nr, nc, CV_64F, cv::Scalar(NULL));
	deep_map = all_deep_map.clone();

	std::vector<cv::Point2f> camera_points;
	std::vector<cv::Point2f> dlp_points;

	//camera_points.resize(nr*nc);
	//dlp_points.resize(nr*nc);

	//#pragma omp parallel for
	for (int r = 0; r< nr;r++)
	{

		double* ptr_x = unwrap_map_x.ptr<double>(r);
		double* ptr_y = unwrap_map_y.ptr<double>(r);
		double* ptr_d = all_deep_map.ptr<double>(r);
		double* ptr_m = deep_map.ptr<double>(r);
		for (int c = 0; c< nc; c++)
		{
			 if(-1 != ptr_x[c])
			 {
				 cv::Point2f dlp_p; 
				 dlp_p.x = dlp_width_*ptr_x[c] / phase_max;
				 dlp_p.y = dlp_height_* ptr_y[c] / phase_max;


				 cv::Point2f camera_p(c, r);
				 camera_points.push_back(camera_p);
				 dlp_points.push_back(dlp_p);
				 //camera_points[r*nc + c] = camera_p;
				 //dlp_points[r*nc + c] = dlp_p;
			 }
			 //else
			 //{
				// camera_points[r*nc + c] = cv::Point2f(-1, -1);
				// dlp_points[r*nc + c] = cv::Point2f(-1, -1);
			 //}
		}
	}


	/*********************************************************************************/

	//std::vector<cv::Point2f> select_camera_points;
	//std::vector<cv::Point2f> select_dlp_points; 
	//std::vector<cv::Point2f>::const_iterator it_c = camera_points.begin();
	//std::vector<cv::Point2f>::const_iterator it_d = dlp_points.begin();

	//int test_num = 0;

	//while(it_c != camera_points.end())
	//{
	//	if(*it_c == *it_d && *it_c == cv::Point2f(-1, -1))
	//	{ 
	//		++it_c;
	//		++it_d; 
	//	}
	//	else
	//	{
	//		select_camera_points.push_back(*it_c);
	//		select_dlp_points.push_back(*it_d);
	//		++it_c;
	//		++it_d;
	//	}  

	//}

	//camera_points = select_camera_points;
	//dlp_points = select_dlp_points;


	//畸变校正
	std::vector<cv::Point2f> correct_camera_points;
	std::vector<cv::Point2f> correct_dlp_points;

	qDebug() << "Select Points Time: " << timer.elapsed();
	timer.restart();
 

	cv::undistortPoints(camera_points, correct_camera_points, camera_intrinsic_, camera_distortion_, cv::noArray(), camera_intrinsic_);
	cv::undistortPoints(dlp_points, correct_dlp_points, project_intrinsic_, projector_distortion_, cv::noArray(), project_intrinsic_);

	qDebug() << "undistort Points Time: " << timer.elapsed();
	timer.restart();
 
	std::vector<cv::Point3f> rebuild_points;
	rebuildPoints(correct_camera_points, correct_dlp_points, rebuild_points);


	qDebug() << "finished time: " << timer.elapsed();







	int points_num = 0;


	std::vector<cv::Point3f> select_rebuild_points;
	cv::Mat deep_map_real_data(deep_map.rows, deep_map.cols, CV_64FC3, cv::Scalar(NULL, NULL, NULL));

	for (int r = 0; r < nr; r++)
	{

		double* ptr_x = unwrap_map_x.ptr<double>(r);
		double* ptr_d = all_deep_map.ptr<double>(r);
		double* ptr_m = deep_map.ptr<double>(r);
		cv::Vec3d * ptr_dr = deep_map_real_data.ptr<cv::Vec3d>(r);
		for (int c = 0; c < nc; c++)
		{
			if (-1 != ptr_x[c])
			{
				if (points_num > rebuild_points.size() - 1)
				{
					return false;
				}

				ptr_d[c] = rebuild_points[points_num].z;

				if (rebuild_points[points_num].z > 600 && rebuild_points[points_num].z < 1600)
				{
					ptr_m[c] = rebuild_points[points_num].z;
					select_rebuild_points.push_back(rebuild_points[points_num]);

					ptr_dr[c][0] = rebuild_points[points_num].x;
					ptr_dr[c][1] = rebuild_points[points_num].y;
					ptr_dr[c][2] = rebuild_points[points_num].z;

				}
				points_num++;
			}

		}

	}


	deep_map = deep_map_real_data.clone();

	/***************************************************************************************************************/

	cv::Mat color_map;

	mapToColor(deep_map, color_map, 600, 1600);

	//cv::Mat show_img = color_map.clone();
	//float rate = 0.5;
	//cv::resize(show_img, show_img, cv::Size(color_map.cols*rate, color_map.rows*rate), cv::INTER_CUBIC);

	//cv::imshow("depth", show_img);

	QString ply_path = "";


	//cv::imwrite("G:/Code/StructureLight/Images/20201123/reconstruct/deep_map.bmp", deep_map);

	//qDebug() << "Save deep map";

	/*QString point_save_path = "G:/Code/StructureLight/Images/20201123/reconstruct/points_variable.txt";

	QFile file(point_save_path);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
	{
		qDebug() << "Save points Error";
		return false;
	}
	else
	{


		QTextStream textStream(&file);
		QString str = "";

		for (int p_i = 0; p_i< select_rebuild_points.size(); p_i++)
		{

			str += QString::number(select_rebuild_points[p_i].x);
			str += ",";
			str += QString::number(select_rebuild_points[p_i].y);
			str += ",";
			str += QString::number(select_rebuild_points[p_i].z);
			str += "\n";
		}

		textStream << str;
		file.close();


		qDebug() << "Save points"<< point_save_path;
	}*/

	return true;

}

cv::Vec3f CalibrateMachine::rotationMatrixToEulerAngles(cv::Mat& R)
{
	float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
	bool singular = sy < 1e-6; // If
	float x, y, z;
	if (!singular)
	{
		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		y = atan2(-R.at<double>(2, 0), sy);
		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
	}
	else
	{
		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		y = atan2(-R.at<double>(2, 0), sy);
		z = 0;
	}
#if 1
	x = x * 180.0f / 3.141592653589793f;
	y = y * 180.0f / 3.141592653589793f;
	z = z * 180.0f / 3.141592653589793f;
#endif
	return cv::Vec3f(x, y, z);
}

double CalibrateMachine::calibrateStereo(std::vector<std::vector<cv::Point2f>> camera_points_list, std::vector<std::vector<cv::Point2f>> dlp_points_list)
{
	std::vector<std::vector<cv::Point3f>> world_feature_points;


	for (int g_i = 0; g_i< camera_points_list.size(); g_i++)
	{  
		std::vector<cv::Point3f> objectCorners;
		for (int i = 0; i< board_size_.height; i++) {
			for (int j = 0; j<board_size_.width; j++) {
				objectCorners.push_back(cv::Point3f(10 * i, 10 * j, 0.0f));
			}
		}  
		world_feature_points.push_back(objectCorners);
	}

	/********************************************************************************************************************/

	//标定
	bool mustInitUndistort = true;
	int flag = 0;

	std::vector<cv::Mat> left_rvecs, left_tvecs;
	std::vector<cv::Mat> right_rvecs, right_tvecs;

	cv::Mat camera_intrinsic,projector_intrinsic;
	cv::Mat camera_distortion, projector_distortion;
	cv::Mat _R, _T, _E, _F;

	double cameraError = cv::calibrateCamera(world_feature_points,
		camera_points_list,
		board_size_,
		camera_intrinsic,
		camera_distortion,
		left_rvecs, left_tvecs,
		flag, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON));


	//qDebug() << "cameraError: " << cameraError;
	//qDebug() << "cameraPoints num: " << camera_points_list.size();

	double dlpError = cv::calibrateCamera(world_feature_points,
		dlp_points_list,
		board_size_,
		projector_intrinsic,
		projector_distortion,
		right_rvecs, right_tvecs,
		flag, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON));

	//qDebug() << "dlpError: " << dlpError;


	//std::cout << "camera_intrinsic: " << "\n" << camera_intrinsic << "\n"; 
	//std::cout << "camera_distortion: " << "\n" << camera_distortion << "\n"; 
	//std::cout << "projector_intrinsic: " << "\n" << projector_intrinsic << "\n"; 
	//std::cout << "projector_distortion: " << "\n" << projector_distortion << "\n";

	std::cout.flush();

	cv::Mat s_camera_intrinsic = camera_intrinsic.clone(), s_project_intrinsic = projector_intrinsic.clone();
	cv::Mat s_camera_distortion = camera_distortion.clone(), s_projector_distortion = projector_distortion.clone();

	cv::TermCriteria term_criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 500, DBL_EPSILON);

	double stereoError = cv::stereoCalibrate(world_feature_points, camera_points_list, dlp_points_list, s_camera_intrinsic, s_camera_distortion,
		s_project_intrinsic, s_projector_distortion, board_size_, _R, _T, _E, _F,/*cv::CALIB_FIX_INTRINSIC*/ cv::CALIB_USE_INTRINSIC_GUESS /*+ cal_flags*/, term_criteria);


	std::cout << "s_camera_intrinsic: " << "\n" << s_camera_intrinsic << "\n"; 
	std::cout << "s_camera_distortion: " << "\n" << s_camera_distortion << "\n"; 
	std::cout << "s_project_intrinsic: " << "\n" << s_project_intrinsic << "\n"; 
	std::cout << "s_projector_distortion: " << "\n" << s_projector_distortion << "\n";


	std::cout << "_R: " << "\n" << _R << "\n";
	std::cout << "_T: " << "\n" << _T << "\n";

	std::cout.flush();

	double dist = sqrt(_T.at<double>(0) * _T.at<double>(0) + _T.at<double>(1) * _T.at<double>(1) + _T.at<double>(2) * _T.at<double>(2));

	cv::Vec3f angle = rotationMatrixToEulerAngles(_R);


	qDebug() << "x angle: " << angle[0];
	qDebug() << "y angle: " << angle[1];
	qDebug() << "z angle: " << angle[2];

	qDebug() << "T distance: " << dist;

	qDebug() << "stereoError: " << stereoError;

	camera_intrinsic_ = s_camera_intrinsic.clone();
	camera_distortion_ = s_camera_distortion.clone();
	project_intrinsic_ = s_project_intrinsic.clone();
	projector_distortion_ = s_projector_distortion.clone();

	/**************************************************************************************************************/


	cv::Mat E_1 = cv::Mat::eye(3, 4, CV_64FC1);
	cv::Mat E_2 = cv::Mat::zeros(3, 4, CV_64FC1);

	M_1_ = cv::Mat::zeros(3, 4, CV_64FC1);
	M_2_ = cv::Mat::zeros(3, 4, CV_64FC1);

	for (int i = 0; i< _R.rows; i++)
	{
		for (int j = 0; j<_R.cols; j++)
		{
			E_2.at<double>(i, j) = _R.at<double>(i, j);
		}
	}

	for (int i = 0; i<3; i++)
	{
		E_2.at<double>(i, 3) = _T.at<double>(i);
	}


	M_1_ = s_camera_intrinsic*E_1;
	M_2_ = s_project_intrinsic*E_2;

	//M_1 = cameraA*cameraM;
	//M_2 = dlpA*dlpM;


	/******************************************************************************************************************/


	writeCalibXml(s_camera_intrinsic, s_camera_distortion, s_project_intrinsic, s_projector_distortion, _R, _T);

	return stereoError;

	/****************************************************************************************************************/

}


double CalibrateMachine::calibrateProjector(std::vector<std::vector<cv::Point2f>> dlp_points_list, std::map<int, bool> &select_group)
{ 
	std::vector<std::vector<cv::Point3f>> world_feature_points;


	for (int g_i = 0; g_i< dlp_points_list.size(); g_i++)
	{
		select_group.insert(std::pair<int, bool>(g_i, true));

		std::vector<cv::Point3f> objectCorners;
		for (int i = 0; i< board_size_.height; i++) {
			for (int j = 0; j<board_size_.width; j++) {
				objectCorners.push_back(cv::Point3f(25 * i, 25 * j, 0.0f));
			}
		}


		world_feature_points.push_back(objectCorners);
	} 




	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	std::vector<cv::Mat> rvecsMat, tvecsMat;

	/* 运行标定函数 */
	double err_first = cv::calibrateCamera(world_feature_points, dlp_points_list, board_size_, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, cv::CALIB_FIX_K3);

	qDebug() << "First calibrate error: " << err_first;


	double total_err = 0.0;            // 所有图像的平均误差的总和 
	double err = 0.0;                  // 每幅图像的平均误差
	double totalErr = 0.0;
	double totalPoints = 0.0;
	std::vector<cv::Point2f> image_points_pro;     // 保存重新计算得到的投影点

	std::vector<std::vector<cv::Point2f>> select_camera_points;
	std::vector<std::vector<cv::Point3f>> select_world_points;

	/******************************************************************************************************/

	double select_err = err_first;
	int max_series = -1;
	double max_err = 0;

	while (select_err> 0.2 && dlp_points_list.size() > 6)
	{
		total_err = 0;

		for (int i = 0; i < dlp_points_list.size(); i++)
		{

			projectPoints(world_feature_points[i], rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points_pro);   //通过得到的摄像机内外参数，对角点的空间三维坐标进行重新投影计算
			err = cv::norm(cv::Mat(dlp_points_list[i]), cv::Mat(image_points_pro), cv::NORM_L2);



			totalErr += err*err;
			totalPoints += world_feature_points[i].size();

			err /= world_feature_points[i].size();
			qDebug() << i + 1 << "err:" << err << "pixel";
			total_err += err;

			if (err> max_err)
			{
				max_err = err;
				max_series = i;
			}

		}

		double ave_err = total_err / dlp_points_list.size();

		if (-1 != max_series)
		{
			for (int g_i = 0, select_i = 0; g_i< select_group.size(); g_i++)
			{
				if (select_group[g_i])
				{
					select_i++;
				}

				if (max_series + 1 == select_i)
				{
					select_group[g_i] = false;

					break;
					//max_series = -10;
				}

			}

			int false_num = 0;
			for (int g_i = 0; g_i< select_group.size(); g_i++)
			{
				if (!select_group[g_i])
				{
					false_num++;
				}
			}

			qDebug() << "False num: " << false_num;


			dlp_points_list.erase(dlp_points_list.begin() + max_series);
			world_feature_points.erase(world_feature_points.begin() + max_series);
			max_series = -1;
			max_err = 0;

			err_first = cv::calibrateCamera(world_feature_points, dlp_points_list, board_size_, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, cv::CALIB_FIX_K3);

			qDebug() << "Dlp error: " << err_first;

		}

		qDebug() << "ave err: " << ave_err;

		select_err = ave_err;

	}

	/******************************************************************************************************/

	std::cout << "DLP cameraMatrix: " << std::endl;
	std::cout << cameraMatrix << std::endl;

	return select_err;
}


double CalibrateMachine::calibrateCamera(std::vector<std::vector<cv::Point2f>> camera_points_list, std::map<int, bool> &select_group)
{
	select_group.clear();
	for(int g_i= 0;g_i< camera_points_list.size();g_i++)
	{
		select_group.insert(std::pair<int, bool>(g_i, true));
	}

	std::vector<std::vector<cv::Point3f>> world_feature_points;  
	for (int g_i = 0; g_i< camera_points_list.size(); g_i++)
	{
		select_group.insert(std::pair<int, bool>(g_i, true));

		std::vector<cv::Point3f> objectCorners;
		for (int i = 0; i< board_size_.height; i++) {
			for (int j = 0; j<board_size_.width; j++) {
				objectCorners.push_back(cv::Point3f(25 * i, 25 * j, 0.0f));
			}
		}


		world_feature_points.push_back(objectCorners);
	}
	 
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	std::vector<cv::Mat> rvecsMat, tvecsMat;

	/* 运行标定函数 */
	double err_first = cv::calibrateCamera(world_feature_points, camera_points_list, board_size_, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, cv::CALIB_FIX_K3);

	qDebug() << "First calibrate error: " << err_first;


	double total_err = 0.0;            // 所有图像的平均误差的总和 
	double err = 0.0;                  // 每幅图像的平均误差
	double totalErr = 0.0;
	double totalPoints = 0.0;
	std::vector<cv::Point2f> image_points_pro;     // 保存重新计算得到的投影点

	std::vector<std::vector<cv::Point2f>> select_camera_points;
	std::vector<std::vector<cv::Point3f>> select_world_points;

	/******************************************************************************************************/

	double select_err = err_first;
	int max_series = -1;
	double max_err = 0;

	while (select_err> 0.1 && camera_points_list.size() > 5)
	{
		total_err = 0;

		for (int i = 0; i < camera_points_list.size(); i++)
		{

			projectPoints(world_feature_points[i], rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points_pro);   //通过得到的摄像机内外参数，对角点的空间三维坐标进行重新投影计算
			err = cv::norm(cv::Mat(camera_points_list[i]), cv::Mat(image_points_pro), cv::NORM_L2);



			totalErr += err*err;
			totalPoints += world_feature_points[i].size();

			err /= world_feature_points[i].size();
			qDebug() << i + 1 << "err:" << err << "pixel";
			total_err += err;

			if (err> max_err)
			{
				max_err = err;
				max_series = i;
			}

		}

		double ave_err = total_err / camera_points_list.size();

		if (-1 != max_series)
		{
			for (int g_i = 0,select_i = 0; g_i< select_group.size(); g_i++)
			{
				if(select_group[g_i])
				{
					select_i++;
				}
 
				if(max_series + 1 == select_i)
				{
					select_group[g_i] = false;
 
					break;
					//max_series = -10;
				}
				 
			}

			int false_num = 0;
			for(int g_i= 0;g_i< select_group.size();g_i++)
			{
				if (!select_group[g_i])
				{
					false_num++;
				}
			}

			qDebug() << "False num: " << false_num;


			camera_points_list.erase(camera_points_list.begin() + max_series);
			world_feature_points.erase(world_feature_points.begin() + max_series);
			max_series = -1;
			max_err = 0;

			err_first = cv::calibrateCamera(world_feature_points, camera_points_list, board_size_, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, cv::CALIB_FIX_K3);

			qDebug() << "calibrate error: " << err_first;

		}

		qDebug() << "ave err: " << ave_err;

		select_err = ave_err;

	}

	/******************************************************************************************************/


	std::cout << "cameraMatrix: " << "\n" << cameraMatrix << "\n";

	return select_err;


}

double CalibrateMachine::calibrate()
{
	


	return -1;
}

bool CalibrateMachine::addWorldPoints()
{
	std::vector<cv::Point3f> objectCorners;


	for (int i = 0; i< board_size_.height; i++) {
		for (int j = 0; j<board_size_.width; j++) {
			objectCorners.push_back(cv::Point3f(25*i, 25*j, 0.0f));
		}
	}
	 


	return true;
}

bool CalibrateMachine::addCameraPoints(std::vector<cv::Point2f> points)
{ 

	return true;
}

bool CalibrateMachine::clearFeaturePoints()
{
  
	return true;
}


bool CalibrateMachine::findCircleBoardFeature(cv::Mat img, std::vector<cv::Point2f> &points)
{
	std::vector<cv::Point2f> circle_points;
	cv::Mat img_inv = inv_image(img);
	bool found = cv::findCirclesGrid(img_inv, board_size_, circle_points, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);

	if (!found)
		return false;

	points = circle_points;


	return true;

}

cv::Mat CalibrateMachine::inv_image(cv::Mat img)
{
	if (!img.data)
	{
		return cv::Mat();
	}

	int nl = img.rows;
	int nc = img.cols* img.channels();

	if (img.isContinuous())
	{

		nc = nc*nl;
		nl = 1;

	}

	cv::Mat result(img.size(), CV_8U, cv::Scalar(0));


	for (int i = 0; i< nl; i++)
	{
		uchar* ptr1 = img.ptr<uchar>(i);

		uchar* ptr_r = result.ptr<uchar>(i);
		for (int j = 0; j< nc; j++)
		{
			ptr_r[j] = 255 - ptr1[j];
		}
	}

	return result;
}