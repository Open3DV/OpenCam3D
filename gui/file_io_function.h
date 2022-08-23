#pragma once

#include<QtCore/QFile>
#include <QtCore/qtextstream.h>
#include <QtCore/QDebug>
#include <QtCore/QFile>
#include <QtCore/QXmlStreamReader>
#include <QtCore/QXmlStreamWriter> 
#include <QtXml/QtXml>
#include <QtCore/QTime>
#include <QtCore/QtCore>
#include <QtCore/QString>
#include <opencv2/core.hpp>


class FileIoFunction
{
public:
	FileIoFunction();
	~FileIoFunction();

	/****************************************************************************************************************/
	//DF8
	bool readImages(std::vector<cv::Mat> &patterns, QString path);

	bool mat_to_float(cv::Mat &mat);

	bool dirConvertToImagePathList(QString dir_path, std::vector<QStringList>& images_path_list);

	/****************************************************************************************************************/

	//保存深度图到txt文件
	bool saveDepthMapToTxt(cv::Mat points_cloud_mat, QString path);
	//保存点云到txt文件
	bool SavePointToTxt(cv::Mat deep_mat, QString path,cv::Mat texture_map = cv::Mat());
	//保存Ascii点云到ply文件
	bool SaveAsciiPointsToPly(cv::Mat deep_mat, QString path, cv::Mat texture_map = cv::Mat());
	//保存Bin点云到ply文件
	bool SaveBinPointsToPly(cv::Mat deep_mat, QString path, cv::Mat texture_map = cv::Mat());
	//读标定文件 
	bool readCalibXml(cv::Mat& camera_intrinsic, cv::Mat& project_intrinsic, cv::Mat& camera_distortion,
		cv::Mat& projector_distortion, cv::Mat& rotation_matrix, cv::Mat& translation_matrix, cv::Mat &M_1, cv::Mat &M_2);

	bool readCalibXml(QString path, cv::Mat& camera_intrinsic,cv::Mat& project_intrinsic,cv::Mat& camera_distortion,
	cv::Mat& projector_distortion,cv::Mat& rotation_matrix,cv::Mat& translation_matrix);
	//保存标定文件 
	bool writeCalibXml(QString path, cv::Mat camera_intrinsic, cv::Mat camera_distortion, cv::Mat projector_instrinsic,
		cv::Mat projector_distortion, cv::Mat s_r, cv::Mat s_t);
	//获取文件夹数目
	bool getFoldersNum(QString path, int &num);
	//获取文件数目
	bool getFileNum(QString path, QString suffix, int &num);
	//xyz-map图转z-map图
	bool mapToColor(cv::Mat deep_map, cv::Mat &color_map, cv::Mat &grey_map,int low_z, int high_z);

	int FileIoFunction::percentile(cv::Mat& image, int percent);

	//深度图转z-map图
	bool depthToColor(cv::Mat depth_map, cv::Mat& color_map, cv::Mat& grey_map, int low_z, int high_z);

	bool cutDeepMapBaseZ(cv::Mat &deep_map,cv::Mat &mask, int low_z, int high_z);

	//截取z-map Roi图
	bool maskZMap(cv::Mat &z_map, cv::Mat mask);
	//保存多组曝光图案到文件夹
	bool saveMoreExposurePatternsToFolder(QString dir_path, std::vector<std::vector<cv::Mat>> patters_list);
	//保存图案至文件夹
	bool savePatterns(QString dir_path, std::vector<cv::Mat> patters);
	//保存图案至文件夹，文件名自定义
	bool savePatternsToFolder(QString dir_path, std::vector<cv::Mat> patters);
	//保存点云和z-map图
	bool savePointsToFolder(QString dir_path,cv::Mat deep_map,cv::Mat mask, cv::Mat texture_map= cv::Mat());
	//相位图转灰度图用于显示
	bool mapToGrey(cv::Mat phase_map, int period_num, cv::Mat &show_map);
	//正反投影图案排序
	bool sortPatternsBaseInvertProject(std::vector<cv::Mat> invert_patterns,std::vector<cv::Mat> &sort_patterns);
	//融合纹理图
	bool mergeTextureMap(std::vector<cv::Mat> patterns, cv::Mat &texture_map);
private:

	bool matToString(cv::Mat in_mat, QString& str);

	bool stringToMat(QString str, cv::Mat& out_mat);


	QString calib_path_;

};

