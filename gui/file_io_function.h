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

	//�������ͼ��txt�ļ�
	bool saveDepthMapToTxt(cv::Mat points_cloud_mat, QString path);
	//������Ƶ�txt�ļ�
	bool SavePointToTxt(cv::Mat deep_mat, QString path,cv::Mat texture_map = cv::Mat());
	//����Ascii���Ƶ�ply�ļ�
	bool SaveAsciiPointsToPly(cv::Mat deep_mat, QString path, cv::Mat texture_map = cv::Mat());
	//����Bin���Ƶ�ply�ļ�
	bool SaveBinPointsToPly(cv::Mat deep_mat, QString path, cv::Mat texture_map = cv::Mat());
	//���궨�ļ� 
	bool readCalibXml(cv::Mat& camera_intrinsic, cv::Mat& project_intrinsic, cv::Mat& camera_distortion,
		cv::Mat& projector_distortion, cv::Mat& rotation_matrix, cv::Mat& translation_matrix, cv::Mat &M_1, cv::Mat &M_2);

	bool readCalibXml(QString path, cv::Mat& camera_intrinsic,cv::Mat& project_intrinsic,cv::Mat& camera_distortion,
	cv::Mat& projector_distortion,cv::Mat& rotation_matrix,cv::Mat& translation_matrix);
	//����궨�ļ� 
	bool writeCalibXml(QString path, cv::Mat camera_intrinsic, cv::Mat camera_distortion, cv::Mat projector_instrinsic,
		cv::Mat projector_distortion, cv::Mat s_r, cv::Mat s_t);
	//��ȡ�ļ�����Ŀ
	bool getFoldersNum(QString path, int &num);
	//��ȡ�ļ���Ŀ
	bool getFileNum(QString path, QString suffix, int &num);
	//xyz-mapͼתz-mapͼ
	bool mapToColor(cv::Mat deep_map, cv::Mat &color_map, cv::Mat &grey_map,int low_z, int high_z);

	int FileIoFunction::percentile(cv::Mat& image, int percent);

	//���ͼתz-mapͼ
	bool depthToColor(cv::Mat depth_map, cv::Mat& color_map, cv::Mat& grey_map, int low_z, int high_z);

	bool cutDeepMapBaseZ(cv::Mat &deep_map,cv::Mat &mask, int low_z, int high_z);

	//��ȡz-map Roiͼ
	bool maskZMap(cv::Mat &z_map, cv::Mat mask);
	//��������ع�ͼ�����ļ���
	bool saveMoreExposurePatternsToFolder(QString dir_path, std::vector<std::vector<cv::Mat>> patters_list);
	//����ͼ�����ļ���
	bool savePatterns(QString dir_path, std::vector<cv::Mat> patters);
	//����ͼ�����ļ��У��ļ����Զ���
	bool savePatternsToFolder(QString dir_path, std::vector<cv::Mat> patters);
	//������ƺ�z-mapͼ
	bool savePointsToFolder(QString dir_path,cv::Mat deep_map,cv::Mat mask, cv::Mat texture_map= cv::Mat());
	//��λͼת�Ҷ�ͼ������ʾ
	bool mapToGrey(cv::Mat phase_map, int period_num, cv::Mat &show_map);
	//����ͶӰͼ������
	bool sortPatternsBaseInvertProject(std::vector<cv::Mat> invert_patterns,std::vector<cv::Mat> &sort_patterns);
	//�ں�����ͼ
	bool mergeTextureMap(std::vector<cv::Mat> patterns, cv::Mat &texture_map);
private:

	bool matToString(cv::Mat in_mat, QString& str);

	bool stringToMat(QString str, cv::Mat& out_mat);


	QString calib_path_;

};

