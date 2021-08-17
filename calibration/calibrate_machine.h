#pragma once

#include "QtCore/QtCore"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>



class CalibrateMachine : public QObject
{
	//Q_OBJECT

public:
	CalibrateMachine();
	~CalibrateMachine();


	double Bilinear_interpolation(double x, double y, cv::Mat &mapping);

	bool findCircleBoardFeature(cv::Mat img, std::vector<cv::Point2f> &points);

	bool clearFeaturePoints();

	bool addCameraPoints(std::vector<cv::Point2f> points);

	bool addWorldPoints();

	double calibrate();

	double calibrateCamera(std::vector<std::vector<cv::Point2f>> camera_points_list, std::map<int, bool> &select_group);

	double repairedPoints(cv::Point pos, cv::Mat map);

	bool cameraPointsToDlp(std::vector<cv::Point2f> camera_points, cv::Mat unwrap_map_hor,cv::Mat unwrap_map_ver, int group_num,int dlp_width,int dlp_height, std::vector<cv::Point2f> &dlp_points);

	double calibrateProjector(std::vector<std::vector<cv::Point2f>> dlp_points_list, std::map<int, bool> &select_group);



	cv::Vec3f rotationMatrixToEulerAngles(cv::Mat& R);

	double calibrateStereo(std::vector<std::vector<cv::Point2f>> camera_points_list, std::vector<std::vector<cv::Point2f>> dlp_points_list);

	bool writeCalibXml(cv::Mat camera_intrinsic,cv::Mat camera_distortion,cv::Mat projector_instrinsic,cv::Mat projector_distortion,cv::Mat s_r,cv::Mat s_t);

	bool readCalibXml();

	bool rebuildData(cv::Mat unwrap_map_x, cv::Mat unwrap_map_y, int group_num, cv::Mat &deep_map,cv::Mat texture_map= cv::Mat()); 

	bool rebuildBaseSingleSideData(cv::Mat unwrap_map_y, int group_num, cv::Mat &deep_map, cv::Mat texture_map = cv::Mat());

	bool rebuildDataBaseOpenmp(cv::Mat unwrap_map_x, cv::Mat unwrap_map_y, int group_num, cv::Mat &deep_map, cv::Mat texture_map = cv::Mat());

	bool rebuildPoint(cv::Point2f camera_p, cv::Point2f dlp_p, cv::Point3f &result_p);

	bool rebuildPoints(std::vector<cv::Point2f> camera_points, std::vector<cv::Point2f> dlp_points, std::vector<cv::Point3f> &rebuild_points); 

	bool rebuildPointsBaseSingleSide(std::vector<cv::Point2f> camera_points, std::vector<cv::Point2f> dlp_points, std::vector<cv::Point3f> &rebuild_points);

	bool mapToColor(cv::Mat deep_map, cv::Mat &color_map, int low_z, int high_z);

	bool matToString(cv::Mat in_mat, QString& str);

	bool stringToMat(QString str, cv::Mat& out_mat);

	double getRebuildValueB();
private:

	void createCorrectPoints(cv::Mat unwrap_map_x, cv::Mat unwrap_map_y, std::vector<cv::Point2f> &l_points, std::vector<cv::Point2f> &r_points);

private:

	cv::Size board_size_;

	cv::Mat inv_image(cv::Mat img);

	cv::Mat M_1_;
	cv::Mat M_2_;

	cv::Mat camera_intrinsic_;
	cv::Mat project_intrinsic_;
	cv::Mat camera_distortion_; 
	cv::Mat projector_distortion_;
	cv::Mat rotation_matrix_;
	cv::Mat translation_matrix_;
	double value_b_;

	double dlp_width_;
	double dlp_height_;

	  


};
