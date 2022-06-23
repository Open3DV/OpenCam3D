#pragma once
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "../firmware/camera_param.h"

class LookupTableFunction
{
public:
	LookupTableFunction();
	~LookupTableFunction();

	virtual bool rebuildData(cv::Mat unwrap_map_x, int group_num, cv::Mat& deep_map, cv::Mat& mask);

	bool mat_double_to_float(cv::Mat org_mat, cv::Mat& dst_mat);

	bool mat_float_to_double(cv::Mat org_mat, cv::Mat& dst_mat);

	bool generate_pointcloud(cv::Mat z, cv::Mat& mask, cv::Mat& map_xyz);

	virtual double depth_per_point_6patterns_combine(double Xc, double Yc, double Xp, cv::Mat xL_rotate_x, cv::Mat xL_rotate_y, cv::Mat single_pattern_mapping, double b, double& disparity);

	double Bilinear_interpolation(double x, double y, cv::Mat& mapping);

	cv::Mat readmapping(int rows, int cols, std::string mapping_file);

	bool readBinMapping(int rows, int cols, std::string mapping_file, cv::Mat& out_map);

	bool readBinMappingFloat(int rows, int cols, std::string mapping_file, cv::Mat& out_map);

	bool TestReadBinMapping(int rows, int cols, std::string mapping_file, cv::Mat& out_map);

	bool saveBinMapping(std::string mapping_file, cv::Mat out_map);

	bool saveBinMappingFloat(std::string mapping_file, cv::Mat out_map);

	bool getRebuildValueB(double& b);

	void setRebuildValueB(double b);

	bool readCalibData(std::string path);

	bool readCalibXml(std::string path);

	bool readTable(std::string dir_path, int rows, int cols);

	virtual bool generateLookTable(cv::Mat& xL_rotate_x, cv::Mat& xL_rotate_y, cv::Mat& rectify_R1, cv::Mat& pattern_mapping);

	bool generateRotateTable(cv::Mat camera_intrinsic_, cv::Mat camera_distortion_, cv::Mat rectify_R,
		cv::Size size, cv::Mat& rotate_x, cv::Mat& rotate_y);

	bool generateGridMapping(cv::Mat rotate_x, cv::Mat rotate_y, cv::Mat& map);

	bool undistortedPoints(std::vector<cv::Point2d> distortPoints, cv::Mat intrinsic, cv::Mat distortion,
		std::vector<cv::Point2d>& undisted_points);

	bool undistortedImage(cv::Mat distort_img, cv::Mat& undistort_img);

	void setCalibData(struct CameraCalibParam calib_param);

	bool getLookTable(cv::Mat& xL_rotate_x, cv::Mat& xL_rotate_y, cv::Mat& rectify_R1, cv::Mat& pattern_mapping);

	virtual bool readTableFloat(std::string dir_path, cv::Mat& xL_rotate_x, cv::Mat& xL_rotate_y, cv::Mat& rectify_R1, cv::Mat& pattern_mapping);

	bool setCameraVersion(int version);
protected:
	void normalizePoint(
		double x, double y,
		double fc_x, double fc_y,
		double cc_x, double cc_y,
		double k1, double k2, double k3, double p1, double p2,
		double& x_norm, double& y_norm);

protected:

	cv::Mat single_pattern_mapping_;
	cv::Mat xL_rotate_x_;
	cv::Mat xL_rotate_y_;
	cv::Mat xR_rotate_x_;
	cv::Mat xR_rotate_y_;
	cv::Mat R_1_;

	cv::Size image_size_;
	float min_low_z_;
	float max_max_z_;

	cv::Mat camera_intrinsic_;
	cv::Mat project_intrinsic_;
	cv::Mat camera_distortion_;
	cv::Mat projector_distortion_;
	cv::Mat rotation_matrix_;
	cv::Mat translation_matrix_;
	double value_b_;

	float dlp_width_;
	float dlp_height_;

};


class MiniLookupTableFunction : public LookupTableFunction
{
public:

	virtual bool rebuildData(cv::Mat unwrap_map_x, int group_num, cv::Mat& deep_map, cv::Mat& mask);

	virtual double depth_per_point_6patterns_combine(double Xc, double Yc, double Xp, cv::Mat xL_rotate_x, cv::Mat xL_rotate_y, cv::Mat single_pattern_mapping, double b);

	double Bilinear_mini_interpolation(double x, double y, cv::Mat& mapping);

	virtual bool readTableFloat(std::string dir_path, cv::Mat& xL_rotate_x, cv::Mat& xL_rotate_y, cv::Mat& rectify_R1, cv::Mat& pattern_minimapping);

	// 生成压缩的map
	virtual bool generateLookTable(cv::Mat& xL_rotate_x, cv::Mat& xL_rotate_y, cv::Mat& rectify_R1, cv::Mat& pattern_minimapping);

	double findTheColByRowAndCol(cv::Mat& _cameraMatrix, cv::Mat& _distCoeffs,
		cv::Mat& _matR, double _rowUndistorted, double _colDistorted);//2.2

	bool cutTheMap(cv::Mat& _originalMap, cv::Mat& _cuttedMap);//3


	bool makeMapFull(cv::Mat& _cameraMatrix, cv::Mat& _distCoeffs,
		cv::Mat& _matR, cv::Mat& _notFullData, cv::Mat& _theMask, cv::Mat& _isFullData);//2.1

	//bool readBinMappingFloat(int rows, int cols, std::string mapping_file, cv::Mat& out_map);//0

	bool generateMiniLookupTable(cv::Mat _theFullGridMap, cv::Mat& _theMiniMap);//4

	bool takeMiniMapBack(cv::Mat& _theMiniMap, cv::Mat& _theOriginalMap, cv::Mat& _theFullSizeMask);//5

	double calculateNormalizedCol(cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
		cv::InputArray _matR, cv::InputArray _newCameraMatrix, double _row_undistorted, double _col_distorted);//2.3

	bool generateMiniGridMapping(cv::Mat& _LookupTable, cv::Mat& _MiniLookupTable);

	bool readCameraCalibData(std::string path, struct CameraCalibParam& param);

protected:

	cv::Mat the_mini_map_;
};

