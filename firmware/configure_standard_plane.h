#pragma once 
#include <iostream> 
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp> 
#include <opencv2/core.hpp>  
#include <opencv2/calib3d.hpp>  
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>  


#include "camera_param.h"
using namespace cv;
 
class ConfigureStandardPlane
{
public:
	ConfigureStandardPlane();
	~ConfigureStandardPlane();

	bool getStandardPlaneParam(const float* ptr_point_cloud,const uchar* bright,float* R,float* T);

	void setCalibrateParam(struct CameraCalibParam param){
		camera_calibration_param_ = param;
	};
	
	bool findCircleBoardFeature(cv::Mat img, std::vector<cv::Point2f>& points); 
	
	// R * pc2 + t = pc1
	// pc1: Mat (N, 3) CV_64F
	// pc2: Mat (N, 3) CV_64F
	// r: Mat (3, 3) CV_64F
	// t: Mat (3, 1) CV_64F
	void svdIcp(const Mat& pc1, const Mat& pc2, Mat& r, Mat& t);


private:
	bool bilinearInterpolationFeaturePoints(std::vector<cv::Point2f> feature_points, std::vector<cv::Point3f>& point_3d, cv::Mat point_cloud);
 
	double Bilinear_interpolation(double x, double y, cv::Mat& mapping); 
	
	std::vector<cv::Point3f> generateAsymmetricWorldFeature(float width, float height);


	cv::Mat inv_image(cv::Mat img);
	/*****************************************************************************************************************************************/
	// arr1: Mat (1, N) CV_64F
	// arr2: Mat (1, M) CV_64F
	// return: Mat (N, M) CV_64F
	Mat kronProductArr(const Mat& arr1, const Mat& arr2);

	// end == -1: take only 1 row or colume
	Rect sliceMask(int rowStart, int rowEnd, int colStart, int colEnd);
	// pc: Mat (N, 3) CV_64F
	// return: pcMean (1, 3) CV_64F
	Mat pcMean(const Mat& pc);



	/****************************************************************************************************************************************/

private:
	int image_width_;
	int image_height_;

	cv::size board_size_;
	// int board_width_;
	// int board_height_;


	//相机标定参数
	struct CameraCalibParam camera_calibration_param_;
 
};