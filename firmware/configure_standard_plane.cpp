#include "configure_standard_plane.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>  

ConfigureStandardPlane::ConfigureStandardPlane()
{
 	image_width_ = 1920;
	image_height_ = 1200;

	board_size_.width = 7;
	board_size_.height = 11;
 
}


ConfigureStandardPlane::~ConfigureStandardPlane()
{
}


std::vector<cv::Point3f> ConfigureStandardPlane::generateAsymmetricWorldFeature(float width, float height)
{
	std::vector<cv::Point3f> objectCorners;
	for (int r = 0; r < board_size_.height; r++)
	{
		for (int c = 0; c < board_size_.width; c++)
		{

			if (0 == r% 2)
			{

				objectCorners.push_back(cv::Point3f(width * c , height * r, 0.0f));
			}
			else if (1 == r % 2)
			{

				objectCorners.push_back(cv::Point3f(width * c + 0.5 * width, height * r, 0.0f));
			}
			  
		}
	}

	return objectCorners;
}

double ConfigureStandardPlane::Bilinear_interpolation(double x, double y, cv::Mat& mapping)
{

	int x1 = floor(x);
	int y1 = floor(y);
	int x2 = x1 + 1;
	int y2 = y1 + 1;


	if (CV_64FC1 == mapping.type())
	{
		double fq11 = mapping.at<double>(y1, x1);
		double fq21 = mapping.at<double>(y1, x2);
		double fq12 = mapping.at<double>(y2, x1);
		double fq22 = mapping.at<double>(y2, x2);

		//if (fq11 < 0 || fq21 < 0 || fq12 < 0 || fq22 < 0)
		//	return -1;

		double out = 0;
		out = fq11 * (x2 - x) * (y2 - y) + fq21 * (x - x1) * (y2 - y) + fq12 * (x2 - x) * (y - y1) + fq22 * (x - x1) * (y - y1);


		return out;
	}
	else if (CV_32FC1 == mapping.type())
	{
		float fq11 = mapping.at<float>(y1, x1);
		float fq21 = mapping.at<float>(y1, x2);
		float fq12 = mapping.at<float>(y2, x1);
		float fq22 = mapping.at<float>(y2, x2);

		//if (fq11 < 0 || fq21 < 0 || fq12 < 0 || fq22 < 0)
		//	return -1;

		float out = 0;
		out = fq11 * (x2 - x) * (y2 - y) + fq21 * (x - x1) * (y2 - y) + fq12 * (x2 - x) * (y - y1) + fq22 * (x - x1) * (y - y1);


		return out;
	}

	return -1;

}


bool ConfigureStandardPlane::bilinearInterpolationFeaturePoints(std::vector<cv::Point2f> feature_points, std::vector<cv::Point3f>& point_3d, cv::Mat point_cloud)
{
	if (point_cloud.empty())
		return false;

 
	std::vector<cv::Mat> point_cloud_channels;
	cv::split(point_cloud, point_cloud_channels);

	point_3d.clear();

	for (int i = 0; i < feature_points.size(); i++)
	{
		cv::Point2f p_0 = feature_points[i];
		cv::Point3f f_p_inter;

		f_p_inter.x = Bilinear_interpolation(p_0.x, p_0.y, point_cloud_channels[0]);
		f_p_inter.y = Bilinear_interpolation(p_0.x, p_0.y, point_cloud_channels[1]);
		f_p_inter.z = Bilinear_interpolation(p_0.x, p_0.y, point_cloud_channels[2]);

		point_3d.push_back(f_p_inter);
	}


	return true;
}


cv::Mat ConfigureStandardPlane::inv_image(cv::Mat img)
{
	if (!img.data)
	{
		return cv::Mat();
	}

	int nl = img.rows;
	int nc = img.cols * img.channels();

	if (img.isContinuous())
	{

		nc = nc * nl;
		nl = 1;

	}

	cv::Mat result(img.size(), CV_8U, cv::Scalar(0));


	for (int i = 0; i < nl; i++)
	{
		uchar* ptr1 = img.ptr<uchar>(i);

		uchar* ptr_r = result.ptr<uchar>(i);
		for (int j = 0; j < nc; j++)
		{
			ptr_r[j] = 255 - ptr1[j];
		}
	}

	return result;
}

bool ConfigureStandardPlane::findCircleBoardFeature(cv::Mat img, std::vector<cv::Point2f>& points)
{
	std::vector<cv::Point2f> circle_points;
	cv::Mat img_inv = inv_image(img); 
	bool found = cv::findCirclesGrid(img_inv, board_size_, circle_points, cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);

	if (!found)
		return false;

	points = circle_points;


	return true;

}

bool ConfigureStandardPlane::getStandardPlaneParam(const float* ptr_point_cloud,const uchar* bright,float* R,float* T)
{

    cv::Mat pointcloud_map(image_height_,image_width_,CV_32FC,ptr_point_cloud);
    cv::Mat bright_map(image_height_,image_width_,CV_8UC,bright);


	cv::Mat cameraMatrix(3, 3, CV_32FC1, camera_calibration_param_.camera_intrinsic);
	cv::Mat distCoeffs = cv::Mat(5, 1, CV_32F, camera_calibration_param_.camera_distortion);  


    cv::Mat undist_img;
	cv::undistort(bright_map, undist_img, cameraMatrix, distCoeffs);
 
	std::vector<cv::Point2f> undist_circle_points;
	bool found = findCircleBoardFeature(undist_img, undist_circle_points);

    if (!found)
	{
		return false;
	}
		 
		/*******************************************************************************************/
		std::vector<cv::Point3f> point_3d;
		bilinearInterpolationFeaturePoints(undist_circle_points, point_3d, points_map);

		
		cv::Mat pc1(point_3d.size(), 3, CV_64F, cv::Scalar(0));
		cv::Mat pc2(point_3d.size(), 3, CV_64F, cv::Scalar(0));

		std::vector<cv::Point3f> world_points = generateAsymmetricWorldFeature(20, 10);

		for (int r = 0; r < point_3d.size(); r++)
		{
			pc2.at<double>(r, 0) = point_3d[r].x;
			pc2.at<double>(r, 1) = point_3d[r].y;
			pc2.at<double>(r, 2) = point_3d[r].z;
		}
		for (int r = 0; r < world_points.size(); r++)
		{
			pc1.at<double>(r, 0) = world_points[r].x;
			pc1.at<double>(r, 1) = world_points[r].y;
			pc1.at<double>(r, 2) = world_points[r].z;
		}

		cv::Mat r(3, 3, CV_32F, cv::Scalar(0));
		cv::Mat t(3, 3, CV_32F, cv::Scalar(0));

		svdIcp(pc1, pc2, r, t);  


        memcpy(R, r.data,sizeof(float) * 9);
        memcpy(T, t.data ,sizeof(float) * 3);
 
    return true;

}


/****************************************************************************************************************************************/



// arr1: Mat (1, N) CV_64F
// arr2: Mat (1, M) CV_64F
// return: Mat (N, M) CV_64F
Mat ConfigureStandardPlane::kronProductArr(const Mat& arr1, const Mat& arr2)
{
	int N = arr1.cols;
	int M = arr2.cols;
	Mat arr1Expand = repeat(arr1.t(), 1, M);
	Mat arr2Expand = repeat(arr2, N, 1);
	Mat kroProduct = arr1Expand.mul(arr2Expand);
	return kroProduct;
}

// R * pc2 + t = pc1
// pc1: Mat (N, 3) CV_64F
// pc2: Mat (N, 3) CV_64F
// r: Mat (3, 3) CV_64F
// t: Mat (3, 1) CV_64F
void ConfigureStandardPlane::svdIcp(const Mat& pc1, const Mat& pc2, Mat& r, Mat& t)
{
	int pointNum = pc1.rows;
	Mat pc1Mean = pcMean(pc1);
	Mat pc2Mean = pcMean(pc2);
	Mat pc1Centered = pc1 - repeat(pc1Mean, pointNum, 1);
	Mat pc2Centered = pc2 - repeat(pc2Mean, pointNum, 1);
	Mat w = Mat::zeros(3, 3, CV_64F);
	for (int i = 0; i < pointNum; i++) {
		Mat q1 = pc1Centered(sliceMask(i, -1, 0, 3));
		Mat q2 = pc2Centered(sliceMask(i, -1, 0, 3));
		w += kronProductArr(q2, q1);
	}
	Mat u, sigma, vt;
	SVD::compute(w, sigma, u, vt);
	r = vt.t() * u.t();
	double det = determinant(r);
	if (det < 0) {
		vt.at<double>(2, 0) = -vt.at<double>(2, 0);
		vt.at<double>(2, 1) = -vt.at<double>(2, 1);
		vt.at<double>(2, 2) = -vt.at<double>(2, 2);
		r = vt.t() * u.t();
	}
	t = pc1Mean - pc2Mean * r.t();
	t = t.t();   // from (1, 3) to (3, 1)
}

// end == -1: take only 1 row or colume
Rect ConfigureStandardPlane::sliceMask(int rowStart, int rowEnd, int colStart, int colEnd)
{
	if (colEnd == -1) { colEnd = colStart + 1; }
	if (rowEnd == -1) { rowEnd = rowStart + 1; }
	return Rect(colStart, rowStart, colEnd - colStart, rowEnd - rowStart);
}


// pc: Mat (N, 3) CV_64F
// return: pcMean (1, 3) CV_64F
Mat ConfigureStandardPlane::pcMean(const Mat& pc)
{
	Mat xList = pc(sliceMask(0, pc.rows, 0, -1));
	Mat yList = pc(sliceMask(0, pc.rows, 1, -1));
	Mat zList = pc(sliceMask(0, pc.rows, 2, -1));
	Mat meanXYZ =
		(Mat_<double>(1, 3) << mean(xList)[0],
			mean(yList)[0],
			mean(zList)[0]);   //[0]: denoted for the first channel(only one channel in this case)
	return meanXYZ;
}

























/******************************************************************************************************************************************/

