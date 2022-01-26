#include "PrecisionTest.h"
#include "QDebug"

PrecisionTest::PrecisionTest()
{

}
PrecisionTest::~PrecisionTest()
{

}


float PrecisionTest::computeTwoPointDistance(cv::Point3f p0, cv::Point3f p1)
{
	cv::Point3f differ_p = p0 - p1;
	double differ_val = std::sqrtf(differ_p.x * differ_p.x + differ_p.y * differ_p.y + differ_p.z * differ_p.z);

	return differ_val;
}


float PrecisionTest::computePointToPlaneDistance(cv::Point3f point, std::vector<float> plane)
{
	float mod_length = std::sqrtf(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
	float rms = std::abs(point.x * plane[0] + point.y * plane[1] + point.z * plane[2] + plane[3]) / mod_length;

	qDebug() << point.x << " " << point.y << " " << point.z << " ";
	qDebug() << plane[0] << " " << plane[1] << " " << plane[2] << " " << plane[3] << " ";

	return rms;
}

float PrecisionTest::fitPlaneBaseLeastSquares(std::vector<cv::Point3f> points_3d, std::vector<float>& plane, cv::Point3f& center_point)
{
	if (points_3d.size() < 3)
		return -1;

	cv::Mat points(points_3d.size(), 3, CV_32FC1);

	int rows = points.rows;
	int cols = points.cols;

	for (int r = 0; r < rows; r++)
	{
		points.at<float>(r, 0) = points_3d[r].x;
		points.at<float>(r, 1) = points_3d[r].y;
		points.at<float>(r, 2) = points_3d[r].z;
	}


	cv::Mat centroid = cv::Mat::zeros(1, cols, CV_32FC1);
	for (int i = 0; i < cols; i++) {
		for (int j = 0; j < rows; j++) {
			centroid.at<float>(0, i) += points.at<float>(j, i);
		}
		centroid.at<float>(0, i) /= rows;

	}

	qDebug() << "center point: " << centroid.at<float>(0, 0) << " , "
	    << centroid.at<float>(0, 1) << " , " << centroid.at<float>(0, 2);

	center_point.x = centroid.at<float>(0, 0);
	center_point.y = centroid.at<float>(0, 1);
	center_point.z = centroid.at<float>(0, 2);

	cv::Mat points2 = cv::Mat::ones(rows, cols, CV_32FC1);
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			points2.at<float>(i, j) = points.at<float>(i, j) - centroid.at<float>(0, j);
		}
	}
	cv::Mat A, W, U, V;
	cv::gemm(points2, points, 1, NULL, 0, A, cv::GEMM_1_T);
	cv::SVD::compute(A, W, U, V);


	cv::Mat plane_mat = cv::Mat::zeros(cols + 1, 1, CV_32FC1);
	for (int c = 0; c < cols; c++) {
		plane_mat.at<float>(c, 0) = V.at<float>(cols - 1, c);
		plane_mat.at<float>(cols, 0) += plane_mat.at<float>(c, 0) * centroid.at<float>(0, c);
	}

	plane.push_back(plane_mat.at<float>(0, 0));
	plane.push_back(plane_mat.at<float>(1, 0));
	plane.push_back(plane_mat.at<float>(2, 0));
	plane.push_back(-1 * plane_mat.at<float>(3, 0));

	qDebug() << "Normal point: " << plane[0] << " , " << plane[1] << " , "
	    << plane[2] << " , " << plane[3];

	double mod_length = std::sqrtf(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
	double rms = 0.0;
	for (int p = 0; p < points_3d.size(); p++)
	{
		rms += std::abs(points_3d[p].x * plane[0] + points_3d[p].y * plane[1] + points_3d[p].z * plane[2] + plane[3]) / mod_length;
	}
	rms /= points_3d.size();

	return rms;
}
