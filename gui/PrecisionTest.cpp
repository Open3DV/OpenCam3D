#include "PrecisionTest.h"
 

PrecisionTest::PrecisionTest()
{

}
PrecisionTest::~PrecisionTest()
{

}


// arr1: Mat (1, N) CV_64F
// arr2: Mat (1, M) CV_64F
// return: Mat (N, M) CV_64F
Mat PrecisionTest::kronProductArr(const Mat& arr1, const Mat& arr2)
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
void PrecisionTest::svdIcp(const Mat& pc1, const Mat& pc2, Mat& r, Mat& t)
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
Rect PrecisionTest::sliceMask(int rowStart, int rowEnd, int colStart, int colEnd)
{
	if (colEnd == -1) { colEnd = colStart + 1; }
	if (rowEnd == -1) { rowEnd = rowStart + 1; }
	return Rect(colStart, rowStart, colEnd - colStart, rowEnd - rowStart);
}


// pc: Mat (N, 3) CV_64F
// return: pcMean (1, 3) CV_64F
Mat PrecisionTest::pcMean(const Mat& pc)
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


double PrecisionTest::computeTwoPointSetDistance(std::vector<cv::Point3f> point_set_0, std::vector<cv::Point3f> point_set_1)
{
	if (point_set_0.size() != point_set_1.size())
	{
		return -1;
	}

	std::vector<double> dist_list;

	double sum_diff = 0;

	for (int i = 0; i < point_set_0.size(); i++)
	{
		cv::Point3f differ_p = point_set_0[i] - point_set_1[i];
		double differ_val = std::sqrtf(differ_p.x * differ_p.x + differ_p.y * differ_p.y + differ_p.z * differ_p.z);
		dist_list.push_back(differ_val);
		//std::cout << differ_val;

		sum_diff += std::abs(differ_val);
	}


	return sum_diff / point_set_0.size();

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
	//float rms = std::abs(point.x * plane[0] + point.y * plane[1] + point.z * plane[2] + plane[3]) / mod_length;
	float rms = point.x * plane[0] + point.y * plane[1] + point.z * plane[2] + plane[3] / mod_length;

 //   std::cout<< point.x << " " << point.y << " " << point.z << " ";
	//std::cout << plane[0] << " " << plane[1] << " " << plane[2] << " " << plane[3] << " ";

	return rms;
}


double PrecisionTest::transformPoints(std::vector<cv::Point3f> org_points, std::vector<cv::Point3f>& trans_points, cv::Mat R, cv::Mat T)
{
	 
	trans_points.clear();
	trans_points.resize(org_points.size());

	double* rotate = R.ptr<double>(0);
	double* translation = T.ptr<double>(0);

	#pragma omp parallel for
	for (int p_i = 0; p_i < org_points.size(); p_i++)
	{
		cv::Point3f o_point = org_points[p_i];
		cv::Point3f trans_point;

		trans_point.x = rotate[0] * o_point.x + rotate[1] * o_point.y + rotate[2] * o_point.z + translation[0];
		trans_point.y = rotate[3] * o_point.x + rotate[4] * o_point.y + rotate[5] * o_point.z + translation[1];
		trans_point.z = rotate[6] * o_point.x + rotate[7] * o_point.y + rotate[8] * o_point.z + translation[2];
 
		trans_points[p_i] = trans_point;
	}


	return true;

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

	std::cout << "center point: " << centroid.at<float>(0, 0) << " , "
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

	std::cout << "Normal point: " << plane[0] << " , " << plane[1] << " , "
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
