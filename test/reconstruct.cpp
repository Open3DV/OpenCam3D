#include "reconstruct.h"
#include <iostream> 
#include <fstream>
#include "triangulation.h"
#include <opencv2/imgcodecs.hpp>
#include "../firmware/protocol.h"

DF_Reconstruct::DF_Reconstruct()
{
	dlp_width_ = 1920;
	dlp_height_ = 1080;


	//calib_path_ = "./param.txt";


	//bool ret = readCalibData(camera_intrinsic_, project_intrinsic_, camera_distortion_, projector_distortion_,
	//	rotation_matrix_, translation_matrix_, M_1_, M_2_);

	//if (!ret)
	//{
	//	std::cout << "C++ Read Calib Data Error" << std::endl;
	//}
 //

	//FileIoFunction io_machine; 
	// ret = io_machine.readCalibXml(camera_intrinsic_, project_intrinsic_, camera_distortion_,projector_distortion_,
	//	rotation_matrix_, translation_matrix_, M_1_, M_2_);

	//if(!ret)
	//{
	//	std::cout << "Read Calib Data Error"<<std::endl;
	//}


}


DF_Reconstruct::~DF_Reconstruct()
{

}


bool DF_Reconstruct::setCameraVersion(int version)
{

	switch (version)
	{
	case DFX_800:
	{
		dlp_width_ = 1280;
		dlp_height_ = 720;

		return true;
	}
	break;

	case DFX_1800:
	{

		dlp_width_ = 1920;
		dlp_height_ = 1080;

		return true;
	}
	break;

	default:
		break;
	}

	return false;


}

bool DF_Reconstruct::setCalibData(CameraCalibParam param)
{
	camera_intrinsic_ = cv::Mat(3, 3, CV_64F, cv::Scalar(0.0));
	camera_distortion_ = cv::Mat(1, 5, CV_64F, cv::Scalar(0.0));

	project_intrinsic_ = cv::Mat(3, 3, CV_64F, cv::Scalar(0.0));
	projector_distortion_ = cv::Mat(1, 5, CV_64F, cv::Scalar(0.0));

	rotation_matrix_ = cv::Mat(3, 3, CV_64F, cv::Scalar(0.0));
	translation_matrix_ = cv::Mat(3, 1, CV_64F, cv::Scalar(0.0));


	double* ptr_c_i = camera_intrinsic_.ptr<double>(0);

	ptr_c_i[0] = param.camera_intrinsic[0];
	ptr_c_i[1] = param.camera_intrinsic[1];
	ptr_c_i[2] = param.camera_intrinsic[2];
	ptr_c_i[3] = param.camera_intrinsic[3];
	ptr_c_i[4] = param.camera_intrinsic[4];
	ptr_c_i[5] = param.camera_intrinsic[5];
	ptr_c_i[6] = param.camera_intrinsic[6];
	ptr_c_i[7] = param.camera_intrinsic[7];
	ptr_c_i[8] = param.camera_intrinsic[8];


	double* ptr_c_d = camera_distortion_.ptr<double>(0);
	ptr_c_d[0] = param.camera_distortion[0];
	ptr_c_d[1] = param.camera_distortion[1];
	ptr_c_d[2] = param.camera_distortion[2];
	ptr_c_d[3] = param.camera_distortion[3];
	ptr_c_d[4] = param.camera_distortion[4];


	double* ptr_p_i = project_intrinsic_.ptr<double>(0);
	ptr_p_i[0] = param.projector_intrinsic[0];
	ptr_p_i[1] = param.projector_intrinsic[1];
	ptr_p_i[2] = param.projector_intrinsic[2];
	ptr_p_i[3] = param.projector_intrinsic[3];
	ptr_p_i[4] = param.projector_intrinsic[4];
	ptr_p_i[5] = param.projector_intrinsic[5];
	ptr_p_i[6] = param.projector_intrinsic[6];
	ptr_p_i[7] = param.projector_intrinsic[7];
	ptr_p_i[8] = param.projector_intrinsic[8];


	double* ptr_p_d = projector_distortion_.ptr<double>(0);
	ptr_p_d[0] = param.projector_distortion[0];
	ptr_p_d[1] = param.projector_distortion[1];
	ptr_p_d[2] = param.projector_distortion[2];
	ptr_p_d[3] = param.projector_distortion[3];
	ptr_p_d[4] = param.projector_distortion[4];


	double* ptr_r = rotation_matrix_.ptr<double>(0);
	ptr_r[0] = param.rotation_matrix[0];
	ptr_r[1] = param.rotation_matrix[1];
	ptr_r[2] = param.rotation_matrix[2];
	ptr_r[3] = param.rotation_matrix[3];
	ptr_r[4] = param.rotation_matrix[4];
	ptr_r[5] = param.rotation_matrix[5];
	ptr_r[6] = param.rotation_matrix[6];
	ptr_r[7] = param.rotation_matrix[7];
	ptr_r[8] = param.rotation_matrix[8];


	double* ptr_t = translation_matrix_.ptr<double>(0);
	ptr_t[0] = param.translation_matrix[0];
	ptr_t[1] = param.translation_matrix[1];
	ptr_t[2] = param.translation_matrix[2];


	cv::Mat E_1 = cv::Mat::eye(3, 4, CV_64FC1);
	cv::Mat E_2 = cv::Mat::zeros(3, 4, CV_64FC1);

	M_1_ = cv::Mat::zeros(3, 4, CV_64FC1);
	M_2_ = cv::Mat::zeros(3, 4, CV_64FC1);

	for (int i = 0; i < rotation_matrix_.rows; i++)
	{
		for (int j = 0; j < rotation_matrix_.cols; j++)
		{
			E_2.at<double>(i, j) = rotation_matrix_.at<double>(i, j);
		}
	}

	for (int i = 0; i < 3; i++)
	{
		E_2.at<double>(i, 3) = translation_matrix_.at<double>(i);
	}


	M_1_ = camera_intrinsic_ * E_1;
	M_2_ = project_intrinsic_ * E_2;

	return true;
}

bool DF_Reconstruct::readCalibData(cv::Mat& camera_intrinsic, cv::Mat& project_intrinsic, cv::Mat& camera_distortion,
	cv::Mat& projector_distortion, cv::Mat& rotation_matrix, cv::Mat& translation_matrix, cv::Mat& M_1, cv::Mat& M_2)
{
	std::ifstream myfile(calib_path_);

	if (!myfile.is_open())
	{
		std::cout << "can not open this file" << std::endl;
		return 0;
	}

	double I[40] = { 0 };

	//从data1文件中读入int数据
	for (int i = 0; i < 40; i++)
	{

		myfile >> I[i];
		//std::cout << I[i]<<std::endl;

	}

	myfile.close();

	camera_intrinsic = cv::Mat(3, 3, CV_64F, cv::Scalar(0.0));
	camera_distortion = cv::Mat(1, 5, CV_64F, cv::Scalar(0.0));

	project_intrinsic = cv::Mat(3, 3, CV_64F, cv::Scalar(0.0));
	projector_distortion = cv::Mat(1, 5, CV_64F, cv::Scalar(0.0));

	rotation_matrix = cv::Mat(3, 3, CV_64F, cv::Scalar(0.0));
	translation_matrix = cv::Mat(3, 1, CV_64F, cv::Scalar(0.0));


	double* ptr_c_i = camera_intrinsic.ptr<double>(0);

	ptr_c_i[0] = I[0];
	ptr_c_i[1] = I[1];
	ptr_c_i[2] = I[2];
	ptr_c_i[3] = I[3];
	ptr_c_i[4] = I[4];
	ptr_c_i[5] = I[5];
	ptr_c_i[6] = I[6];
	ptr_c_i[7] = I[7];
	ptr_c_i[8] = I[8];


	double* ptr_c_d = camera_distortion.ptr<double>(0);
	ptr_c_d[0] = I[9];
	ptr_c_d[1] = I[10];
	ptr_c_d[2] = I[11];
	ptr_c_d[3] = I[12];
	ptr_c_d[4] = I[13];


	double* ptr_p_i = project_intrinsic.ptr<double>(0);
	ptr_p_i[0] = I[14];
	ptr_p_i[1] = I[15];
	ptr_p_i[2] = I[16];
	ptr_p_i[3] = I[17];
	ptr_p_i[4] = I[18];
	ptr_p_i[5] = I[19];
	ptr_p_i[6] = I[20];
	ptr_p_i[7] = I[21];
	ptr_p_i[8] = I[22];


	double* ptr_p_d = projector_distortion.ptr<double>(0);
	ptr_p_d[0] = I[23];
	ptr_p_d[1] = I[24];
	ptr_p_d[2] = I[25];
	ptr_p_d[3] = I[26];
	ptr_p_d[4] = I[27];


	double* ptr_r = rotation_matrix.ptr<double>(0);
	ptr_r[0] = I[28];
	ptr_r[1] = I[29];
	ptr_r[2] = I[30];
	ptr_r[3] = I[31];
	ptr_r[4] = I[32];
	ptr_r[5] = I[33];
	ptr_r[6] = I[34];
	ptr_r[7] = I[35];
	ptr_r[8] = I[36];


	double* ptr_t = translation_matrix.ptr<double>(0);
	ptr_t[0] = I[37];
	ptr_t[1] = I[38];
	ptr_t[2] = I[39];


	cv::Mat E_1 = cv::Mat::eye(3, 4, CV_64FC1);
	cv::Mat E_2 = cv::Mat::zeros(3, 4, CV_64FC1);

	M_1 = cv::Mat::zeros(3, 4, CV_64FC1);
	M_2 = cv::Mat::zeros(3, 4, CV_64FC1);

	for (int i = 0; i < rotation_matrix.rows; i++)
	{
		for (int j = 0; j < rotation_matrix.cols; j++)
		{
			E_2.at<double>(i, j) = rotation_matrix.at<double>(i, j);
		}
	}

	for (int i = 0; i < 3; i++)
	{
		E_2.at<double>(i, 3) = translation_matrix.at<double>(i);
	}


	M_1 = camera_intrinsic * E_1;
	M_2 = project_intrinsic * E_2;

	return true;
}

bool DF_Reconstruct::setCalibrateData(cv::Mat camera_intrinsic, cv::Mat project_intrinsic, cv::Mat camera_distortion,
	cv::Mat projector_distortion, cv::Mat rotation_matrix, cv::Mat& translation_matrix)
{
	if (!camera_distortion.data || !project_intrinsic.data || !camera_distortion.data
		|| !projector_distortion.data || !rotation_matrix.data || !translation_matrix.data)
	{
		return false;
	}

	cv::Mat E_1 = cv::Mat::eye(3, 4, CV_64FC1);
	cv::Mat E_2 = cv::Mat::zeros(3, 4, CV_64FC1);

	cv::Mat M_1 = cv::Mat::zeros(3, 4, CV_64FC1);
	cv::Mat M_2 = cv::Mat::zeros(3, 4, CV_64FC1);

	for (int i = 0; i < rotation_matrix.rows; i++)
	{
		for (int j = 0; j < rotation_matrix.cols; j++)
		{
			E_2.at<double>(i, j) = rotation_matrix.at<double>(i, j);
		}
	}

	for (int i = 0; i < 3; i++)
	{
		E_2.at<double>(i, 3) = translation_matrix.at<double>(i);
	}


	M_1 = camera_intrinsic * E_1;
	M_2 = project_intrinsic * E_2;


	camera_intrinsic_ = camera_intrinsic.clone();
	camera_distortion_ = camera_distortion_.clone();

	project_intrinsic_ = project_intrinsic.clone();
	projector_distortion_ = projector_distortion.clone();

	rotation_matrix_ = rotation_matrix.clone();
	translation_matrix_ = translation_matrix.clone();

	M_1_ = M_1.clone();
	M_2_ = M_2.clone();

	return true;
}


bool DF_Reconstruct::undistortedImage(cv::Mat distort_img, cv::Mat& undistort_img)
{
	if (distort_img.empty())
	{
		return false;
	}

	if (camera_intrinsic_.empty() || camera_distortion_.empty())
	{
		return false;
	}

	cv::undistort(distort_img, undistort_img, camera_intrinsic_, camera_distortion_);


	return true;

}

bool DF_Reconstruct::undistortedPoints(std::vector<cv::Point2d> distortPoints, cv::Mat intrinsic, cv::Mat distortion, std::vector<cv::Point2d>& undisted_points)
{

	if (!intrinsic.data)
	{
		return false;
	}

	if (!distortion.data)
	{
		return false;
	}

	if (distortPoints.empty())
	{
		return false;
	}


	double fc_x = intrinsic.at<double>(0, 0);
	double fc_y = intrinsic.at<double>(1, 1);
	double cc_x = intrinsic.at<double>(0, 2);
	double cc_y = intrinsic.at<double>(1, 2);

	double k1 = distortion.at<double>(0, 0);
	double k2 = distortion.at<double>(0, 1);
	double p1 = distortion.at<double>(0, 2);
	double p2 = distortion.at<double>(0, 3);
	double k3 = distortion.at<double>(0, 4);


	undisted_points.clear();

	for (int i = 0; i < distortPoints.size(); i++)
	{
		cv::Point2d n_p;

		normalizePoint(distortPoints[i].x, distortPoints[i].y, fc_x, fc_y,
			cc_x, cc_y, k1, k2, k3, p1, p2, n_p.x, n_p.y);

		undisted_points.push_back(n_p);
	}


	return true;
}


bool DF_Reconstruct::rebuildPoints(std::vector<cv::Point2d> camera_points, std::vector<cv::Point2d> dlp_points, std::vector<cv::Point3d>& rebuild_points, std::vector<double>& error_list)
{

	if (camera_points.size() != dlp_points.size() || camera_points.empty())
	{
		return false;
	}


	double* R_data = (double*)rotation_matrix_.data;
	double* T_data = (double*)translation_matrix_.data;

	rebuild_points.clear();
	rebuild_points.resize(camera_points.size());
	error_list.clear();
	error_list.resize(camera_points.size());

#pragma omp parallel for
	for (int i = 0; i < camera_points.size(); i++)
	{

		double X_L;
		double Y_L;
		double Z_L;

		double X_R;
		double Y_R;
		double Z_R;
		double error;

		triangulation(camera_points[i].x, camera_points[i].y,
			dlp_points[i].x, dlp_points[i].y,
			R_data, T_data,
			X_L, Y_L, Z_L, X_R, Y_R, Z_R, error);


		error_list[i] = error;
		rebuild_points[i] = cv::Point3d(X_L, Y_L, Z_L);

	}


	return true;

}

bool DF_Reconstruct::pointError(cv::Mat point_cloud_0, cv::Mat point_cloud_1, cv::Mat& error_map)
{
	if (point_cloud_0.empty() || point_cloud_1.empty())
	{
		return false;
	}

	int nr = point_cloud_0.rows;
	int nc = point_cloud_0.cols;

	cv::Mat err(nr, nc, CV_64F, cv::Scalar(0.0));

	for (int r = 0; r < point_cloud_0.rows; r++)
	{

		double* ptr_err = err.ptr<double>(r);
		cv::Vec3d* ptr_p0 = point_cloud_0.ptr<cv::Vec3d>(r);
		cv::Vec3d* ptr_p1 = point_cloud_1.ptr<cv::Vec3d>(r);

		for (int c = 0; c < point_cloud_1.cols; c++)
		{

			double x = ptr_p1[c][0] - ptr_p0[c][0];
			double y = ptr_p1[c][1] - ptr_p0[c][1];
			double z = ptr_p1[c][2] - ptr_p0[c][2];

			double err = std::sqrt(x * x + y * y + z * z);

			ptr_err[c] = 1.0 * err;
		}
	}

	error_map = err.clone();


	return true;

}


bool DF_Reconstruct::depthTransformPointcloud(cv::Mat depth_map, cv::Mat& point_cloud_map)
{
	if (depth_map.empty())
	{
		return false;
	}

	double camera_fx = camera_intrinsic_.at<double>(0, 0);
	double camera_fy = camera_intrinsic_.at<double>(1, 1);

	double camera_cx = camera_intrinsic_.at<double>(0, 2);
	double camera_cy = camera_intrinsic_.at<double>(1, 2);

	float k1 = camera_distortion_.at<double>(0, 0);
	float k2 = camera_distortion_.at<double>(0, 1);
	float p1 = camera_distortion_.at<double>(0, 2);
	float p2 = camera_distortion_.at<double>(0, 3);
	float k3 = camera_distortion_.at<double>(0, 4);

	int nr = depth_map.rows;
	int nc = depth_map.cols;


	if (depth_map.type() == CV_64F)
	{
		cv::Mat points_map(nr, nc, CV_64FC3, cv::Scalar(0, 0, 0));


		for (int r = 0; r < nr; r++)
		{

			double* ptr_d = depth_map.ptr<double>(r);
			cv::Vec3d* ptr_p = points_map.ptr<cv::Vec3d>(r);

			for (int c = 0; c < nc; c++)
			{

				if (ptr_d[c] > 0)
				{
					/**********************************************************/
					double undistort_x = c;
					double undistort_y = r;

					//undistortPoint(c, r, camera_fx, camera_fy,
					//	camera_cx, camera_cy, k1, k2, k3, p1, p2, undistort_x, undistort_y);

					/*********************************************************/

					cv::Point3d p;
					p.z = ptr_d[c];

					p.x = (undistort_x - camera_cx) * p.z / camera_fx;
					p.y = (undistort_y - camera_cy) * p.z / camera_fy;


					ptr_p[c][0] = p.x;
					ptr_p[c][1] = p.y;
					ptr_p[c][2] = p.z;
				}


			}


		}


		point_cloud_map = points_map.clone();
	}
	else if (depth_map.type() == CV_32F)
	{
		cv::Mat points_map(nr, nc, CV_32FC3, cv::Scalar(0, 0, 0));


		for (int r = 0; r < nr; r++)
		{

			float* ptr_d = depth_map.ptr<float>(r);
			cv::Vec3f* ptr_p = points_map.ptr<cv::Vec3f>(r);

			for (int c = 0; c < nc; c++)
			{

				if (ptr_d[c] > 0)
				{
					/****************************************************************************************/

					double undistort_x = c;
					double undistort_y = r;

					//undistortPoint(c, r, camera_fx, camera_fy,
					//	camera_cx, camera_cy, k1, k2, k3, p1, p2, undistort_x, undistort_y);

					/***************************************************************************************/

					cv::Point3d p;
					p.z = ptr_d[c];

					p.x = (undistort_x - camera_cx) * p.z / camera_fx;
					p.y = (undistort_y - camera_cy) * p.z / camera_fy;


					ptr_p[c][0] = p.x;
					ptr_p[c][1] = p.y;
					ptr_p[c][2] = p.z;
				}


			}


		}


		point_cloud_map = points_map.clone();
	}


	return true;
}



bool DF_Reconstruct::rebuildPoints(std::vector<cv::Point2d> camera_points, std::vector<cv::Point2d> dlp_points, std::vector<cv::Point3d>& rebuild_points)
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

#pragma omp parallel for
	for (int p_i = 0; p_i < camera_points.size(); p_i++)
	{
		cv::Point2d camera_p = camera_points[p_i];
		cv::Point2d dlp_p = dlp_points[p_i];
		cv::Point3d result_p;

		cv::Mat leftM = cv::Mat::zeros(4, 3, CV_64FC1);
		cv::Mat rightM = cv::Mat::zeros(4, 1, CV_64FC1);
		cv::Mat point = cv::Mat::zeros(3, 1, CV_64FC1);


		leftM.at<double>(0, 0) = camera_p.x * M_1_.at<double>(2, 0) - M_1_.at<double>(0, 0);
		leftM.at<double>(0, 1) = camera_p.x * M_1_.at<double>(2, 1) - M_1_.at<double>(0, 1);
		leftM.at<double>(0, 2) = camera_p.x * M_1_.at<double>(2, 2) - M_1_.at<double>(0, 2);

		leftM.at<double>(1, 0) = camera_p.y * M_1_.at<double>(2, 0) - M_1_.at<double>(1, 0);
		leftM.at<double>(1, 1) = camera_p.y * M_1_.at<double>(2, 1) - M_1_.at<double>(1, 1);
		leftM.at<double>(1, 2) = camera_p.y * M_1_.at<double>(2, 2) - M_1_.at<double>(1, 2);


		leftM.at<double>(2, 0) = dlp_p.x * M_2_.at<double>(2, 0) - M_2_.at<double>(0, 0);
		leftM.at<double>(2, 1) = dlp_p.x * M_2_.at<double>(2, 1) - M_2_.at<double>(0, 1);
		leftM.at<double>(2, 2) = dlp_p.x * M_2_.at<double>(2, 2) - M_2_.at<double>(0, 2);

		leftM.at<double>(3, 0) = dlp_p.y * M_2_.at<double>(2, 0) - M_2_.at<double>(1, 0);
		leftM.at<double>(3, 1) = dlp_p.y * M_2_.at<double>(2, 1) - M_2_.at<double>(1, 1);
		leftM.at<double>(3, 2) = dlp_p.y * M_2_.at<double>(2, 2) - M_2_.at<double>(1, 2);



		rightM.at<double>(0, 0) = M_1_.at<double>(0, 3) - camera_p.x * M_1_.at<double>(2, 3);
		rightM.at<double>(1, 0) = M_1_.at<double>(1, 3) - camera_p.y * M_1_.at<double>(2, 3);

		rightM.at<double>(2, 0) = M_2_.at<double>(0, 3) - dlp_p.x * M_2_.at<double>(2, 3);
		rightM.at<double>(3, 0) = M_2_.at<double>(1, 3) - dlp_p.y * M_2_.at<double>(2, 3);

		cv::solve(leftM, rightM, point, cv::DECOMP_SVD);

		result_p.x = point.at<double>(0, 0);
		result_p.y = point.at<double>(1, 0);
		result_p.z = point.at<double>(2, 0);

		rebuild_points[p_i] = result_p;

	}


	return true;
}


bool DF_Reconstruct::phaseToCoord(cv::Mat unwrap_map, float size, cv::Mat& coord_map)
{
	if (unwrap_map.empty())
	{
		return false;
	}

	int nr = unwrap_map.rows;
	int nc = unwrap_map.cols;

	cv::Mat map(nr, nc, CV_64F, cv::Scalar(0));

	for (int r = 0; r < nr; r++)
	{
		double* ptr_unwrap = unwrap_map.ptr<double>(r);
		double* ptr_map = map.ptr<double>(r);

		for (int c = 0; c < nc; c++)
		{
			ptr_map[c] = size * ptr_unwrap[c] / CV_2PI;
		}

	}


	coord_map = map.clone();

	return true;
}


bool DF_Reconstruct::rebuildData(cv::Mat unwrap_map_x, cv::Mat unwrap_map_y, int period_nun, cv::Mat& deep_map, cv::Mat& err_map)
{
	if (!M_1_.data || !M_2_.data)
	{
		return false;
	}

	if (!unwrap_map_x.data || !unwrap_map_y.data)
	{
		return false;
	}


	double phase_max = 2 * CV_PI * period_nun;

	int nr = unwrap_map_x.rows;
	int nc = unwrap_map_x.cols;


	cv::Mat mask(nr, nc, CV_8U, cv::Scalar(0));

	std::vector<cv::Point2d> camera_points;
	std::vector<cv::Point2d> dlp_points;
	std::vector<double> error_list;
	//map to dlp pos

	for (int r = 0; r < nr; r++)
	{
		double* ptr_x = unwrap_map_x.ptr<double>(r);
		double* ptr_y = unwrap_map_y.ptr<double>(r);
		uchar* ptr_m = mask.ptr<uchar>(r);
		for (int c = 0; c < nc; c++)
		{
			if (0 != ptr_x[c])
			{
				cv::Point2d dlp_p;
				dlp_p.x = dlp_width_ * ptr_x[c] / phase_max;
				dlp_p.y = dlp_height_ * ptr_y[c] / phase_max;

				cv::Point2d camera_p(c, r);
				camera_points.push_back(camera_p);
				dlp_points.push_back(dlp_p);

				ptr_m[c] = 255;

			}

		}
	}
	/*********************************************************************************/


	//畸变校正
	std::vector<cv::Point2d> correct_camera_points;
	std::vector<cv::Point2d> correct_dlp_points;




	undistortedPoints(camera_points, camera_intrinsic_, camera_distortion_, correct_camera_points);
	undistortedPoints(dlp_points, project_intrinsic_, projector_distortion_, correct_dlp_points);


	//std::vector<cv::Point2d> correct_camera_points_user;
	//std::vector<cv::Point2d> correct_dlp_points_user;
	//cv::undistortPoints(camera_points, correct_camera_points_user, camera_intrinsic_, camera_distortion_, cv::noArray(), camera_intrinsic_);
	//cv::undistortPoints(dlp_points, correct_dlp_points_user, project_intrinsic_, projector_distortion_, cv::noArray(), project_intrinsic_);



	std::vector<cv::Point3d> rebuild_points_test;
	std::vector<cv::Point3d> rebuild_points;

	rebuildPoints(correct_camera_points, correct_dlp_points, rebuild_points, error_list);

	int points_num = 0;

	cv::Mat deep_map_real_data(nr, nc, CV_64FC3, cv::Scalar(0, 0, 0));
	cv::Mat deep_map_real_data_test(nr, nc, CV_64FC3, cv::Scalar(0, 0, 0));
	cv::Mat error_map(nr, nc, CV_64FC1, cv::Scalar(0));

	for (int r = 0; r < nr; r++)
	{
		uchar* ptr_m = mask.ptr<uchar>(r);
		cv::Vec3d* ptr_dr = deep_map_real_data.ptr<cv::Vec3d>(r);
		cv::Vec3d* ptr_dr_test = deep_map_real_data_test.ptr<cv::Vec3d>(r);
		double* ptr_err = error_map.ptr<double>(r);
		for (int c = 0; c < nc; c++)
		{
			if (0 != ptr_m[c])
			{
				if (points_num < rebuild_points.size())
				{

					ptr_err[c] = error_list[points_num];

					if (error_list[points_num] < 2)
					{
						ptr_dr[c][0] = rebuild_points[points_num].x;
						ptr_dr[c][1] = rebuild_points[points_num].y;
						ptr_dr[c][2] = rebuild_points[points_num].z;
					}

					points_num++;
				}


			}

		}

	}


	deep_map = deep_map_real_data.clone();

	err_map = error_map.clone();
	/***************************************************************************************************************/



	return true;
}

bool DF_Reconstruct::rebuildData(cv::Mat unwrap_map_x, cv::Mat unwrap_map_y, int period_nun, cv::Mat& deep_map)
{
	if (!M_1_.data || !M_2_.data)
	{
		return false;
	}

	if (!unwrap_map_x.data || !unwrap_map_y.data)
	{
		return false;
	}


	double phase_max = 2 * CV_PI * period_nun;

	int nr = unwrap_map_x.rows;
	int nc = unwrap_map_x.cols;


	cv::Mat mask(nr, nc, CV_8U, cv::Scalar(0));

	std::vector<cv::Point2d> camera_points;
	std::vector<cv::Point2d> dlp_points;
	std::vector<double> error_list;
	//map to dlp pos

	for (int r = 0; r < nr; r++)
	{
		double* ptr_x = unwrap_map_x.ptr<double>(r);
		double* ptr_y = unwrap_map_y.ptr<double>(r);
		uchar* ptr_m = mask.ptr<uchar>(r);
		for (int c = 0; c < nc; c++)
		{
			if (0 != ptr_x[c])
			{
				cv::Point2d dlp_p;
				dlp_p.x = dlp_width_ * ptr_x[c] / phase_max;
				dlp_p.y = dlp_height_ * ptr_y[c] / phase_max;

				cv::Point2d camera_p(c, r);
				camera_points.push_back(camera_p);
				dlp_points.push_back(dlp_p);

				ptr_m[c] = 255;

			}

		}
	}
	/*********************************************************************************/


	//畸变校正
	std::vector<cv::Point2d> correct_camera_points;
	std::vector<cv::Point2d> correct_dlp_points;




	undistortedPoints(camera_points, camera_intrinsic_, camera_distortion_, correct_camera_points);
	undistortedPoints(dlp_points, project_intrinsic_, projector_distortion_, correct_dlp_points);


	//std::vector<cv::Point2d> correct_camera_points_user;
	//std::vector<cv::Point2d> correct_dlp_points_user;
	//cv::undistortPoints(camera_points, correct_camera_points_user, camera_intrinsic_, camera_distortion_, cv::noArray(), camera_intrinsic_);
	//cv::undistortPoints(dlp_points, correct_dlp_points_user, project_intrinsic_, projector_distortion_, cv::noArray(), project_intrinsic_);



	std::vector<cv::Point3d> rebuild_points_test;
	std::vector<cv::Point3d> rebuild_points;

	rebuildPoints(correct_camera_points, correct_dlp_points, rebuild_points, error_list);

	int points_num = 0;

	cv::Mat deep_map_real_data(nr, nc, CV_64FC3, cv::Scalar(0, 0, 0));
	cv::Mat deep_map_real_data_test(nr, nc, CV_64FC3, cv::Scalar(0, 0, 0));
	cv::Mat error_map(nr, nc, CV_64FC1, cv::Scalar(0));

	for (int r = 0; r < nr; r++)
	{
		uchar* ptr_m = mask.ptr<uchar>(r);
		cv::Vec3d* ptr_dr = deep_map_real_data.ptr<cv::Vec3d>(r);
		cv::Vec3d* ptr_dr_test = deep_map_real_data_test.ptr<cv::Vec3d>(r);
		double* ptr_err = error_map.ptr<double>(r);
		for (int c = 0; c < nc; c++)
		{
			if (0 != ptr_m[c])
			{
				if (points_num < rebuild_points.size())
				{

					ptr_err[c] = error_list[points_num];

					if (error_list[points_num] < 2)
					{
						ptr_dr[c][0] = rebuild_points[points_num].x;
						ptr_dr[c][1] = rebuild_points[points_num].y;
						ptr_dr[c][2] = rebuild_points[points_num].z;
					}

					points_num++;
				}


			}

		}

	}


	deep_map = deep_map_real_data.clone();
	/***************************************************************************************************************/

	//cv::Rect rect(500, 300, 200, 200);



	//cv::Mat roi = error_map(rect);

	//cv::Mat mat_mean, mat_stddev;
 //
	//cv::meanStdDev(roi, mat_mean, mat_stddev);

	//double mean = mat_mean.at<double>(0, 0);
	//double stddev = mat_stddev.at<double>(0, 0);

	//std::cout<< "mean: " << mean<<std::endl;
	//std::cout<< "stddev: " << stddev << std::endl;

	//cv::Mat show_rect = error_map.clone();
	//cv::rectangle(show_rect, rect, cv::Scalar(255), 3);

	//cv::imwrite("../test_err_00.tiff", error_map);

	return true;

}

