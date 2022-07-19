#include "reconstruct.h"
#include <iostream> 
#include <fstream>
#include "../src/triangulation.h"
#include <opencv2/imgcodecs.hpp>
#include "../firmware/protocol.h"

Reconstruct::Reconstruct()
{
	dlp_width_ = 1920;
	dlp_height_ = 1080;
}


Reconstruct::~Reconstruct()
{

}

/*******************************************************************************************************************/

bool Reconstruct::setCameraVersion(int version)
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

bool Reconstruct::setCalibData(CameraCalibParam param)
{
	camera_intrinsic_ = cv::Mat(3, 3, CV_32F, cv::Scalar(0.0));
	camera_distortion_ = cv::Mat(1, 5, CV_32F, cv::Scalar(0.0));

	project_intrinsic_ = cv::Mat(3, 3, CV_32F, cv::Scalar(0.0));
	projector_distortion_ = cv::Mat(1, 5, CV_32F, cv::Scalar(0.0));

	rotation_matrix_ = cv::Mat(3, 3, CV_32F, cv::Scalar(0.0));
	translation_matrix_ = cv::Mat(3, 1, CV_32F, cv::Scalar(0.0));


	float* ptr_c_i = camera_intrinsic_.ptr<float>(0);

	ptr_c_i[0] = param.camera_intrinsic[0];
	ptr_c_i[1] = param.camera_intrinsic[1];
	ptr_c_i[2] = param.camera_intrinsic[2];
	ptr_c_i[3] = param.camera_intrinsic[3];
	ptr_c_i[4] = param.camera_intrinsic[4];
	ptr_c_i[5] = param.camera_intrinsic[5];
	ptr_c_i[6] = param.camera_intrinsic[6];
	ptr_c_i[7] = param.camera_intrinsic[7];
	ptr_c_i[8] = param.camera_intrinsic[8];


	float* ptr_c_d = camera_distortion_.ptr<float>(0);
	ptr_c_d[0] = param.camera_distortion[0];
	ptr_c_d[1] = param.camera_distortion[1];
	ptr_c_d[2] = param.camera_distortion[2];
	ptr_c_d[3] = param.camera_distortion[3];
	ptr_c_d[4] = param.camera_distortion[4];


	float* ptr_p_i = project_intrinsic_.ptr<float>(0);
	ptr_p_i[0] = param.projector_intrinsic[0];
	ptr_p_i[1] = param.projector_intrinsic[1];
	ptr_p_i[2] = param.projector_intrinsic[2];
	ptr_p_i[3] = param.projector_intrinsic[3];
	ptr_p_i[4] = param.projector_intrinsic[4];
	ptr_p_i[5] = param.projector_intrinsic[5];
	ptr_p_i[6] = param.projector_intrinsic[6];
	ptr_p_i[7] = param.projector_intrinsic[7];
	ptr_p_i[8] = param.projector_intrinsic[8];


	float* ptr_p_d = projector_distortion_.ptr<float>(0);
	ptr_p_d[0] = param.projector_distortion[0];
	ptr_p_d[1] = param.projector_distortion[1];
	ptr_p_d[2] = param.projector_distortion[2];
	ptr_p_d[3] = param.projector_distortion[3];
	ptr_p_d[4] = param.projector_distortion[4];


	float* ptr_r = rotation_matrix_.ptr<float>(0);
	ptr_r[0] = param.rotation_matrix[0];
	ptr_r[1] = param.rotation_matrix[1];
	ptr_r[2] = param.rotation_matrix[2];
	ptr_r[3] = param.rotation_matrix[3];
	ptr_r[4] = param.rotation_matrix[4];
	ptr_r[5] = param.rotation_matrix[5];
	ptr_r[6] = param.rotation_matrix[6];
	ptr_r[7] = param.rotation_matrix[7];
	ptr_r[8] = param.rotation_matrix[8];


	float* ptr_t = translation_matrix_.ptr<float>(0);
	ptr_t[0] = param.translation_matrix[0];
	ptr_t[1] = param.translation_matrix[1];
	ptr_t[2] = param.translation_matrix[2];


	cv::Mat E_1 = cv::Mat::eye(3, 4, CV_32FC1);
	cv::Mat E_2 = cv::Mat::zeros(3, 4, CV_32FC1);

	M_1_ = cv::Mat::zeros(3, 4, CV_32FC1);
	M_2_ = cv::Mat::zeros(3, 4, CV_32FC1);

	for (int i = 0; i < rotation_matrix_.rows; i++)
	{
		for (int j = 0; j < rotation_matrix_.cols; j++)
		{
			E_2.at<float>(i, j) = rotation_matrix_.at<float>(i, j);
		}
	}

	for (int i = 0; i < 3; i++)
	{
		E_2.at<float>(i, 3) = translation_matrix_.at<float>(i);
	}


	M_1_ = camera_intrinsic_ * E_1;
	M_2_ = project_intrinsic_ * E_2;

	return true;
}

bool Reconstruct::undistortedPoints(std::vector<cv::Point2f> distortPoints, cv::Mat intrinsic, cv::Mat distortion, std::vector<cv::Point2f>& undisted_points)
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


	float fc_x = intrinsic.at<float>(0, 0);
	float fc_y = intrinsic.at<float>(1, 1);
	float cc_x = intrinsic.at<float>(0, 2);
	float cc_y = intrinsic.at<float>(1, 2);

	float k1 = distortion.at<float>(0, 0);
	float k2 = distortion.at<float>(0, 1);
	float p1 = distortion.at<float>(0, 2);
	float p2 = distortion.at<float>(0, 3);
	float k3 = distortion.at<float>(0, 4);


	undisted_points.clear();

	for (int i = 0; i < distortPoints.size(); i++)
	{
		cv::Point2f n_p;

		normalizePoint(distortPoints[i].x, distortPoints[i].y, fc_x, fc_y,
			cc_x, cc_y, k1, k2, k3, p1, p2, n_p.x, n_p.y);

		undisted_points.push_back(n_p);
	}


	return true;
}


bool Reconstruct::rebuildPoints(std::vector<cv::Point2f> camera_points, std::vector<cv::Point2f> dlp_points, std::vector<cv::Point3f>& rebuild_points, std::vector<float>& error_list)
{

	if (camera_points.size() != dlp_points.size() || camera_points.empty())
	{
		return false;
	}


	float* R_data = (float*)rotation_matrix_.data;
	float* T_data = (float*)translation_matrix_.data;

	rebuild_points.clear();
	rebuild_points.resize(camera_points.size());
	error_list.clear();
	error_list.resize(camera_points.size());

#pragma omp parallel for
	for (int i = 0; i < camera_points.size(); i++)
	{

		float X_L;
		float Y_L;
		float Z_L;

		float X_R;
		float Y_R;
		float Z_R;
		float error;

		triangulation(camera_points[i].x, camera_points[i].y,
			dlp_points[i].x, dlp_points[i].y,
			R_data, T_data,
			X_L, Y_L, Z_L, X_R, Y_R, Z_R, error);


		error_list[i] = error;
		rebuild_points[i] = cv::Point3f(X_L, Y_L, Z_L);

	}


	return true;

}

bool Reconstruct::rebuildData(cv::Mat unwrap_map_x, cv::Mat unwrap_map_y, int period_nun, cv::Mat& deep_map, cv::Mat& err_map)
{
	if (!M_1_.data || !M_2_.data)
	{
		return false;
	}

	if (!unwrap_map_x.data || !unwrap_map_y.data)
	{
		return false;
	}


	float phase_max = 2 * CV_PI * period_nun;

	int nr = unwrap_map_x.rows;
	int nc = unwrap_map_x.cols;


	cv::Mat mask(nr, nc, CV_8U, cv::Scalar(0));

	std::vector<cv::Point2f> camera_points;
	std::vector<cv::Point2f> dlp_points;
	std::vector<float> error_list;
	//map to dlp pos

	for (int r = 0; r < nr; r++)
	{
		float* ptr_x = unwrap_map_x.ptr<float>(r);
		float* ptr_y = unwrap_map_y.ptr<float>(r);
		uchar* ptr_m = mask.ptr<uchar>(r);
		for (int c = 0; c < nc; c++)
		{
			if (0 != ptr_x[c])
			{
				cv::Point2f dlp_p;
				dlp_p.x = dlp_width_ * ptr_x[c] / phase_max;
				dlp_p.y = dlp_height_ * ptr_y[c] / phase_max;

				cv::Point2f camera_p(c, r);
				camera_points.push_back(camera_p);
				dlp_points.push_back(dlp_p);

				ptr_m[c] = 255;

			}

		}
	}
	/*********************************************************************************/


	//畸变校正
	std::vector<cv::Point2f> correct_camera_points;
	std::vector<cv::Point2f> correct_dlp_points;



	undistortedPoints(camera_points, camera_intrinsic_, camera_distortion_, correct_camera_points);
	undistortedPoints(dlp_points, project_intrinsic_, projector_distortion_, correct_dlp_points);


	//std::vector<cv::Point2f> correct_camera_points_user;
	//std::vector<cv::Point2f> correct_dlp_points_user;
	//cv::undistortPoints(camera_points, correct_camera_points_user, camera_intrinsic_, camera_distortion_, cv::noArray(), camera_intrinsic_);
	//cv::undistortPoints(dlp_points, correct_dlp_points_user, project_intrinsic_, projector_distortion_, cv::noArray(), project_intrinsic_);



	std::vector<cv::Point3f> rebuild_points_test;
	std::vector<cv::Point3f> rebuild_points;

	rebuildPoints(correct_camera_points, correct_dlp_points, rebuild_points, error_list);

	int points_num = 0;

	cv::Mat deep_map_real_data(nr, nc, CV_32FC3, cv::Scalar(0, 0, 0));
	cv::Mat deep_map_real_data_test(nr, nc, CV_32FC3, cv::Scalar(0, 0, 0));
	cv::Mat error_map(nr, nc, CV_32FC1, cv::Scalar(0));

	for (int r = 0; r < nr; r++)
	{
		uchar* ptr_m = mask.ptr<uchar>(r);
		cv::Vec3f* ptr_dr = deep_map_real_data.ptr<cv::Vec3f>(r);
		cv::Vec3f* ptr_dr_test = deep_map_real_data_test.ptr<cv::Vec3f>(r);
		float* ptr_err = error_map.ptr<float>(r);
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

