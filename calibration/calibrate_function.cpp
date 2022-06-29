#ifdef _WIN32  
#include <windows.h>
#elif __linux 
#include <cstring>
#endif 
#include "calibrate_function.h" 
#include <assert.h>
#include <fstream> 
#include <iomanip>

Calibrate_Function::Calibrate_Function()
{
	board_size_.width = 7;
	board_size_.height = 11;

	dlp_width_ = 1920;
	dlp_height_ = 1080;

	board_width_ = 40;
	board_height_ = 20;
}
Calibrate_Function::~Calibrate_Function()
{

}


void Calibrate_Function::setCalibrationBoard(int board_num)
{
	switch (board_num)
	{
	case 20:
	{
		setBoardMessage(11, 7, 20, 10);
	}
	break;

	case 40:
	{
		setBoardMessage(11, 7, 40, 20);
	}
	break;
	default:
		break;
	}
}

void Calibrate_Function::setBoardMessage(int rows, int cols, int width, int height)
{
	board_size_.width = cols;
	board_size_.height = rows;

	board_width_ = width;
	board_height_ = height;
}

double Calibrate_Function::computeLineError(std::vector<cv::Point2f> points, double max_err)
{
	double err = 0;

	cv::Point2f f_p = points.front();
	cv::Point2f l_p = points.back();

	double k = (f_p.y - l_p.y) / (f_p.x - l_p.x);

	double b = f_p.y - k * f_p.x;

	double max_e = 0;

	for (int i = 0; i < points.size(); i++)
	{
		double e = std::abs((k * points[i].x - points[i].y + b) / (std::sqrt(k * k + 1)));
		err += e;

		if (max_e < e)
		{
			max_e = e;
		}



	}

	std::cout << "max err: " << max_e << "      ";

	max_err = max_e;

	err /= points.size();

	return err;
}


bool Calibrate_Function::saveMatTxt(cv::Mat mat, std::string path)
{
	if (mat.empty())
	{
		return false;
	}

	int nr = mat.rows;
	int nc = mat.cols;

	//����txt 
	std::ofstream stream(path, std::ios::trunc);

	for (int r = 0; r < nr; r++)
	{
		for (int c = 0; c < nc; c++)
		{
			stream << mat.at<double>(r, c) << " ";
		}

		stream << "\n";
	}
	stream.close();


	return true;
}

bool Calibrate_Function::savePointsTxt(std::vector<cv::Point2f> points, std::string path)
{

	if (points.empty())
		return false;

	/*************************************************************************************************************************/
	//����txt 
	std::ofstream stream(path, std::ios::trunc);

	for (int i = 0; i < points.size(); i++)
	{
		stream << points[i].x << " " << points[i].y << "\n";
	}
	stream.close();

	return false;
}


//дxml
bool Calibrate_Function::writeCalibXml(cv::Mat camera_intrinsic, cv::Mat camera_distortion, cv::Mat projector_instrinsic, cv::Mat projector_distortion, cv::Mat s_r, cv::Mat s_t)
{
	if (!camera_intrinsic.data || !camera_distortion.data || !projector_instrinsic.data || !projector_distortion.data || !s_r.data || !s_t.data)
	{
		return false;
	}

	cv::FileStorage fswrite("./calib.xml", cv::FileStorage::WRITE);

	fswrite << "camera_intrinsic" << camera_intrinsic;
	fswrite << "camera_distortion" << camera_distortion;
	fswrite << "projector_instrinsic" << projector_instrinsic;
	fswrite << "projector_distortion" << projector_distortion;
	fswrite << "s_r" << s_r;
	fswrite << "s_t" << s_t;

	fswrite.release();

	std::cout << "Write Calib Success!" << std::endl;

	return true;
}

bool Calibrate_Function::writeCalibTxt(cv::Mat camera_intrinsic, cv::Mat camera_distortion, cv::Mat projector_instrinsic,
	cv::Mat projector_distortion, cv::Mat s_r, cv::Mat s_t, std::string path)
{
	if (!camera_intrinsic.data || !camera_distortion.data || !projector_instrinsic.data || !projector_distortion.data || !s_r.data || !s_t.data)
	{
		return false;
	}

	/*************************************************************************************************************************/
	//����txt 
	std::ofstream stream(path, std::ios::trunc);

	stream << camera_intrinsic.at<double>(0, 0) << "\n";
	stream << camera_intrinsic.at<double>(0, 1) << "\n";
	stream << camera_intrinsic.at<double>(0, 2) << "\n";
	stream << camera_intrinsic.at<double>(1, 0) << "\n";
	stream << camera_intrinsic.at<double>(1, 1) << "\n";
	stream << camera_intrinsic.at<double>(1, 2) << "\n";
	stream << camera_intrinsic.at<double>(2, 0) << "\n";
	stream << camera_intrinsic.at<double>(2, 1) << "\n";
	stream << camera_intrinsic.at<double>(2, 2) << "\n";


	stream << camera_distortion.at<double>(0, 0) << "\n";
	stream << camera_distortion.at<double>(0, 1) << "\n";
	stream << camera_distortion.at<double>(0, 2) << "\n";
	stream << camera_distortion.at<double>(0, 3) << "\n";
	stream << camera_distortion.at<double>(0, 4) << "\n";



	stream << projector_instrinsic.at<double>(0, 0) << "\n";
	stream << projector_instrinsic.at<double>(0, 1) << "\n";
	stream << projector_instrinsic.at<double>(0, 2) << "\n";
	stream << projector_instrinsic.at<double>(1, 0) << "\n";
	stream << projector_instrinsic.at<double>(1, 1) << "\n";
	stream << projector_instrinsic.at<double>(1, 2) << "\n";
	stream << projector_instrinsic.at<double>(2, 0) << "\n";
	stream << projector_instrinsic.at<double>(2, 1) << "\n";
	stream << projector_instrinsic.at<double>(2, 2) << "\n";


	stream << projector_distortion.at<double>(0, 0) << "\n";
	stream << projector_distortion.at<double>(0, 1) << "\n";
	stream << projector_distortion.at<double>(0, 2) << "\n";
	stream << projector_distortion.at<double>(0, 3) << "\n";
	stream << projector_distortion.at<double>(0, 4) << "\n";

	stream << s_r.at<double>(0, 0) << "\n";
	stream << s_r.at<double>(0, 1) << "\n";
	stream << s_r.at<double>(0, 2) << "\n";
	stream << s_r.at<double>(1, 0) << "\n";
	stream << s_r.at<double>(1, 1) << "\n";
	stream << s_r.at<double>(1, 2) << "\n";
	stream << s_r.at<double>(2, 0) << "\n";
	stream << s_r.at<double>(2, 1) << "\n";
	stream << s_r.at<double>(2, 2) << "\n";

	stream << s_t.at<double>(0, 0) << "\n";
	stream << s_t.at<double>(1, 0) << "\n";
	stream << s_t.at<double>(2, 0) << "\n";

	//for(int r= 0;r< s_r.rows;r++)
	//{
	//	
	//	for(int c= 0;c< s_r.cols;c++)
	//	{
	//		stream << QString::number(s_r.at<double>(r, c));

	//		if(c!= s_r.cols -1)
	//		{
	//			stream << " ";
	//		}
	//	}

	//	stream << "\n";
	//}

	stream.close();

	/*************************************************************************************************************************/

	return true;

}


cv::Vec3f Calibrate_Function::rotationMatrixToEulerAngles(cv::Mat& R)
{
	float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
	bool singular = sy < 1e-6; // If
	float x, y, z;
	if (!singular)
	{
		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		y = atan2(-R.at<double>(2, 0), sy);
		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
	}
	else
	{
		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		y = atan2(-R.at<double>(2, 0), sy);
		z = 0;
	}
#if 1
	x = x * 180.0f / 3.141592653589793f;
	y = y * 180.0f / 3.141592653589793f;
	z = z * 180.0f / 3.141592653589793f;
#endif
	return cv::Vec3f(x, y, z);
}


double Calibrate_Function::calibrateStereo(std::vector<std::vector<cv::Point2f>> camera_points_list, std::vector<std::vector<cv::Point2f>> dlp_points_list, std::string path)
{
	std::vector<std::vector<cv::Point3f>> world_feature_points;


	for (int g_i = 0; g_i < camera_points_list.size(); g_i++)
	{
		std::vector<cv::Point3f> objectCorners = generateAsymmetricWorldFeature(board_width_, board_height_);
		world_feature_points.push_back(objectCorners);
	}

	/********************************************************************************************************************/



	//�궨
	//bool mustInitUndistort = true;
	//int flag = 0;

	//std::vector<cv::Mat> left_rvecs, left_tvecs;
	//std::vector<cv::Mat> right_rvecs, right_tvecs;

	//cv::Mat camera_intrinsic, projector_intrinsic;
	//cv::Mat camera_distortion, projector_distortion;
	//cv::Mat _R, _T, _E, _F;

	//double cameraError = cv::calibrateCamera(world_feature_points,
	//	camera_points_list,
	//	board_size_,
	//	camera_intrinsic,
	//	camera_distortion,
	//	left_rvecs, left_tvecs,
	//	flag, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 500, DBL_EPSILON));

	//std::cout << "camera intrinsic: " << "\n" << camera_intrinsic << "\n";

	//double dlpError = cv::calibrateCamera(world_feature_points,
	//	dlp_points_list,
	//	board_size_,
	//	projector_intrinsic,
	//	projector_distortion,
	//	right_rvecs, right_tvecs,
	//	flag, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 500, DBL_EPSILON));


	cv::Mat _R, _T, _E, _F;

	cv::Mat s_camera_intrinsic = camera_intrinsic_.clone(), s_project_intrinsic = project_intrinsic_.clone();
	cv::Mat s_camera_distortion = camera_distortion_.clone(), s_projector_distortion = projector_distortion_.clone();

	cv::TermCriteria term_criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 500, DBL_EPSILON);

	double stereoError = cv::stereoCalibrate(world_feature_points, camera_points_list, dlp_points_list, s_camera_intrinsic, s_camera_distortion,
		s_project_intrinsic, s_projector_distortion, board_size_, _R, _T, _E, _F,/*cv::CALIB_FIX_INTRINSIC*/ cv::CALIB_USE_INTRINSIC_GUESS /*+ cal_flags*/, term_criteria);

	//double stereoError = cv::stereoCalibrate(world_feature_points, camera_points_list, dlp_points_list, s_camera_intrinsic, s_camera_distortion,
	//	s_project_intrinsic, s_projector_distortion, board_size_, _R, _T, _E, _F, cv::CALIB_FIX_INTRINSIC /*+ cal_flags*/, term_criteria);
	/***********************************************************************************************************/



	std::cout << "camera intrinsic: " << "\n" << s_camera_intrinsic << "\n";
	std::cout << "camera distortion: " << "\n" << s_camera_distortion << "\n";
	std::cout << "project intrinsic: " << "\n" << s_project_intrinsic << "\n";
	std::cout << "projector distortion: " << "\n" << s_projector_distortion << "\n";


	std::cout << "R: " << "\n" << _R << "\n";
	std::cout << "T: " << "\n" << _T << "\n";

	std::cout.flush();

	/******************************************************************************************************************/

	bool ret = writeCalibTxt(s_camera_intrinsic, s_camera_distortion, s_project_intrinsic, s_projector_distortion, _R, _T, path);
	//writeCalibXml(s_camera_intrinsic, s_camera_distortion, s_project_intrinsic, s_projector_distortion, _R, _T);

	if (ret)
	{
		std::cout << "Save Calib: " << path << std::endl;
	}
	else
	{
		std::cout << "Save Calib Error: " << path << std::endl;
	}


	return stereoError;

	/****************************************************************************************************************/

}



double Calibrate_Function::calibrateProjector(std::vector<std::vector<cv::Point2f>> dlp_points_list, std::map<int, bool>& select_group)
{
	std::vector<std::vector<cv::Point3f>> world_feature_points;


	for (int g_i = 0; g_i < dlp_points_list.size(); g_i++)
	{
		select_group.insert(std::pair<int, bool>(g_i, true));

		std::vector<cv::Point3f> objectCorners = generateAsymmetricWorldFeature(board_width_, board_height_);
		world_feature_points.push_back(objectCorners);
	}


	if (dlp_points_list.size() < 6)
	{

		std::cout << "Error: " << std::endl;
		std::cout << "Group Num: " << dlp_points_list.size() << std::endl;
		return false;
	}

	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	std::vector<cv::Mat> rvecsMat, tvecsMat;
	int flag = 0;
	/* ���б궨���� */
	double err_first = cv::calibrateCamera(world_feature_points, dlp_points_list, board_size_, cameraMatrix, distCoeffs, rvecsMat, tvecsMat,
		flag, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 500, DBL_EPSILON));

	//std::cout << "First calibrate error: " << err_first << std::endl;


	double total_err = 0.0;            // ����ͼ���ƽ�������ܺ� 
	double err = 0.0;                  // ÿ��ͼ���ƽ�����
	double totalErr = 0.0;
	double totalPoints = 0.0;
	std::vector<cv::Point2f> image_points_pro;     // �������¼���õ���ͶӰ��

	std::vector<std::vector<cv::Point2f>> select_camera_points;
	std::vector<std::vector<cv::Point3f>> select_world_points;

	/******************************************************************************************************/

	double select_err = err_first;
	int max_series = -1;
	double max_err = 0;

	while (select_err > 0.1 && dlp_points_list.size() > 6)
	{
		total_err = 0;

		for (int i = 0; i < dlp_points_list.size(); i++)
		{

			projectPoints(world_feature_points[i], rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points_pro);   //ͨ���õ������������������Խǵ�Ŀռ���ά�����������ͶӰ����
			err = cv::norm(cv::Mat(dlp_points_list[i]), cv::Mat(image_points_pro), cv::NORM_L2);



			totalErr += err * err;
			totalPoints += world_feature_points[i].size();

			err /= world_feature_points[i].size();
			//std::cout << i + 1 << " err: " << err << " pixel" << std::endl;
			total_err += err;

			if (err > max_err)
			{
				max_err = err;
				max_series = i;
			}

		}

		double ave_err = total_err / dlp_points_list.size();

		if (-1 != max_series)
		{
			for (int g_i = 0, select_i = 0; g_i < select_group.size(); g_i++)
			{
				if (select_group[g_i])
				{
					select_i++;
				}

				if (max_series + 1 == select_i)
				{
					select_group[g_i] = false;

					break;
					//max_series = -10;
				}

			}

			int false_num = 0;
			for (int g_i = 0; g_i < select_group.size(); g_i++)
			{
				if (!select_group[g_i])
				{
					false_num++;
				}
			}

			//std::cout << "False num: " << false_num << std::endl;


			dlp_points_list.erase(dlp_points_list.begin() + max_series);
			world_feature_points.erase(world_feature_points.begin() + max_series);
			max_series = -1;
			max_err = 0;

			err_first = cv::calibrateCamera(world_feature_points, dlp_points_list, board_size_, cameraMatrix, distCoeffs, rvecsMat, tvecsMat,
				flag, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 500, DBL_EPSILON));

			//std::cout << "Dlp error: " << err_first << std::endl;

		}

		std::cout << "projector calibrate err: " << ave_err << std::endl;

		select_err = ave_err;

	}

	project_intrinsic_ = cameraMatrix.clone();
	projector_distortion_ = distCoeffs.clone();
	std::cout << "project_intrinsic_: " << "\n" << project_intrinsic_ << "\n";
	std::cout << "projector_distortion_: " << "\n" << projector_distortion_ << "\n";
	/******************************************************************************************************/
	return select_err;
}


bool Calibrate_Function::bilinearInterpolationFeaturePoints(std::vector<cv::Point2f> feature_points, std::vector<cv::Point3f>& point_3d, cv::Mat point_cloud)
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


bool Calibrate_Function::fillThePhaseHole(cv::Mat& phase, bool is_hor)
{
	if (phase.empty())
	{
		return false;
	}

	if (is_hor)
	{
		for (int r = 1; r < phase.rows - 1; r++)
		{
			double* ptr = phase.ptr<double>(r);
			for (int c = 1; c < phase.cols - 1; c++)
			{
				if (ptr[c] <= 0)
				{

					int c0 = c - 1;
					int c2 = c + 1;

					if (ptr[c0] > 0 && ptr[c2] > 0)
					{
						ptr[c] = (ptr[c0] + ptr[c2]) / 2.0;
					}

				}

			}

		}
	}
	else
	{
		for (int r = 1; r < phase.rows - 1; r++)
		{
			double* ptr = phase.ptr<double>(r);
			for (int c = 1; c < phase.cols - 1; c++)
			{
				if (ptr[c] <= 0)
				{

					int r0 = r - 1;
					int r2 = r + 1;

					if (r0 > 0 && r2 < phase.rows)
					{

						double* ptr_r0 = phase.ptr<double>(r0);
						double* ptr_r2 = phase.ptr<double>(r2);

						if (ptr_r0[c] > 0 && ptr_r2[c] > 0)
						{
							ptr[c] = (ptr_r0[c] + ptr_r2[c]) / 2.0;
						}
					}

				}

			}

		}
	}

	//for (int r = 1; r < phase.rows-1; r++)
	//{
	//	double* ptr = phase.ptr<double>(r);
	//	for (int c = 1; c < phase.cols -1; c++)
	//	{
	//		if (ptr[c] <= 0)
	//		{

	//			int c0 = c - 1;
	//			int c2 = c + 1;
	//			int r0 = r - 1;
	//			int r2 = r + 1;

	//			if (c0 > 0 && r0 > 0 && c2< phase.cols && r2 < phase.rows)
	//			{

	//				double* ptr_r0 = phase.ptr<double>(r0);
	//				double* ptr_r2 = phase.ptr<double>(r2);

	//				if (ptr[c0] > 0 && ptr[c2] > 0 && ptr_r0[c] > 0 && ptr_r2[c] > 0)
	//				{
	//					ptr[c] = (ptr[c0] + ptr[c2] + ptr_r0[c] + ptr_r2[c]) / 4.0;
	//				}
	//			}

	//		}

	//	}

	//}

	return true;
}

double Calibrate_Function::Bilinear_interpolation(double x, double y, cv::Mat& mapping)
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

bool Calibrate_Function::cameraPointsToDlp(std::vector<cv::Point2f> camera_points, cv::Mat unwrap_map_hor, cv::Mat unwrap_map_ver, int group_num, int dlp_width, int dlp_height, std::vector<cv::Point2f>& dlp_points)
{
	if (camera_points.empty() || !unwrap_map_hor.data || !unwrap_map_ver.data)
		return false;

	bool ret = true;

	dlp_points.clear();

	double phase_max = 2 * CV_PI * std::pow(2.0, group_num - 1);

	for (int p_i = 0; p_i < camera_points.size(); p_i++)
	{
		cv::Point2f pos = camera_points[p_i];

		//cv::Point d_p;
		//d_p.x = pos.x + 0.5;
		//d_p.y = pos.y + 0.5;
		//���Բ�ֵ�Ż�


		double  hor_val = Bilinear_interpolation(pos.x, pos.y, unwrap_map_hor);
		double  ver_val = Bilinear_interpolation(pos.x, pos.y, unwrap_map_ver);

		if (hor_val < 0 || ver_val < 0)
		{
			ret = false;
		}

		//int x = pos.x;
		//int y = pos.y;

		//double hor_val_0 = unwrap_map_hor.at<double>(y, x);
		//double hor_val_1 = unwrap_map_hor.at<double>(y+1, x);

		//double ver_val_0 = unwrap_map_ver.at<double>(y,  x);
		//double ver_val_1 = unwrap_map_ver.at<double>(y , x+1);

		//double hor_val = hor_val_0 + (pos.y - y) * (hor_val_1 - hor_val_0);
		//double ver_val = ver_val_0 + (pos.x - x) * (ver_val_1 - ver_val_0);


		//if (hor_val_0 < 0.5 || hor_val_1 < 0.5 || ver_val_0 < 0.5 || ver_val_1 < 0.5)
		//{
		//	ret = false;
		//}



		cv::Point2f dlp_p;

		dlp_p.x = dlp_width * ver_val / phase_max;
		dlp_p.y = dlp_height * hor_val / phase_max;

		dlp_points.push_back(dlp_p);

	}

	return ret;

}


std::vector<cv::Point3f> Calibrate_Function::generateSymmetricWorldFeature(float width, float height)
{
	std::vector<cv::Point3f> objectCorners;
	for (int r = 0; r < board_size_.height; r++)
	{
		for (int c = 0; c < board_size_.width; c++)
		{
			objectCorners.push_back(cv::Point3f(width * c, height * r, 0.0f));
		}
	}

	return objectCorners;
}

std::vector<cv::Point3f> Calibrate_Function::generateAsymmetricWorldFeature(float width, float height)
{
	std::vector<cv::Point3f> objectCorners;
	for (int r = 0; r < board_size_.height; r++)
	{
		for (int c = 0; c < board_size_.width; c++)
		{

			if (0 == r % 2)
			{

				objectCorners.push_back(cv::Point3f(width * c, height * r, 0.0f));
			}
			else if (1 == r % 2)
			{

				objectCorners.push_back(cv::Point3f(width * c + 0.5 * width, height * r, 0.0f));
			}

		}
	}

	return objectCorners;
}

double Calibrate_Function::calibrateCamera(std::vector<std::vector<cv::Point2f>> camera_points_list, std::map<int, bool>& select_group)
{
	select_group.clear();
	for (int g_i = 0; g_i < camera_points_list.size(); g_i++)
	{
		select_group.insert(std::pair<int, bool>(g_i, true));
	}

	std::vector<std::vector<cv::Point3f>> world_feature_points;
	for (int g_i = 0; g_i < camera_points_list.size(); g_i++)
	{
		select_group.insert(std::pair<int, bool>(g_i, true));

		std::vector<cv::Point3f> objectCorners = generateAsymmetricWorldFeature(board_width_, board_height_);

		world_feature_points.push_back(objectCorners);
	}

	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	std::vector<cv::Mat> rvecsMat, tvecsMat;
	int flag = 0;

	/* ���б궨���� */
	double err_first = cv::calibrateCamera(world_feature_points, camera_points_list, board_size_, cameraMatrix, distCoeffs, rvecsMat, tvecsMat,
		flag, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 500, DBL_EPSILON));

	//std::cout << "First calibrate error: " << err_first<<std::endl;


	double total_err = 0.0;            // ����ͼ���ƽ�������ܺ� 
	double err = 0.0;                  // ÿ��ͼ���ƽ�����
	double totalErr = 0.0;
	double totalPoints = 0.0;
	std::vector<cv::Point2f> image_points_pro;     // �������¼���õ���ͶӰ��

	std::vector<std::vector<cv::Point2f>> select_camera_points;
	std::vector<std::vector<cv::Point3f>> select_world_points;

	/******************************************************************************************************/

	double select_err = err_first;
	int max_series = -1;
	double max_err = 0;

	while (select_err > 0.1 && camera_points_list.size() > 5)
	{
		total_err = 0;

		for (int i = 0; i < camera_points_list.size(); i++)
		{

			projectPoints(world_feature_points[i], rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points_pro);   //ͨ���õ������������������Խǵ�Ŀռ���ά�����������ͶӰ����
			err = cv::norm(cv::Mat(camera_points_list[i]), cv::Mat(image_points_pro), cv::NORM_L2);



			totalErr += err * err;
			totalPoints += world_feature_points[i].size();

			err /= world_feature_points[i].size();
			//std::cout << i + 1 << " err:" << err << " pixel" << std::endl;
			total_err += err;

			if (err > max_err)
			{
				max_err = err;
				max_series = i;
			}

		}

		double ave_err = total_err / camera_points_list.size();

		if (-1 != max_series)
		{
			for (int g_i = 0, select_i = 0; g_i < select_group.size(); g_i++)
			{
				if (select_group[g_i])
				{
					select_i++;
				}

				if (max_series + 1 == select_i)
				{
					select_group[g_i] = false;

					break;
					//max_series = -10;
				}

			}

			int false_num = 0;
			for (int g_i = 0; g_i < select_group.size(); g_i++)
			{
				if (!select_group[g_i])
				{
					false_num++;
				}
			}

			//std::cout << "False num: " << false_num << std::endl;


			camera_points_list.erase(camera_points_list.begin() + max_series);
			world_feature_points.erase(world_feature_points.begin() + max_series);
			max_series = -1;
			max_err = 0;

			err_first = cv::calibrateCamera(world_feature_points, camera_points_list, board_size_, cameraMatrix, distCoeffs, rvecsMat, tvecsMat,
				flag, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 500, DBL_EPSILON));

			//std::cout  << "calibrate error: " << err_first << std::endl;

		}

		std::cout << "camera calibrate err: " << ave_err << std::endl;

		select_err = ave_err;

	}

	/******************************************************************************************************/
	camera_intrinsic_ = cameraMatrix.clone();
	camera_distortion_ = distCoeffs.clone();


	std::cout << "camera_intrinsic_: " << "\n" << camera_intrinsic_ << "\n";
	std::cout << "camera_distortion_: " << "\n" << camera_distortion_ << "\n";

	return select_err;


}



int Calibrate_Function::testOverExposure(cv::Mat img, std::vector<cv::Point2f> points)
{
	if (img.empty() || points.empty())
		return -1;

	for (int p = 0; p < points.size(); p++)
	{
		cv::Point2f point = points[p];

		if (255 == img.at<uchar>(point.y, point.x))
		{
			return 0;
		}
	}

	return 1;
}

bool Calibrate_Function::findCircleBoardFeature(cv::Mat img, std::vector<cv::Point2f>& points)
{
	std::vector<cv::Point2f> circle_points;
	cv::Mat img_inv = inv_image(img);
	bool found = cv::findCirclesGrid(img_inv, board_size_, circle_points, cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);

	if (!found)
		return false;

	points = circle_points;


	return true;

}

cv::Mat Calibrate_Function::inv_image(cv::Mat img)
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