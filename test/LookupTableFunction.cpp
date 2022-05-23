#include "LookupTableFunction.h"
#include "iostream"
#include <fstream>
#include "../firmware/protocol.h"
//#include "FileIoFunction.h" 

LookupTableFunction::LookupTableFunction()
{
	image_size_.width = 1920;
	image_size_.height = 1200;

	min_low_z_ = 300;
	max_max_z_ = 3000;

	dlp_width_ = 1920;
	dlp_height_ = 1080;
}


LookupTableFunction::~LookupTableFunction()
{
}

bool LookupTableFunction::setCameraVersion(int version)
{

	switch (version)
	{
	case DFX_800:
	{
		dlp_width_ = 1280;
		dlp_height_ = 720;

		min_low_z_ = 100;
		max_max_z_ = 5000;

		return true;
	}
	break;

	case DFX_1800:
	{

		dlp_width_ = 1920;
		dlp_height_ = 1080;

		min_low_z_ = 300;
		max_max_z_ = 10000;

		return true;
	}
	break;

	default:
		break;
	}

	return false;


}

/*******************************************************************************************************/

bool LookupTableFunction::undistortedPoints(std::vector<cv::Point2d> distortPoints, cv::Mat intrinsic, cv::Mat distortion, std::vector<cv::Point2d>& undisted_points)
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


void LookupTableFunction::normalizePoint(
	double x, double y,
	double fc_x, double fc_y,
	double cc_x, double cc_y,
	double k1, double k2, double k3, double p1, double p2,
	double& x_norm, double& y_norm)
{
	double x_distort = (x - cc_x) / fc_x;
	double y_distort = (y - cc_y) / fc_y;

	double x_iter = x_distort;
	double y_iter = y_distort;

	for (int i = 0; i < 20; i++)
	{
		double r_2 = x_iter * x_iter + y_iter * y_iter;
		double r_4 = r_2 * r_2;
		double r_6 = r_4 * r_2;
		double k_radial = 1 + k1 * r_2 + k2 * r_4 + k3 * r_6;
		double delta_x = 2 * p1 * x_iter * y_iter + p2 * (r_2 + 2 * x_iter * x_iter);
		double delta_y = p1 * (r_2 + 2 * y_iter * y_iter) + 2 * p2 * x_iter * y_iter;
		x_iter = (x_distort - delta_x) / k_radial;
		y_iter = (y_distort - delta_y) / k_radial;
	}
	//x_norm = x_iter*fc_x+ cc_x;
	//y_norm = y_iter*fc_y + cc_y;

	x_norm = x_iter;
	y_norm = y_iter;

	return;
}


bool LookupTableFunction::generateRotateTable(cv::Mat camera_intrinsic_, cv::Mat camera_distortion_, cv::Mat rectify_R,
	cv::Size size, cv::Mat& rotate_x, cv::Mat& rotate_y)
{

	int nr = size.height;
	int nc = size.width;

	cv::Mat undistort_map_x(nr, nc, CV_64F, cv::Scalar(-2));
	cv::Mat undistort_map_y(nr, nc, CV_64F, cv::Scalar(-2));

	std::vector<cv::Point2d> camera_points;

	for (int r = 0; r < nr; r++)
	{
		for (int c = 0; c < nc; c++)
		{
			cv::Point2d camera_p(c, r);
			camera_points.push_back(camera_p);
		}
	}
	/*********************************************************************************/


	//����У��
	std::vector<cv::Point2d> correct_camera_points;
	undistortedPoints(camera_points, camera_intrinsic_, camera_distortion_, correct_camera_points);

	//r1 rotate
	for (int r = 0; r < nr; r++)
	{
		double* ptr_x = undistort_map_x.ptr<double>(r);
		double* ptr_y = undistort_map_y.ptr<double>(r);
		for (int c = 0; c < nc; c++)
		{
			cv::Mat vec(3, 1, CV_64FC1, cv::Scalar(1));

			int offset = r * nc + c;

			vec.at<double>(0, 0) = correct_camera_points[offset].x;
			vec.at<double>(1, 0) = correct_camera_points[offset].y;

			cv::Mat rotate = rectify_R * vec;

			rotate = rotate / rotate.at<double>(2, 0);

			ptr_x[c] = rotate.at<double>(0, 0);
			ptr_y[c] = rotate.at<double>(1, 0);

		}
	}

	rotate_x = undistort_map_x.clone();
	rotate_y = undistort_map_y.clone();

	return true;
}


bool LookupTableFunction::generateGridMapping(cv::Mat rotate_x, cv::Mat rotate_y, cv::Mat& map)
{
	int table_rows = 4000;
	int table_cols = 2000;

	cv::Mat interpolation_map(table_rows, table_cols, CV_64FC1, cv::Scalar(-2));

	std::cout << "interpolation_map!" << std::endl;
	int nr = rotate_x.rows;
	int nc = rotate_x.cols;
	std::cout << nr << std::endl;
	std::cout << nc << std::endl;

	for (int r = 0; r < nr; r++)
	{
		double* ptr_x = rotate_x.ptr<double>(r);
		double* ptr_y = rotate_y.ptr<double>(r);

		for (int c = 0; c < nc; c++)
		{
			int m_r = 2000 * (ptr_y[c] + 1);

			if (m_r >= table_rows || m_r < 0)
			{
				// std::cout << "error m_r: " << m_r << std::endl;
			}
			else
			{
				interpolation_map.at<double>(m_r, c) = ptr_x[c];
			}

		}
	}

	//��ֵ

	for (int r = 1; r < interpolation_map.rows - 1; r++)
	{
		double* ptr_up = interpolation_map.ptr<double>(r - 1);
		double* ptr_data = interpolation_map.ptr<double>(r);
		double* ptr_next = interpolation_map.ptr<double>(r + 1);
		for (int c = 0; c < interpolation_map.cols; c++)
		{
			if (-2 == ptr_data[c])
			{
				if (-2 != ptr_up[c] && -2 != ptr_next[c])
				{
					ptr_data[c] = (ptr_up[c] + ptr_next[c]) / 2.0;
				}
			}
		}

	}


	map = interpolation_map.clone();

	return true;
}



bool LookupTableFunction::generateLookTable(cv::Mat& xL_rotate_x, cv::Mat& xL_rotate_y, cv::Mat& rectify_R1, cv::Mat& pattern_mapping)
{

	if (!camera_intrinsic_.data || !camera_distortion_.data || !project_intrinsic_.data ||
		!projector_distortion_.data || !rotation_matrix_.data || !translation_matrix_.data)
	{
		return false;
	}

	cv::Mat R1, R2, P1, P2, Q, V1, V2;

	cv::stereoRectify(camera_intrinsic_, camera_distortion_, project_intrinsic_, projector_distortion_,
		image_size_, rotation_matrix_, translation_matrix_,
		R1, R2, P1, P2, Q);

	int nr = image_size_.height;
	int nc = image_size_.width;

	cv::Mat xL_undistort_map_x(nr, nc, CV_64F, cv::Scalar(-2));
	cv::Mat xL_undistort_map_y(nr, nc, CV_64F, cv::Scalar(-2));

	cv::Mat xR_undistort_map_x(nr, nc, CV_64F, cv::Scalar(-2));
	cv::Mat xR_undistort_map_y(nr, nc, CV_64F, cv::Scalar(-2));

	//cv::Mat mask(nr, nc, CV_8U, cv::Scalar(0));


	generateRotateTable(camera_intrinsic_, camera_distortion_, R1, image_size_, xL_undistort_map_x, xL_undistort_map_y);
	generateRotateTable(project_intrinsic_, projector_distortion_, R2, cv::Size(1920, 1200), xR_undistort_map_x, xR_undistort_map_y);


	cv::Mat mapping;
	generateGridMapping(xR_undistort_map_x, xR_undistort_map_y, mapping);


	single_pattern_mapping_ = mapping.clone();
	xL_rotate_x_ = xL_undistort_map_x.clone();
	xL_rotate_y_ = xL_undistort_map_y.clone();
	R_1_ = R1.clone();

	xL_rotate_x = xL_undistort_map_x.clone();
	xL_rotate_y = xL_undistort_map_y.clone();
	rectify_R1 = R1.clone();
	pattern_mapping = mapping.clone();


	return true;
}

/******************************************************************************************************/

bool LookupTableFunction::readTableFloat(std::string dir_path, cv::Mat& xL_rotate_x, cv::Mat& xL_rotate_y, cv::Mat& rectify_R1, cv::Mat& pattern_mapping)
{

	/****************************************************************************************************************************/

	if (!readBinMappingFloat(3, 3, dir_path + "/R1.bin", R_1_))
	{
		return false;
	}

	if (!readBinMappingFloat(4000, 2000, dir_path + "/single_pattern_mapping.bin", single_pattern_mapping_))
	{
		return false;
	}

	if (!readBinMappingFloat(1200, 1920, dir_path + "/combine_xL_rotate_x_cam1_iter.bin", xL_rotate_x_))
	{
		return false;
	}

	if (!readBinMappingFloat(1200, 1920, dir_path + "/combine_xL_rotate_y_cam1_iter.bin", xL_rotate_y_))
	{
		return false;
	}
	/****************************************************************************************************************************/



	if (!single_pattern_mapping_.data || !xL_rotate_x_.data || !xL_rotate_y_.data)
	{
		return false;
	}

	xL_rotate_x = xL_rotate_x_.clone();
	xL_rotate_y = xL_rotate_y_.clone();
	rectify_R1 = R_1_.clone();
	pattern_mapping = single_pattern_mapping_.clone();

	return true;
}

bool LookupTableFunction::readTable(std::string dir_path, int rows, int cols)
{

	//single_pattern_mapping_ = readmapping(4000, 2000, "../config_files/single_pattern_mapping.txt");
	//xL_rotate_x_ = readmapping(rows, cols, "../config_files/combine_xL_rotate_x_cam1_iter.txt");
	//xL_rotate_y_ = readmapping(rows, cols, "../config_files/combine_xL_rotate_y_cam1_iter.txt");
	//R_1_ = readmapping(3, 3, "../config_files/R1.txt");



	/****************************************************************************************************************************/

	if (!readBinMapping(3, 3, dir_path + "/R1.bin", R_1_))
	{
		return false;
	}

	if (!readBinMapping(4000, 2000, dir_path + "/single_pattern_mapping.bin", single_pattern_mapping_))
	{
		return false;
	}

	if (!readBinMapping(rows, cols, dir_path + "/combine_xL_rotate_x_cam1_iter.bin", xL_rotate_x_))
	{
		return false;
	}

	if (!readBinMapping(rows, cols, dir_path + "/combine_xL_rotate_y_cam1_iter.bin", xL_rotate_y_))
	{
		return false;
	}
	/****************************************************************************************************************************/

	cv::Mat test_single_pattern_mapping_ = single_pattern_mapping_.clone();
	cv::Mat test_xL_rotate_x_ = xL_rotate_x_.clone();
	cv::Mat test_xL_rotate_y_ = xL_rotate_y_.clone();
	cv::Mat test_R_1_ = R_1_.clone();


	//xR_rotate_x_ = readmapping(rows, cols, "../config_files/combine_xR_rotate_x_cam1_iter.txt");
	//xR_rotate_y_ = readmapping(rows, cols, "../config_files/combine_xR_rotate_y_cam1_iter.txt");

	if (!single_pattern_mapping_.data || !xL_rotate_x_.data || !xL_rotate_y_.data)
	{
		return false;
	}


	//saveBinMapping("../test_write_single_pattern_mapping.dat", single_pattern_mapping_);

	//cv::Mat test_read_mat;

	//TestReadBinMapping(4000, 2000, "../test_write_single_pattern_mapping.dat", test_read_mat);

	//if (!readBinMapping(4000, 2000, dir_path + "/single_pattern_mapping.dat", single_pattern_mapping_))
	//{
	//	return false;
	//}

	return true;

}


bool LookupTableFunction::getLookTable(cv::Mat& xL_rotate_x, cv::Mat& xL_rotate_y, cv::Mat& rectify_R1, cv::Mat& pattern_mapping)
{
	if (!single_pattern_mapping_.data || !xL_rotate_x_.data || !xL_rotate_y_.data)
	{
		return false;
	}

	xL_rotate_x = xL_rotate_x_.clone();
	xL_rotate_y = xL_rotate_y_.clone();
	rectify_R1 = R_1_.clone();
	pattern_mapping = single_pattern_mapping_.clone();

	return true;
}

void LookupTableFunction::setCalibData(struct CameraCalibParam calib_param)
{
	cv::Mat camera_intrinsic = cv::Mat(3, 3, CV_32F, calib_param.camera_intrinsic);
	cv::Mat camera_distortion = cv::Mat(1, 5, CV_32F, calib_param.camera_distortion);

	cv::Mat project_intrinsic = cv::Mat(3, 3, CV_32F, calib_param.projector_intrinsic);
	cv::Mat projector_distortion = cv::Mat(1, 5, CV_32F, calib_param.projector_distortion);

	cv::Mat rotation_matrix = cv::Mat(3, 3, CV_32F, calib_param.rotation_matrix);
	cv::Mat translation_matrix = cv::Mat(3, 1, CV_32F, calib_param.translation_matrix);

	camera_intrinsic.convertTo(camera_intrinsic, CV_64F);
	camera_distortion.convertTo(camera_distortion, CV_64F);
	project_intrinsic.convertTo(project_intrinsic, CV_64F);
	projector_distortion.convertTo(projector_distortion, CV_64F);
	rotation_matrix.convertTo(rotation_matrix, CV_64F);
	translation_matrix.convertTo(translation_matrix, CV_64F);

	camera_intrinsic_ = camera_intrinsic.clone();
	project_intrinsic_ = project_intrinsic.clone();
	camera_distortion_ = camera_distortion.clone();
	projector_distortion_ = projector_distortion.clone();
	rotation_matrix_ = rotation_matrix.clone();
	translation_matrix_ = translation_matrix.clone();

	value_b_ = sqrt(pow(translation_matrix_.at<double>(0, 0), 2) + pow(translation_matrix_.at<double>(1, 0), 2) + pow(translation_matrix_.at<double>(2, 0), 2));

}

bool LookupTableFunction::readCalibData(std::string path)
{
	std::ifstream myfile(path);

	if (!myfile.is_open())
	{
		std::cout << "can not open this file" << std::endl;
		return 0;
	}

	double I[40] = { 0 };

	//��data1�ļ��ж���int����
	for (int i = 0; i < 40; i++)
	{

		myfile >> I[i];
		//std::cout << I[i]<<std::endl;

	}

	myfile.close();

	cv::Mat camera_intrinsic = cv::Mat(3, 3, CV_64F, cv::Scalar(0.0));
	cv::Mat camera_distortion = cv::Mat(1, 5, CV_64F, cv::Scalar(0.0));

	cv::Mat project_intrinsic = cv::Mat(3, 3, CV_64F, cv::Scalar(0.0));
	cv::Mat projector_distortion = cv::Mat(1, 5, CV_64F, cv::Scalar(0.0));

	cv::Mat rotation_matrix = cv::Mat(3, 3, CV_64F, cv::Scalar(0.0));
	cv::Mat translation_matrix = cv::Mat(3, 1, CV_64F, cv::Scalar(0.0));


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


	camera_intrinsic_ = camera_intrinsic.clone();
	project_intrinsic_ = project_intrinsic.clone();
	camera_distortion_ = camera_distortion.clone();
	projector_distortion_ = projector_distortion.clone();
	rotation_matrix_ = rotation_matrix.clone();
	translation_matrix_ = translation_matrix.clone();

	value_b_ = sqrt(pow(translation_matrix_.at<double>(0, 0), 2) + pow(translation_matrix_.at<double>(1, 0), 2) + pow(translation_matrix_.at<double>(2, 0), 2));

	return true;
}

bool LookupTableFunction::readCalibXml(std::string path)
{

	//FileIoFunction file_io_machine;
 //
	//file_io_machine.readCalibXml(QString::fromStdString(path), camera_intrinsic_, project_intrinsic_, camera_distortion_,
	//	projector_distortion_, rotation_matrix_, translation_matrix_);

	///**************************************************************************************************************/
	//if (!camera_intrinsic_.data || !camera_distortion_.data || !project_intrinsic_.data ||
	//	!projector_distortion_.data || !rotation_matrix_.data || !translation_matrix_.data)
	//{
	//	return false;
	//}

	//value_b_ = sqrt(pow(translation_matrix_.at<double>(0, 0), 2) + pow(translation_matrix_.at<double>(1, 0), 2) + pow(translation_matrix_.at<double>(2, 0), 2));


	return true;



}


void LookupTableFunction::setRebuildValueB(double  b)
{
	value_b_ = b;

}

bool LookupTableFunction::getRebuildValueB(double& b)
{
	if (!translation_matrix_.data)
	{
		return false;
	}
	b = value_b_;

	return true;
}


bool LookupTableFunction::TestReadBinMapping(int rows, int cols, std::string mapping_file, cv::Mat& out_map)
{
	cv::Mat map(rows, cols, CV_64F, cv::Scalar(0));

	std::ifstream inFile(mapping_file, std::ios::in | std::ios::binary); //�����ƶ���ʽ��
	if (!inFile) {
		std::cout << "error" << std::endl;
		return false;
	}
	while (inFile.read((char*)map.data, sizeof(double) * rows * cols)) { //һֱ�����ļ�����
		int readedBytes = inFile.gcount(); //���ղŶ��˶����ֽ�
		std::cout << readedBytes << std::endl;
	}
	inFile.close();

	out_map = map.clone();
	return true;
}


bool LookupTableFunction::saveBinMappingFloat(std::string mapping_file, cv::Mat out_map)
{
	std::ofstream outFile(mapping_file, std::ios::out | std::ios::binary);

	outFile.write((char*)out_map.data, sizeof(float) * out_map.rows * out_map.cols);
	outFile.close();
	return 0;
}


bool LookupTableFunction::saveBinMapping(std::string mapping_file, cv::Mat out_map)
{
	std::ofstream outFile(mapping_file, std::ios::out | std::ios::binary);

	outFile.write((char*)out_map.data, sizeof(double) * out_map.rows * out_map.cols);
	outFile.close();
	return 0;
}


bool LookupTableFunction::readBinMappingFloat(int rows, int cols, std::string mapping_file, cv::Mat& out_map)
{
	cv::Mat map(rows, cols, CV_32F, cv::Scalar(0));
	std::ifstream inFile(mapping_file, std::ios::in | std::ios::binary); //�����ƶ���ʽ��
	if (!inFile) {
		std::cout << "error" << std::endl;
		return false;
	}
	while (inFile.read((char*)map.data, sizeof(float) * rows * cols)) { //һֱ�����ļ�����
		int readedBytes = inFile.gcount(); //���ղŶ��˶����ֽ�
		//std::cout << readedBytes << std::endl;
	}
	inFile.close();

	out_map = map.clone();
	return true;
}

bool LookupTableFunction::readBinMapping(int rows, int cols, std::string mapping_file, cv::Mat& out_map)
{
	//QFile file(QString::fromStdString(mapping_file)); 

	//if (!file.open(QIODevice::ReadOnly))
	//{
	//	return false;
	//}
	//QDataStream in(&file);
	//cv::Mat m = cv::Mat(rows, cols, CV_64F);
	//for (int i = 0; i < rows; i++) {
	//	for (int j = 0; j < cols; j++) {
	//		in >> m.at<double>(i, j);
	//	}
	//}

	//out_map = m.clone();

	//return true;

	cv::Mat map(rows, cols, CV_64F, cv::Scalar(0));
	std::ifstream inFile(mapping_file, std::ios::in | std::ios::binary); //�����ƶ���ʽ��
	if (!inFile) {
		std::cout << "error" << std::endl;
		return false;
	}
	while (inFile.read((char*)map.data, sizeof(double) * rows * cols)) { //һֱ�����ļ�����
		int readedBytes = inFile.gcount(); //���ղŶ��˶����ֽ�
		//std::cout << readedBytes << std::endl;
	}
	inFile.close();

	out_map = map.clone();
	return true;
}

cv::Mat LookupTableFunction::readmapping(int rows, int cols, std::string mapping_file)
{

	cv::Mat m = cv::Mat(rows, cols, CV_64F);
	std::ifstream in(mapping_file);
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			in >> m.at<double>(i, j);
			//cout << m.at<double>(i, j) << endl;
		}
	}
	return m;

}

double LookupTableFunction::Bilinear_interpolation(double x, double y, cv::Mat& mapping)
{

	int x1 = floor(x);
	int y1 = floor(y);
	int x2 = x1 + 1;
	int y2 = y1 + 1;

	//row-y,col-x

	if (x1 == 1919) {
		double out = mapping.at<double>(y1, x1);
		return out;
	}
	else {
		double fq11 = mapping.at<double>(y1, x1);
		double fq21 = mapping.at<double>(y1, x2);
		double fq12 = mapping.at<double>(y2, x1);
		double fq22 = mapping.at<double>(y2, x2);

		double out = fq11 * (x2 - x) * (y2 - y) + fq21 * (x - x1) * (y2 - y) + fq12 * (x2 - x) * (y - y1) + fq22 * (x - x1) * (y - y1);

		return out;
	}


}

double LookupTableFunction::depth_per_point_6patterns_combine(double Xc, double Yc, double Xp, cv::Mat xL_rotate_x, cv::Mat xL_rotate_y, cv::Mat single_pattern_mapping, double b)
{


	double Xcr = Bilinear_interpolation(Xc, Yc, xL_rotate_x);
	double Ycr = Bilinear_interpolation(Xc, Yc, xL_rotate_y);
	double Xpr = Bilinear_interpolation(Xp, (Ycr + 1) * 2000, single_pattern_mapping);
	double delta_X = std::abs(Xcr - Xpr);
	double Z = b / delta_X;
	return Z;

}


bool LookupTableFunction::mat_float_to_double(cv::Mat org_mat, cv::Mat& dst_mat)
{
	if (!org_mat.data)
	{
		return false;
	}

	if (org_mat.type() != CV_32F)
	{
		return false;
	}

	int rows = org_mat.rows;
	int cols = org_mat.cols;

	cv::Mat dst(rows, cols, CV_64F, cv::Scalar(0));


	for (int r = 0; r < rows; r++)
	{
		float* ptr_o = org_mat.ptr<float>(r);
		double* ptr_d = dst.ptr<double>(r);
		for (int c = 0; c < cols; c++)
		{
			ptr_d[c] = (double)ptr_o[c];
		}

	}

	dst_mat = dst.clone();


	return true;
}

bool LookupTableFunction::mat_double_to_float(cv::Mat org_mat, cv::Mat& dst_mat)
{
	if (!org_mat.data)
	{
		return false;
	}

	if (org_mat.type() != CV_64F)
	{
		return false;
	}

	int rows = org_mat.rows;
	int cols = org_mat.cols;

	cv::Mat dst(rows, cols, CV_32F, cv::Scalar(0));


	for (int r = 0; r < rows; r++)
	{
		double* ptr_o = org_mat.ptr<double>(r);
		float* ptr_d = dst.ptr<float>(r);
		for (int c = 0; c < cols; c++)
		{
			ptr_d[c] = (float)ptr_o[c];
		}

	}

	dst_mat = dst.clone();


	return true;
}

bool LookupTableFunction::undistortedImage(cv::Mat distort_img, cv::Mat& undistort_img)
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

bool LookupTableFunction::rebuildData(cv::Mat unwrap_map_x, int group_num, cv::Mat& deep_map, cv::Mat& mask)
{

	if (!unwrap_map_x.data) {
		return false;
	}

	if (!single_pattern_mapping_.data || !xL_rotate_x_.data || !xL_rotate_y_.data)
	{
		std::cout << "Please read table first!";
		return false;
	}

	cv::Mat single_pattern_mapping = single_pattern_mapping_.clone();
	cv::Mat xL_rotate_y = xL_rotate_y_.clone();
	cv::Mat xL_rotate_x = xL_rotate_x_.clone();
	double b = value_b_;


	//��ʼ�����������DMD_Points�ľ���ֵ
	double phase_max = 2 * CV_PI * pow(2.0, group_num - 1);
	int nr = unwrap_map_x.rows;
	int nc = unwrap_map_x.cols;

	//��ʼ�����ͼ
	cv::Mat all_deep_map = cv::Mat(nr, nc, CV_64F, cv::Scalar(0));
	deep_map = all_deep_map.clone();


#pragma omp parallel for
	for (int Yc = 0; Yc < nr; Yc++) {
		double* ptr_x = unwrap_map_x.ptr<double>(Yc);
		double* ptr_d = deep_map.ptr<double>(Yc);
		uchar* ptr_m = mask.ptr<uchar>(Yc);

		for (int Xc = 0; Xc < nc; Xc++) {
			//����Xp��������
			double Xp = dlp_width_ * ptr_x[Xc] / phase_max;
			//��Ҫ��mask������������Ƿ���Ҫ
			if (ptr_m[Xc] == 255 && Xp > 0) {
				//������ȣ���Ҫ�õ�����ĵ�����꣬����λ�õ���Xp���꣬������������ֵ����ľ����Ӳ�����õ���b
				ptr_d[Xc] = depth_per_point_6patterns_combine(Xc, Yc, Xp, xL_rotate_x, xL_rotate_y, single_pattern_mapping, b);
			}
			else {
				ptr_d[Xc] = 0;
			}


		}
	}
	return true;
}


bool LookupTableFunction::generate_pointcloud(cv::Mat z, cv::Mat& mask, cv::Mat& map_xyz)
{

	if (!z.data)
	{
		return false;
	}

	unsigned char* row_pointer_z = (unsigned char*)z.data;
	double* pix_pointer_z;
	unsigned char* row_pointer_mask = (unsigned char*)mask.data;
	unsigned char* pix_pointer_mask;
	int width_step_float = z.step;
	int width_step_uchar = mask.step;
	std::vector<cv::Point3d> pcd;

	cv::Mat R1 = R_1_.clone();

	cv::Mat R1_t = R1.t();
	double R1_t1 = R1_t.at<double>(0, 0);
	double R1_t2 = R1_t.at<double>(0, 1);
	double R1_t3 = R1_t.at<double>(0, 2);
	double R1_t4 = R1_t.at<double>(1, 0);
	double R1_t5 = R1_t.at<double>(1, 1);
	double R1_t6 = R1_t.at<double>(1, 2);
	double R1_t7 = R1_t.at<double>(2, 0);
	double R1_t8 = R1_t.at<double>(2, 1);
	double R1_t9 = R1_t.at<double>(2, 2);


	cv::Mat xL_rotate_y = xL_rotate_y_.clone();
	cv::Mat xL_rotate_x = xL_rotate_x_.clone();

	int img_height = mask.rows;
	int img_width = mask.cols;

	cv::Mat xyz_mat(img_height, img_width, CV_64FC3, cv::Scalar(NULL, NULL, NULL));


	//#pragma omp parallel for
	for (int yc = 0; yc < img_height; yc++) {
		pix_pointer_z = (double*)(row_pointer_z + yc * width_step_float);
		pix_pointer_mask = row_pointer_mask + yc * width_step_uchar;
		cv::Vec3d* ptr_xyz = xyz_mat.ptr<cv::Vec3d>(yc);

		for (int xc = 0; xc < img_width; xc++) {

			cv::Point3d point;
			if (*pix_pointer_mask != 0) {
				double xcr = Bilinear_interpolation(xc, yc, xL_rotate_x);
				double ycr = Bilinear_interpolation(xc, yc, xL_rotate_y);

				point.x = (*pix_pointer_z) * xcr * R1_t1 + (*pix_pointer_z) * ycr * R1_t2 + (*pix_pointer_z) * R1_t3;
				point.y = (*pix_pointer_z) * xcr * R1_t4 + (*pix_pointer_z) * ycr * R1_t5 + (*pix_pointer_z) * R1_t6;
				point.z = (*pix_pointer_z) * xcr * R1_t7 + (*pix_pointer_z) * ycr * R1_t8 + (*pix_pointer_z) * R1_t9;


				if (point.z > min_low_z_ && point.z < max_max_z_)
				{
					ptr_xyz[xc][0] = point.x;
					ptr_xyz[xc][1] = point.y;
					ptr_xyz[xc][2] = point.z;
				}
				else
				{
					ptr_xyz[xc][0] = 0;
					ptr_xyz[xc][1] = 0;
					ptr_xyz[xc][2] = 0;
				}



				pcd.push_back(point);
			}
			pix_pointer_mask++;
			pix_pointer_z++;
		}
		//row_pointer_z += width_step_float;
		//row_pointer_mask += width_step_uchar;

	}

	//std::cout << "Point Cloud Numbers:" << pcd.size() << std::endl;

	map_xyz = xyz_mat.clone();


	return true;
}