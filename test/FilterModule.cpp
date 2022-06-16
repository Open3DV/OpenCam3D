#include "FilterModule.h"
#include <iostream> 
#include <numeric>


FilterModule::FilterModule()
{
}


FilterModule::~FilterModule()
{
}


bool FilterModule::statisticOutlierRemoval(cv::Mat& point_cloud_map, int num_neighbors, double threshold_radio)
{
	if (!point_cloud_map.data)
		return false;

	int nr = point_cloud_map.rows;
	int nc = point_cloud_map.cols;

	cv::Mat mean_map(nr, nc, CV_64FC1, cv::Scalar(0));

	std::vector<double> distance_list;
	for (int r = 0; r < nr; r++)
	{
		cv::Vec3d* ptr_p = point_cloud_map.ptr<cv::Vec3d>(r);
		double* ptr_mean = mean_map.ptr<double>(r);

		for (int c = 0; c < nc; c++)
		{
			if (ptr_p[c][2] > 0)
			{
				getDistanctList(point_cloud_map, cv::Point(c, r), num_neighbors, ptr_mean[c], distance_list);
			}
		}
	}

	double sum = 0;
	for (int i = 0; i < distance_list.size(); i++)
	{
		sum += distance_list[i];
	}

	double mean = sum / distance_list.size();

	double standard_dev = 0;
	double standard_sum = 0;

	for (int i = 0; i < distance_list.size(); i++)
	{
		standard_sum += std::pow(distance_list[i] - mean, 2);
	}
	standard_dev = std::sqrt(standard_sum / distance_list.size());

	std::cout << "mean: " << mean << std::endl;
	std::cout << "standard_dev: " << standard_dev << std::endl;

	for (int r = 0; r < nr; r++)
	{
		cv::Vec3d* ptr_p = point_cloud_map.ptr<cv::Vec3d>(r);
		double* ptr_mean = mean_map.ptr<double>(r);

		for (int c = 0; c < nc; c++)
		{
			if (ptr_p[c][2] > 0)
			{
				if (ptr_mean[c] > (mean + 0.5 * standard_dev) || 0 == ptr_mean[c])
				{
					ptr_p[c][0] = 0;
					ptr_p[c][1] = 0;
					ptr_p[c][2] = 0;
				}


			}
		}
	}


	return true;
}


bool FilterModule::computeNormalDistribution(cv::Mat& point_cloud_map, int num_neighbors, double& mean, double& sd)
{
	if (!point_cloud_map.data)
		return false;

	int nr = point_cloud_map.rows;
	int nc = point_cloud_map.cols;


	return true;
}

bool FilterModule::RadiusOutlierRemoval(cv::Mat& point_cloud_map, cv::Mat& mask, double dot_spacing, double radius, int points_num)
{
	if (!point_cloud_map.data)
		return false;

	int nr = point_cloud_map.rows;
	int nc = point_cloud_map.cols;

	//cv::Mat result_mask(nr, nc, CV_8U, cv::Scalar(0));

	//int rest_num = findNearPointsnum(point_cloud_map, mask, cv::Point(808, 630), radius);
	//#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{
		cv::Vec3d* ptr_p = point_cloud_map.ptr<cv::Vec3d>(r);
		uchar* ptr_m = mask.ptr<uchar>(r);
		//uchar* ptr_r = result_mask.ptr<uchar>(r);

		for (int c = 0; c < nc; c++)
		{
			if (ptr_m[c] > 0)
			{
				int num = findNearPointsnum(point_cloud_map, mask, cv::Point(c, r), dot_spacing, radius);

				if (num > points_num)
				{
					ptr_m[c] = 255;

				}
				else
				{
					ptr_m[c] = 0;

					ptr_p[c][0] = 0;
					ptr_p[c][1] = 0;
					ptr_p[c][2] = 0;
				}
			}
		}
	}

	return true;
}


/***************************************************************************************************************/



bool FilterModule::getDistanctList(cv::Mat point_cloud_map, cv::Point pos, int num_neighbors, double& mean, std::vector<double>& dist_list)
{
	if (!point_cloud_map.data)
		return false;

	int nr = point_cloud_map.rows;
	int nc = point_cloud_map.cols;

	int w = 2;

	int s_r = pos.y - w;
	int s_c = pos.x - w;

	int e_r = pos.y + w;
	int e_c = pos.x + w;

	if (s_r < 0)
	{
		s_r = 0;
	}
	if (s_c < 0)
	{
		s_c = 0;
	}

	if (e_r >= nr)
	{
		e_r = nr - 1;
	}

	if (e_c >= nc)
	{
		e_c = nc - 1;
	}

	std::vector<cv::Point3d> points_list;
	cv::Point3d target_point;
	//cv::Point3d sum_point;
	int num = 0;
	for (int r = s_r; r <= e_r; r++)
	{
		cv::Vec3d* ptr_p = point_cloud_map.ptr<cv::Vec3d>(r);

		for (int c = s_c; c <= e_c; c++)
		{
			if (pos.y == r && pos.x == c)
			{
				target_point.x = ptr_p[c][0];
				target_point.y = ptr_p[c][1];
				target_point.z = ptr_p[c][2];
				continue;
			}
			if (ptr_p[c][2] > 0)
			{
				cv::Point3d point(ptr_p[c][0], ptr_p[c][1], ptr_p[c][2]);
				points_list.push_back(point);

			}
		}
	}

	if (points_list.size() < num_neighbors)
	{
		return false;
	}


	std::vector<double> d_list;
	for (int i = 0; i < points_list.size(); i++)
	{
		double dist = computePointsDistance(target_point, points_list[i]);
		d_list.push_back(dist);
	}
	std::sort(d_list.begin(), d_list.end(), std::less<double>());



	double sum = 0;
	for (int i = 0; i < 6; i++)
	{

		sum += d_list[i];
	}

	mean = sum / num_neighbors;

	if (mean > 600)
	{
		return false;
	}

	dist_list.push_back(mean);

	return true;
}



bool FilterModule::statisticRegion(cv::Mat& point_cloud_map, cv::Point pos, int num_neighbors, double threshold_radio)
{
	if (!point_cloud_map.data)
		return false;

	int nr = point_cloud_map.rows;
	int nc = point_cloud_map.cols;

	int w = num_neighbors;

	int s_r = pos.y - w;
	int s_c = pos.x - w;

	int e_r = pos.y + w;
	int e_c = pos.x + w;

	if (s_r < 0)
	{
		s_r = 0;
	}
	if (s_c < 0)
	{
		s_c = 0;
	}

	if (e_r >= nr)
	{
		e_r = nr - 1;
	}

	if (e_c >= nc)
	{
		e_c = nc - 1;
	}

	std::vector<cv::Point3d> points_list;
	cv::Point3d sum_point;
	int num = 0;
	for (int r = s_r; r <= e_r; r++)
	{
		cv::Vec3d* ptr_p = point_cloud_map.ptr<cv::Vec3d>(r);

		for (int c = s_c; c <= e_c; c++)
		{
			if (ptr_p[c][2] > 0)
			{
				cv::Point3d point(ptr_p[c][0], ptr_p[c][1], ptr_p[c][2]);
				sum_point += point;

				points_list.push_back(point);

			}
		}
	}

	cv::Point3d center_point(sum_point.x / points_list.size(), sum_point.y / points_list.size(), sum_point.z / points_list.size());

	std::vector<double> dist_list;
	double sum = 0;
	double mean = 0;
	for (int i = 0; i < points_list.size(); i++)
	{
		double dist = computePointsDistance(center_point, points_list[i]);
		sum += dist;
		dist_list.push_back(dist);
	}

	mean = sum / points_list.size();

	double sd = 0;

	for (int i = 0; i < points_list.size(); i++)
	{
		sd += std::pow(dist_list[i] - mean, 2);
	}
	sd /= points_list.size();

	sd = std::sqrt(sd);

	cv::Point3d target_point(point_cloud_map.ptr<cv::Vec3d>(pos.y)[pos.x][0], point_cloud_map.ptr<cv::Vec3d>(pos.y)[pos.x][1], point_cloud_map.ptr<cv::Vec3d>(pos.y)[pos.x][2]);

	double dist = computePointsDistance(center_point, target_point);

	if (dist < mean + threshold_radio * sd)
	{
		return true;
	}
	else
	{
		return false;
	}
}

//相机像素为5.4um、焦距12mm。dot_spacing = 5.4*distance/12000 mm，典型值0.54mm（1200） 
int FilterModule::findNearPointsnum(cv::Mat& point_cloud_map, cv::Mat& mask, cv::Point pos, double dot_spacing, double radius)
{
	if (!point_cloud_map.data)
		return -1;

	int nr = point_cloud_map.rows;
	int nc = point_cloud_map.cols;

	int w = 1 + radius / dot_spacing;

	int s_r = pos.y - w;
	int s_c = pos.x - w;

	int e_r = pos.y + w;
	int e_c = pos.x + w;

	if (s_r < 0)
	{
		s_r = 0;
	}
	if (s_c < 0)
	{
		s_c = 0;
	}

	if (e_r >= nr)
	{
		e_r = nr - 1;
	}

	if (e_c >= nc)
	{
		e_c = nc - 1;
	}



	//cv::Mat mask_color;

	//cv::cvtColor(mask, mask_color, cv::COLOR_GRAY2BGR);  
	//cv::rectangle(mask_color, cv::Point(s_c, s_r), cv::Point(e_c, e_r), cv::Scalar(255,0,255), 3, 1, 0);

	int num = 0;

	cv::Vec3d p_0 = point_cloud_map.at<cv::Vec3d>(pos.y, pos.x);

	for (int r = s_r; r <= e_r; r++)
	{
		cv::Vec3d* ptr_p = point_cloud_map.ptr<cv::Vec3d>(r);
		uchar* ptr_m = mask.ptr<uchar>(r);

		for (int c = s_c; c <= e_c; c++)
		{
			if (ptr_m[c] > 0)
			{
				double dist = computePointsDistance(p_0, ptr_p[c]);

				//std::cout << c << " , " << r << " : " << dist << " ; "<<std::endl;

				if (radius > dist)
				{
					num++;
				}
			}
		}
	}

	//std::cout << std::endl;

	return num;


	//cv::Vec3d p0 = point_cloud_map.at<cv::Vec3d>(pos.y, pos.x);


	//cv::Vec3d p1 = point_cloud_map.at<cv::Vec3d>(pos.y - 1, pos.x - 1);
	//cv::Vec3d p2 = point_cloud_map.at<cv::Vec3d>(pos.y - 1, pos.x + 0);
	//cv::Vec3d p3 = point_cloud_map.at<cv::Vec3d>(pos.y - 1, pos.x + 1);
	//cv::Vec3d p4 = point_cloud_map.at<cv::Vec3d>(pos.y, pos.x + 1);
	//cv::Vec3d p5 = point_cloud_map.at<cv::Vec3d>(pos.y + 1, pos.x + 1);
	//cv::Vec3d p6 = point_cloud_map.at<cv::Vec3d>(pos.y + 1, pos.x);
	//cv::Vec3d p7 = point_cloud_map.at<cv::Vec3d>(pos.y + 1, pos.x - 1);
	//cv::Vec3d p8 = point_cloud_map.at<cv::Vec3d>(pos.y, pos.x - 1);

	//double d1 = computePointsDistance(p0, p1);
	//double d2 = computePointsDistance(p0, p2);
	//double d3 = computePointsDistance(p0, p3);
	//double d4 = computePointsDistance(p0, p4);
	//double d5 = computePointsDistance(p0, p5);
	//double d6 = computePointsDistance(p0, p6);
	//double d7 = computePointsDistance(p0, p7);
	//double d8 = computePointsDistance(p0, p8);

	//std::cout << d1 << " , " << d2 << " , " << d3 << " , " << d4 << std::endl;

	//return -1;
}


double FilterModule::computePointsDistance(cv::Vec3d p0, cv::Vec3d p1)
{
	cv::Vec3d p_d = p1 - p0;


	//std::cout << p_d[0] << " , " << p_d[1] << " , " << p_d[2] << std::endl;

	double val = sqrt(p_d[0] * p_d[0] + p_d[1] * p_d[1] + p_d[2] * p_d[2]);


	//std::cout << val << std::endl;

	return val;
}

double FilterModule::computePointsDistance(cv::Point3f p0, cv::Point3f p1)
{
	cv::Point3f p_d = p1 - p0;

	return sqrt(p_d.x * p_d.x + p_d.y * p_d.y + p_d.z * p_d.z);
}

double FilterModule::computePointsZDistance(cv::Point3d p0, cv::Point3d p1)
{
	cv::Point3d p_d = p1 - p0;

	return std::abs(p_d.z);
}


double FilterModule::computePointsDistance(cv::Point3d p0, cv::Point3d p1)
{
	cv::Point3d p_d = p1 - p0;

	return sqrt(p_d.x * p_d.x + p_d.y * p_d.y + p_d.z * p_d.z);
}


double FilterModule::computePointsDistance(cv::Point2f p0, cv::Point2f p1)
{
	cv::Point2f p_d = p1 - p0;

	return sqrt(p_d.x * p_d.x + p_d.y * p_d.y);
}














/****************************************************************************************************************/