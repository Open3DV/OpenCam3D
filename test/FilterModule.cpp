#include "FilterModule.h"
#include <iostream>


FilterModule::FilterModule()
{
}


FilterModule::~FilterModule()
{
}


bool FilterModule::RadiusOutlierRemoval(cv::Mat &point_cloud_map, cv::Mat &mask, double radius, int points_num)
{
	if (!point_cloud_map.data)
		return false;

	int nr = point_cloud_map.rows;
	int nc = point_cloud_map.cols;

	//cv::Mat result_mask(nr, nc, CV_8U, cv::Scalar(0));

	//int rest_num = findNearPointsnum(point_cloud_map, mask, cv::Point(808, 630), radius);
	//#pragma omp parallel for
	for(int r= 0;r< nr;r++)
	{
		cv::Vec3d* ptr_p = point_cloud_map.ptr<cv::Vec3d>(r);
		uchar* ptr_m = mask.ptr<uchar>(r);
		//uchar* ptr_r = result_mask.ptr<uchar>(r);

		for(int c = 0;c< nc;c++)
		{
			if(ptr_m[c] > 0)
			{
				int num= findNearPointsnum(point_cloud_map, mask, cv::Point(c, r), radius); 

				if(num> points_num)
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
//œ‡¡⁄µ„æ‡¿Î∂®0.4mm
int FilterModule::findNearPointsnum(cv::Mat &point_cloud_map, cv::Mat &mask, cv::Point pos, double radius)
{
	if (!point_cloud_map.data)
		return -1;

	int nr = point_cloud_map.rows;
	int nc = point_cloud_map.cols;

	int w = radius *2.5;

	int s_r = pos.y - w;
	int s_c = pos.x - w;

	int e_r = pos.y + w;
	int e_c = pos.x + w;

	if(s_r< 0)
	{
		s_r = 0;
	}
	if(s_c < 0)
	{
		s_c = 0;
	}

	if(e_r>= nr)
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
 
	for(int r= s_r;r<= e_r;r++)
	{
		cv::Vec3d* ptr_p = point_cloud_map.ptr<cv::Vec3d>(r);
		uchar* ptr_m = mask.ptr<uchar>(r);

		for(int c= s_c;c<= e_c;c++)
		{
			if(ptr_m[c] > 0)
			{
				double dist = computePointsDistance(p_0, ptr_p[c]);

				//std::cout << c << " , " << r << " : " << dist << " ; "<<std::endl;

				if(radius> dist)
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

	double val = sqrt(p_d[0]*p_d[0] + p_d[1] *p_d[1] + p_d[2] * p_d[2]);


	//std::cout << val << std::endl;

	return val;
}

double FilterModule::computePointsDistance(cv::Point3f p0, cv::Point3f p1)
{
	cv::Point3f p_d = p1 - p0;

	return sqrt(p_d.x*p_d.x + p_d.y*p_d.y + p_d.z*p_d.z);
}

double FilterModule::computePointsDistance(cv::Point3d p0, cv::Point3d p1)
{
	cv::Point3d p_d = p1 - p0; 

	return sqrt(p_d.x*p_d.x + p_d.y*p_d.y + p_d.z*p_d.z);
}


double FilterModule::computePointsDistance(cv::Point2f p0, cv::Point2f p1)
{
	cv::Point2f p_d = p1 - p0;

	return sqrt(p_d.x*p_d.x + p_d.y*p_d.y);
}














/****************************************************************************************************************/