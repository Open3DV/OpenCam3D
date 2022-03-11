#include "AnalyseError.h"
#include "iostream"

AnalyseError::AnalyseError()
{

}



AnalyseError::~AnalyseError()
{

}



/**
* @brief expandEdge 扩展边界函数
* @param img:输入图像，单通道二值图，深度为8
* @param edge  边界数组，存放4条边界值
* @param edgeID 当前边界号
* @return 布尔值 确定当前边界是否可以扩展
*/

bool AnalyseError::expandEdge(const cv::Mat& img, int edge[], const int edgeID)
{
	//[1] --初始化参数
	int nc = img.cols;
	int nr = img.rows;
	switch (edgeID) {
	case 0:
		if (edge[0] > nr)
			return false;
		for (int i = edge[3]; i <= edge[1]; ++i)
		{
			if (img.at<uchar>(edge[0], i) == 255)//遇见255像素表明碰到边缘线
				return false;
		}
		edge[0]++;
		return true;
		break;
	case 1:
		if (edge[1] > nc)
			return false;
		for (int i = edge[2]; i <= edge[0]; ++i)
		{
			if (img.at<uchar>(i, edge[1]) == 255)//遇见255像素表明碰到边缘线
				return false;
		}
		edge[1]++;
		return true;
		break;
	case 2:
		if (edge[2] < 0)
			return false;
		for (int i = edge[3]; i <= edge[1]; ++i)
		{
			if (img.at<uchar>(edge[2], i) == 255)//遇见255像素表明碰到边缘线
				return false;
		}
		edge[2]--;
		return true;
		break;
	case 3:
		if (edge[3] < 0)
			return false;
		for (int i = edge[2]; i <= edge[0]; ++i)
		{
			if (img.at<uchar>(i, edge[3]) == 255)//遇见255像素表明碰到边缘线
				return false;
		}
		edge[3]--;
		return true;
		break;
	default:
		return false;
		break;
	}

}
 

/**
* @brief 求取连通区域内接矩
* @param img:输入图像，单通道二值图，深度为8
* @param center:最小外接矩的中心
* @return  最大内接矩形
* 基于中心扩展算法
*/
cv::Rect AnalyseError::InSquare(cv::Mat& img, const cv::Point center)
{
	// --[1]参数检测
	if (img.empty() || img.channels() > 1 || img.depth() > 8)
		return cv::Rect();
	// --[2] 初始化变量
	int edge[4];
	edge[0] = center.y + 1;//top
	edge[1] = center.x + 1;//right
	edge[2] = center.y - 1;//bottom
	edge[3] = center.x - 1;//left
						   //[2]
						   // --[3]边界扩展(中心扩散法)

	bool EXPAND[4] = { 1,1,1,1 };//扩展标记位
	int n = 0;
	while (EXPAND[0] || EXPAND[1] || EXPAND[2] || EXPAND[3])
	{
		int edgeID = n % 4;
		EXPAND[edgeID] = expandEdge(img, edge, edgeID);
		n++;
	}
	//[3]
	//qDebug() << edge[0] << edge[1] << edge[2] << edge[3];
	cv::Point tl = cv::Point(edge[3], edge[0]);
	cv::Point br = cv::Point(edge[1], edge[2]);
	return cv::Rect(tl, br);
}

 

double AnalyseError::computeError(cv::Mat err_map)
{
	if (!err_map.data)
	{
		return -1;
	}
	double err = -1;

	int nr = err_map.rows;
	int nc = err_map.cols;

	cv::Mat binary_map(err_map.size(), CV_8U, cv::Scalar(0));

	cv::Mat median_map = err_map.clone();

	median_map.convertTo(median_map, CV_32F);

	cv::medianBlur(median_map, median_map, 3);

	for (int r = 0; r < nr; r++)
	{
		uchar* ptr_b = binary_map.ptr<uchar>(r);
		double* ptr_dr = err_map.ptr<double>(r);

		for (int c = 0; c < nc; c++)
		{

			if (ptr_dr[c] > 0)
			{
				ptr_b[c] = 255;
			}
			 

		}
	}

	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(binary_map, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

	int max_size = 0;
	std::vector<cv::Point> max_contour;

	for (int i = 0; i < contours.size(); i++)
	{
		if (contours[i].size() > max_size)
		{
			max_size = contours[i].size();
			max_contour = contours[i];
		}
	}

	cv::Mat region_map(nr, nc, CV_8U, cv::Scalar(0));
	std::vector<std::vector<cv::Point>> max_contours;
	max_contours.push_back(max_contour);
	cv::drawContours(region_map, max_contours, -1, cv::Scalar(255), -1);

	cv::Rect bounding_rect = cv::boundingRect(max_contour);
	
	bounding_rect.width *= 0.9;
	bounding_rect.height *= 0.8;
	bounding_rect.x += 0.05* bounding_rect.width;
	bounding_rect.y += 0.1 * bounding_rect.height;

  
	cv::rectangle(binary_map, bounding_rect, cv::Scalar(100), 1, 8);
 


	cv::Mat roi_map(err_map.size(), CV_8U, cv::Scalar(0));
	cv::rectangle(roi_map, bounding_rect, cv::Scalar(255), -1, 8);

	int count_use = 0;
	double sum_err = 0;

	for (int r = 0; r < nr; r++)
	{
		uchar* ptr_b = roi_map.ptr<uchar>(r);
		float* ptr_dr = median_map.ptr<float>(r);
		//double* ptr_dr = err_map.ptr<double>(r);

		for (int c = 0; c < nc; c++)
		{

			if (ptr_b[c] > 0)
			{
				if (ptr_dr[c] > 0 && ptr_dr[c] < 10)
				{
					sum_err += ptr_dr[c];
					count_use++;

				}
			}


		}
	}

	err = sum_err / count_use;

	return err;
}



