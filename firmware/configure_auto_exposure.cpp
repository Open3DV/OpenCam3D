#include "configure_auto_exposure.h"

ConfigureAutoExposure::ConfigureAutoExposure()
{
 
}

ConfigureAutoExposure::~ConfigureAutoExposure()
{
} 

bool ConfigureAutoExposure::evaluateBrightnessParam(cv::Mat brightness_mat, cv::Mat mask, float& average_pixel, float& over_exposure_rate)
{
	if (!brightness_mat.data)
	{
		return false;
	}

	cv::Mat img = brightness_mat.clone();

	int nr = img.rows;
	int nc = img.cols;

	if (!mask.data)
	{
		cv::Mat gen_mask(nr, nc, CV_8U, cv::Scalar(255));
        int threshold_val = 0; 
        uchar *ptr_org_2 = img.ptr<uchar>(2);
        uchar *ptr_org_n2 = img.ptr<uchar>(nr - 2);

        for (int c = 0; c < nc; c++)
        {
            threshold_val += ptr_org_2[c];
            threshold_val += ptr_org_n2[c];
        }

        threshold_val /= 2 * nc;
        threshold_val += 20;

        for (int r = 0; r < nr; r++)
        {
            uchar *ptr_mask = gen_mask.ptr<uchar>(r);
            uchar *ptr_org = img.ptr<uchar>(r);

            for (int c = 0; c < nc; c++)
            {
                if (ptr_org[c] < threshold_val)
                {
                    ptr_mask[c] = 0;
                }
            }
        }

        mask = gen_mask.clone();
    }



	//设置提取直方图的相关变量
	cv::Mat hist;  //用于存放直方图计算结果
	const int channels[1] = { 0 };  //通道索引
	float inRanges[2] = { 0,256 };
	const float* ranges[1] = { inRanges };  //像素灰度值范围
	const int bins[1] = { 256 };  //直方图的维度，其实就是像素灰度值的最大值
	calcHist(&img, 1, channels, mask, hist, 1, bins, ranges);  //计算图像直方图
	//准备绘制直方图
	// int hist_w = 512;
	// int hist_h = 2000;
	// int width = 2;
	// cv::Mat histImage = cv::Mat::zeros(hist_h, hist_w, CV_8U);

	/// 将直方图归一化到范围 [ 0, histImage.rows ]
	//cv::normalize(hist, hist, 1, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());

	// for (int i = 1; i <= hist.rows; i++)
	// {
	// 	rectangle(histImage, cv::Point(width * (i - 1), hist_h - 1), cv::Point(width * i - 1, hist_h - cvRound(hist.at<float>(i - 1) / 15)), cv::Scalar(255), -1);
	// } 

	//过曝点
	/********************************************************************************************************************************/
	cv::Mat over_exposure_mask;
	cv::threshold(img, over_exposure_mask, 254, 255, cv::THRESH_BINARY);

	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::Mat erode_mask;

	cv::erode(over_exposure_mask, erode_mask, element);

	int over_exposure_points = 0;
	int all_points = 0;
	float sum_pixels = 0;

	for (int r = 0; r < nr; r++)
	{
		uchar* ptr_erode = erode_mask.ptr<uchar>(r);
		uchar* ptr_mask = mask.ptr<uchar>(r);
		uchar* ptr_org = img.ptr<uchar>(r);

		for (int c = 0; c < nc; c++)
		{
			if (ptr_erode[c] == 255)
			{
				over_exposure_points++;
			}

			if (ptr_mask[c] > 0)
			{
				all_points++;
				sum_pixels += ptr_org[c];
			}
		}

	}


	int light_points = 0;

	for (int val = 128; val < 256; val++)
	{
		light_points += hist.at<float>(val);
	}

	int good_points = light_points - over_exposure_points;

	float good_rate = 100.0 * good_points / all_points;
	LOG(INFO) << "good_rate: " << good_rate;

	over_exposure_rate = 100.0 * over_exposure_points / all_points;
	LOG(INFO) << "over_exposure_rate: " << over_exposure_rate;

	average_pixel = sum_pixels / all_points;
	LOG(INFO) << "average_pixel: " << average_pixel;

	return true;
}
