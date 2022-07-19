#include "encode.h"
#include "iostream" 

Encode::Encode()
{
}


Encode::~Encode()
{
}


bool Encode::sixStepPhaseShift(std::vector<cv::Mat> patterns, cv::Mat& wrap_map, cv::Mat& mask, cv::Mat& confidence)
{
	if (6 != patterns.size())
	{
		return false;
	}

	cv::Mat img0 = patterns[0 + 3];
	cv::Mat img1 = patterns[1 + 3];
	cv::Mat img2 = patterns[2 + 3];
	cv::Mat img3 = patterns[3 - 3];
	cv::Mat img4 = patterns[4 - 3];
	cv::Mat img5 = patterns[5 - 3];


	cv::Mat result(img1.rows, img1.cols, CV_32F, cv::Scalar(-1));
	cv::Mat confidence_map(img1.rows, img1.cols, CV_32F, cv::Scalar(0));

	if (!mask.data)
	{
		mask = cv::Mat(img1.rows, img1.cols, CV_8U, cv::Scalar(255));
	}

	int nl = img1.rows;
	int nc = img1.cols * img1.channels();


	if (img0.isContinuous())
	{
		if (img1.isContinuous())
		{
			if (img2.isContinuous())
			{
				if (img3.isContinuous())
				{
					if (img4.isContinuous())
					{
						if (img5.isContinuous())
						{

							if (mask.isContinuous())
							{
								nc = nc * nl;
								nl = 1;
							}

						}
					}
				}
			}

		}
	}

	if (CV_16U == img0.type())
	{

#pragma omp parallel for
		for (int r = 0; r < nl; r++)
		{

			ushort* ptr0 = img0.ptr<ushort>(r);
			ushort* ptr1 = img1.ptr<ushort>(r);
			ushort* ptr2 = img2.ptr<ushort>(r);
			ushort* ptr3 = img3.ptr<ushort>(r);
			ushort* ptr4 = img4.ptr<ushort>(r);
			ushort* ptr5 = img5.ptr<ushort>(r);
			uchar* ptr_m = mask.ptr<uchar>(r);
			float* ptr_con = confidence_map.ptr<float>(r);

			float* optr = result.ptr<float>(r);
			for (int c = 0; c < nc; c++)
			{
				int exposure_num = 0;
				if (ptr_m[c])
				{
					//float a = ptr4[j] - ptr2[j];
					//float b = ptr1[j] - ptr3[j];
					if (255 == ptr0[c])
					{
						exposure_num++;
					}

					if (255 == ptr1[c])
					{
						exposure_num++;
					}
					if (255 == ptr2[c])
					{
						exposure_num++;
					}
					if (255 == ptr3[c])
					{
						exposure_num++;
					}
					if (255 == ptr4[c])
					{
						exposure_num++;
					}

					if (255 == ptr5[c])
					{
						exposure_num++;
					}


					float b = ptr0[c] * std::sin(0 * CV_2PI / 6.0) + ptr1[c] * std::sin(1 * CV_2PI / 6.0) + ptr2[c] * std::sin(2 * CV_2PI / 6.0)
						+ ptr3[c] * std::sin(3 * CV_2PI / 6.0) + ptr4[c] * std::sin(4 * CV_2PI / 6.0) + ptr5[c] * std::sin(5 * CV_2PI / 6.0);


					float a = ptr0[c] * std::cos(0 * CV_2PI / 6.0) + ptr1[c] * std::cos(1 * CV_2PI / 6.0) + ptr2[c] * std::cos(2 * CV_2PI / 6.0)
						+ ptr3[c] * std::cos(3 * CV_2PI / 6.0) + ptr4[c] * std::cos(4 * CV_2PI / 6.0) + ptr5[c] * std::cos(5 * CV_2PI / 6.0);

					float r = std::sqrt(a * a + b * b);

					//if (r > 255)
					//{
					//	r = 255;
					//}

					ptr_con[c] = r;

					/***********************************************************************/

					if (exposure_num > 3)
					{
						ptr_m[c] = 0;
						ptr_con[c] = 0;
						optr[c] = -1;
					}
					else
					{
						optr[c] = CV_PI + std::atan2(a, b);
					}

				}
			}
		}


		/*****************************************************************************************************************************/

		confidence = confidence_map.clone();

		wrap_map = result.clone();

		return true;
	}
	else if (CV_8U == img0.type())
	{

#pragma omp parallel for
		for (int r = 0; r < nl; r++)
		{

			uchar* ptr0 = img0.ptr<uchar>(r);
			uchar* ptr1 = img1.ptr<uchar>(r);
			uchar* ptr2 = img2.ptr<uchar>(r);
			uchar* ptr3 = img3.ptr<uchar>(r);
			uchar* ptr4 = img4.ptr<uchar>(r);
			uchar* ptr5 = img5.ptr<uchar>(r);
			uchar* ptr_m = mask.ptr<uchar>(r);
			float* ptr_con = confidence_map.ptr<float>(r);

			float* optr = result.ptr<float>(r);
			for (int c = 0; c < nc; c++)
			{
				int exposure_num = 0;
				if (ptr_m[c])
				{
					//float a = ptr4[j] - ptr2[j];
					//float b = ptr1[j] - ptr3[j];
					if (255 == ptr0[c])
					{
						exposure_num++;
					}

					if (255 == ptr1[c])
					{
						exposure_num++;
					}
					if (255 == ptr2[c])
					{
						exposure_num++;
					}
					if (255 == ptr3[c])
					{
						exposure_num++;
					}
					if (255 == ptr4[c])
					{
						exposure_num++;
					}

					if (255 == ptr5[c])
					{
						exposure_num++;
					}


					float b = ptr0[c] * std::sin(0 * CV_2PI / 6.0) + ptr1[c] * std::sin(1 * CV_2PI / 6.0) + ptr2[c] * std::sin(2 * CV_2PI / 6.0)
						+ ptr3[c] * std::sin(3 * CV_2PI / 6.0) + ptr4[c] * std::sin(4 * CV_2PI / 6.0) + ptr5[c] * std::sin(5 * CV_2PI / 6.0);


					float a = ptr0[c] * std::cos(0 * CV_2PI / 6.0) + ptr1[c] * std::cos(1 * CV_2PI / 6.0) + ptr2[c] * std::cos(2 * CV_2PI / 6.0)
						+ ptr3[c] * std::cos(3 * CV_2PI / 6.0) + ptr4[c] * std::cos(4 * CV_2PI / 6.0) + ptr5[c] * std::cos(5 * CV_2PI / 6.0);

					float r = std::sqrt(a * a + b * b);

					//if (r > 255)
					//{
					//	r = 255;
					//}

					ptr_con[c] = r;

					/***********************************************************************/

					if (exposure_num > 3)
					{
						ptr_m[c] = 0;
						ptr_con[c] = 0;
						optr[c] = -1;
					}
					else
					{
						optr[c] = CV_PI + std::atan2(a, b);
					}

				}
			}
		}


		/*****************************************************************************************************************************/

		confidence = confidence_map.clone();

		wrap_map = result.clone();

		return true;
	}

	return false;
}

bool Encode::fourStepPhaseShift(std::vector<cv::Mat> patterns, cv::Mat& wrap_map, cv::Mat& mask, cv::Mat& confidence)
{
	if (4 != patterns.size())
	{
		return false;
	}

	cv::Mat img1 = patterns[0];
	cv::Mat img2 = patterns[1];
	cv::Mat img3 = patterns[2];
	cv::Mat img4 = patterns[3];


	cv::Mat result(img1.rows, img1.cols, CV_32F, cv::Scalar(-1));
	cv::Mat confidence_map(img1.rows, img1.cols, CV_32F, cv::Scalar(0));

	if (!mask.data)
	{
		mask = cv::Mat(img1.rows, img1.cols, CV_8U, cv::Scalar(255));
	}

	int nl = img1.rows;
	int nc = img1.cols * img1.channels();

	if (img1.isContinuous())
	{
		if (img2.isContinuous())
		{
			if (img3.isContinuous())
			{
				if (img4.isContinuous())
				{
					if (mask.isContinuous())
					{
						nc = nc * nl;
						nl = 1;
					}
				}
			}
		}

	}


	if (CV_16U == img1.type())
	{

#pragma omp parallel for
		for (int i = 0; i < nl; i++)
		{
			ushort* ptr1 = img1.ptr<ushort>(i);
			ushort* ptr2 = img2.ptr<ushort>(i);
			ushort* ptr3 = img3.ptr<ushort>(i);
			ushort* ptr4 = img4.ptr<ushort>(i);
			uchar* ptr_m = mask.ptr<uchar>(i);
			float* ptr_con = confidence_map.ptr<float>(i);

			float* optr = result.ptr<float>(i);
			for (int j = 0; j < nc; j++)
			{
				int exposure_num = 0;
				if (ptr_m[j] == 255)
				{
					if (255 == ptr1[j])
					{
						exposure_num++;
					}
					if (255 == ptr2[j])
					{
						exposure_num++;
					}
					if (255 == ptr3[j])
					{
						exposure_num++;
					}
					if (255 == ptr4[j])
					{
						exposure_num++;
					}

					float a = ptr4[j] - ptr2[j];
					float b = ptr1[j] - ptr3[j];

					float r = std::sqrt(a * a + b * b) + 0.5;

					//if(r> 255)
					//{
					//	r = 255;
					//}

					ptr_con[j] = r;

					/***********************************************************************/

					if (exposure_num > 1)
					{
						ptr_m[j] = 0;
						ptr_con[j] = 0;
						optr[j] = -1;
					}
					else
					{
						optr[j] = CV_PI + std::atan2(a, b);
					}

				}
			}
		}


		/*****************************************************************************************************************************/

		confidence = confidence_map.clone();

		wrap_map = result.clone();

		return true;

	}
	else if (CV_8U == img1.type())
	{

#pragma omp parallel for
		for (int i = 0; i < nl; i++)
		{
			uchar* ptr1 = img1.ptr<uchar>(i);
			uchar* ptr2 = img2.ptr<uchar>(i);
			uchar* ptr3 = img3.ptr<uchar>(i);
			uchar* ptr4 = img4.ptr<uchar>(i);
			uchar* ptr_m = mask.ptr<uchar>(i);
			float* ptr_con = confidence_map.ptr<float>(i);

			float* optr = result.ptr<float>(i);
			for (int j = 0; j < nc; j++)
			{
				int exposure_num = 0;
				if (ptr_m[j] == 255)
				{
					if (255 == ptr1[j])
					{
						exposure_num++;
					}
					if (255 == ptr2[j])
					{
						exposure_num++;
					}
					if (255 == ptr3[j])
					{
						exposure_num++;
					}
					if (255 == ptr4[j])
					{
						exposure_num++;
					}

					float a = ptr4[j] - ptr2[j];
					float b = ptr1[j] - ptr3[j];

					float r = std::sqrt(a * a + b * b) + 0.5;

					//if(r> 255)
					//{
					//	r = 255;
					//}

					ptr_con[j] = r;

					/***********************************************************************/

					if (exposure_num > 1)
					{
						ptr_m[j] = 0;
						ptr_con[j] = 0;
						optr[j] = -1;
					}
					else
					{
						optr[j] = CV_PI + std::atan2(a, b);
					}

				}
			}
		}


		/*****************************************************************************************************************************/

		confidence = confidence_map.clone();
		wrap_map = result.clone();

		return true;
	}



	return false;
}

bool Encode::computePhaseBaseFourStep(std::vector<cv::Mat> patterns, std::vector<cv::Mat>& wrap_list, std::vector<cv::Mat>& mask_list, std::vector<cv::Mat>& confidence_list)
{

	std::vector<cv::Mat> wrap_img_list;
	std::vector<cv::Mat> mask_map_list;
	std::vector<cv::Mat> confidence_map_list;
	std::vector<int> number_list;



#pragma omp parallel for
	for (int i = 0; i < patterns.size() - 1; i += 4)
	{
		cv::Mat wrap_img;
		cv::Mat confidence;
		cv::Mat mask;

		std::vector<cv::Mat> phase_list(patterns.begin() + i, patterns.begin() + i + 4);
		fourStepPhaseShift(phase_list, wrap_img, mask, confidence);


#pragma omp critical
		{
			number_list.push_back(i / 4);
			wrap_img_list.push_back(wrap_img);
			mask_map_list.push_back(mask);
			confidence_map_list.push_back(confidence);
		}
	}


	std::vector<cv::Mat> sort_img_list;
	sort_img_list.resize(wrap_img_list.size());

	std::vector<cv::Mat> sort_mask_list;
	sort_mask_list.resize(mask_map_list.size());

	std::vector<cv::Mat> sort_confidencce_list;
	sort_confidencce_list.resize(confidence_map_list.size());

	for (int i = 0; i < wrap_img_list.size(); i++)
	{
		sort_img_list[number_list[i]] = wrap_img_list[i];
		sort_mask_list[number_list[i]] = mask_map_list[i];
		sort_confidencce_list[number_list[i]] = confidence_map_list[i];
	}

	wrap_list = sort_img_list;
	mask_list = sort_mask_list;
	confidence_list = sort_confidencce_list;

	return true;
}

bool Encode::maskBaseConfidence(cv::Mat confidence, int threshold, cv::Mat& mask)
{
	if (!confidence.data || !mask.data)
	{
		return false;
	}

	int nr = confidence.rows;
	int nc = confidence.cols;


	for (int r = 0; r < nr; r++)
	{
		float* ptr_c = confidence.ptr<float>(r);
		uchar* ptr_m = mask.ptr<uchar>(r);

		for (int c = 0; c < nc; c++)
		{
			if (ptr_c[c] < threshold)
			{
				ptr_m[c] = 0;
			}

		}
	}

	return true;

}


bool Encode::maskMap(cv::Mat mask, cv::Mat& map)
{

	if (!mask.data)
	{
		return false;
	}

	if (!map.data)
	{
		return false;
	}

	if (CV_32FC3 == map.type())
	{
		for (int r = 0; r < map.rows; r++)
		{

			cv::Vec3f* ptr_map = map.ptr<cv::Vec3f>(r);
			uchar* ptr_mask = mask.ptr<uchar>(r);

			for (int c = 0; c < map.cols; c++)
			{
				if (0 == ptr_mask[c])
				{
					ptr_map[c][0] = 0;
					ptr_map[c][1] = 0;
					ptr_map[c][2] = 0;
				}
			}

		}
	}
	else if (CV_32FC1 == map.type())
	{
		for (int r = 0; r < map.rows; r++)
		{
			float* ptr_map = map.ptr<float>(r);
			uchar* ptr_mask = mask.ptr<uchar>(r);

			for (int c = 0; c < map.cols; c++)
			{
				if (0 == ptr_mask[c])
				{
					ptr_map[c] = 0;
				}
			}

		}
	}



	return true;
}

bool Encode::unwrapVariableWavelength(cv::Mat l_unwrap, cv::Mat h_wrap, float rate, cv::Mat& h_unwrap, cv::Mat& k_Mat, float threshold, cv::Mat& err_mat)
{

	if (l_unwrap.empty() || h_wrap.empty())
	{
		return false;
	}

	int nr = l_unwrap.rows;
	int nc = l_unwrap.cols;

	if (l_unwrap.isContinuous())
	{
		if (h_wrap.isContinuous())
		{
			if (k_Mat.isContinuous())
			{

				nc = nc * nr;
				nr = 1;
			}
		}

	}

	cv::Mat err_map(l_unwrap.size(), CV_32F, cv::Scalar(0.0));

	for (int r = 0; r < nr; r++)
	{
		float* l_ptr = l_unwrap.ptr<float>(r);
		float* h_ptr = h_wrap.ptr<float>(r);
		uchar* k_ptr = k_Mat.ptr<uchar>(r);
		float* h_unwrap_ptr = h_unwrap.ptr<float>(r);

		float* ptr_err = err_map.ptr<float>(r);

		for (int c = 0; c < nc; c++)
		{

			float temp = 0.5 + (rate * l_ptr[c] - h_ptr[c]) / (2 * CV_PI);
			int k = temp;
			h_unwrap_ptr[c] = 2 * CV_PI * k + h_ptr[c];

			ptr_err[c] = fabs(h_unwrap_ptr[c] - rate * l_ptr[c]);

			k_ptr[c] = k;

			if (ptr_err[c] > threshold)
			{
				h_unwrap_ptr[c] = -10;

			}

		}
	}

	err_mat = err_map.clone();

	return true;
}

bool Encode::unwrapVariableWavelengthPatterns(std::vector<cv::Mat> wrap_img_list, std::vector<float> rate_list, cv::Mat& unwrap_img, cv::Mat& mask)
{
	if (wrap_img_list.empty())
	{
		return false;
	}
	if (wrap_img_list.size() != rate_list.size() + 1)
	{
		return false;
	}

	std::vector<float> threshold_list;

	for (int i = 0; i < rate_list.size(); i++)
	{
		threshold_list.push_back(CV_PI);
	}


	if (threshold_list.size() >= 3)
	{
		threshold_list[0] = CV_PI;
		threshold_list[1] = CV_PI;
		threshold_list[2] = 1.5;
	}


	int nr = wrap_img_list[0].rows;
	int nc = wrap_img_list[0].cols;

	bool unwrap_filter = false;

	if (mask.data)
	{
		unwrap_filter = true;
	}

	cv::Mat h_unwrap_map(nr, nc, CV_32F, cv::Scalar(0));

	cv::Mat err_map_l(nr, nc, CV_32F, cv::Scalar(0));
	cv::Mat err_map_h(nr, nc, CV_32F, cv::Scalar(0));

	cv::Mat unwrap_map = wrap_img_list[0];

	cv::Mat k_mat(nr, nc, CV_8U, cv::Scalar(0));

	for (int g_i = 1; g_i < wrap_img_list.size(); g_i++)
	{
		cv::Mat wrap_map = wrap_img_list[g_i];
		cv::Mat h_unwrap_map(nr, nc, CV_32F, cv::Scalar(0));
		cv::Mat err_map;

		unwrapVariableWavelength(unwrap_map, wrap_map, rate_list[g_i - 1], h_unwrap_map, k_mat, threshold_list[g_i - 1], err_map);

		unwrap_map = h_unwrap_map.clone();
	}

	unwrap_img = unwrap_map.clone();

	return true;
}

