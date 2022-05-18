#include "encode.h"
#include "iostream" 

DF_Encode::DF_Encode()
{
}


DF_Encode::~DF_Encode()
{
}


bool DF_Encode::sixStepPhaseShift(std::vector<cv::Mat> patterns, cv::Mat& wrap_map, cv::Mat& mask, cv::Mat& confidence)
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


	cv::Mat result(img1.rows, img1.cols, CV_64F, cv::Scalar(-1));
	cv::Mat confidence_map(img1.rows, img1.cols, CV_64F, cv::Scalar(0));

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
		double* ptr_con = confidence_map.ptr<double>(r);

		double* optr = result.ptr<double>(r);
		for (int c = 0; c < nc; c++)
		{
			if (ptr_m[c])
			{
				//double a = ptr4[j] - ptr2[j];
				//double b = ptr1[j] - ptr3[j];


				double b = ptr0[c] * std::sin(0 * CV_2PI / 6.0) + ptr1[c] * std::sin(1 * CV_2PI / 6.0) + ptr2[c] * std::sin(2 * CV_2PI / 6.0)
					+ ptr3[c] * std::sin(3 * CV_2PI / 6.0) + ptr4[c] * std::sin(4 * CV_2PI / 6.0) + ptr5[c] * std::sin(5 * CV_2PI / 6.0);


				double a = ptr0[c] * std::cos(0 * CV_2PI / 6.0) + ptr1[c] * std::cos(1 * CV_2PI / 6.0) + ptr2[c] * std::cos(2 * CV_2PI / 6.0)
					+ ptr3[c] * std::cos(3 * CV_2PI / 6.0) + ptr4[c] * std::cos(4 * CV_2PI / 6.0) + ptr5[c] * std::cos(5 * CV_2PI / 6.0);

				double r = std::sqrt(a * a + b * b);

				//if (r > 255)
				//{
				//	r = 255;
				//}

				ptr_con[c] = r;

				/***********************************************************************/

				if (255 == ptr1[c] || 255 == ptr2[c] || 255 == ptr3[c] || 255 == ptr4[c] || 255 == ptr4[c] || 255 == ptr5[c])
				{
					ptr_m[c] = 0;
					ptr_con[c] = 0;
					optr[c] = 0;
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

bool DF_Encode::fourStepPhaseShift(std::vector<cv::Mat> patterns, cv::Mat& wrap_map, cv::Mat& mask, cv::Mat &confidence)
{
	if (4 != patterns.size())
	{
		return false;
	}
	 
	cv::Mat img1 = patterns[0];
	cv::Mat img2 = patterns[1];
	cv::Mat img3 = patterns[2];
	cv::Mat img4 = patterns[3];


	cv::Mat result(img1.rows, img1.cols, CV_64F, cv::Scalar(-1));
	cv::Mat confidence_map(img1.rows, img1.cols, CV_64F, cv::Scalar(0));

	if (!mask.data)
	{
		mask = cv::Mat(img1.rows, img1.cols, CV_8U, cv::Scalar(255));
	} 

	int nl = img1.rows;
	int nc = img1.cols* img1.channels();

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
						nc = nc*nl;
						nl = 1;
					}
				}
			}
		}

	}
	#pragma omp parallel for
		for (int i = 0; i< nl; i++)
		{
			uchar* ptr1 = img1.ptr<uchar>(i);
			uchar* ptr2 = img2.ptr<uchar>(i);
			uchar* ptr3 = img3.ptr<uchar>(i);
			uchar* ptr4 = img4.ptr<uchar>(i);
			uchar* ptr_m = mask.ptr<uchar>(i);
			double* ptr_con = confidence_map.ptr<double>(i);

			double* optr = result.ptr<double>(i);
			for (int j = 0; j< nc; j++)
			{
				if (ptr_m[j] == 255)
				{
					double a = ptr4[j] - ptr2[j];
					double b = ptr1[j] - ptr3[j];
 
					double r = std::sqrt(a*a + b*b) + 0.5; 

					//if(r> 255)
					//{
					//	r = 255;
					//}

					ptr_con[j] = r;

					/***********************************************************************/

					if (255 == ptr1[j] || 255 == ptr2[j] || 255 == ptr3[j] || 255 == ptr4[j])
					{
						ptr_m[j] = 0;
						ptr_con[j] = 0;
						optr[j] = 0;
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
 

bool DF_Encode::unwrapVariableWavelength(cv::Mat l_unwrap, cv::Mat h_wrap, double rate, cv::Mat& h_unwrap, cv::Mat& k_Mat,cv::Mat err_mat)
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

	cv::Mat err_map(l_unwrap.size(), CV_64F, cv::Scalar(0.0));

	for (int r = 0; r < nr; r++)
	{
		double* l_ptr = l_unwrap.ptr<double>(r);
		double* h_ptr = h_wrap.ptr<double>(r);
		uchar* k_ptr = k_Mat.ptr<uchar>(r);
		double* h_unwrap_ptr = h_unwrap.ptr<double>(r);

		double* ptr_err = err_map.ptr<double>(r);

		for (int c = 0; c < nc; c++)
		{

			//double temp = 0.5 + l_ptr[j] / (1 * CV_PI) - h_ptr[j] / (rate * CV_PI); 

			double temp = 0.5 + (rate * l_ptr[c] - h_ptr[c]) / (2*CV_PI);
			int k = temp;
			h_unwrap_ptr[c] = 2 * CV_PI * k + h_ptr[c];

			ptr_err[c] = fabs(h_unwrap_ptr[c] - rate * l_ptr[c]);

			k_ptr[c] = k;

			if (ptr_err[c] > 0.8)
			{
				h_unwrap_ptr[c] = -10;

			}

			//int k = temp; 
			//k_ptr[j] = k; 
			//h_unwrap_ptr[j] = 2 * CV_PI * k + h_ptr[j];
			 

		}
	}

	err_mat = err_map.clone();

	return true;
}

void DF_Encode::unwarpDualWavelength(cv::Mat l_unwrap, cv::Mat h_wrap, cv::Mat& h_unwrap, cv::Mat& k_Mat)
{


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


	for (int i = 0; i < nr; i++)
	{
		double* l_ptr = l_unwrap.ptr<double>(i);
		double* h_ptr = h_wrap.ptr<double>(i);
		uchar* k_ptr = k_Mat.ptr<uchar>(i);
		double* h_unwrap_ptr = h_unwrap.ptr<double>(i);
		for (int j = 0; j < nc; j++)
		{

			double temp = 0.5 + l_ptr[j] / (1 * CV_PI) - h_ptr[j] / (2 * CV_PI);

			int k = temp;


			k_ptr[j] = k;

			h_unwrap_ptr[j] = 2 * CV_PI * k + h_ptr[j];




		}
	}


}

bool DF_Encode::unwrapHalfWavelengthPatternsOpenmp(std::vector<cv::Mat> wrap_img_list, cv::Mat& unwrap_img, cv::Mat& mask)
{
	if (wrap_img_list.empty())
	{
		return false;
	}

 

	bool unwrap_filter = false;
	 

	if (mask.data)
	{
		unwrap_filter = true;
	}

	std::vector<cv::Mat> unwrap_img_list;
	unwrap_img_list.push_back(wrap_img_list[0]);

	int nr = wrap_img_list[0].rows;
	int nc = wrap_img_list[0].cols;

	for (int w_i = 0; w_i < wrap_img_list.size() - 1; w_i++)
	{
		cv::Mat img_1 = unwrap_img_list[w_i];
		cv::Mat img_2 = wrap_img_list[w_i + 1];

		std::cout << "w_i: " << w_i;

		cv::Mat k_mat(nr, nc, CV_8U, cv::Scalar(0));
		cv::Mat unwrap_mat(nr, nc, CV_64F, cv::Scalar(0));

		unwarpDualWavelength(img_1, img_2, unwrap_mat, k_mat); 
		unwrap_img_list.push_back(unwrap_mat);

	}
	 

	float period_num = std::pow(2, unwrap_img_list.size() - 1);

	 

	//unwrap_img = unwrap_img_list[unwrap_img_list.size() - 1].clone()/ period_num;

	unwrap_img = unwrap_img_list[unwrap_img_list.size() - 1].clone();
	 

	return true;
}


bool DF_Encode::unwrapVariableWavelengthPatterns(std::vector<cv::Mat> wrap_img_list, std::vector<double> rate_list, cv::Mat& unwrap_img, cv::Mat& mask)
{
	if (wrap_img_list.empty())
	{
		return false;
	}
	if (wrap_img_list.size() != rate_list.size()+1)
	{
		return false;
	}


	int nr = wrap_img_list[0].rows;
	int nc = wrap_img_list[0].cols;

	bool unwrap_filter = false;

	if (mask.data)
	{
		unwrap_filter = true;
	}

	cv::Mat h_unwrap_map(nr, nc, CV_64F, cv::Scalar(0));

	cv::Mat err_map_l(nr, nc, CV_64F, cv::Scalar(0));
	cv::Mat err_map_h(nr, nc, CV_64F, cv::Scalar(0));

	cv::Mat unwrap_map = wrap_img_list[0];

	cv::Mat k_mat(nr,nc,CV_8U,cv::Scalar(0));

	for (int g_i = 1; g_i < wrap_img_list.size(); g_i++)
	{
		cv::Mat wrap_map = wrap_img_list[g_i];
		cv::Mat h_unwrap_map(nr, nc, CV_64F, cv::Scalar(0));

		unwrapVariableWavelength(unwrap_map, wrap_map, rate_list[g_i - 1], h_unwrap_map, k_mat);

		unwrap_map = h_unwrap_map.clone();
	}

	unwrap_img = unwrap_map.clone();

	return true;
}

bool DF_Encode::unwrapVariableWavelengthPatternsOpenmp(std::vector<cv::Mat> wrap_img_list, std::vector<double> rate_list, cv::Mat &unwrap_img, cv::Mat &mask)
{

	if (wrap_img_list.empty())
	{
		return false;
	}
	if (3 != wrap_img_list.size())
	{
		return false;
	}

	if (2 != rate_list.size())
	{
		return false;
	}

	int nr = wrap_img_list[0].rows;
	int nc = wrap_img_list[0].cols;

	bool unwrap_filter = false;

	if (mask.data)
	{
		unwrap_filter = true;
	}

	cv::Mat h_unwrap_map(nr, nc, CV_64F, cv::Scalar(0));

	cv::Mat err_map_l(nr, nc, CV_64F, cv::Scalar(0));
	cv::Mat err_map_h(nr, nc, CV_64F, cv::Scalar(0));

	#pragma omp parallel for
	for (int r = 0; r< nr; r++)
	{
		double* ptr_0 = wrap_img_list[0].ptr<double>(r);
		double* ptr_1 = wrap_img_list[1].ptr<double>(r);
		double* ptr_2 = wrap_img_list[2].ptr<double>(r);

		double* ptr_err_l = err_map_l.ptr<double>(r);
		double* ptr_err_h = err_map_h.ptr<double>(r);

		uchar* ptr_mask = mask.ptr<uchar>(r);

		double* ptr_h = h_unwrap_map.ptr<double>(r);

		for (int c = 0; c< nc; c++)
		{

			double temp = 0.5 + (rate_list[0] * ptr_0[c] - ptr_1[c]) / (CV_PI);
			int k = temp;
			ptr_h[c] = CV_PI*k + ptr_1[c];

			if (unwrap_filter)
			{
				double error = fabs(ptr_h[c] - ptr_0[c] * rate_list[0]);
				ptr_err_l[c] = error * 1;
				//backup 0.5
				if (error > 1.0)
				{
					ptr_h[c] = 0;
					ptr_mask[c] = 0;
				}
			}




			/******************************************************************/
			temp = 0.5 + (rate_list[1] * ptr_h[c] - ptr_2[c]) / (CV_PI);
			k = temp;

			double old_ptr_h = ptr_h[c];
			ptr_h[c] = CV_PI*k + ptr_2[c];

			if (unwrap_filter)
			{
				double error = fabs(ptr_h[c] - old_ptr_h * rate_list[1]);
				ptr_err_h[c] = error * 1;
				//backup 0.2
				if (error > 0.4)
				{
					ptr_h[c] = 0;
					ptr_mask[c] = 0;
				}
			}


			/********************************************************************************/
		}

	}

	unwrap_img = h_unwrap_map.clone();

	//unwrap_img = unwrap_img / 32;

	return true;
}



bool DF_Encode::computePhaseBaseSixStep(std::vector<cv::Mat> patterns, std::vector<cv::Mat>& wrap_maps, cv::Mat& mask_img, cv::Mat& confidence)
{
	std::vector<cv::Mat> wrap_img_list;
	std::vector<cv::Mat> confidence_map_list;
	std::vector<int> number_list;



#pragma omp parallel for
	for (int i = 0; i < patterns.size() - 1; i += 6)
	{
		cv::Mat wrap_img;
		cv::Mat confidence;

		std::vector<cv::Mat> phase_list(patterns.begin() + i, patterns.begin() + i + 6);
		sixStepPhaseShift(phase_list, wrap_img, mask_img, confidence);


#pragma omp critical
		{
			number_list.push_back(i / 6);
			wrap_img_list.push_back(wrap_img);
			confidence_map_list.push_back(confidence);
		}


	}


	std::vector<cv::Mat> sort_img_list;
	sort_img_list.resize(wrap_img_list.size());

	std::vector<cv::Mat> sort_confidencce_list;
	sort_confidencce_list.resize(confidence_map_list.size());

	for (int i = 0; i < wrap_img_list.size(); i++)
	{

		sort_img_list[number_list[i]] = wrap_img_list[i];
		sort_confidencce_list[number_list[i]] = confidence_map_list[i];
	}

	wrap_maps = sort_img_list;

	cv::Mat confid_map = sort_confidencce_list[0].clone();

	int nr = sort_confidencce_list[0].rows;
	int nc = sort_confidencce_list[0].cols;

	//for (int r = 0; r < nr; r++)
	//{

	//	double* ptr_0 = sort_confidencce_list[0].ptr<double>(r);
	//	double* ptr_1 = sort_confidencce_list[1].ptr<double>(r);
	//	double* ptr_2 = sort_confidencce_list[2].ptr<double>(r);

	//	double* ptr_confid = confid_map.ptr<double>(r);

	//	for (int c = 0; c < nc; c++)
	//	{

	//		double max_v = 0;
	//		if (ptr_0[c] > ptr_1[c])
	//		{
	//			max_v = ptr_0[c];
	//		}
	//		else
	//		{
	//			max_v = ptr_1[c];
	//		}
	//		if (max_v < ptr_2[c])
	//		{
	//			max_v = ptr_2[c];
	//		}
	//		ptr_confid[c] = max_v;

	//	}
	//}

	confidence = confid_map.clone();


	return true;
}

bool DF_Encode::computePhaseBaseFourStep(std::vector<cv::Mat> patterns, std::vector<cv::Mat> &wrap_maps, cv::Mat &mask_img, cv::Mat& confidence)
{

	std::vector<cv::Mat> wrap_img_list;
	std::vector<cv::Mat> confidence_map_list;
	std::vector<int> number_list;



	#pragma omp parallel for
	for (int i = 0; i < patterns.size() - 1; i += 4)
	{
		cv::Mat wrap_img;
		cv::Mat confidence;

		std::vector<cv::Mat> phase_list(patterns.begin() + i, patterns.begin() + i + 4);
		fourStepPhaseShift(phase_list, wrap_img, mask_img, confidence);


		#pragma omp critical
		{
			number_list.push_back(i / 4);
			wrap_img_list.push_back(wrap_img);
			confidence_map_list.push_back(confidence);
		}


	}


	std::vector<cv::Mat> sort_img_list;
	sort_img_list.resize(wrap_img_list.size());

	std::vector<cv::Mat> sort_confidencce_list;
	sort_confidencce_list.resize(confidence_map_list.size());

	for (int i = 0; i < wrap_img_list.size(); i++)
	{

		sort_img_list[number_list[i]] = wrap_img_list[i];
		sort_confidencce_list[number_list[i]] = confidence_map_list[i];
	}

	wrap_maps = sort_img_list;

	cv::Mat confid_map = sort_confidencce_list[0].clone();

	int nr = sort_confidencce_list[0].rows;
	int nc = sort_confidencce_list[0].cols;

	for (int r = 0; r < nr; r++)
	{

		uchar* ptr_0 = sort_confidencce_list[0].ptr<uchar>(r);
		uchar* ptr_1 = sort_confidencce_list[1].ptr<uchar>(r);
		uchar* ptr_2 = sort_confidencce_list[2].ptr<uchar>(r);

		uchar* ptr_confid = confid_map.ptr<uchar>(r);

		for (int c = 0; c < nc; c++)
		{
			uchar min_v = 255;
			if (ptr_0[c] < ptr_1[c])
			{
				min_v = ptr_0[c];
			}
			else
			{
				min_v = ptr_1[c];
			}
			if (min_v > ptr_2[c])
			{
				min_v = ptr_2[c];
			}
			ptr_confid[c] = min_v;

			//uchar max_v = 0;
			//if (ptr_0[c] > ptr_1[c])
			//{
			//	max_v = ptr_0[c];
			//}
			//else
			//{
			//	max_v = ptr_1[c];
			//}
			//if (max_v < ptr_2[c])
			//{
			//	max_v = ptr_2[c];
			//}
			//ptr_confid[c] = max_v;

		}
	}

	confidence = confid_map.clone();


	return true;
}


bool DF_Encode::maskMap(cv::Mat mask, cv::Mat &map)
{
	if(!mask.data)
	{
		return false;
	}

	if(!map.data)
	{
		return false;
	}


	if(CV_64FC3 == map.type())
	{
		for(int r= 0;r< map.rows;r++)
		{
			
			cv::Vec3d* ptr_map = map.ptr<cv::Vec3d>(r);
			uchar* ptr_mask = mask.ptr<uchar>(r);

			for(int c= 0;c< map.cols;c++)
			{
				if(0 == ptr_mask[c])
				{
					ptr_map[c][0] = 0;
					ptr_map[c][1] = 0;
					ptr_map[c][2] = 0;
				}
			}

		}
	}
	else if(CV_64FC1 == map.type())
	{
		for (int r = 0; r< map.rows; r++)
		{ 
			double* ptr_map = map.ptr<double>(r);
			uchar* ptr_mask = mask.ptr<uchar>(r);

			for (int c = 0; c< map.cols; c++)
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

bool DF_Encode::selectMaskBaseConfidence(cv::Mat confidence, int threshold, cv::Mat& mask)
{
	if (!confidence.data)
	{
		return false;
	}

	int nr = confidence.rows;
	int nc = confidence.cols;

	//mask = cv::Mat(nr, nc, CV_8U, cv::Scalar(0));

	for (int r = 0; r< nr; r++)
	{
		double* ptr_c = confidence.ptr<double>(r);
		uchar* ptr_m = mask.ptr<uchar>(r);

		for (int c = 0; c< nc; c++)
		{
			if (ptr_c[c] < threshold)
			{
				ptr_m[c] = 0;
			}
 
		}
	}

	return true;

}
