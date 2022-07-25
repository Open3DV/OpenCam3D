#include "evaluate.h"



bool singlePixelCompare(cv::Mat org_map, cv::Mat dst_map, float threshold_val)
{
	if (org_map.empty() || dst_map.empty())
	{
		return false;
	}

	int nr = org_map.rows;
	int nc = dst_map.cols;


	if (CV_32F == org_map.type())
	{
		for (int r = 0; r < nr; r++)
		{
			float* ptr_o = org_map.ptr<float>(r);
			float* ptr_d = dst_map.ptr<float>(r);

			for (int c = 0; c < nc; c++)
			{
				float err = abs(ptr_o[c] - ptr_d[c]);

				if (err > threshold_val)
				{
					return false;
				}
			}

		}
	}
	else if (CV_8U == org_map.type())
	{
		for (int r = 0; r < nr; r++)
		{
			uchar* ptr_o = org_map.ptr<uchar>(r);
			uchar* ptr_d = dst_map.ptr<uchar>(r);

			for (int c = 0; c < nc; c++)
			{
				float err = abs(ptr_o[c] - ptr_d[c]);

				if (err > threshold_val)
				{
					return false;
				}
			}

		}
	}

	return true;

}