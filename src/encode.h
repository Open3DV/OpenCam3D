#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

class Encode
{
public:
	Encode();
	~Encode();

	bool sixStepPhaseShift(std::vector<cv::Mat> patterns, cv::Mat& wrap_map, cv::Mat& mask, cv::Mat& confidence, cv::Mat& merge_brightness);

	bool fourStepPhaseShift(std::vector<cv::Mat> patterns, cv::Mat& wrap_map, cv::Mat& mask, cv::Mat& confidence, cv::Mat& merge_brightness);

	bool computePhaseBaseFourStep(std::vector<cv::Mat> patterns, std::vector<cv::Mat>& wrap_list, std::vector<cv::Mat>& mask_list, std::vector<cv::Mat>& confidence_list, std::vector<cv::Mat>& brightness_list);

	bool maskBaseConfidence(cv::Mat confidence, int threshold, cv::Mat& mask);

	bool maskMap(cv::Mat mask, cv::Mat& map);

	bool unwrapVariableWavelength(cv::Mat l_unwrap, cv::Mat h_wrap, float rate, cv::Mat& h_unwrap, cv::Mat& k_Mat, float threshold, cv::Mat& err_mat);

	bool unwrapVariableWavelengthPatterns(std::vector<cv::Mat> wrap_img_list, std::vector<float> rate_list, cv::Mat& unwrap_img, cv::Mat& mask);
};

