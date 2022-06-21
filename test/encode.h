#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

class DF_Encode
{
public:
	DF_Encode();
	~DF_Encode();

	bool sixStepPhaseShift(std::vector<cv::Mat> patterns, cv::Mat& wrap_map, cv::Mat& mask, cv::Mat& confidence);

	bool fourStepPhaseShift(std::vector<cv::Mat> patterns, cv::Mat& wrap_map, cv::Mat& mask, cv::Mat& confidence);

	bool computePhaseBaseSixStep(std::vector<cv::Mat> patterns, std::vector<cv::Mat>& wrap_maps, cv::Mat& mask_img, cv::Mat& confidence);

	bool computePhaseBaseFourStep(std::vector<cv::Mat> patterns, std::vector<cv::Mat>& wrap_maps, cv::Mat& mask_img, cv::Mat& confidence);

	void unwarpDualWavelength(cv::Mat l_unwrap, cv::Mat h_wrap, cv::Mat& h_unwrap, cv::Mat& k_Mat);

	bool unwrapVariableWavelength(cv::Mat l_unwrap, cv::Mat h_wrap, double rate, cv::Mat& h_unwrap, cv::Mat& k_Mat, cv::Mat err_mat = cv::Mat());

	bool unwrapVariableWavelengthPatterns(std::vector<cv::Mat> wrap_img_list, std::vector<double> rate_list, cv::Mat& unwrap_img, cv::Mat& mask);

	bool unwrapVariableWavelengthPatternsOpenmp(std::vector<cv::Mat> wrap_img_list, std::vector<double> rate_list, cv::Mat& unwrap_img, cv::Mat& mask);

	bool unwrapHalfWavelengthPatternsOpenmp(std::vector<cv::Mat> wrap_img_list, cv::Mat& unwrap_img, cv::Mat& mask);

	bool selectMaskBaseConfidence(cv::Mat confidence, int threshold, cv::Mat& mask);

	bool maskMap(cv::Mat mask, cv::Mat& map);

	bool mergePatterns(std::vector<std::vector<cv::Mat>> patterns_list, std::vector<cv::Mat>& patterns);
};

