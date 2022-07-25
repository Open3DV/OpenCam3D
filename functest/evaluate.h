#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

bool singlePixelCompare(cv::Mat org_map, cv::Mat dst_map, float threshold_val);