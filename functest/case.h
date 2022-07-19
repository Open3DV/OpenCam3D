#pragma once

#include <stdio.h>
#include "../sdk/open_cam3d.h" 
#include "opencv2/opencv.hpp"

void get_point_cloud(const char* ip);

bool get_frame_01(const char* ip);