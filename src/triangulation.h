#pragma once

void normalizePoint(float x, float y,
	float fc_x, float fc_y,
	float cc_x, float cc_y,
	float k1, float k2, float k3, float p1, float p2,
	float& x_norm, float& y_norm);

void undistortPoint(float x, float y,
	float fc_x, float fc_y,
	float cc_x, float cc_y,
	float k1, float k2, float k3, float p1, float p2,
	float& x_undistort, float& y_undistort);

void triangulation(float x_norm_L, float y_norm_L,
	float x_norm_R, float y_norm_R,
	float* R, float* T,
	float& X_L, float& Y_L, float& Z_L,
	float& X_R, float& Y_R, float& Z_R,
	float& error);

