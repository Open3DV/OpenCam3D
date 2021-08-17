#pragma once

void normalizePoint(double x, double y,
	double fc_x, double fc_y,
	double cc_x, double cc_y,
	double k1, double k2, double k3, double p1, double p2,
	double& x_norm, double& y_norm);


void triangulation(double x_norm_L, double y_norm_L,
    double x_norm_R, double y_norm_R,
    double* R, double* T,
    double& X_L, double& Y_L, double& Z_L,
    double& X_R, double& Y_R, double& Z_R,
    double& error);

 