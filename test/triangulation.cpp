#include "triangulation.h"
#include <cmath>

void normalizePoint(
    double x, double y,
	double fc_x, double fc_y,
	double cc_x, double cc_y,
	double k1, double k2, double k3, double p1, double p2,
	double& x_norm, double& y_norm)
{
    double x_distort = (x - cc_x) / fc_x;
    double y_distort = (y - cc_y) / fc_y;

    double x_iter = x_distort;
    double y_iter = y_distort;
    
    for (int i = 0; i < 20; i++)
    {
        double r_2 = x_iter * x_iter + y_iter * y_iter;
        double r_4 = r_2 * r_2;
        double r_6 = r_4 * r_2;
        double k_radial = 1 + k1 * r_2 + k2 * r_4 + k3 * r_6;
        double delta_x = 2 * p1 * x_iter * y_iter + p2 * (r_2 + 2 * x_iter * x_iter);
        double delta_y = p1 * (r_2 + 2 * y_iter * y_iter) + 2 * p2 * x_iter * y_iter;
        x_iter = (x_distort - delta_x) / k_radial;
        y_iter = (y_distort - delta_y) / k_radial;
    }
    //x_norm = x_iter*fc_x+ cc_x;
    //y_norm = y_iter*fc_y + cc_y;

	x_norm = x_iter;
	y_norm = y_iter;

    return;
}

void undistortPoint(double x, double y,
    double fc_x, double fc_y,
    double cc_x, double cc_y,
    double k1, double k2, double k3, double p1, double p2,
    double& x_undistort, double& y_undistort)
{
    double x_distort = (x - cc_x) / fc_x;
    double y_distort = (y - cc_y) / fc_y;

    double x_iter = x_distort;
    double y_iter = y_distort;

    for (int i = 0; i < 20; i++)
    {
        double r_2 = x_iter * x_iter + y_iter * y_iter;
        double r_4 = r_2 * r_2;
        double r_6 = r_4 * r_2;
        double k_radial = 1 + k1 * r_2 + k2 * r_4 + k3 * r_6;
        double delta_x = 2 * p1 * x_iter * y_iter + p2 * (r_2 + 2 * x_iter * x_iter);
        double delta_y = p1 * (r_2 + 2 * y_iter * y_iter) + 2 * p2 * x_iter * y_iter;
        x_iter = (x_distort - delta_x) / k_radial;
        y_iter = (y_distort - delta_y) / k_radial;
    }
    x_undistort = x_iter*fc_x+ cc_x;
    y_undistort = y_iter*fc_y + cc_y;

    //x_undistort = x_iter;
    //y_undistort = y_iter;

    return;
}

void triangulation(double x_norm_L, double y_norm_L,
    double x_norm_R, double y_norm_R,
    double* R, double* T,
    double& X_L, double& Y_L, double& Z_L,
    double& X_R, double& Y_R, double& Z_R,
    double& error)
{
    double u_x_L = R[0] * x_norm_L + R[1] * y_norm_L + R[2];
    double u_y_L = R[3] * x_norm_L + R[4] * y_norm_L + R[5];
    double u_w_L = R[6] * x_norm_L + R[7] * y_norm_L + R[8];

    double n_x2_L = x_norm_L * x_norm_L + y_norm_L * y_norm_L + 1;
    double n_x2_R = x_norm_R * x_norm_R + y_norm_R * y_norm_R + 1;

    double D = u_x_L * x_norm_R + u_y_L * y_norm_R + u_w_L;
    double DD = n_x2_L * n_x2_R - D * D;

    double dot_uT = u_x_L * T[0] + u_y_L * T[1] + u_w_L * T[2];
    double dot_xttT = x_norm_R * T[0] + y_norm_R * T[1] + T[2];
    double dot_xttu = u_x_L * x_norm_R + u_y_L * y_norm_R + u_w_L;

    double NN1 = dot_xttu*dot_xttT - n_x2_R*dot_uT;
    double NN2 = n_x2_L*dot_xttT - dot_uT*dot_xttu;

    double Zt = NN1/DD;
    double Ztt = NN2/DD;

    double X1 = x_norm_L * Zt;
    double Y1 = y_norm_L * Zt;
    double Z1 = Zt;

    double X2_R = x_norm_R * Ztt - T[0];
    double Y2_R = y_norm_R * Ztt - T[1];
    double Z2_R = Ztt - T[2];

    double X2 = R[0] * X2_R + R[3] * Y2_R + R[6] * Z2_R;
    double Y2 = R[1] * X2_R + R[4] * Y2_R + R[7] * Z2_R;
    double Z2 = R[2] * X2_R + R[5] * Y2_R + R[8] * Z2_R;

	X_L = (X1 + X2) / 2.0;
	Y_L = (Y1 + Y2) / 2.0;
	Z_L = (Z1 + Z2) / 2.0;

	//X_L = X1;
	//Y_L = Y1;
	//Z_L = Z1;

    //XR = R * XL + T;
    X_R = R[0] * X_L + R[1] * Y_L + R[2] * Z_L + T[0];
    Y_R = R[3] * X_L + R[4] * Y_L + R[5] * Z_L + T[1];
    Z_R = R[6] * X_L + R[7] * Y_L + R[8] * Z_L + T[2];

    error = sqrt((X1 - X2) * (X1 - X2) + (Y1 - Y2) * (Y1 - Y2) + (Z1 - Z2) * (Z1 - Z2));
	//+++++++++++++++++++++++++++++++++++++++++++++
    return;
}