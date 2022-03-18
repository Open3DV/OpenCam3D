#include "calibration_param_gui.h"

CalibrationParamGui::CalibrationParamGui(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
}

CalibrationParamGui::~CalibrationParamGui()
{
}


bool CalibrationParamGui::setShowCalibrationMessage(struct SystemConfigParam config_param, struct CameraCalibParam calibration_param)
{
	system_config_param_ = config_param;
	camera_calibration_param_ = calibration_param;

	QString distort_str = "";

	for (int i = 0; i < 5; i++)
	{
		distort_str += QString::number(calibration_param.camera_distortion[i]);
		distort_str += "\t";
	}
	ui.textEdit_distortion->setText(distort_str);

	float extrinsic[12] = { 1,0,0,0,1,0,0,0,1,0,0,0 };

	QString extrinsic_str;

	for (int r = 0; r < 3; r++)
	{
		for (int c = 0; c < 3; c++)
		{
			extrinsic_str += QString::number(extrinsic[r * 3 + c]);
			extrinsic_str += "\t \t";
		}
		extrinsic_str += QString::number(extrinsic[9 + r]);
		extrinsic_str += "\r\n";
	}
	 
	ui.textEdit_extrinsic->setText(extrinsic_str);


	QString intrinsic_str = "";

	for (int r = 0; r < 3; r++)
	{
		for (int c = 0; c < 3; c++)
		{
			intrinsic_str += QString::number(calibration_param.camera_intrinsic[r*3 + c]);
			intrinsic_str += "\t \t";
		}
		intrinsic_str += "\r\n";
	} 

	ui.textEdit_intrinsic->setText(intrinsic_str);



	QString plane_str = "";

	for (int r = 0; r < 3; r++)
	{ 
		for (int c = 0; c < 3; c++)
		{
			plane_str += QString::number(config_param.standard_plane_external_param[r * 3 + c]);
			plane_str += "\t";
		}
		plane_str += QString::number(config_param.standard_plane_external_param[9 + r]);
		plane_str += "\r\n";
	} 
	ui.textEdit_standard_plane->setText(plane_str);

	return true;
}