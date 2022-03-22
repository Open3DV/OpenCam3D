#pragma once

#include <QDialog>
#include "ui_calibration_param_gui.h"
#include "../firmware/camera_param.h"
#include "../firmware/system_config_settings.h"



class CalibrationParamGui : public QDialog
{
	Q_OBJECT

public:
	CalibrationParamGui(QWidget *parent = Q_NULLPTR);
	~CalibrationParamGui();

	bool setShowCalibrationMessage(struct SystemConfigParam config_param, struct CameraCalibParam calibration_param);
	 

private:
	Ui::CalibrationParamGui ui;

	//���ϵͳ���ò���
	struct SystemConfigParam system_config_param_;
	//����궨����
	struct CameraCalibParam camera_calibration_param_;
};
