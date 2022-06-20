#include "camera_gui.h"
#include "camera_capture_gui.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "file_io_function.h"
#include <QDebug>
#include <iostream>
#include <QMouseEvent>
#include <QMessageBox>
#include <QLabel>
#include <QFileDialog>
#include "../firmware/version.h"
#include "select_calibration_board_gui.h"

camera_gui::camera_gui(QWidget* parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	connect(ui.actionAbout, &QAction::triggered, []() {
		QMessageBox::about(NULL, "About", _VERSION_);
		});


	default_config_path_ = "../camera_config.json";
	last_config_path_ = "../dfx802_config.json";

	label_temperature_ = new QLabel(this);
	ui.statusBar->addPermanentWidget(label_temperature_);

	connect(ui.tab_capture, SIGNAL(send_temperature_update(float)), this, SLOT(do_update_temperature(float)));
	connect(ui.action_load_camera_config, SIGNAL(triggered()), this, SLOT(do_action_load_camera_config()));
	connect(ui.action_save_camera_config, SIGNAL(triggered()), this, SLOT(do_action_save_camera_config()));
	connect(ui.action_exit, SIGNAL(triggered()), this, SLOT(do_action_exit()));
	connect(ui.action_get_calibration_param, SIGNAL(triggered()), this, SLOT(do_action_show_calibration_param()));
	connect(ui.action_select_calibration_board, SIGNAL(triggered()), this, SLOT(do_action_select_calibration_board()));
	connect(this, SIGNAL(send_network_drop()), this, SLOT(do_slot_handle_network()));
}

camera_gui::~camera_gui()
{

}

void  camera_gui::do_slot_handle_network()
{
	ui.tab_capture->addLogMessage(u8"心跳停止");
	ui.tab_capture->do_pushButton_disconnect();
	ui.tab_capture->addLogMessage(u8"重新连接...");
	ui.tab_capture->do_pushButton_connect();
}

bool camera_gui::handle_network_drop()
{
	//ui.tab_capture->do_pushButton_disconnect();

	emit send_network_drop();

	return true;
}

void camera_gui::setOnDrop(int (*p_function)(void*))
{
	ui.tab_capture->setOnDrop(p_function);
}

void camera_gui::do_action_load_camera_config()
{
	QString path = QFileDialog::getOpenFileName(this, u8"加载配置文件", last_config_path_, "*.json");

	if (path.isEmpty())
	{
		return;
	}

	last_config_path_ = path;

	bool ret = ui.tab_capture->loadSettingData(path);


	if (ret)
	{
		QString log = u8"成功加载配置文件： " + path;
		ui.tab_capture->addLogMessage(log);
	}
	else
	{
		QString log = u8"加载配置文件失败： " + path;
		ui.tab_capture->addLogMessage(log);
	}

}

void camera_gui::do_action_save_camera_config()
{

	QString path = QFileDialog::getSaveFileName(this, u8"保存配置文件", last_config_path_, "*.json");

	if (path.isEmpty())
	{
		return;
	}

	last_config_path_ = path;



	bool ret = ui.tab_capture->saveSettingData(path);

	if (ret)
	{
		QString log = u8"保存配置文件： " + path;
		ui.tab_capture->addLogMessage(log);
	}
	else
	{
		QString log = u8"保存配置文件失败： " + path;
		ui.tab_capture->addLogMessage(log);
	}
}


void camera_gui::do_action_select_calibration_board()
{
	SelectCalibrationBoardGui board_widget;
	struct GuiConfigDataStruct gui_param;
	ui.tab_capture->getGuiConfigParam(gui_param);
	board_widget.set_board_type(gui_param.Instance().calibration_board);

	if (QDialog::Accepted == board_widget.exec())
	{
		int flag = board_widget.get_board_type();
		qDebug() << "board: " << flag;

		ui.tab_capture->setCalibrationBoard(flag);
	}



}

void camera_gui::do_action_show_calibration_param()
{
	struct SystemConfigParam config_param;
	struct CameraCalibParam calibration_param;

	bool ret = ui.tab_capture->getShowCalibrationMessage(config_param, calibration_param);

	if (ret)
	{
		show_calib_param_gui_.setShowCalibrationMessage(config_param, calibration_param);
		show_calib_param_gui_.exec();
	}
	else
	{
		ui.tab_capture->addLogMessage(u8"请连接相机");
	}


}

void camera_gui::do_action_exit()
{
	this->close();
}

void camera_gui::do_update_temperature(float val)
{
	QString str = QString::number(val) + u8" ℃";

	label_temperature_->setText(str);
}

void camera_gui::closeEvent(QCloseEvent* e)
{
	if (QMessageBox::question(this,
		u8"提示",
		u8"确定退出软件？",
		QMessageBox::Yes, QMessageBox::No)
		== QMessageBox::Yes) {
		e->accept();//不会将事件传递给组件的父组件


		ui.tab_capture->saveSettingData(default_config_path_);

		if (ui.tab_capture->isConnect())
		{
			ui.tab_capture->do_pushButton_disconnect();
		}

	}
	else
	{
		e->ignore();
	}
}

bool camera_gui::setShowImages(cv::Mat brightness, cv::Mat depth)
{
	cv::Mat img_color;
	cv::Mat gray_map;
	FileIoFunction io_machine;

	int low_z = processing_gui_settings_data_.Instance().low_z_value;
	int high_z = processing_gui_settings_data_.Instance().high_z_value;

	io_machine.depthToColor(depth, img_color, gray_map, low_z, high_z);


	return true;
}

bool camera_gui::setSettingsData(GuiConfigDataStruct& settings_data)
{
	processing_gui_settings_data_ = settings_data;


	//cv::Mat img_b = cv::imread("G:/Code/GitCode/DF8/DF15_SDK/x64/Release/capture_data/frame03_data/0604/data_01.bmp", 0);
	//cv::Mat img_depth = cv::imread("G:/Code/GitCode/DF8/DF15_SDK/x64/Release/capture_data/frame03_data/0604/data_01.tiff", cv::IMREAD_UNCHANGED);
	//setShowImages(img_b, img_depth);

	return true;
}

bool camera_gui::setUiData()
{
	//ui.tab_capture->setSettingData(processing_settings_data_);
	//ui.tab_capture->setUiData();


	return true;

}

