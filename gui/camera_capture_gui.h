#pragma once

#include <QWidget>
#include "ui_camera_capture_gui.h"
#include "settings_file_function.h"
#include <opencv2/core.hpp>
#include "../sdk/open_cam3d.h"
#include <QThread>
#include <QDebug>
#include <QtCore/QTimer>

#define SELECT_BRIGHTNESS_FLAG_ 1;
#define SELECT_GRAY_DEPTH_FLAG_ 2;
#define SELECT_COLOR_DEPTH_FLAG_ 3;

class CameraCaptureGui : public QWidget
{
	Q_OBJECT

public:
	CameraCaptureGui(QWidget *parent = Q_NULLPTR);
	~CameraCaptureGui();
	 
	bool setShowImages(cv::Mat brightness, cv::Mat depth);

	void setSettingData(ProcessingDataStruct& settings_data_);

	bool saveSettingData(QString path);

	void setUiData(); 

	bool connectCamera(QString ip);

	bool capture_one_frame_data();

	bool capture_one_frame_and_render();

	bool capture_brightness();

	bool initializeFunction();

	bool saveOneFrameData(QString path_name);

	void testThread(QString path_name);

	void addLogMessage(QString str); 

	//更新多曝光参数
	void update_many_exposure_param();

	bool many_exposure_param_has_changed();
private:
	bool showImage();

	bool setImage(cv::Mat img);

	bool renderDepthImage(cv::Mat depth);

	bool renderBrightnessImage(cv::Mat brightness);

	void undateSystemConfigUiData();

	double computePointsDistance(cv::Point2f p_0, cv::Point2f p_1, cv::Mat point_cloud);
	 
signals:
	void send_temperature_update(float val);

public slots:


	void do_timeout_slot();

 


private slots:
	void do_QRadioButton_toggled_brightness(bool state);

	void do_QRadioButton_toggled_color_depth(bool state);

	void do_QRadioButton_toggled_gray_depth(bool state);

	void add_exposure_item(int row,int col,int val);

	bool remove_exposure_item(int row);  

	double get_exposure_item_value(int row);

private slots:
	void do_checkBox_toggled_hdr(bool state);

	void do_spin_exposure_num_changed(int val);

	void do_spin_min_z_changed(int val);

	void do_spin_max_z_changed(int val);

	void do_pushButton_connect();

	void do_pushButton_disconnect();

	void do_pushButton_capture_one_frame();

	void do_pushButton_capture_many_frame();

	void do_pushButton_test_accuracy();

	void do_pushButton_capture_continuous();

	void do_spin_led_current_changed(int val);
	  

	/******************************************************************************************/

	void do_pushButton_save_as();

	 

private:
	Ui::CameraCaptureGui ui;

	ProcessingDataStruct processing_settings_data_;

	//6个exposure输入框
	std::vector<QSpinBox*> exposure_time_list_;

	int radio_button_flag_;

	cv::Mat depth_map_;
	cv::Mat brightness_map_;
	cv::Mat render_image_brightness_;
	cv::Mat render_image_gray_depth_;
	cv::Mat render_image_color_depth_;


	//Camera 
	bool connected_flag_;
	int camera_width_;
	int camera_height_;

	//相机系统配置参数
	struct SystemConfigParam system_config_param_;
	//相机标定参数
	struct CameraCalibParam camera_calibration_param_;

	QString last_path_;
	QString sys_path_;
	 
	QThread* capture_thread_;

 

	bool start_timer_flag_;
	QTimer capture_timer_;

	QString camera_ip_;
 

};
