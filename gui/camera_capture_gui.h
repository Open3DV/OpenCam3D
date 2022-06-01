#pragma once

#include <QWidget>
#include "ui_camera_capture_gui.h"
#include "settings_file_function.h"
#include <opencv2/core.hpp>
#include "../sdk/open_cam3d.h"
//#include "../firmware/system_config_settings.h"
#include "../firmware/protocol.h"
#include <QThread>
#include <QDebug>
#include <QtCore/QTimer>

#define SELECT_BRIGHTNESS_FLAG_ 1;
#define SELECT_HEIGHT_MAP_FLAG_ 2;
#define SELECT_COLOR_DEPTH_FLAG_ 3;

#define GENERATE_BRIGHTNESS_DEFAULT_ 1;
#define GENERATE_BRIGHTNESS_ILLUMINATION_ 2;
#define GENERATE_BRIGHTNESS_DARKNESS_ 3;

class CameraCaptureGui : public QWidget
{
	Q_OBJECT

public:
	CameraCaptureGui(QWidget* parent = Q_NULLPTR);
	~CameraCaptureGui();

	void setOnDrop(int (*p_function)(void*));

	bool setShowImages(cv::Mat brightness, cv::Mat depth);

	void setGuiSettingData(GuiConfigDataStruct& settings_data_);

	bool saveSettingData(QString path);

	bool loadSettingData(QString path);

	void setUiData();

	bool connectCamera(QString ip);

	bool stopCapturingOneFrameBaseThread();

	void captureOneFrameBaseThread(bool hdr);

	bool captureOneFrameData();

	bool captureOneFrameAndRender();

	bool captureBrightness();

	bool initializeFunction();

	bool saveOneFrameData(QString path_name);

	void addLogMessage(QString str);

	//更新多曝光参数
	void updateManyExposureParam();

	bool manyExposureParamHasChanged();

	bool isConnect();

	bool setCameraConfigParam();

	bool getCameraConfigParam();

	void sleep(int sectime);

	bool getShowCalibrationMessage(struct SystemConfigParam& config_param, struct CameraCalibParam& calibration_param);

	void getGuiConfigParam(struct GuiConfigDataStruct& gui_param);

	//更新生成亮度图参数
	void updateGenerateBrightnessParam();

	void setCalibrationBoard(int flag);

private:
	bool showImage();

	bool setImage(cv::Mat img);

	bool renderHeightImage(cv::Mat height);

	bool renderDepthImage(cv::Mat depth);

	bool renderBrightnessImage(cv::Mat brightness);

	void undateSystemConfigUiData();

	double computePointsDistance(cv::Point2f p_0, cv::Point2f p_1, cv::Mat point_cloud);

	bool bilinearInterpolationFeaturePoints(std::vector<cv::Point2f> feature_points, std::vector<cv::Point3f>& point_3d, cv::Mat point_cloud);


signals:
	void send_temperature_update(float val);

	void send_images_update();

public slots:

	void do_timeout_capture_slot();

	void do_undate_show_slot();

	void do_pushButton_connect();

	void do_pushButton_disconnect();

private slots:
	void do_QRadioButton_toggled_brightness(bool state);

	void do_QRadioButton_toggled_color_depth(bool state);

	void do_QRadioButton_toggled_gray_depth(bool state);

	void do_QRadioButton_toggled_generate_brightness_default(bool state);

	void do_QRadioButton_toggled_generate_brightness_illumination(bool state);

	void do_QRadioButton_toggled_generate_brightness_darkness(bool state);

	void add_exposure_item(int row, int exposure, int led);

	bool remove_exposure_item(int row);

	double get_exposure_item_value(int row);

private slots:
	void do_checkBox_toggled_bilateral_filter(bool state);

	void do_checkBox_toggled_hdr(bool state);

	void do_spin_exposure_num_changed(int val);

	void do_spin_min_z_changed(int val);

	void do_spin_max_z_changed(int val);

	void do_pushButton_capture_one_frame();

	void do_pushButton_test_accuracy();

	void do_pushButton_calibrate_external_param();

	void do_pushButton_capture_continuous();

	void do_spin_led_current_changed(int val);

	void do_spin_camera_exposure_changed(int val);

	void do_spin_generate_brightness_exposure_changed(int val);
	/******************************************************************************************/

	void do_pushButton_save_as();

	void do_pushButton_open_folder();



private:
	Ui::CameraCaptureGui ui;

	bool capture_show_flag_;


	//std::mutex	capture_m_mutex_;
	bool capturing_flag_;
	bool camera_setting_flag_;

	//6个exposure输入框
	std::vector<QSpinBox*> exposure_time_list_;
	std::vector<QSpinBox*> led_current_list_;

	int radio_button_flag_;

	cv::Mat depth_map_;
	cv::Mat brightness_map_;
	cv::Mat height_map_;
	cv::Mat render_image_brightness_;
	cv::Mat render_image_gray_depth_;
	cv::Mat render_image_color_depth_;
	cv::Mat render_image_color_height_;

	int min_depth_value_;
	int max_depth_value_;


	//Camera 
	bool connected_flag_;
	int camera_width_;
	int camera_height_;


	GuiConfigDataStruct processing_gui_settings_data_;
	SettingsFileFunction config_system_param_machine_;

	//相机系统配置参数
	struct SystemConfigParam system_config_param_;
	struct FirmwareConfigParam firmware_config_param_;
	//相机标定参数
	struct CameraCalibParam camera_calibration_param_;

	QString last_path_;
	QString sys_path_;

	QThread* capture_thread_;

	bool start_timer_flag_;
	QTimer capture_timer_;
	QString camera_ip_;

	std::vector<cv::Point3f> center_points_list_;
	std::vector<float> rms_list_;
	std::vector<std::vector<float>> plane_list_;
	std::vector<std::vector<cv::Point3f>> feature_points_list_;

	//生成亮度图模式
	int generate_brightness_model_;
	float generate_brightness_exposure_;

	cv::Size2f board_size_;

	int calibration_board_flag_;
	int camera_version_;


	int (*m_p_OnDropped_)(void*);
};
