#pragma once

#include <QMainWindow>
#include "ui_camera_gui.h"
#include "settings_file_function.h"
#include "opencv2/core.hpp"
#include "calibration_param_gui.h"

class camera_gui : public QMainWindow
{
    Q_OBJECT

public:
    camera_gui(QWidget* parent = Q_NULLPTR);
    ~camera_gui();

    bool setSettingsData(GuiConfigDataStruct& settings_data);

    bool setShowImages(cv::Mat brightness, cv::Mat depth);

public slots:
    void do_update_temperature(float val);

    void do_action_load_camera_config();

    void do_action_save_camera_config();

    void do_action_exit();

    void do_action_show_calibration_param();
protected:
    void closeEvent(QCloseEvent * e);

private:
    Ui::camera_gui ui;

    CalibrationParamGui show_calib_param_gui_;

    bool setUiData();

    GuiConfigDataStruct processing_gui_settings_data_;

    QLabel* label_temperature_;

    QString default_config_path_;
    QString last_config_path_;
};