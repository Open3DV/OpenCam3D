#pragma once

#include <QMainWindow>
#include "ui_camera_gui.h"
#include "settings_file_function.h"
#include "opencv2/core.hpp"

class camera_gui : public QMainWindow
{
    Q_OBJECT

public:
    camera_gui(QWidget* parent = Q_NULLPTR);
    ~camera_gui();

    bool setSettingsData(ProcessingDataStruct & settings_data);

    bool setShowImages(cv::Mat brightness, cv::Mat depth);

protected:
    void closeEvent(QCloseEvent * e);

private:
    Ui::camera_gui ui;

    bool setUiData();

    ProcessingDataStruct processing_settings_data_;
};