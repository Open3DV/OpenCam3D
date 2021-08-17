#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_dexforce_camera_gui.h"

#include "SettingsFileFunction.h"

class dexforce_camera_gui : public QMainWindow
{
    Q_OBJECT

public:
    dexforce_camera_gui(QWidget *parent = Q_NULLPTR);

    bool setSettingsData(ProcessingDataStruct& settings_data);

    bool setShowImages(cv::Mat brightness, cv::Mat depth);

protected:
    void closeEvent(QCloseEvent* e);

private:
    Ui::dexforce_camera_guiClass ui;

    bool setUiData();

    ProcessingDataStruct processing_settings_data_;
};
