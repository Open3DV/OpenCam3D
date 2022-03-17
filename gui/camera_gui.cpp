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

camera_gui::camera_gui(QWidget *parent)
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

}

camera_gui::~camera_gui()
{

}

void camera_gui::do_action_load_camera_config()
{ 
    QString path = QFileDialog::getOpenFileName(this, QString::fromLocal8Bit("加载配置文件"), last_config_path_, "*.json");

    if (path.isEmpty())
    {
        return;
    }

    last_config_path_ = path;

    bool ret =  ui.tab_capture->loadSettingData(path);
   

    if (ret)
    {
        QString log = QString::fromLocal8Bit("成功加载配置文件： ") + path;
        ui.tab_capture->addLogMessage(log);
    }
    else
    {
        QString log = QString::fromLocal8Bit("加载配置文件失败： ") + path;
        ui.tab_capture->addLogMessage(log);
    }

}

void camera_gui::do_action_save_camera_config()
{
      
    QString path = QFileDialog::getSaveFileName(this, QString::fromLocal8Bit("保存配置文件"), last_config_path_,"*.json");

    if (path.isEmpty())
    {
        return;
    }

    last_config_path_ = path;



    bool ret = ui.tab_capture->saveSettingData(path); 

    if (ret)
    {
        QString log = QString::fromLocal8Bit("保存配置文件： ") + path;
        ui.tab_capture->addLogMessage(log);
    }
    else
    {
        QString log = QString::fromLocal8Bit("保存配置文件失败： ") + path;
        ui.tab_capture->addLogMessage(log);
    }
}

void camera_gui::do_action_exit()
{
	this->close();
}

void camera_gui::do_update_temperature(float val)
{
    QString str = QString::number(val) + QString::fromLocal8Bit(" ℃");

    label_temperature_->setText(str);
}

void camera_gui::closeEvent(QCloseEvent* e)
{
    if (QMessageBox::question(this,
        QString::fromLocal8Bit("提示"),
        QString::fromLocal8Bit("确定退出软件？"),
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

