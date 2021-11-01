#include "camera_capture_gui.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "file_io_function.h"
#include <QDebug>
#include <iostream>
#include <QMouseEvent>
#include <QtWidgets/qfiledialog.h>



CameraCaptureGui::CameraCaptureGui(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);




	exposure_time_list_.clear();

	add_exposure_item(1.0); 
	 

	ui.tableWidget_more_exposure->setEditTriggers(QAbstractItemView::NoEditTriggers); //设置不可编辑
	ui.tableWidget_more_exposure->setFrameShape(QFrame::Box);

	qDebug() << "Capture";


	//cv::Mat img_b = cv::imread("G:/Code/GitCode/Df8/Df15_SDK/x64/Release/capture_data/frame03_data/0604/data_01.bmp", 0);
	//cv::Mat img_depth = cv::imread("G:/Code/GitCode/Df8/Df15_SDK/x64/Release/capture_data/frame03_data/0604/data_01.tiff", cv::IMREAD_UNCHANGED);


	processing_settings_data_.loadFromSettings("../processing_settings.ini");
	

	//brightness_map_ = img_b.clone();
	//depth_map_ = img_depth.clone();
	//render_image_brightness_ = brightness_map_.clone();
 

	renderBrightnessImage(brightness_map_);
	renderDepthImage(depth_map_);

	radio_button_flag_ = SELECT_BRIGHTNESS_FLAG_;
	showImage();

	setUiData();

	initializeFunction();

	last_path_ = "../TestData";
	sys_path_ = "../TestData";
	QDir dir(last_path_);
	QString path = dir.absolutePath();

	if (!dir.exists(path))
	{
		bool res = dir.mkpath(path); 
	}

	ui.tableWidget_more_exposure->hide();
	ui.spinBox_exposure_num->hide();
	ui.label_exposure_num->hide();
	ui.label_confidence->hide();
	ui.spinBox_confidence->hide();
	ui.pushButton_capture_many_frame->hide();
	ui.spinBox_multiframe->hide();

	ui.radioButton_depth_grey->hide();
	 
}

CameraCaptureGui::~CameraCaptureGui()
{
}


bool CameraCaptureGui::initializeFunction()
{
	/*******************************************************************************************************************/

	connect(ui.spinBox_exposure_num, SIGNAL(valueChanged(int)), this, SLOT(do_spin_exposure_num_changed(int)));
	connect(ui.spinBox_min_z, SIGNAL(valueChanged(int)), this, SLOT(do_spin_min_z_changed(int)));
	connect(ui.spinBox_max_z, SIGNAL(valueChanged(int)), this, SLOT(do_spin_max_z_changed(int)));

	connect(ui.spinBox_led, SIGNAL(valueChanged(int)), this, SLOT(do_spin_led_current_changed(int)));

	connect(ui.radioButton_brightness, SIGNAL(toggled(bool)), this, SLOT(on_QRadioButton_toggled_brightness(bool)));
	connect(ui.radioButton_depth_color, SIGNAL(toggled(bool)), this, SLOT(on_QRadioButton_toggled_color_depth(bool)));
	connect(ui.radioButton_depth_grey, SIGNAL(toggled(bool)), this, SLOT(on_QRadioButton_toggled_gray_depth(bool)));

	 

	connect(ui.pushButton_connect, SIGNAL(clicked()), this, SLOT(do_pushButton_connect()));
	connect(ui.pushButton_capture_one_frame, SIGNAL(clicked()), this, SLOT(do_pushButton_capture_one_frame()));
	connect(ui.pushButton_capture_continuous, SIGNAL(clicked()), this, SLOT(do_pushButton_capture_continuous()));
	connect(ui.pushButton_capture_many_frame, SIGNAL(clicked()), this, SLOT(do_pushButton_capture_many_frame()));


	connect(ui.pushButton_save_as, SIGNAL(clicked()), this, SLOT(do_pushButton_save_as()));

	connect(&capture_timer_, SIGNAL(timeout()), this, SLOT(do_timeout_slot()));
	capture_timer_.setInterval(1000);
	start_timer_flag_ = false;
	/**********************************************************************************************************************/




	return true;
}


void CameraCaptureGui::addLogMessage(QString str)
{

	QString StrCurrentTime =  QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");

	QString log = StrCurrentTime + " " + str;

	ui.textBrowser_log->append(log);

	ui.textBrowser_log->repaint();
}

bool CameraCaptureGui::saveOneFrameData(QString path_name)
{
	if (path_name.isEmpty())
	{
		return false;
	}

	QDir dir(path_name);
	path_name = dir.absolutePath();


	addLogMessage(QString::fromLocal8Bit("保存亮度图："));
	QString brightness_str = path_name + ".bmp";
	cv::imwrite(brightness_str.toStdString(), brightness_map_);
	addLogMessage(brightness_str);

	addLogMessage(QString::fromLocal8Bit("保存深度图："));
	QString depth_str = path_name + ".tiff";
	cv::imwrite(depth_str.toStdString(), depth_map_);
	addLogMessage(depth_str);


	addLogMessage(QString::fromLocal8Bit("保存点云："));
	QString points_str = path_name + ".xyz"; 
	cv::Mat points_map(brightness_map_.size(), CV_32FC3, cv::Scalar(0., 0., 0.));

	depthTransformPointcloud((float*)depth_map_.data, (float*)points_map.data);


	FileIoFunction file_io_machine;
	file_io_machine.SavePointToTxt(points_map, points_str, brightness_map_);
	addLogMessage(points_str);

	return true;
}

bool CameraCaptureGui::saveSettingData(QString path)
{

	return processing_settings_data_.saveToSettings(path);
	
}


bool CameraCaptureGui::renderBrightnessImage(cv::Mat brightness)
{
	
	cv::Mat color_map(brightness.size(), CV_8UC3, cv::Scalar(0, 0, 0));

	int nr = color_map.rows;
	int nc = color_map.cols; 

	for (int r = 0; r < nr; r++)
	{
		uchar* ptr_b = brightness.ptr<uchar>(r);
		cv::Vec3b* ptr_cb = color_map.ptr<cv::Vec3b>(r);
		for (int c = 0; c < nc; c++)
		{
			if (ptr_b[c] == 255)
			{
				ptr_cb[c][0] = 255;
				ptr_cb[c][1] = 0;
				ptr_cb[c][2] = 0;
			}
			else
			{
				ptr_cb[c][0] = ptr_b[c];
				ptr_cb[c][1] = ptr_b[c];
				ptr_cb[c][2] = ptr_b[c];
			}
		}

	}

	render_image_brightness_ = color_map.clone();
	return true;
}

bool CameraCaptureGui::renderDepthImage(cv::Mat depth)
{
	if (depth.empty())
	{
		return false;
	}


	FileIoFunction io_machine;

	int low_z = processing_settings_data_.Instance().low_z_value;
	int high_z = processing_settings_data_.Instance().high_z_value;

	io_machine.depthToColor(depth, render_image_color_depth_, render_image_gray_depth_, low_z, high_z);

	return true;

}

bool CameraCaptureGui::setShowImages(cv::Mat brightness, cv::Mat depth)
{
	cv::Mat img_color;
	cv::Mat gray_map;
	FileIoFunction io_machine;

	int low_z = processing_settings_data_.Instance().low_z_value;
	int high_z = processing_settings_data_.Instance().high_z_value;

	io_machine.depthToColor(depth, img_color, gray_map, low_z, high_z);


	return true;
}


void CameraCaptureGui::setSettingData(ProcessingDataStruct& settings_data_)
{
	processing_settings_data_ = settings_data_;
}



void CameraCaptureGui::undateSystemConfigUiData()
{
	ui.spinBox_led->setValue(system_config_param_.led_current);
}

void CameraCaptureGui::setUiData()
{
	ui.spinBox_min_z->setValue(processing_settings_data_.Instance().low_z_value);
	ui.spinBox_max_z->setValue(processing_settings_data_.Instance().high_z_value);
	ui.lineEdit_ip->setText(processing_settings_data_.Instance().ip);
}


double CameraCaptureGui::get_exposure_item_value(int row)
{
	if (row > ui.tableWidget_more_exposure->rowCount() - 1)
		return -1;

	return exposure_time_list_[row]->value();
 
}


bool CameraCaptureGui::remove_exposure_item(int row)
{

	int item_count = ui.tableWidget_more_exposure->rowCount();

	if (row > item_count - 1)
		return false;

	ui.tableWidget_more_exposure->removeRow(row);

	exposure_time_list_.erase(exposure_time_list_.begin() + row);

	return true;
}

void CameraCaptureGui::add_exposure_item(double val)
{

	int item_count = ui.tableWidget_more_exposure->rowCount(); 

	ui.tableWidget_more_exposure->setRowCount(item_count+1); 

	QDoubleSpinBox *upperSpinBoxItem = new QDoubleSpinBox();
	upperSpinBoxItem->setRange(0, 99);//设置数值显示范围
	upperSpinBoxItem->setValue(val);
	ui.tableWidget_more_exposure->setItem(item_count, 0, new QTableWidgetItem(QString::number(item_count+1)));
	ui.tableWidget_more_exposure->setCellWidget(item_count, 1, upperSpinBoxItem);//i为所在行，j+2为所在列

	ui.tableWidget_more_exposure->item(item_count, 0)->setTextAlignment(Qt::AlignHCenter | Qt::AlignCenter);
	ui.tableWidget_more_exposure->item(item_count, 0)->setSelected(false);

	exposure_time_list_.push_back(upperSpinBoxItem);
}

void CameraCaptureGui::do_spin_min_z_changed(int val)
{
	processing_settings_data_.Instance().low_z_value = val;

	renderDepthImage( depth_map_);
	showImage();

	//cv::Mat img_b = cv::imread("G:/Code/GitCode/Df8/Df15_SDK/x64/Release/capture_data/frame03_data/0604/data_01.bmp", 0);
	//cv::Mat img_depth = cv::imread("G:/Code/GitCode/Df8/Df15_SDK/x64/Release/capture_data/frame03_data/0604/data_01.tiff", cv::IMREAD_UNCHANGED);
	//setShowImages(img_b, img_depth);
 
}





void CameraCaptureGui::do_spin_led_current_changed(int val)
{
	if (connected_flag_)
	{
		system_config_param_.led_current = val;

		int ret_code = DfSetSystemConfigParam(system_config_param_);
		if (0 != ret_code)
		{
			qDebug() << "Get Param Error;";
			return;
		}

		QString str = QString::fromLocal8Bit("设置投影亮度: ") + QString::number(val);

		addLogMessage(str);
	}
}

void CameraCaptureGui::do_spin_max_z_changed(int val)
{

	processing_settings_data_.Instance().high_z_value = val;


	renderDepthImage(depth_map_);
	showImage();

	//cv::Mat img_b = cv::imread("G:/Code/GitCode/Df8/Df15_SDK/x64/Release/capture_data/frame03_data/0604/data_01.bmp", 0);
	//cv::Mat img_depth = cv::imread("G:/Code/GitCode/Df8/Df15_SDK/x64/Release/capture_data/frame03_data/0604/data_01.tiff", cv::IMREAD_UNCHANGED);
	//setShowImages(img_b, img_depth);
}

void CameraCaptureGui::do_spin_exposure_num_changed(int val)
{
	qDebug() << "val: " << val;

	int item_num = ui.tableWidget_more_exposure->rowCount();

	if(val == item_num)
	{
		return;
	}

	if(val< item_num)
	{ 

		for(int row= item_num-1; row>= val; row--)
		{
			remove_exposure_item(row); 
		}
	}
	else
	{

		for(int row= item_num;row< val;row++)
		{ 
			double val = get_exposure_item_value(row-1);
			add_exposure_item(val + 1.0);
		}

	}

	//ui.tableWidget_more_exposure->scrollToTop();
	//ui.tableWidget_more_exposure->scrollToBottom();
	ui.tableWidget_more_exposure->repaint();
	ui.verticalLayout->update();
}

/*************************************************************************************************************************************/

bool CameraCaptureGui::connectCamera(QString ip)
{

	return true;
}


bool CameraCaptureGui::capture_brightness()
{
	if (!connected_flag_)
	{
		return false;
	}

	int width = camera_width_;
	int height = camera_height_; 

 
	 

	int image_size = width * height;

	cv::Mat brightness(cv::Size(width, height), CV_8U,cv::Scalar(0));
	unsigned char* brightness_buf = (unsigned char*)brightness.data;

	int ret_code = DfGetCameraData(0, 0,brightness_buf, image_size,0, 0,0, 0);
	 

	if (0 == ret_code)
	{
		brightness_map_ = brightness.clone();
		return true;
	}

	return false;
}

bool CameraCaptureGui::capture_one_frame_data()
{

	if (!connected_flag_)
	{
		return false;
	}

	int width = camera_width_;
	int height = camera_height_;

	//分配内存保存采集结果
	//float* point_cloud_data = (float*)malloc(sizeof(float) * width * height * 3);
	//memset(point_cloud_data, 0, sizeof(float) * width * height * 3);

	//ushort* depth_data = (ushort*)malloc(sizeof(ushort) * width * height);
	//memset(depth_data, 0, sizeof(ushort) * width * height);

	//char* timestamp_data = (char*)malloc(sizeof(uchar) * 30);
	//memset(timestamp_data, 0, sizeof(uchar) * 30);

	//uchar* brightness_data = (uchar*)malloc(sizeof(uchar) * width * height);
	//memset(brightness_data, 0, sizeof(uchar) * width * height);
 

	addLogMessage(QString::fromLocal8Bit("采集数据："));
 

	/*****************************************************************************/
	int image_size = width * height;

	cv::Mat brightness(height, width, CV_8U, cv::Scalar(0));
	cv::Mat depth(height, width, CV_32F, cv::Scalar(0.));

	int depth_buf_size = image_size * 1 * 4;
	float* depth_buf = (float*)(new char[depth_buf_size]);

	int brightness_bug_size = image_size;
	unsigned char* brightness_buf = new unsigned char[brightness_bug_size];

	int ret_code = 0;

	if (ui.checkBox_hdr->isChecked())
	{ 
		ret_code = DfGetFrameHdr((float*)depth.data, depth_buf_size, (uchar*)brightness.data, brightness_bug_size);
	}
	else
	{ 
		ret_code = DfGetFrame03((float*)depth.data, depth_buf_size, (uchar*)brightness.data, brightness_bug_size);
	}




	/***************************************************************************/
	 

	if (0 == ret_code)
	{
		brightness_map_ = brightness.clone();
		depth_map_ = depth.clone();

		addLogMessage(QString::fromLocal8Bit("采集完成！"));
		return true;
	}

	addLogMessage(QString::fromLocal8Bit("采集失败！"));
	return false;
	 

	//int ret_code = DfCaptureData(1, timestamp_data);

 
		
 

		//if (0 == ret_code)
		//{
		//	DfGetBrightnessData(brightness_data);
		//	cv::Mat test_brightness(height, width, CV_8U, brightness_data);

		//	brightness_map_ = test_brightness.clone();

		//	//cv::imwrite("./demo_brightness.bmp", test_brightness);

		//	DfGetDepthData(depth_data);

		//	cv::Mat test_depth(height, width, CV_16U, depth_data);
		//	 
		//	test_depth /= 10;

		//	test_depth.convertTo(test_depth, CV_32FC1);


		//	depth_map_ = test_depth.clone();


		//	//cv::imwrite("./demo_depth.tiff", test_depth);

		//	//DfGetPointcloudData(point_cloud_data);
		//	//cv::Mat test_point_cloud(height, width, CV_32FC3, point_cloud_data);

		//	free(brightness_data);
		//	free(depth_data);
		//	free(timestamp_data);

		//	return true;

		//}
		//else
		//{
		//	free(brightness_data);
		//	free(depth_data);
		//	free(timestamp_data);
		//	return false;
		//}
	 
}

void  CameraCaptureGui::do_pushButton_connect()
{



	if (!connected_flag_)
	{

		camera_ip_ = ui.lineEdit_ip->text();
		if (camera_ip_.isEmpty())
		{
			addLogMessage(QString::fromLocal8Bit("请设置IP！"));
			return;
		}


		addLogMessage(QString::fromLocal8Bit("连接相机："));
		int ret_code = DfConnect(camera_ip_.toStdString().c_str());

		if (0 == ret_code)
		{
			//必须连接相机成功后，才可获取相机分辨率
			ret_code = DfGetCameraResolution(&camera_width_, &camera_height_);
			std::cout << "Width: " << camera_width_ << "    Height: " << camera_height_ << std::endl;

			if (0 != ret_code)
			{
				qDebug() << "Connect Error!;";
				return;
			}

			addLogMessage(QString::fromLocal8Bit("连接相机成功！"));
			//保存ip配置
			processing_settings_data_.Instance().ip = camera_ip_;

			ret_code = DfGetSystemConfigParam(system_config_param_);
			if (0 != ret_code)
			{
				qDebug() << "Get Param Error;";
				return;
			}

			undateSystemConfigUiData();
		}
		else
		{
			std::cout << "Connect Camera Error!";
			addLogMessage(QString::fromLocal8Bit("连接相机失败！"));
			return;
		}


		connected_flag_ = true;

		CalibrationParam calib_param;
		ret_code = DfGetCalibrationParam(&calib_param);

		if (0 == ret_code)
		{
			std::cout << "intrinsic: " << std::endl;
			for (int r = 0; r < 3; r++)
			{
				for (int c = 0; c < 3; c++)
				{
					std::cout << calib_param.intrinsic[3 * r + c] << "\t";
				}
				std::cout << std::endl;
			}

			std::cout << "extrinsic: " << std::endl;
			for (int r = 0; r < 4; r++)
			{
				for (int c = 0; c < 4; c++)
				{
					std::cout << calib_param.extrinsic[4 * r + c] << "\t";
				}
				std::cout << std::endl;
			}

			std::cout << "distortion: " << std::endl;
			for (int r = 0; r < 1; r++)
			{
				for (int c = 0; c < 12; c++)
				{
					std::cout << calib_param.distortion[1 * r + c] << "\t";
				}
				std::cout << std::endl;
			}
		}
		else
		{
			std::cout << "Get Calibration Data Error!";
			return;
		}

		ui.pushButton_connect->setIcon(QIcon(":/dexforce_camera_gui/image/disconnect.png"));
	}
	else
	{
		//断开相机
		do_pushButton_disconnect();

		ui.pushButton_connect->setIcon(QIcon(":/dexforce_camera_gui/image/connect.png"));
	}



}

void  CameraCaptureGui::do_pushButton_disconnect()
{
	
	if (connected_flag_)
	{
		DfDisconnect(camera_ip_.toStdString().c_str());
		addLogMessage(QString::fromLocal8Bit("断开相机！")); 
		connected_flag_ = false;
	}


}


void CameraCaptureGui::do_pushButton_save_as()
{


	QString StrCurrentTime = "/" + QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss");

	QString path = QFileDialog::getSaveFileName(this, "Set Save Name",last_path_ + StrCurrentTime);

	if (path.isEmpty())
	{
		return;
	}

	last_path_ = path;

	QStringList str_list = path.split(".");

	QString name = str_list[0];

	saveOneFrameData(name);

	//QString brightness_str = name + ".bmp";
	//cv::imwrite(brightness_str.toStdString(), brightness_map_);

	//QString depth_str = name + ".tiff";
	//cv::imwrite(depth_str.toStdString(), depth_map_);


	//QString points_str = name + ".txt";

	//cv::Mat points_map(brightness_map_.size(), CV_32FC3, cv::Scalar(0., 0., 0.));

	//DfDepthTransformPointcloud(depth_map_, points_map);
	// 

	//FileIoFunction file_io_machine;
	//file_io_machine.SavePointToTxt(points_map, points_str, brightness_map_);
	qDebug() << name;

}


bool CameraCaptureGui::capture_one_frame_and_render()
{
	bool ret = capture_one_frame_data(); 


	if (ret)
	{

		renderBrightnessImage(brightness_map_);
		renderDepthImage(depth_map_);
		showImage();

		if (ui.checkBox_auto_save->isChecked())
		{
			QString StrCurrentTime = "/" + QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss");
			QString path_name = sys_path_ + StrCurrentTime;
			saveOneFrameData(path_name);
		}
	}
	else
	{
		return false;
	}

	return true;
}


void  CameraCaptureGui::do_pushButton_capture_one_frame()
{
	if (!connected_flag_)
	{
		return;
	}

 

	capture_one_frame_and_render();
	 




}

 
 

void  CameraCaptureGui::do_timeout_slot()
{

	capture_timer_.stop();

	if (start_timer_flag_)
	{
		//do_pushButton_capture_one_frame();


		capture_one_frame_and_render();

		//bool ret =capture_brightness();


		//if (ret)
		//{

		//	renderBrightnessImage(brightness_map_);
		//	showImage();
		//}

		//qDebug() << "Timer";
		capture_timer_.start();

	}


}

void  CameraCaptureGui::do_pushButton_capture_many_frame()
{


}

void  CameraCaptureGui::do_pushButton_capture_continuous()
{

	if (start_timer_flag_)
	{
		start_timer_flag_ = false;

		ui.pushButton_capture_continuous->setIcon(QIcon(":/dexforce_camera_gui/image/video_start.png"));
	}
	else
	{
		start_timer_flag_ = true;
		capture_timer_.start();

		ui.pushButton_capture_continuous->setIcon(QIcon(":/dexforce_camera_gui/image/video_stop.png"));
	}

	//capture_thread_ = new QThread(this);
	//capture_thread_->start();

	//capture_timer_ = new QTimer(0);
	//capture_timer_->setInterval(100);
	//capture_timer_->moveToThread(capture_thread_);
	//connect(capture_timer_, SIGNAL(timeout()), this, SLOT(workSlot()), Qt::DirectConnection);
	//connect(capture_thread_, SIGNAL(started()), capture_timer_, SLOT(start()));


	//disconnect(capture_timer_, SIGNAL(timeout()), this, SLOT(workSlot()));
	//disconnect(capture_thread_, SIGNAL(started()), capture_timer_, SLOT(start()));
	//capture_timer_->stop();
	//delete capture_timer_;
}



/*****************************************************************************************************************************/
//显示相关
void CameraCaptureGui::on_QRadioButton_toggled_brightness(bool state)
{
	if (state)
	{
		radio_button_flag_ = SELECT_BRIGHTNESS_FLAG_;
		//qDebug() << "state: " << radio_button_flag_;
		showImage();
	}
}

void CameraCaptureGui::on_QRadioButton_toggled_color_depth(bool state)
{
	if (state)
	{
		radio_button_flag_ = SELECT_COLOR_DEPTH_FLAG_;
		//qDebug() << "state: " << radio_button_flag_;
		showImage();
	}
}

void CameraCaptureGui::on_QRadioButton_toggled_gray_depth(bool state)
{
	if (state)
	{
		radio_button_flag_ = SELECT_GRAY_DEPTH_FLAG_;
		//qDebug() << "state: " << radio_button_flag_;
		showImage();
	}
}



bool CameraCaptureGui::showImage()
{
	switch (radio_button_flag_)
	{
	case 1:
	{
		setImage(render_image_brightness_);
	}
	break;

	case 2:
	{
		setImage(render_image_gray_depth_);
	}
	break;

	case 3:
	{
		setImage(render_image_color_depth_);
	}
	break;

	default:
		break;
	}

	return true;
}

bool CameraCaptureGui::setImage(cv::Mat img)
{
	cv::Mat show_img = img.clone();

	if (img.empty())
	{
		return false;
	}

	QImage qimg;

	if (show_img.channels() == 3)//RGB Img
	{
		cv::Mat Rgb;
		cv::cvtColor(show_img, Rgb, cv::COLOR_BGR2RGB);//颜色空间转换
		//qimg = QImage((const uchar*)(Rgb.data), Rgb.cols, Rgb.rows, Rgb.step, QImage::Format_RGB888);

		qimg = QImage(show_img.data, show_img.cols, show_img.rows, show_img.step, QImage::Format_RGB888);

	}
	else//Gray Img
	{
		if (CV_8UC1 == show_img.type())
		{
			qimg = QImage((const uchar*)(show_img.data), show_img.cols, show_img.rows, show_img.cols * show_img.channels(), QImage::Format_Indexed8);
		}
		else if (CV_32FC1 == show_img.type())
		{

			show_img.convertTo(show_img, CV_8U);

			qimg = QImage((const uchar*)(show_img.data), show_img.cols, show_img.rows, show_img.cols * show_img.channels(), QImage::Format_Indexed8);

		}

	}



	//pixmap_ = QPixmap::fromImage(qimg);
	 

	ui.label_image->setPixmap(QPixmap::fromImage(qimg));
	ui.label_image->setScaledContents(true);
	//ui.label_image->setPixmap(pixmap_.scaled(ui.label_image->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));  // 保持比例 & 平滑缩放(无锯齿)

	return true;
}


/************************************************************************************************************************************/