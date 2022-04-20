#include "settings_file_function.h"
#include "qsettings.h"
#include "qfile.h"
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QString>
#include <QDebug>
#include <QFile>
#include <QDateTime>
#include <QDir>

/***************************************************************************************************************************/
GuiConfigDataStruct GuiConfigDataStruct::instance_;
GuiConfigDataStruct::GuiConfigDataStruct()
{

}

bool GuiConfigDataStruct::loadFromSettings(const QString& f)
{

	//if (QFile::exists(f))
	//{
		QSettings settings(f, QSettings::IniFormat);
 
		instance_.low_z_value = settings.value("low_z_value", 400.0).toInt(); 
		instance_.high_z_value = settings.value("high_z_value", 600.0).toInt();
		instance_.ip = settings.value("ip", "").toString();
		 
		return true;
	//}
	//else
	//{
	//	return false;
	//}

}

bool GuiConfigDataStruct::saveToSettings(const QString& f)
{
	QSettings settings(f, QSettings::IniFormat);
	settings.setValue("low_z_value", Instance().low_z_value);
	settings.setValue("high_z_value", Instance().high_z_value);
	settings.setValue("ip", Instance().ip);
   
	return true;
}


/***************************************************************************************************************************/


SettingsFileFunction::SettingsFileFunction()
{
	gui_config_.Instance().ip = "";
	gui_config_.Instance().low_z_value = 300;
	gui_config_.Instance().high_z_value = 1200;
}

SettingsFileFunction::~SettingsFileFunction()
{
}


bool SettingsFileFunction::loadProcessingSettingsFile(QString path)
{
	//打开文件
	QFile file(path);
	file.open(QIODevice::ReadOnly);
	QByteArray data = file.readAll();
	file.close();


	QJsonParseError jsonParseError;
	QJsonDocument jsonDocument(QJsonDocument::fromJson(data, &jsonParseError));
	if (QJsonParseError::NoError != jsonParseError.error)
	{
		qDebug() << QString("JsonParseError: %1").arg(jsonParseError.errorString());
		return false;
	}

	QJsonObject rootObject = jsonDocument.object();
	QStringList rootKeys = rootObject.keys();
	qDebug()<<"roots keys: " << rootKeys << "\r\n";
	 
	//QStringList keys = rootObject.keys();
	//for (int i = 0; i < keys.size(); i++)
	//{
	//	qDebug() << "key" << i << " is:" << keys.at(i);
	//}

	if (rootObject.contains("firmware") && rootObject["firmware"].isObject())
	{
		QJsonObject firmware_Obj = rootObject["firmware"].toObject();

		if (firmware_Obj.contains("led_current") && firmware_Obj["led_current"].isDouble())
		{
			qDebug() << "led_current is:" << firmware_Obj.value("led_current").toInt();
			camera_config_.Instance().config_param_.led_current = firmware_Obj.value("led_current").toInt();
		}

		if (firmware_Obj.contains("exposure_num") && firmware_Obj["led_current"].isDouble())
		{
			qDebug() << "exposure_num is:" << firmware_Obj.value("exposure_num").toInt();
			camera_config_.Instance().config_param_.exposure_num = firmware_Obj.value("exposure_num").toInt();
		}

		if (firmware_Obj.contains("exposure_param") && firmware_Obj["exposure_param"].isArray())
		{
			QJsonArray j_array = firmware_Obj.value("exposure_param").toArray();

			if (6 == j_array.size())
			{ 
				qDebug() << "exposure_param is:" << firmware_Obj.value("exposure_param").toArray();

				for (int i = 0; i < 6; i++)
				{
					camera_config_.Instance().config_param_.exposure_param[i] = j_array[i].toInt();
				}

			}
		}

		if (firmware_Obj.contains("standard_plane_external_param") && firmware_Obj["standard_plane_external_param"].isArray())
		{
			QJsonArray j_array = firmware_Obj.value("standard_plane_external_param").toArray();

			if (12 == j_array.size())
			{
				qDebug() << "standard_plane_external_param is:" << j_array;
				for (int i = 0; i < 12; i++)
				{
					camera_config_.Instance().config_param_.standard_plane_external_param[i] = j_array[i].toDouble();
				}
			}

		}

		if (firmware_Obj.contains("camera_exposure_time") && firmware_Obj["camera_exposure_time"].isDouble())
		{
			qDebug() << "camera_exposure_time is:" << firmware_Obj.value("camera_exposure_time").toInt();
			camera_config_.Instance().config_param_.camera_exposure_time = firmware_Obj.value("camera_exposure_time").toInt();
		}

		
	}

	/******************************************************************************************************************************/


	if (rootObject.contains("gui") && rootObject["gui"].isObject())
	{
		QJsonObject gui_Obj = rootObject["gui"].toObject();

		if (gui_Obj.contains("low_z_value") && gui_Obj["low_z_value"].isDouble())
		{ 
			gui_config_.Instance().low_z_value = gui_Obj.value("low_z_value").toInt();
		}

		if (gui_Obj.contains("high_z_value") && gui_Obj["high_z_value"].isDouble())
		{
			gui_config_.Instance().high_z_value = gui_Obj.value("high_z_value").toInt();
		}

		if (gui_Obj.contains("ip") && gui_Obj["ip"].isString())
		{
			gui_config_.Instance().ip = gui_Obj.value("ip").toString();
		}
	}

















	/*********************************************************************************************************************************/

	

	return true;
}


bool SettingsFileFunction::saveProcessingSettingsFile(QString path)
{
	  
	QFile file(path);
	if (!file.open(QIODevice::ReadWrite)) {
		qDebug() << "File open error";
	}
	else {
		qDebug() << "File open!";
	}
	// 使用QJsonObject对象插入键值对。
	QJsonObject rootObject;


	/***************************************************************************************************************/
	//firmware config param

	QJsonObject jsonObject_firmware;
	jsonObject_firmware.insert("led_current", camera_config_.Instance().config_param_.led_current);
	jsonObject_firmware.insert("exposure_num", camera_config_.Instance().config_param_.exposure_num);

	QJsonArray exposure_param_array; 
	for (int i = 0; i < 6; i++)
	{
		exposure_param_array.append(camera_config_.Instance().config_param_.exposure_param[i]);
	}
	jsonObject_firmware.insert("exposure_param", exposure_param_array);


	QJsonArray standard_plane_external_param_array;
	for (int i = 0; i < 12; i++)
	{
		standard_plane_external_param_array.append(camera_config_.Instance().config_param_.standard_plane_external_param[i]);
	}

	jsonObject_firmware.insert("standard_plane_external_param", standard_plane_external_param_array);


	jsonObject_firmware.insert("camera_exposure_time", camera_config_.Instance().config_param_.camera_exposure_time);

	// 使用QJsonDocument设置该json对象
	QJsonDocument jsonDoc;
	rootObject.insert("firmware", jsonObject_firmware);

	/***********************************************************************************************************************************/

	QJsonObject jsonObject_gui;
	jsonObject_gui.insert("low_z_value", gui_config_.Instance().low_z_value);
	jsonObject_gui.insert("high_z_value", gui_config_.Instance().high_z_value);
	jsonObject_gui.insert("ip", gui_config_.Instance().ip);

	rootObject.insert("gui", jsonObject_gui);



	/************************************************************************************************************************************/

	jsonDoc.setObject(rootObject);

	// 将json以文本形式写入文件并关闭文件。
	file.write(jsonDoc.toJson());
	file.close();

	qDebug() << "Write to file: "<< path;
	return true;
}


void SettingsFileFunction::getGuiConfigData(struct GuiConfigDataStruct& param)
{
	param = gui_config_.Instance();
}

void SettingsFileFunction::setGuiConfigData(struct GuiConfigDataStruct param)
{
	gui_config_.Instance() = param.Instance();
}


void SettingsFileFunction::getFirmwareConfigData(struct SystemConfigParam& param)
{
	param = camera_config_.Instance().config_param_;
}

void SettingsFileFunction::setFirmwareConfigData(struct SystemConfigParam param)
{
	camera_config_.Instance().config_param_ = param;
}