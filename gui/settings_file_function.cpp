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
	gui_config_.Instance().ip = "192.168.0.100";
	gui_config_.Instance().low_z_value = 300;
	gui_config_.Instance().high_z_value = 3000;
	gui_config_.Instance().calibration_board = 20;
	gui_config_.Instance().repetition_count = 0;

	firmware_config_param_.generate_brightness_exposure = 12000;
	firmware_config_param_.generate_brightness_model = 1;
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
	qDebug() << "roots keys: " << rootKeys << "\r\n";

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

		if (firmware_Obj.contains("camera_exposure_time") && firmware_Obj["camera_exposure_time"].isDouble())
		{
			qDebug() << "camera_exposure_time is:" << firmware_Obj.value("camera_exposure_time").toInt();
			camera_config_.Instance().config_param_.camera_exposure_time = firmware_Obj.value("camera_exposure_time").toInt();
		}

		if (firmware_Obj.contains("camera_gain") && firmware_Obj["camera_gain"].isDouble())
		{
			qDebug() << "camera_gain is:" << firmware_Obj.value("camera_gain").toInt();
			camera_config_.Instance().config_param_.camera_gain = firmware_Obj.value("camera_gain").toInt();
		}

		if (firmware_Obj.contains("mixed_exposure_num") && firmware_Obj["mixed_exposure_num"].isDouble())
		{
			qDebug() << "camera_exposure_time is:" << firmware_Obj.value("mixed_exposure_num").toInt();
			camera_config_.Instance().firwmare_param_.mixed_exposure_num = firmware_Obj.value("mixed_exposure_num").toInt();
		}

		if (firmware_Obj.contains("mixed_exposure_param_list") && firmware_Obj["mixed_exposure_param_list"].isArray())
		{
			QJsonArray j_array = firmware_Obj.value("mixed_exposure_param_list").toArray();

			if (6 == j_array.size())
			{
				qDebug() << "mixed_exposure_param_list is:" << firmware_Obj.value("mixed_exposure_param_list").toArray();

				for (int i = 0; i < 6; i++)
				{
					camera_config_.Instance().firwmare_param_.mixed_exposure_param_list[i] = j_array[i].toInt();
				}

			}
		}

		if (firmware_Obj.contains("mixed_led_param_list") && firmware_Obj["mixed_led_param_list"].isArray())
		{
			QJsonArray j_array = firmware_Obj.value("mixed_led_param_list").toArray();

			if (6 == j_array.size())
			{
				qDebug() << "mixed_led_param_list is:" << firmware_Obj.value("mixed_led_param_list").toArray();

				for (int i = 0; i < 6; i++)
				{
					camera_config_.Instance().firwmare_param_.mixed_led_param_list[i] = j_array[i].toInt();
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

		if (firmware_Obj.contains("generate_brightness_model") && firmware_Obj["generate_brightness_model"].isDouble())
		{
			qDebug() << "generate_brightness_model is:" << firmware_Obj.value("generate_brightness_model").toInt();
			camera_config_.Instance().firwmare_param_.generate_brightness_model = firmware_Obj.value("generate_brightness_model").toInt();
		}

		if (firmware_Obj.contains("generate_brightness_exposure") && firmware_Obj["generate_brightness_exposure"].isDouble())
		{
			qDebug() << "generate_brightness_exposure is:" << firmware_Obj.value("generate_brightness_exposure").toInt();
			camera_config_.Instance().firwmare_param_.generate_brightness_exposure = firmware_Obj.value("generate_brightness_exposure").toInt();
		}

		if (firmware_Obj.contains("use_bilateral_filter") && firmware_Obj["use_bilateral_filter"].isDouble())
		{
			qDebug() << "use_bilateral_filter is:" << firmware_Obj.value("use_bilateral_filter").toInt();
			camera_config_.Instance().firwmare_param_.use_bilateral_filter = firmware_Obj.value("use_bilateral_filter").toInt();
		}

		if (firmware_Obj.contains("bilateral_filter_param_d") && firmware_Obj["bilateral_filter_param_d"].isDouble())
		{
			qDebug() << "bilateral_filter_param_d is:" << firmware_Obj.value("bilateral_filter_param_d").toInt();
			camera_config_.Instance().firwmare_param_.bilateral_filter_param_d = firmware_Obj.value("bilateral_filter_param_d").toInt();
		}

		if (firmware_Obj.contains("confidence") && firmware_Obj["confidence"].isDouble())
		{
			qDebug() << "confidence is:" << firmware_Obj.value("confidence").toInt();
			camera_config_.Instance().firwmare_param_.confidence = firmware_Obj.value("confidence").toInt();
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

		if (gui_Obj.contains("use_hdr_model") && gui_Obj["use_hdr_model"].isBool())
		{
			gui_config_.Instance().use_hdr_model = gui_Obj.value("use_hdr_model").toBool();
		}

		if (gui_Obj.contains("calibration_board") && gui_Obj["calibration_board"].isDouble())
		{
			gui_config_.Instance().calibration_board = gui_Obj.value("calibration_board").toInt();
		}

		if (gui_Obj.contains("repetition_count") && gui_Obj["repetition_count"].isDouble())
		{
			gui_config_.Instance().repetition_count = gui_Obj.value("repetition_count").toInt();
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
	jsonObject_firmware.insert("camera_exposure_time", camera_config_.Instance().config_param_.camera_exposure_time);

	QJsonArray exposure_param_array;
	for (int i = 0; i < 6; i++)
	{
		exposure_param_array.append(camera_config_.Instance().firwmare_param_.mixed_exposure_param_list[i]);
	}
	jsonObject_firmware.insert("mixed_exposure_param_list", exposure_param_array);

	QJsonArray led_param_array;
	for (int i = 0; i < 6; i++)
	{
		led_param_array.append(camera_config_.Instance().firwmare_param_.mixed_led_param_list[i]);
	}
	jsonObject_firmware.insert("mixed_led_param_list", led_param_array);
	jsonObject_firmware.insert("mixed_exposure_num", camera_config_.Instance().firwmare_param_.mixed_exposure_num);

	QJsonArray standard_plane_external_param_array;
	for (int i = 0; i < 12; i++)
	{
		standard_plane_external_param_array.append(camera_config_.Instance().config_param_.standard_plane_external_param[i]);
	}

	jsonObject_firmware.insert("standard_plane_external_param", standard_plane_external_param_array);


	jsonObject_firmware.insert("camera_exposure_time", camera_config_.Instance().config_param_.camera_exposure_time);
	jsonObject_firmware.insert("camera_gain", camera_config_.Instance().config_param_.camera_gain);


	jsonObject_firmware.insert("generate_brightness_model", camera_config_.Instance().firwmare_param_.generate_brightness_model);
	jsonObject_firmware.insert("generate_brightness_exposure", camera_config_.Instance().firwmare_param_.generate_brightness_exposure);

	jsonObject_firmware.insert("use_bilateral_filter", camera_config_.Instance().firwmare_param_.use_bilateral_filter);
	jsonObject_firmware.insert("bilateral_filter_param_d", camera_config_.Instance().firwmare_param_.bilateral_filter_param_d);

	jsonObject_firmware.insert("confidence", camera_config_.Instance().firwmare_param_.confidence);

	// 使用QJsonDocument设置该json对象
	QJsonDocument jsonDoc;
	rootObject.insert("firmware", jsonObject_firmware);

	/***********************************************************************************************************************************/

	QJsonObject jsonObject_gui;
	jsonObject_gui.insert("low_z_value", gui_config_.Instance().low_z_value);
	jsonObject_gui.insert("high_z_value", gui_config_.Instance().high_z_value);
	jsonObject_gui.insert("ip", gui_config_.Instance().ip);
	jsonObject_gui.insert("use_hdr_model", gui_config_.Instance().use_hdr_model);
	jsonObject_gui.insert("calibration_board", gui_config_.Instance().calibration_board);
	jsonObject_gui.insert("repetition_count", gui_config_.Instance().repetition_count);

	rootObject.insert("gui", jsonObject_gui);



	/************************************************************************************************************************************/

	jsonDoc.setObject(rootObject);

	// 将json以文本形式写入文件并关闭文件。
	file.write(jsonDoc.toJson());
	file.close();

	qDebug() << "Write to file: " << path;
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


void SettingsFileFunction::getFirmwareConfigData(struct FirmwareConfigParam& param)
{
	param = camera_config_.Instance().firwmare_param_;
}

void SettingsFileFunction::setFirmwareConfigData(struct FirmwareConfigParam param)
{
	camera_config_.Instance().firwmare_param_ = param;
}

void SettingsFileFunction::getSystemConfigData(struct SystemConfigParam& param)
{
	param = camera_config_.Instance().config_param_;
}

void SettingsFileFunction::setSystemConfigData(struct SystemConfigParam param)
{
	camera_config_.Instance().config_param_ = param;
}