#include "settings_file_function.h"
#include "qsettings.h"
#include "qfile.h"

/***************************************************************************************************************************/
ProcessingDataStruct ProcessingDataStruct::instance_;
ProcessingDataStruct::ProcessingDataStruct()
{

}

bool ProcessingDataStruct::loadFromSettings(const QString& f)
{

	//if (QFile::exists(f))
	//{
		QSettings settings(f, QSettings::IniFormat);
 
		instance_.low_z_value = settings.value("low_z_value", 400.0).toInt(); 
		instance_.high_z_value = settings.value("high_z_value", 600.0).toInt();
		 
		return true;
	//}
	//else
	//{
	//	return false;
	//}

}

bool ProcessingDataStruct::saveToSettings(const QString& f)
{
	QSettings settings(f, QSettings::IniFormat);
	settings.setValue("low_z_value", Instance().low_z_value);
	settings.setValue("high_z_value", Instance().high_z_value);
 


	return true;
}


/***************************************************************************************************************************/


SettingsFileFunction::SettingsFileFunction(QObject *parent)
	: QObject(parent)
{
}

SettingsFileFunction::~SettingsFileFunction()
{
}


bool SettingsFileFunction::loadProcessingSettingsFile(QString path)
{
 
 
	return true;
}

bool SettingsFileFunction::saveProcessingSettingsFile(QString path)
{

	return true;
}
