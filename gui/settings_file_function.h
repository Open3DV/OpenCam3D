#pragma once

#include <QObject>
#include "../firmware/system_config_settings.h"

struct  GuiConfigDataStruct
{
	GuiConfigDataStruct();

	int low_z_value;
	int high_z_value;

	QString ip;
	
  
	bool loadFromSettings(const QString& f);
	bool saveToSettings(const QString& f);

	static GuiConfigDataStruct& Instance()
	{
		return instance_;
	}

private:
	static GuiConfigDataStruct instance_;
};

class SettingsFileFunction : public QObject
{
	Q_OBJECT

public:
	SettingsFileFunction();
	~SettingsFileFunction();

	void setFirmwareConfigData(struct SystemConfigParam param);

	void getFirmwareConfigData(struct SystemConfigParam& param);

	void setGuiConfigData(struct GuiConfigDataStruct param);

	void getGuiConfigData(struct GuiConfigDataStruct& param);

	bool loadProcessingSettingsFile(QString path);

	bool saveProcessingSettingsFile(QString path);

private:
	struct SystemConfigDataStruct camera_config_;
	struct  GuiConfigDataStruct gui_config_;
};
