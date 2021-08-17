#pragma once

#include <QObject>


struct  ProcessingDataStruct
{
	ProcessingDataStruct();

	int low_z_value;
	int high_z_value;
	
  
	bool loadFromSettings(const QString& f);
	bool saveToSettings(const QString& f);
 
	static ProcessingDataStruct& Instance()
	{
		return instance_;
	}

private:
	static ProcessingDataStruct instance_;
};

class SettingsFileFunction : public QObject
{
	Q_OBJECT

public:
	SettingsFileFunction(QObject *parent);
	~SettingsFileFunction();

	bool loadProcessingSettingsFile(QString path);

	bool saveProcessingSettingsFile(QString path);
};
