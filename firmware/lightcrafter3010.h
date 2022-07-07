#pragma once
#include "i2c.h"
#include <stdio.h>
#include <string.h>

#define Write_Image_Freeze				0x1A
#define Write_Operating_Mode_Select		0x05
#define Write_Input_Image_Size			0x2E
#define Write_Image_Crop				0x10
#define Write_Checkerboard				0x0B
#define Write_Display_Size				0x12
#define Write_Rgb_Led_Enable			0x52

#define Write_Internal_Pattern_Control	0x9E
#define Write_Flash_Data_Type_Select	0xDE
#define Read_Flash_Update_Precheck		0xDD
#define Write_Flash_Erase				0xE0
#define Read_Short_Status				0xD0
#define Write_Flash_Data_Length			0xDF
#define Write_Flash_Start				0xE1
#define Write_Flash_Continue			0xE2
#define Read_Flash_Start 				0xE3
#define Read_Flash_Continue				0xE4
#define Write_Pattern_Order_Table_Entry	0x98
#define Read_Device_ID					0xD4

class LightCrafter3010
{
private:
	I2CDevice _device;
	I2CDevice _MCP3221;
	size_t read(char inner_addr, void* buffer, size_t buffer_size);
	size_t write(char inner_addr, void* buffer, size_t buffer_size);
	size_t read_mcp3221(void* buffer, size_t buffer_size);

public:
	LightCrafter3010();

	~LightCrafter3010();

	void init();
		
	void start_pattern_sequence();

	void stop_pattern_sequence();
	
	void write_pattern_table(unsigned char* pattern_index, int len);
	
	void write_pattern_table(unsigned char* pattern_index, int len,float camera_exposure);

	void pattern_mode01();
	void pattern_mode02();
	void pattern_mode03();
	void pattern_mode04();
	
	void pattern_mode03_repetition(int repetition_count);
	void pattern_mode04_repetition(int repetition_count);

	void pattern_mode_brightness();

	void read_pattern_table(int i);
	void read_pattern_status();
	
	float get_temperature();
    void SetLedCurrent(unsigned short R, unsigned short G, unsigned short B);

	void enable_checkerboard();
	void disable_checkerboard();

	void enable_solid_field();
	void disable_solid_field();

	void set_internal_pattern_stop();
	void set_flash_data_type();
	bool set_flash_build_data_size(unsigned int data_size);
	void set_erase_flash();
	bool check_erase_flash_status();
	void set_flash_data_length(unsigned short dataLen);
	int write_data_into_the_flash(unsigned char writeFlashCmd, char *TxBuffer, unsigned short dataLen);
	void read_data_from_the_flash(unsigned char readFlashCmd, char *RxBuffer, unsigned short dataLen);
	void reload_pattern_order_table_from_flash();
	float get_projector_temperature();

	void set_camera_exposure(float exposure){
		camera_exposure_ = exposure;
	}

	void read_dmd_device_id(int& version);
private:
	float camera_exposure_;
};

