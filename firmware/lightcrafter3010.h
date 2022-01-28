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

class LightCrafter3010
{
private:
	I2CDevice _device;
	size_t read(char inner_addr, void* buffer, size_t buffer_size);
	size_t write(char inner_addr, void* buffer, size_t buffer_size);

public:
	LightCrafter3010();

	~LightCrafter3010();

	void init();
		
	void start_pattern_sequence();
	
	void write_pattern_table(unsigned char* pattern_index, int len);

	void pattern_mode01();
	void pattern_mode02();
	void pattern_mode03();
	void pattern_mode04();

	void pattern_mode_brightness();

	void read_pattern_table(int i);
	void read_pattern_status();
	
	float get_temperature();
    void SetLedCurrent(unsigned short R, unsigned short G, unsigned short B);

	void enable_checkerboard();
	void disable_checkerboard();
};

