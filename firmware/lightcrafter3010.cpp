#include "lightcrafter3010.h"
#include "i2c.h"
#include <stdio.h>
#include <string.h>


LightCrafter3010::LightCrafter3010()
{
	int fd;
	if((fd = i2c_open("/dev/i2c-1")) == -1)
	{
		perror("Open i2c bus error");
		return;
	}
	memset(&_device, 0, sizeof(_device));
	i2c_init_device(&_device);
	_device.bus = fd;
        _device.addr = 0x1b & 0x3ff;  //0x1b is LC3010 device address
        _device.page_bytes = 32;
        _device.iaddr_bytes = 1;
}

size_t LightCrafter3010::read(char inner_addr, void* buffer, size_t buffer_size)
{
	return	i2c_read(&_device, inner_addr, buffer, buffer_size);
}

size_t LightCrafter3010::write(char inner_addr, void* buffer, size_t buffer_size)
{
	return	i2c_write_3010(&_device, inner_addr, buffer, buffer_size);
}

LightCrafter3010::~LightCrafter3010()
{
	i2c_close(_device.bus);
}


void LightCrafter3010::init()
{
	char buffer[6];

	//internal test pattern mode
	buffer[0] = 0x04;
	write(0x05, buffer, 1);

	//LED set to brightest
	SetLedCurrent(1023, 1023, 1023);

	//enable tigger out 1
	buffer[4] = 0x00;
	buffer[3] = 0x00;
	buffer[2] = 0x00;
	buffer[1] = 0x00;
	buffer[0] = 0x02;
	write(0x92, buffer, 5);

	//enable trigger out 2
	//delay = -500us (0xfffffe0c)
	buffer[4] = 0xff;
	buffer[3] = 0xff;
	buffer[2] = 0xfe;
	buffer[1] = 0x0c;
	buffer[0] = 0x03;
	write(0x92, buffer, 5);

	pattern_mode01();
	//write_pattern_table();
}

void LightCrafter3010::SetLedCurrent(unsigned short R, unsigned short G, unsigned short B)
{
    if (R>1023) R=1023;
    if (G>1023) G=1023;
    if (B>1023) B=1023;
    
    char buffer[6];
    buffer[5] = ((B>>8)&0x03);
    buffer[4] = (B&0xff);
    buffer[3] = ((G>>8)&0x03);
    buffer[2] = (G&0xff);
    buffer[1] = ((R>>8)&0x03);
    buffer[0] = (R & 0xff);

    write(0x54, buffer, 6);
} 

void LightCrafter3010::enable_checkerboard()
{
    unsigned char TxBuffer[8];
    
    TxBuffer[0] = 0x01;
    write(Write_Image_Freeze, TxBuffer, 1);

    TxBuffer[0] = 0x01;
    write(Write_Operating_Mode_Select, TxBuffer, 1);
/*
    TxBuffer[0] = 0x00; 
    TxBuffer[1] = 0x05; 
    TxBuffer[2] = 0xD0; 
    TxBuffer[3] = 0x02;
    write(Write_Input_Image_Size, TxBuffer, 4);

    memset(TxBuffer, 0x00, 8);
    TxBuffer[5] = 0x05;
    TxBuffer[6] = 0xD0;
    TxBuffer[7] = 0x02;
    write(Write_Image_Crop, TxBuffer, 8);
*/
    TxBuffer[0] = 0x87;
    TxBuffer[1] = 0x30;
    TxBuffer[2] = 0x0F;
    TxBuffer[3] = 0x00;
    TxBuffer[4] = 0x0F;
    TxBuffer[5] = 0x00;
    write(Write_Checkerboard, TxBuffer, 6);
/*
    memset(TxBuffer, 0x00, 8);
    TxBuffer[5] = 0x05;
    TxBuffer[6] = 0xD0;
    TxBuffer[7] = 0x02;
    write(Write_Display_Size, TxBuffer, 8);
*/
    TxBuffer[0] = 0x07;
    write(Write_Rgb_Led_Enable, TxBuffer, 1);

    TxBuffer[0] = 0x00;
    write(Write_Image_Freeze, TxBuffer, 1);
} 

void LightCrafter3010::disable_checkerboard()
{
    unsigned char TxBuffer[8];
    
    TxBuffer[0] = 0x01;
    write(Write_Image_Freeze, TxBuffer, 1);

    TxBuffer[0] = 0x00;
    TxBuffer[1] = 0x00;
    TxBuffer[2] = 0x0F;
    TxBuffer[3] = 0x00;
    TxBuffer[4] = 0x0F;
    TxBuffer[5] = 0x00;
    write(Write_Checkerboard, TxBuffer, 6);

    TxBuffer[0] = 0x00;
    write(Write_Image_Freeze, TxBuffer, 1);
} 

void LightCrafter3010::write_pattern_table(unsigned char* pattern_index, int len)
{
    unsigned char buffer[24];
    buffer[0] = 0x01;  //Start
    buffer[1] = 0x00;  //Pattern Set Index

    buffer[2] = 0x06;  //Number of pattern to display
    //buffer[3] = 0x07;  //RGB
    buffer[3] = 0x04;  //RGB


    // Pattern Invert
    buffer[4] = 0; 
    buffer[5] = 0;
    buffer[6] = 0;
    buffer[7] = 0;
    buffer[8] = 0;
    buffer[9] = 0;
    buffer[10] = 0;
    buffer[11] = 0;

    // Illumination Time = 11000us
    buffer[12] = 0xf8;
    buffer[13] = 0x2a;
    buffer[14] = 0x00;
    buffer[15] = 0x00;

    // Pre-illumination Dark Time = 500us
    buffer[16] = 0xf4;
    buffer[17] = 0x01;
    buffer[18] = 0x00;
    buffer[19] = 0x00;

    // Post-illumination Dark Time = 1000us
    buffer[20] = 0xb8;
    buffer[21] = 0x0b;
    buffer[22] = 0x00;
    buffer[23] = 0x00;

    for(int i=0; i<len; i++)
    {
	buffer[1] = pattern_index[i];
        write(0x98, buffer, 24);
	buffer[0] = 0x00;
    }
}

void LightCrafter3010::pattern_mode01()
{
    // unsigned char pattern_index[] = {12,13,15,16};
    unsigned char pattern_index[] = {0,1,3,4};
    write_pattern_table(pattern_index, 4);
}

void LightCrafter3010::pattern_mode02()
{
    unsigned char pattern_index[] = {0,1,2,3,4,5,6};
    write_pattern_table(pattern_index, 7);
}

void LightCrafter3010::pattern_mode_brightness()
{
    unsigned char pattern_index[] = {6};
    write_pattern_table(pattern_index, 1);
}

void LightCrafter3010::pattern_mode03()
{
    unsigned char pattern_index[] = {0,1,2,3,4,6};
    write_pattern_table(pattern_index, 6);
}


void LightCrafter3010::pattern_mode03_repetition(int repetition_count)
{
    if(repetition_count< 1)
    {
        repetition_count = 1;
    }

    int group_count = 5+repetition_count;

    unsigned char pattern_index[group_count];

    pattern_index[0] = 0;
    pattern_index[1] = 1;

    for(int i= 0;i< repetition_count;i++)
    {
        pattern_index[2+i] = 2;
    }
 
    pattern_index[2+repetition_count] = 3;
    pattern_index[3+repetition_count] = 4;
    pattern_index[4+repetition_count] = 6;
    write_pattern_table(pattern_index, group_count);

}	

void LightCrafter3010::pattern_mode04()
{
    // unsigned char pattern_index[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};
    // write_pattern_table(pattern_index, 18);
}


void LightCrafter3010::read_pattern_status()
{
    char buffer[7];

    read(0x9f, buffer, 7);
    printf("read pattern status\n");
    for(int i=0; i<7; i++)
    {
	printf("%d   %x\n", i, buffer[i]);
    }
}

void LightCrafter3010::read_pattern_table(int i)
{
    //0x99
    char buffer[24];

    buffer[0] = i;
    write(0x99, buffer, 1);
    read(0x99, buffer, 24);
    for(int i=0; i<24; i++)
    {
	printf("%x ", buffer[i]);
    }
    printf("\n");
}

		
void LightCrafter3010::start_pattern_sequence()
{
	char buffer[2] = {0x00, 0x00};
	write(0x9e, buffer, 2);
}

float LightCrafter3010::get_temperature()
{
    char buffer[2];
    read(0xd6, buffer, 2);
    bool negative = buffer[1] & 0x80;
    float temperature = ((*((unsigned short*)buffer)) & 0x7FFF)/10.0;
    if(negative)
    {
	temperature = -temperature;
    }
    return temperature;
}

