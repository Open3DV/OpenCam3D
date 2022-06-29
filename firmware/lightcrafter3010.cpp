#include "lightcrafter3010.h"
#include "i2c.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "easylogging++.h"

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
    _device.addr = 0x1b & 0x3ff;        //0x1b is LC3010 device address
    _device.page_bytes = 256;//32;
    _device.iaddr_bytes = 1;

    camera_exposure_ = 12000;

    // ----- MCP3221 init
	memset(&_MCP3221, 0, sizeof(_MCP3221));
	i2c_init_device(&_MCP3221);
    _MCP3221.bus = fd;
    _MCP3221.addr = 0x4d;
    _MCP3221.page_bytes = 256;
    _MCP3221.iaddr_bytes = 1;  
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


void LightCrafter3010::read_dmd_device_id(int& version)
{
    
    unsigned char TxBuffer[8];

    read(Read_Device_ID,TxBuffer,1);

    int value = int(TxBuffer[0]);

    // LOG(INFO)<<"DMD: "<<value;

    if(11 == value)
    {
        version = 800;
    }
    else if(12 == value)
    {
        version = 1800;
    }
}

void LightCrafter3010::enable_solid_field()
{
    unsigned char TxBuffer[8];
    
    TxBuffer[0] = 0x01;
    write(Write_Image_Freeze, TxBuffer, 1);

    TxBuffer[0] = 0x01;
    write(Write_Operating_Mode_Select, TxBuffer, 1);
 
    TxBuffer[0] = 0x00;
    TxBuffer[1] = 0x70;
    TxBuffer[2] = 0x0F;
    TxBuffer[3] = 0x00;
    TxBuffer[4] = 0x0F;
    TxBuffer[5] = 0x00;
    write(Write_Checkerboard, TxBuffer, 6);
 
    TxBuffer[0] = 0x07;
    write(Write_Rgb_Led_Enable, TxBuffer, 1);

    TxBuffer[0] = 0x00;
    write(Write_Image_Freeze, TxBuffer, 1);
}

void LightCrafter3010::disable_solid_field()
{
    disable_checkerboard();
    init();
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
    TxBuffer[1] = 0x70;
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

void LightCrafter3010::set_internal_pattern_stop()
{
    unsigned char TxBuffer[8];
    
    TxBuffer[0] = 0x01;
    TxBuffer[1] = 0x00;
    write(Write_Internal_Pattern_Control, TxBuffer, 2);
}

void LightCrafter3010::set_flash_data_type()
{
    unsigned char TxBuffer[8];
    
    TxBuffer[0] = 0xD0;
    write(Write_Flash_Data_Type_Select, TxBuffer, 1);
}

bool LightCrafter3010::set_flash_build_data_size(unsigned int data_size)
{
    bool ack = true;
    unsigned char MxBuffer[8];
    
    MxBuffer[0] = (data_size >> 0) & 0xff;
    MxBuffer[1] = (data_size >> 8) & 0xff;
    MxBuffer[2] = (data_size >> 16) & 0xff;
    MxBuffer[3] = (data_size >> 24) & 0xff;
    write(Read_Flash_Update_Precheck, MxBuffer, 4);

    return ack;
}

void LightCrafter3010::set_erase_flash()
{
    unsigned char TxBuffer[8];
    
    TxBuffer[0] = 0xAA;
    TxBuffer[1] = 0xBB;
    TxBuffer[2] = 0xCC;
    TxBuffer[3] = 0xDD;
    write(Write_Flash_Erase, TxBuffer, 4);
}

bool LightCrafter3010::check_erase_flash_status()
{
    bool ack = true;
    unsigned char RxBuffer[8];
    int i;
    
    for (i = 0; i < 30; i++)
    {
        read(Read_Short_Status, RxBuffer, 1);

        char string[50] = {'\0'};
        sprintf(string, "Read_Short_Status: %d- 0x%X", i, RxBuffer[0]);
        LOG(INFO)<<string;

        if ((RxBuffer[0] & 0x7E) != 0) {
            ack = false;
        } else {
            ack = true;
            break;
        }

        usleep(10000);
    }

    return ack;
}

void LightCrafter3010::set_flash_data_length(unsigned short dataLen)
{
    unsigned char TxBuffer[8];
    
    TxBuffer[0] = (dataLen >> 0) & 0xff;
    TxBuffer[1] = (dataLen >> 8) & 0xff;
    write(Write_Flash_Data_Length, TxBuffer, 2);
}

int LightCrafter3010::write_data_into_the_flash(unsigned char writeFlashCmd, char *TxBuffer, unsigned short dataLen)
{
    return write(writeFlashCmd, TxBuffer, dataLen);
}

void LightCrafter3010::read_data_from_the_flash(unsigned char readFlashCmd, char *RxBuffer, unsigned short dataLen)
{
    read(readFlashCmd, RxBuffer, dataLen);
}

void LightCrafter3010::reload_pattern_order_table_from_flash()
{
    unsigned char TxBuffer[24];
    
    memset(TxBuffer, 0, sizeof(TxBuffer));
    TxBuffer[0] = 0x02;
    write(Write_Pattern_Order_Table_Entry, TxBuffer, 24);
}


void LightCrafter3010::write_pattern_table(unsigned char* pattern_index, int len,float camera_exposure)
{
    unsigned char buffer[24];
    buffer[0] = 0x01;  //Start
    buffer[1] = 0x00;  //Pattern Set Index

    buffer[2] = 0x06;  //Number of pattern to display
    // buffer[3] = 0x07;  //RGB
    buffer[3] = 0x04;  //BLUE

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
    int illumination_time = camera_exposure - 1000;
  

    std::vector<int> remainder_list;
    for(int i= 0;i< 8;i++)
    {
        int remainder = illumination_time%16;
        remainder_list.push_back(remainder);
        illumination_time /=16; 
    }

 
    buffer[12] = remainder_list[1]*16+remainder_list[0];
    buffer[13] = remainder_list[3]*16+remainder_list[2];
    buffer[14] = remainder_list[5]*16+remainder_list[4];
    buffer[15] = remainder_list[7]*16+remainder_list[6];

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
    write_pattern_table(pattern_index, 4, camera_exposure_);
}

void LightCrafter3010::pattern_mode02()
{
    unsigned char pattern_index[] = {0,1,2,3,4,5,6};
    write_pattern_table(pattern_index, 7, camera_exposure_);
}

void LightCrafter3010::pattern_mode_brightness()
{
    unsigned char pattern_index[] = {6};
    write_pattern_table(pattern_index, 1, camera_exposure_);
}

void LightCrafter3010::pattern_mode03()
{
    unsigned char pattern_index[] = {0,1,2,3,4,6};
    write_pattern_table(pattern_index, 6, camera_exposure_);
}


void LightCrafter3010::pattern_mode04_repetition(int repetition_count)
{
    if(repetition_count< 1)
    {
        repetition_count = 1;
    }

    int group_count = 3+repetition_count;

    unsigned char pattern_index[group_count];
    pattern_index[0] = 0;
    pattern_index[1] = 1;

    for(int i= 0;i< repetition_count;i++)
    {
        pattern_index[2+i] = 2;
    }
  
    pattern_index[group_count-1] = 6;
    write_pattern_table(pattern_index, group_count, camera_exposure_);

 
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
    write_pattern_table(pattern_index, group_count, camera_exposure_);

}	

void LightCrafter3010::pattern_mode04()
{
    unsigned char pattern_index[] = {0,1,2,6};
    write_pattern_table(pattern_index, 4, camera_exposure_);
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


void LightCrafter3010::stop_pattern_sequence()
{
	char buffer[2] = {0x01, 0x00};
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

size_t LightCrafter3010::read_mcp3221(void* buffer, size_t buffer_size)
{
	return	i2c_read(&_MCP3221, 0, buffer, buffer_size);
}

float LightCrafter3010::get_projector_temperature()
{
    unsigned char buffer[2];
    read_mcp3221(buffer, 2);

    short OutputCode;
    OutputCode = ((buffer[0] << 8) & 0xff00) | buffer[1];
    printf("The AD data = 0x%x = %d\n", OutputCode, OutputCode);

    // Rntc = 10 * (4096 - AD) / AD, unit=KO
    float fAD = OutputCode;
    float fRntc = 10.0 * (4096.0 - fAD) / fAD;

    return fRntc;
}
