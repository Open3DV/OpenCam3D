#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <iostream>
#include <errno.h> 
#include "camera_dh.h"
#include <cassert>
#include "protocol.h"
#include <random>
#include <time.h>
#include <mutex>
#include <thread>
#include "lightcrafter3010.h"
#include "easylogging++.h"
#include "encode_cuda.cuh"
#include "system_config_settings.h"


INITIALIZE_EASYLOGGINGPP 

std::random_device rd;
std::mt19937 rand_num(rd());
bool connected = false;
long long current_token = 0;
time_t last_time;
std::mutex mtx_last_time;
std::thread heartbeat_thread;
CameraDh camera;
LightCrafter3010 lc3010;
struct CameraCalibParam param;

int brightness_current = 100;
 
SystemConfigDataStruct system_config_settings_machine_;

bool readSystemConfig()
{
    return system_config_settings_machine_.loadFromSettings("../system_config.ini");
}

bool saveSystemConfig()
{
    return system_config_settings_machine_.saveToSettings("../system_config.ini");
}

int heartbeat_check()
{
    while(connected)
    {
	std::this_thread::sleep_for(std::chrono::milliseconds(1));

	time_t current_time;
	time(&current_time);

	mtx_last_time.lock();
	double seconds = difftime(current_time, last_time);
	mtx_last_time.unlock();
	
	if(seconds>30)
	{
	    LOG(INFO)<<"HeartBeat stopped!";
	    connected = false;
            current_token = 0;
	}
    }

    return 0;
}

long long generate_token()
{
    long long token = rand_num();
    return token;
}

int send_buffer(int sock, const char* buffer, int buffer_size)
{
   /* 
  struct tcp_info info; 
  int len=sizeof(info); 
  getsockopt(sock, IPPROTO_TCP, TCP_INFO, &info, (socklen_t *)&len); 
  if((info.tcpi_state==TCP_ESTABLISHED))
  {
         LOG(INFO)<<"ok";
  }
  else
  {
	 return DF_FAILED; 
  }

  */

 
	
	
    int size = 0;
    int ret = send(sock, (char*)&buffer_size, sizeof(buffer_size), MSG_NOSIGNAL);
    LOG(INFO)<<"send buffer_size ret="<<ret;
    if (ret == -1)
    {
        return DF_FAILED;
    }

    int sent_size = 0;
    ret = send(sock, buffer, buffer_size, MSG_NOSIGNAL);
    LOG(INFO)<<"send buffer ret="<<ret;
    if (ret == -1)
    {
        return DF_FAILED;
    }
    sent_size += ret;
    while(sent_size != buffer_size)
    {
	buffer += ret;
	LOG(INFO)<<"sent_size="<<sent_size;
	ret = send(sock, buffer, buffer_size-sent_size, MSG_NOSIGNAL);
        LOG(INFO)<<"ret="<<ret;
        if (ret == -1)
        {
            return DF_FAILED;
        }
	sent_size += ret;
    }

    return DF_SUCCESS;
}

int recv_buffer(int sock, char* buffer, int buffer_size)
{
    int size = 0;
    int ret = recv(sock, (char*)&size, sizeof(size), 0);
    assert(buffer_size >= size);
    int n_recv = 0;
    ret = DF_SUCCESS;

    while (ret != -1)
    {
        ret = recv(sock, buffer, buffer_size, 0);
        //std::cout << "ret="<<ret << std::endl;
        if (ret > 0)
        {
            buffer_size -= ret;
            n_recv += ret;
            buffer += ret;
        }

        if (buffer_size == 0)
        {
            assert(n_recv == size);
            return DF_SUCCESS;
        }
    }
    return DF_FAILED;
}

int send_command(int sock, int command)
{
    return send_buffer(sock, (const char*)&command, sizeof(int));
}

int recv_command(int sock, int* command)
{
    return recv_buffer(sock, (char*)command, sizeof(int));
}

float read_temperature(int flag)
{
    float val = -1.0;

    switch(flag)
    {
        case 0:
        {
            char data[100];
            std::ifstream infile;
            infile.open("/sys/class/thermal/thermal_zone0/temp");
            infile >> data;
            // std::cout << "first read data from file1.dat == " << data << std::endl;
 
            val = (float)std::atoi(data) / 1000.0; 

        }
        break;

        case 1:
        {
            char data[100];
            std::ifstream infile;
            infile.open("/sys/class/thermal/thermal_zone1/temp");
            infile >> data;
            
            val = (float)std::atoi(data) / 1000.0; 
        }
        break;

        case 2:
        {
            char data[100];
            std::ifstream infile;
            infile.open("/sys/class/thermal/thermal_zone2/temp");
            infile >> data;
            
            val =(float)std::atoi(data) / 1000.0; 
        }
        break;

        default:
        break;
    }
   
 

    return val;
}

int setup_socket(int port)
{
    int server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(server_sock<0)
    {
        perror("ERROR: socket()");
        exit(0);
    }

    int flags = 3;
    setsockopt(server_sock, SOL_TCP, TCP_KEEPIDLE, (void*)&flags, sizeof(flags));
    flags = 3;
    setsockopt(server_sock, SOL_TCP, TCP_KEEPCNT, (void*)&flags, sizeof(flags));
    flags = 1;
    setsockopt(server_sock, SOL_TCP, TCP_KEEPINTVL, (void*)&flags, sizeof(flags));


    //将套接字和IP、端口绑定
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));  //每个字节都用0填充
    serv_addr.sin_family = AF_INET;  //使用IPv4地址
    serv_addr.sin_addr.s_addr = INADDR_ANY;  //具体的IP地址
    serv_addr.sin_port = htons(port);  //端口
    int ret = bind(server_sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
    if(ret==-1)
    {
        printf("bind ret=%d, %s\n", ret, strerror(errno));
        close(server_sock);
        return DF_FAILED;
    }

    //进入监听状态，等待用户发起请求
    ret = listen(server_sock, 1);
    if(ret == -1)
    {
        printf("listen ret=%d, %s\n", ret, strerror(errno));
        close(server_sock);
        return DF_FAILED;
    }
    return server_sock;
}

int accept_new_connection(int server_sock)
{
    //std::cout<<"listening"<<std::endl;
    //接收客户端请求
    struct sockaddr_in clnt_addr;
    socklen_t clnt_addr_size = sizeof(clnt_addr);
    int client_sock = accept(server_sock, (struct sockaddr*)&clnt_addr, &clnt_addr_size);

    //print address
    char buffer[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &clnt_addr.sin_addr, buffer, sizeof(buffer));

    struct timeval timeout = {1,0};
    int ret = setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
    ret = setsockopt(client_sock, SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout, sizeof(timeout));
    //int flags = 1;
    //setsockopt(client_sock, SOL_SOCKET, SO_KEEPALIVE, (void*)&flags, sizeof(flags));

    //std::cout<<"accepted connection from "<<buffer<<std::endl;
    
    return client_sock;
}

int handle_cmd_connect(int client_sock)
{
    int ret;
    if (connected)
    {
	std::cout<<"new connection rejected"<<std::endl;
	return send_command(client_sock, DF_CMD_REJECT);
    }
    else
    {
        ret = send_command(client_sock, DF_CMD_OK);
	if(ret == DF_FAILED)
	{
	    return DF_FAILED;
	}
	long long token = generate_token();
	ret = send_buffer(client_sock, (char*)&token, sizeof(token));
	if(ret == DF_FAILED)
	{
	    return DF_FAILED;
	}
	connected = true;
	current_token = token;
	
	mtx_last_time.lock();
	time(&last_time);
	mtx_last_time.unlock();

	if(heartbeat_thread.joinable())
	{
	    heartbeat_thread.join();
	}
	heartbeat_thread = std::thread(heartbeat_check);

	//std::cout<<"connection established, current token is: "<<current_token<<std::endl;
	return DF_SUCCESS;
    }
}

int check_token(int client_sock)
{
    long long token = 0;
    int ret = recv_buffer(client_sock, (char*)&token, sizeof(token));
    //std::cout<<"token ret = "<<ret<<std::endl;
    //std::cout<<"checking token:"<<token<<std::endl;
    if(ret == DF_FAILED)
    {
	return DF_FAILED;
    }

    if(token == current_token)
    {
	//std::cout<<"ok"<<std::endl;
	ret = send_command(client_sock, DF_CMD_OK);
	return DF_SUCCESS;
    }
    else
    {
	std::cout<<"reject"<<std::endl;
	ret = send_command(client_sock, DF_CMD_REJECT);
	return DF_FAILED;
    }
}

int handle_cmd_disconnect(int client_sock)
{
    std::cout<<"handle_cmd_disconnect"<<std::endl;
    long long token = 0;
    int ret = recv_buffer(client_sock, (char*)&token, sizeof(token));
    std::cout<<"token "<<token<<" trying to disconnect"<<std::endl;
    if(ret == DF_FAILED)
    {
	return DF_FAILED;
    }
    if(token == current_token)
    {
	connected = false;
	current_token = 0;
	std::cout<<"client token="<<token<<" disconnected"<<std::endl;
	ret = send_command(client_sock, DF_CMD_OK);
	if(ret == DF_FAILED)
	{
	    return DF_FAILED;
	}
    }
    else
    {
	std::cout<<"disconnect rejected"<<std::endl;
	ret = send_command(client_sock, DF_CMD_REJECT);
	if(ret == DF_FAILED)
	{
	    return DF_FAILED;
	}
    }
    return DF_SUCCESS;
}

int handle_cmd_get_brightness(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    lc3010.pattern_mode_brightness();

    int image_num= 1;

    int buffer_size = 1920*1200*image_num;
    char* buffer = new char[buffer_size];
   
     LOG(TRACE)<<"capture single image";
    
    camera.captureRawTest(image_num,buffer);



    //int buffer_size = 1920*1200;
    //char* buffer = new char[buffer_size];
   //camera.captureSingleImage(buffer);
    LOG(TRACE)<<"start send image, image_size="<<buffer_size;
    int ret = send_buffer(client_sock, buffer, buffer_size);
    delete [] buffer;
    if(ret == DF_FAILED)
    {
        LOG(ERROR)<<"send error, close this connection!";
	return DF_FAILED;
    }
    LOG(TRACE)<<"image sent!";
    return DF_SUCCESS;
}


int handle_cmd_get_raw_03(int client_sock)
{
    lc3010.pattern_mode03();

    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }


    int image_num= 31;

    //cv::Mat image = get_mat();
    //lc3010.start_pattern_sequence();
    int buffer_size = 1920*1200*image_num;
    char* buffer = new char[buffer_size];
    camera.captureRawTest(image_num,buffer);

    printf("start send image, buffer_size=%d\n", buffer_size);
    int ret = send_buffer(client_sock, buffer, buffer_size);
    printf("ret=%d\n", ret);
    if(ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
	delete [] buffer;
	return DF_FAILED;
    }
    printf("image sent!\n");
    delete [] buffer;
    return DF_SUCCESS;
}


int handle_cmd_get_raw_02(int client_sock)
{
    lc3010.pattern_mode02();

    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }


    int image_num= 37;

    //cv::Mat image = get_mat();
    //lc3010.start_pattern_sequence();
    int buffer_size = 1920*1200*image_num;
    char* buffer = new char[buffer_size];
    camera.captureRawTest(image_num,buffer);

    printf("start send image, buffer_size=%d\n", buffer_size);
    int ret = send_buffer(client_sock, buffer, buffer_size);
    printf("ret=%d\n", ret);
    if(ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
	delete [] buffer;
	return DF_FAILED;
    }
    printf("image sent!\n");
    delete [] buffer;
    return DF_SUCCESS;
}


int handle_cmd_get_raw_01(int client_sock)
{
    //camera.warmupCamera();
	
    lc3010.pattern_mode01();

    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    int capture_num = 24;

    //cv::Mat image = get_mat();
    //lc3010.start_pattern_sequence();
    int buffer_size = 1920*1200*capture_num;
    char* buffer = new char[buffer_size];
    //camera.captureRawPhaseImages(buffer);
    camera.captureRawTest(capture_num,buffer);
    
    printf("start send image, buffer_size=%d\n", buffer_size);
    int ret = send_buffer(client_sock, buffer, buffer_size);
    printf("ret=%d\n", ret);
    if(ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
	delete [] buffer;
	return DF_FAILED;
    }
    printf("image sent!\n");
    delete [] buffer;
    return DF_SUCCESS;
}
   

int handle_cmd_get_raw(int client_sock)
{
    //camera.warmupCamera();
	
    lc3010.pattern_mode01();

    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    int capture_num = 24;

    //cv::Mat image = get_mat();
    //lc3010.start_pattern_sequence();
    int buffer_size = 1920*1200*capture_num;
    char* buffer = new char[buffer_size];
    //camera.captureRawPhaseImages(buffer);
    camera.captureRawTest(capture_num,buffer);
    
    printf("start send image, buffer_size=%d\n", buffer_size);
    int ret = send_buffer(client_sock, buffer, buffer_size);
    printf("ret=%d\n", ret);
    if(ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
	delete [] buffer;
	return DF_FAILED;
    }
    printf("image sent!\n");
    delete [] buffer;
    return DF_SUCCESS;
}
   
int handle_cmd_get_frame_03_more_exposure(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    LOG(INFO)<<"HDR Exposure:";

    int led_current_h = brightness_current * 1.5;
    int led_current_m = brightness_current;
    int led_current_l = brightness_current * 0.5;

    if(led_current_h > 1023)
    {
        led_current_h = 1023;
    }


    std::vector<float*> depth_map_list;
    std::vector<unsigned char*> brightness_list;
 

    lc3010.SetLedCurrent(led_current_h,led_current_h,led_current_h);	
    lc3010.pattern_mode03();


    int image_count = 31;

    int buffer_size = 1920*1200*image_count;
    char* buffer = new char[buffer_size];
    camera.captureRawTest(image_count,buffer);
    std::vector<unsigned char*> patterns_ptr_list;
    for(int i=0; i<image_count; i++)
    {
	    patterns_ptr_list.push_back(((unsigned char*)(buffer+i*1920*1200)));
    }

    int depth_buf_size = 1920*1200*4;
    float* depth_map_0 = new float[depth_buf_size];

    int brightness_buf_size = 1920*1200*1;
    unsigned char* brightness_0 = new unsigned char[brightness_buf_size]; 

    // int ret= cuda_get_frame_03_hdr(patterns_ptr_list, 0,(float*)depth_map_0,brightness_0);
    int ret= cuda_get_frame_03(patterns_ptr_list,(float*)depth_map_0,brightness_0);

    depth_map_list.push_back(depth_map_0);
    brightness_list.push_back(brightness_0);


    /*************************************************************************************************************/
    
  

    lc3010.SetLedCurrent(led_current_m,led_current_m,led_current_m);	
    lc3010.pattern_mode03();


    camera.captureRawTest(image_count,buffer);
    patterns_ptr_list.clear();
    for(int i=0; i<image_count; i++)
    {
    	patterns_ptr_list.push_back(((unsigned char*)(buffer+i*1920*1200)));
    }

    float* depth_map_1 = new float[depth_buf_size];

    unsigned char* brightness_1 = new unsigned char[brightness_buf_size]; 

    // ret= cuda_get_frame_03_hdr(patterns_ptr_list,1, (float*)depth_map_1,brightness_1);
    ret= cuda_get_frame_03(patterns_ptr_list, (float*)depth_map_1,brightness_1);

     depth_map_list.push_back(depth_map_1);
    brightness_list.push_back(brightness_1);

  
   /**********************************************************************************************************/ 
     

    lc3010.SetLedCurrent(led_current_l,led_current_l,led_current_l);	
    lc3010.pattern_mode03();


    camera.captureRawTest(image_count,buffer);
    patterns_ptr_list.clear();
    for(int i=0; i<image_count; i++)
    {
    	patterns_ptr_list.push_back(((unsigned char*)(buffer+i*1920*1200)));
    }

    float* depth_map_2 = new float[depth_buf_size];

    unsigned char* brightness_2 = new unsigned char[brightness_buf_size]; 

    // ret= cuda_get_frame_03_hdr(patterns_ptr_list,2, (float*)depth_map_2,brightness_2);
    ret= cuda_get_frame_03(patterns_ptr_list, (float*)depth_map_2,brightness_2);

    depth_map_list.push_back(depth_map_2);
    brightness_list.push_back(brightness_2);

 

    /**********************************************************************************************************/


    float* depth_map = new float[depth_buf_size]; 
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 

    cuda_merge_hdr_data(depth_map_list,brightness_list,depth_map,brightness);

    //merge

    /********************************************************************************************************/
    
    printf("start send depth, buffer_size=%d\n", depth_buf_size);
    ret = send_buffer(client_sock, (const char*)depth_map, depth_buf_size);
    printf("depth ret=%d\n", ret);

    if(ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
	delete [] buffer;
    delete [] depth_map;
	delete [] brightness;
	delete [] depth_map_0;
	delete [] brightness_0;
	delete [] depth_map_1;
	delete [] brightness_1;
	delete [] depth_map_2;
	delete [] brightness_2; 
	


	return DF_FAILED;
    }
    
    printf("start send brightness, buffer_size=%d\n", brightness_buf_size);
    ret = send_buffer(client_sock, (const char*)brightness, brightness_buf_size);
    printf("brightness ret=%d\n", ret);

    LOG(INFO)<<"Send Frame03";

    if(ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
	delete [] buffer;
    delete [] depth_map;
	delete [] brightness;
	delete [] depth_map_0;
	delete [] brightness_0;
	delete [] depth_map_1;
	delete [] brightness_1;
	delete [] depth_map_2;
	delete [] brightness_2; 
	
	return DF_FAILED;
    }
    printf("frame sent!\n");
    delete [] buffer;
    delete [] depth_map;
	delete [] brightness;
	delete [] depth_map_0;
	delete [] brightness_0;
    delete [] depth_map_1;
	delete [] brightness_1;
	delete [] depth_map_2;
	delete [] brightness_2;
	
    

    LOG(INFO)<<"More Exposure Finished!";

    return DF_SUCCESS;
}

/*********************************************************************************************/

int handle_cmd_get_frame_03_hdr_parallel(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    LOG(INFO)<<"HDR Exposure:"; 
  
    std::vector<int> led_current_list; 
    for(int i= 0;i< system_config_settings_machine_.Instance().config_param_.exposure_num;i++)
    {
        led_current_list.push_back(system_config_settings_machine_.Instance().config_param_.exposure_param[i]);
    }

    int depth_buf_size = 1920*1200*4;  
    int brightness_buf_size = 1920*1200*1;

    float* depth_map = new float[depth_buf_size]; 
    unsigned char* brightness = new unsigned char[brightness_buf_size];


   std::sort(led_current_list.begin(),led_current_list.end(),std::greater<int>());

    for(int i= 0;i< led_current_list.size();i++)
    {
        int led_current = led_current_list[i];
        lc3010.SetLedCurrent(led_current,led_current,led_current);	
        
        std::cout << "set led: " << led_current << std::endl;

        lc3010.pattern_mode03();
    
        camera.captureFrame03ToGpu();   
        parallel_cuda_copy_result_to_hdr(i); 
    }

    
    lc3010.SetLedCurrent(brightness_current,brightness_current,brightness_current);
 
	cudaDeviceSynchronize();
    parallel_cuda_merge_hdr_data(led_current_list.size(), depth_map, brightness); 

    
    /******************************************************************************/
    //send data
    printf("start send depth, buffer_size=%d\n", depth_buf_size);
    int ret = send_buffer(client_sock, (const char*)depth_map, depth_buf_size);
    printf("depth ret=%d\n", ret);

    if(ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
        delete[] depth_map;
        delete[] brightness;

        return DF_FAILED;
    }
    
    printf("start send brightness, buffer_size=%d\n", brightness_buf_size);
    ret = send_buffer(client_sock, (const char*)brightness, brightness_buf_size);
    printf("brightness ret=%d\n", ret);

    LOG(INFO)<<"Send Frame03";

    float temperature = read_temperature(0);
    
    LOG(INFO)<<"temperature: "<<temperature<<" deg";

    if(ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
        
	delete [] depth_map;
	delete [] brightness;
	
	return DF_FAILED;
    }
    printf("frame sent!\n");
    
    delete [] depth_map;
    delete [] brightness;
    


    return DF_SUCCESS;

}

int handle_cmd_get_frame_03_parallel(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    int depth_buf_size = 1920*1200*4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = 1920*1200*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 


    lc3010.pattern_mode03(); 
    camera.captureFrame03ToGpu();
  
    int ret= parallel_cuda_copy_result_from_gpu((float*)depth_map,brightness);

    
    printf("start send depth, buffer_size=%d\n", depth_buf_size);
    ret = send_buffer(client_sock, (const char*)depth_map, depth_buf_size);
    printf("depth ret=%d\n", ret);

    if(ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
	// delete [] buffer;
	delete [] depth_map;
	delete [] brightness;
	
	return DF_FAILED;
    }
    
    printf("start send brightness, buffer_size=%d\n", brightness_buf_size);
    ret = send_buffer(client_sock, (const char*)brightness, brightness_buf_size);
    printf("brightness ret=%d\n", ret);

    LOG(INFO)<<"Send Frame03";

    float temperature = read_temperature(0);
    
    LOG(INFO)<<"temperature: "<<temperature<<" deg";

    if(ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
	// delete [] buffer;
	delete [] depth_map;
	delete [] brightness;
	
	return DF_FAILED;
    }
    printf("frame sent!\n");
    // delete [] buffer;
    delete [] depth_map;
    delete [] brightness;
    return DF_SUCCESS;
    

}
   
int handle_cmd_get_frame_03(int client_sock)
{
    lc3010.pattern_mode03();

    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }


    int image_count = 31;

    int buffer_size = 1920*1200*image_count;
    char* buffer = new char[buffer_size];
    camera.captureRawTest(image_count,buffer);
    std::vector<unsigned char*> patterns_ptr_list;
    for(int i=0; i<image_count; i++)
    {
	patterns_ptr_list.push_back(((unsigned char*)(buffer+i*1920*1200)));
    }

    int depth_buf_size = 1920*1200*4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = 1920*1200*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 

    int ret= cuda_get_frame_03(patterns_ptr_list, (float*)depth_map,brightness);

    printf("start send depth, buffer_size=%d\n", depth_buf_size);
    ret = send_buffer(client_sock, (const char*)depth_map, depth_buf_size);
    printf("depth ret=%d\n", ret);

    if(ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
	delete [] buffer;
	delete [] depth_map;
	delete [] brightness;
	
	return DF_FAILED;
    }
    
    printf("start send brightness, buffer_size=%d\n", brightness_buf_size);
    ret = send_buffer(client_sock, (const char*)brightness, brightness_buf_size);
    printf("brightness ret=%d\n", ret);

    LOG(INFO)<<"Send Frame03";

    float temperature = read_temperature(0);
    
    LOG(INFO)<<"temperature: "<<temperature<<" deg";

    if(ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
	delete [] buffer;
	delete [] depth_map;
	delete [] brightness;
	
	return DF_FAILED;
    }
    printf("frame sent!\n");
    delete [] buffer;
    delete [] depth_map;
    delete [] brightness;
    return DF_SUCCESS;
}



int handle_cmd_get_frame_01(int client_sock)
{
    lc3010.pattern_mode01();

    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    int buffer_size = 1920*1200*24;
    char* buffer = new char[buffer_size];
    camera.captureRawPhaseImages(buffer);
    std::vector<unsigned char*> patterns_ptr_list;
    for(int i=0; i<24; i++)
    {
	patterns_ptr_list.push_back(((unsigned char*)(buffer+i*1920*1200)));
    }

    int depth_buf_size = 1920*1200*4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = 1920*1200*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 

    int ret= cuda_get_frame_base_24(patterns_ptr_list, (float*)depth_map,brightness);

    printf("start send depth, buffer_size=%d\n", depth_buf_size);
    ret = send_buffer(client_sock, (const char*)depth_map, depth_buf_size);
    printf("depth ret=%d\n", ret);

    printf("start send brightness, buffer_size=%d\n", brightness_buf_size);
    ret = send_buffer(client_sock, (const char*)brightness, brightness_buf_size);
    printf("brightness ret=%d\n", ret);


    if(ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
	delete [] buffer;
	delete [] depth_map;
	delete [] brightness;
	
	return DF_FAILED;
    }
    printf("frame sent!\n");
    delete [] buffer;
    delete [] depth_map;
    delete [] brightness;
    return DF_SUCCESS;
}

 
int handle_cmd_get_point_cloud(int client_sock)
{
    lc3010.pattern_mode01();

    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    int buffer_size = 1920*1200*24;
    char* buffer = new char[buffer_size];
    camera.captureRawPhaseImages(buffer);
    std::vector<unsigned char*> patterns_ptr_list;
    for(int i=0; i<24; i++)
    {
	patterns_ptr_list.push_back(((unsigned char*)(buffer+i*1920*1200)));
    }

    int point_cloud_buf_size = 1920*1200*3*4;
    float* point_cloud_map = new float[point_cloud_buf_size];
    int ret= cuda_reconstruct_base_24(patterns_ptr_list, (float*)point_cloud_map);

    printf("start send point cloud, buffer_size=%d\n", point_cloud_buf_size);
    ret = send_buffer(client_sock, (const char*)point_cloud_map, point_cloud_buf_size);
    printf("ret=%d\n", ret);
    if(ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
	delete [] buffer;
	delete [] point_cloud_map;
	return DF_FAILED;
    }
    printf("image sent!\n");
    delete [] buffer;
    delete [] point_cloud_map;
    return DF_SUCCESS;
}

int handle_heartbeat(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    mtx_last_time.lock();
    time(&last_time);
    mtx_last_time.unlock();

    return DF_SUCCESS;
}
    
int handle_get_temperature(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	return DF_FAILED;
    }
    // float temperature = lc3010.get_temperature();
    float temperature = read_temperature(0);
    LOG(INFO)<<"temperature:"<<temperature;
    int ret = send_buffer(client_sock, (char*)(&temperature), sizeof(temperature));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	return DF_FAILED;
    }
    return DF_SUCCESS;

}
    
int read_calib_param()
{
    std::ifstream ifile;
    ifile.open("calib_param.txt");
    int n_params = sizeof(param)/sizeof(float);
    for(int i=0; i<n_params; i++)
    {
	ifile>>(((float*)(&param))[i]);
    }
    ifile.close();
    return DF_SUCCESS;
}

int handle_get_camera_parameters(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	return DF_FAILED;
    }

    read_calib_param();
	
    int ret = send_buffer(client_sock, (char*)(&param), sizeof(param));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	return DF_FAILED;
    }
    return DF_SUCCESS;

}

/*****************************************************************************************/
//system config param 
int handle_get_system_config_parameters(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	return DF_FAILED;
    }

    read_calib_param();
	
    int ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().config_param_), sizeof(system_config_settings_machine_.Instance().config_param_));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	return DF_FAILED;
    }
    return DF_SUCCESS;

}

bool set_system_config(SystemConfigParam &rect_config_param)
{

    //set led current
    if(rect_config_param.led_current != system_config_settings_machine_.Instance().config_param_.led_current)
    { 
        if(0<= rect_config_param.led_current && rect_config_param.led_current< 1024)
        {
            brightness_current = rect_config_param.led_current;
            lc3010.SetLedCurrent(brightness_current,brightness_current,brightness_current);

            system_config_settings_machine_.Instance().config_param_.led_current = brightness_current;
        }
 
    }

    //set many exposure param
    system_config_settings_machine_.Instance().config_param_.exposure_num = rect_config_param.exposure_num;

    std::memcpy(system_config_settings_machine_.Instance().config_param_.exposure_param , rect_config_param.exposure_param,sizeof(rect_config_param.exposure_param));
 
 

    return true;
}

int handle_set_system_config_parameters(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	
     

    SystemConfigParam rect_config_param;


    int ret = recv_buffer(client_sock, (char*)(&rect_config_param), sizeof(rect_config_param));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }

    bool ok = set_system_config(rect_config_param);    

    if(!ok)
    {
        return DF_FAILED;
    }

    return DF_SUCCESS;

}


/*****************************************************************************************/

int write_calib_param()
{
    std::ofstream ofile;
    ofile.open("calib_param.txt");
    int n_params = sizeof(param)/sizeof(float);
    for(int i=0; i<n_params; i++)
    {
	    ofile<<(((float*)(&param))[i])<<std::endl;
    }
    ofile.close();
    return DF_SUCCESS;
}
    
int handle_set_camera_parameters(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	return DF_FAILED;
    }
	
    int ret = recv_buffer(client_sock, (char*)(&param), sizeof(param));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	return DF_FAILED;
    }
    write_calib_param();

    return DF_SUCCESS;

}

/*****************************************************************************************/
bool config_checkerboard(bool enable)
{
    if (enable) {
        lc3010.enable_checkerboard();
    } else {
        lc3010.disable_checkerboard();
        lc3010.init();
    }

    return true;
}
//*****************************************************************************************/
// Enable and disable checkerboard, by wangtong, 2022-01-27
//*****************************************************************************************/
int handle_enable_checkerboard(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }

    LOG(INFO)<<"enable checkerboard!";
    config_checkerboard(true);

    float temperature = read_temperature(1);
    int ret = send_buffer(client_sock, (char*)(&temperature), sizeof(temperature));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    
    return DF_SUCCESS;
}

int handle_disable_checkerboard(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }

    LOG(INFO)<<"disable checkerboard!";
    config_checkerboard(false);

    float temperature = read_temperature(2);
    int ret = send_buffer(client_sock, (char*)(&temperature), sizeof(temperature));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    
    return DF_SUCCESS;
}
/*****************************************************************************************/
bool set_internal_pattern_stop()
{
    bool ack = true;

    lc3010.set_internal_pattern_stop();

    return ack;
}

bool set_flash_data_type()
{
    bool ack = true;

    lc3010.set_flash_data_type();

    return ack;
}

bool set_flash_build_data_size(unsigned int data_size)
{
    bool ack = true;

    ack = lc3010.set_flash_build_data_size(data_size);

    return ack;
}

bool set_erase_flash()
{
    bool ack = true;

    lc3010.set_erase_flash();

    return ack;
}

bool check_erase_flash_status()
{
    bool ack = true;

    ack = lc3010.check_erase_flash_status();

    return ack;
}

bool set_flash_data_length(unsigned short dataLen)
{
    bool ack = true;

    lc3010.set_flash_data_length(dataLen);

    return ack;
}

bool write_internal_pattern_data_into_the_flash(char *WriteData, unsigned int data_size)
{
    bool ack = true;
    int i = 0;
    unsigned int send_package = data_size / WRITE_PACKAGE_SIZE;
    unsigned int send_separately = data_size % WRITE_PACKAGE_SIZE;

 //   char string[50] = {'\0'};
    int wtCnt = 0;
    wtCnt = lc3010.write_data_into_the_flash(Write_Flash_Start, WriteData, WRITE_PACKAGE_SIZE);
 //   sprintf(string, "Write_Flash_Start size: %d", wtCnt);
 //   LOG(INFO)<<string;

    for (i = 1; i < send_package; i++) {
        lc3010.write_data_into_the_flash(Write_Flash_Continue, &WriteData[i*WRITE_PACKAGE_SIZE], WRITE_PACKAGE_SIZE);
    }

    lc3010.set_flash_data_length(send_separately);
    wtCnt = lc3010.write_data_into_the_flash(Write_Flash_Continue, &WriteData[i*WRITE_PACKAGE_SIZE], send_separately);
//    sprintf(string, "Write_Flash_Continue size: %d", wtCnt);
//    LOG(INFO)<<string;
/*
	FILE* fw;
	fw = fopen("pattern_data_1.dat", "wb");
    if (fw != NULL) {
		fwrite(WriteData, 1, data_size, fw);
		fclose(fw);
	}
	else {
        LOG(INFO)<< "save pattern data fail";
	}

    LOG(INFO)<< "data size--" << data_size;
    LOG(INFO)<< "send_package--" << send_package;
    LOG(INFO)<< "send_separately--" << send_separately;
*/
    return ack;
}

bool read_internal_pattern_data_from_the_flash(char *ReadData, unsigned int data_size)
{
    bool ack = true;
    int i = 0;
    unsigned int read_package = data_size / READ_PACKAGE_SIZE;
    unsigned int read_separately = data_size % READ_PACKAGE_SIZE;

    lc3010.read_data_from_the_flash(Read_Flash_Start, ReadData, READ_PACKAGE_SIZE);

    for (i = 1; i < read_package; i++) {
        lc3010.read_data_from_the_flash(Read_Flash_Continue, &ReadData[i*READ_PACKAGE_SIZE], READ_PACKAGE_SIZE);
    }

    lc3010.set_flash_data_length(read_separately);
    lc3010.read_data_from_the_flash(Read_Flash_Continue, &ReadData[i*READ_PACKAGE_SIZE], read_separately);
/*
	FILE* fw;
	fw = fopen("read_pattern_data.dat", "wb");
    if (fw != NULL) {
		fwrite(ReadData, 1, data_size, fw);
		fclose(fw);
	}
	else {
        LOG(INFO)<< "save pattern data fail";
	}
*/
    return ack;
}

bool reload_pattern_order_table_from_flash()
{
    bool ack = true;

    lc3010.reload_pattern_order_table_from_flash();

    return ack;
}
/*****************************************************************************************/
bool load_pattern_process(char *ReadData, unsigned int data_size, char *string)
{
    bool ack = true;

    // step1 -- Set Internal Pattern Stop, Do not repeat (run once)
    if ( !set_internal_pattern_stop() )
    {
        strcpy(string, "step1 -- set_internal_pattern_stop error.");
        return false;
    }

    // step2 -- Flash Data Type (D0h for pattern data)
    if ( !set_flash_data_type() )
    {
        strcpy(string, "step2 -- set_flash_data_type error.");
        return false;
    }

    // step3 -- set read internal pattern data length, 256 bytes once opation.
    if ( !set_flash_data_length(0x0100) )
    {
        strcpy(string, "step5 -- set_flash_data_length error.");
        return false;
    }

    // step4 -- start to read flash data, 256 bytes once opation. 
    if ( !read_internal_pattern_data_from_the_flash(ReadData, data_size) )
    {
        strcpy(string, "step6 -- read_internal_pattern_data_from_the_flash error.");
        return false;
    }

    return ack;
}

bool program_pattern_process(char *WriteData, char *ReadData, unsigned int data_size, char *string)
{
    bool ack = true;
 
    // step1 -- Set Internal Pattern Stop, Do not repeat (run once)
    if ( !set_internal_pattern_stop() )
    {
        strcpy(string, "step1 -- set_internal_pattern_stop error.");
        return false;
    }

    // step2 -- Flash Data Type (D0h for pattern data)
    if ( !set_flash_data_type() )
    {
        strcpy(string, "step2 -- set_flash_data_type error.");
        return false;
    }

    // step3 -- set Flash Build Data Size (LSB ~ MSB), return err or not. 
    if ( !set_flash_build_data_size(data_size) )
    {
        strcpy(string, "step3 -- set_flash_build_data_size error.");
        return false;
    }

    // step4 -- Flash Data Type (D0h for pattern data)
    if ( !set_flash_data_type() )
    {
        strcpy(string, "step4 -- set_flash_data_type error.");
        return false;
    }

    // step5 -- Signature: Value = AAh, BBh, CCh, DDh.
    if ( !set_erase_flash() )
    {
        strcpy(string, "step5 -- set_erase_flash error.");
        return false;
    }

    // step6 -- erase flash status check.
    if ( !check_erase_flash_status() )
    {
        strcpy(string, "step6 -- check_erase_flash_status error.");
        return false;
    }

    // step7 -- Flash Data Type (D0h for pattern data)
    if ( !set_flash_data_type() )
    {
        strcpy(string, "step7 -- set_flash_data_type error.");
        return false;
    }

    // step8 -- Set  Flash Data Length 256 bytes.
    if ( !set_flash_data_length(0x0100) )
    {
        strcpy(string, "step8 -- set_flash_data_length error.");
        return false;
    }

    // step9 -- write internal pattern data into the flash. Write 256 bytes once operation.
    if ( !write_internal_pattern_data_into_the_flash(WriteData, data_size) )
    {
        strcpy(string, "step9 -- write_internal_pattern_data_into_the_flash error.");
        return false;
    }

    // step10 -- Flash Data Type (D0h for pattern data)
    if ( !set_flash_data_type() )
    {
        strcpy(string, "step2 -- set_flash_data_type error.");
        return false;
    }

    // step11 -- set read internal pattern data length, 256 bytes once opation.
    if ( !set_flash_data_length(0x0100) )
    {
        strcpy(string, "step5 -- set_flash_data_length error.");
        return false;
    }

    // step12 -- start to read flash data, 256 bytes once opation. 
    if ( !read_internal_pattern_data_from_the_flash(ReadData, data_size) )
    {
        strcpy(string, "step6 -- read_internal_pattern_data_from_the_flash error.");
        return false;
    }

    // step13 --Reload from flash
    if ( !reload_pattern_order_table_from_flash() )
    {
        strcpy(string, "step13 -- reload_pattern_order_table_from_flash error.");
        return false;
    }

    return ack;
}
/*****************************************************************************************/
int handle_load_pattern_data(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }

    unsigned int data_size;
    int ret = recv_buffer(client_sock, (char*)(&data_size), sizeof(data_size));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }

    char string[100] = {'\0'};
    sprintf(string, "load pattern data size: 0x%X", data_size);
    char *ReadData = new char[data_size];
    memset(ReadData, 0, data_size);
    load_pattern_process(ReadData, data_size, string);

    LOG(INFO)<<string;

    ret = send_buffer(client_sock, ReadData, data_size);
    delete [] ReadData;
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    
    return DF_SUCCESS;
}

int handle_program_pattern_data(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }

    unsigned int pattern_size;
    int ret = recv_buffer(client_sock, (char*)(&pattern_size), sizeof(pattern_size));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"recv error, close this connection!\n";
	    return DF_FAILED;
    }

    char string[100] = {'\0'};
    sprintf(string, "program pattern data size: 0x%X", pattern_size);

    char *org_buffer = new char[pattern_size];
    char *back_buffer = new char[pattern_size];
    memset(back_buffer, 0, pattern_size);
/*
	FILE* fr;
    fr = fopen("pattern_data.dat", "rb");
	if (fr != NULL) {
		fread(org_buffer, 1, pattern_size, fr);
		fclose(fr);
	}
	else {
		sprintf(string, "read pattern data fail");
	}
*/
    ret = recv_buffer(client_sock, org_buffer, pattern_size);
    if (ret == DF_FAILED)
    {
        delete [] org_buffer;
        delete [] back_buffer;
        LOG(INFO)<<"recv error, close this connection!\n";
	    return DF_FAILED;
    }

    program_pattern_process(org_buffer, back_buffer, pattern_size, string);
    LOG(INFO)<<string;

    ret = send_buffer(client_sock, back_buffer, pattern_size);
    delete [] org_buffer;
    delete [] back_buffer;
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    
    return DF_SUCCESS;
}
/*****************************************************************************************/
int read_bandwidth()
{
    int val = 0;
    char data[100];

    std::ifstream infile;
    infile.open("/sys/class/net/eth0/speed");
    infile >> data;
    val = (int)std::atoi(data);
    infile.close();

    return val;
}

int handle_get_network_bandwidth(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }

    LOG(INFO)<<"get network bandwidth!";

    int speed = read_bandwidth();
    int ret = send_buffer(client_sock, (char*)(&speed), sizeof(speed));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    
    return DF_SUCCESS;
}
/*****************************************************************************************/
int handle_commands(int client_sock)
{
    int command;
    int ret = recv_command(client_sock, &command); 
    LOG(INFO)<<"command:"<<command;
    
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"connection command not received";
	    close(client_sock);
        return DF_FAILED;
    }

    switch(command)
    {
	case DF_CMD_CONNECT:
	    LOG(INFO)<<"DF_CMD_CONNECT";
	    handle_cmd_connect(client_sock);
	    break;
	case DF_CMD_DISCONNECT:
	    LOG(INFO)<<"DF_CMD_DISCONNECT";
	    handle_cmd_disconnect(client_sock);
	    break;
	case DF_CMD_GET_BRIGHTNESS:
	    LOG(INFO)<<"DF_CMD_GET_BRIGHTNESS";
	    handle_cmd_get_brightness(client_sock);
	    break;
	case DF_CMD_GET_RAW:
	    LOG(INFO)<<"DF_CMD_GET_RAW";
    //	    camera.warmupCamera();
	    handle_cmd_get_raw(client_sock);
	    break;
	case DF_CMD_GET_RAW_TEST:
	    LOG(INFO)<<"DF_CMD_GET_RAW_TEST";
    //	    camera.warmupCamera();
	    handle_cmd_get_raw_02(client_sock);
	    break;
	case DF_CMD_GET_RAW_03:
	    LOG(INFO)<<"DF_CMD_GET_RAW_03";
    //	    camera.warmupCamera();
	    handle_cmd_get_raw_03(client_sock);
	    break;
	case DF_CMD_GET_FRAME_01:
	    LOG(INFO)<<"DF_CMD_GET_FRAME_01";
    //	    camera.warmupCamera();
	    handle_cmd_get_frame_01(client_sock);
        // handle_cmd_get_frame_03_more_exposure(client_sock);
	    break;
    case DF_CMD_GET_FRAME_HDR:
	    LOG(INFO)<<"DF_CMD_GET_FRAME_HDR"; 
    	// handle_cmd_get_frame_03_more_exposure(client_sock);
    	handle_cmd_get_frame_03_hdr_parallel(client_sock);
	    break;
 
	case DF_CMD_GET_FRAME_03:
	    LOG(INFO)<<"DF_CMD_GET_FRAME_03";  
    	// handle_cmd_get_frame_03(client_sock);
    	handle_cmd_get_frame_03_parallel(client_sock);
	    break;
	case DF_CMD_GET_POINTCLOUD:
	    LOG(INFO)<<"DF_CMD_GET_POINTCLOUD";
  //  	    camera.warmupCamera();
	    handle_cmd_get_point_cloud(client_sock);
	    break;
	case DF_CMD_HEARTBEAT:
	    LOG(INFO)<<"DF_CMD_HEARTBEAT";
	    handle_heartbeat(client_sock);
	    break;
	case DF_CMD_GET_TEMPERATURE:
	    LOG(INFO)<<"DF_CMD_GET_TEMPERATURE";
	    handle_get_temperature(client_sock);
	    break;
	case DF_CMD_GET_CAMERA_PARAMETERS:
	    LOG(INFO)<<"DF_CMD_GET_CAMERA_PARAMETERS";
	    handle_get_camera_parameters(client_sock);
	    break;
	case DF_CMD_SET_CAMERA_PARAMETERS:
	    LOG(INFO)<<"DF_CMD_SET_CAMERA_PARAMETERS";
	    handle_set_camera_parameters(client_sock);
        read_calib_param();
            cuda_copy_calib_data(param.camera_intrinsic, 
		         param.projector_intrinsic, 
			 param.camera_distortion,
	                 param.projector_distortion, 
			 param.rotation_matrix, 
			 param.translation_matrix);
	    break;

    // -----------------------------------------------------------
    // -- Enable and disable checkerboard, by wangtong, 2022-01-27 
	case DF_CMD_ENABLE_CHECKER_BOARD:
	    LOG(INFO)<<"DF_CMD_ENABLE_CHECKER_BOARD";
	    handle_enable_checkerboard(client_sock);
	    break;

	case DF_CMD_DISABLE_CHECKER_BOARD:
	    LOG(INFO)<<"DF_CMD_DISABLE_CHECKER_BOARD";
	    handle_disable_checkerboard(client_sock);
	    break;
    // -----------------------------------------------------------

    case DF_CMD_LOAD_PATTERN_DATA:
	    LOG(INFO)<<"DF_CMD_LOAD_PATTERN_DATA";
	    handle_load_pattern_data(client_sock);
        break;

    case DF_CMD_PROGRAM_PATTERN_DATA:
	    LOG(INFO)<<"DF_CMD_PROGRAM_PATTERN_DATA";
	    handle_program_pattern_data(client_sock);
        break;

	case DF_CMD_GET_NETWORK_BANDWIDTH:
	    LOG(INFO)<<"DF_CMD_GET_NETWORK_BANDWIDTH";
	    handle_get_network_bandwidth(client_sock);
	    break;

	case DF_CMD_GET_SYSTEM_CONFIG_PARAMETERS:
	    LOG(INFO)<<"DF_CMD_GET_SYSTEM_CONFIG_PARAMETERS";
	    handle_get_system_config_parameters(client_sock);
	    break;
	case DF_CMD_SET_SYSTEM_CONFIG_PARAMETERS:
	    LOG(INFO)<<"DF_CMD_SET_SYSTEM_CONFIG_PARAMETERS";
	    handle_set_system_config_parameters(client_sock);
        saveSystemConfig();
	    break;
	default:
	    LOG(INFO)<<"DF_CMD_UNKNOWN";
	    break;
    }

    close(client_sock);
    return DF_SUCCESS;
}

int init()
{
    readSystemConfig();

    brightness_current = system_config_settings_machine_.Instance().config_param_.led_current;

    camera.openCamera();
    camera.warmupCamera();
    lc3010.SetLedCurrent(brightness_current,brightness_current,brightness_current);
    cuda_malloc_memory();
    read_calib_param();
       
    cuda_copy_calib_data(param.camera_intrinsic, 
		         param.projector_intrinsic, 
			 param.camera_distortion,
	                 param.projector_distortion, 
			 param.rotation_matrix, 
			 param.translation_matrix);

    float temperature_val = read_temperature(0); 
    LOG(INFO)<<"temperature: "<<temperature_val<<" deg";

    return DF_SUCCESS;
}

int main()
{
    LOG(INFO)<<"server started";
    init();
    LOG(INFO)<<"inited";

    int server_sock;
    do
    {
        server_sock = setup_socket(DF_PORT);
        sleep(1);
    }
    while(server_sock == DF_FAILED);
    std::cout<<"listening"<<std::endl;
    
    while(true)
    {
        int client_sock = accept_new_connection(server_sock);
        if(client_sock!=-1)
	    {
            handle_commands(client_sock);
        }
    }

    close(server_sock);

    return 0;
}
