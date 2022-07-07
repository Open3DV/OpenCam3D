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
// #include "lightcrafter3010.h"
#include "easylogging++.h"
#include "encode_cuda.cuh"
#include "system_config_settings.h"
#include "version.h"
#include "configure_standard_plane.h"
#include "../test/LookupTableFunction.h" 
#include "configure_auto_exposure.h"
#include <JetsonGPIO.h>
#include "scan3d.h"

INITIALIZE_EASYLOGGINGPP
#define OUTPUT_PIN     12       // BOARD pin 32, BCM pin 12


Scan3D scan3d_machine_;

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
float generate_brightness_exposure_time = 12000;
int generate_brightness_model = 1;
 
float max_camera_exposure_ = 28000;
float min_camera_exposure_ = 6000;
 

SystemConfigDataStruct system_config_settings_machine_;

bool readSystemConfig()
{
    return system_config_settings_machine_.loadFromSettings("../system_config.ini");
}

bool saveSystemConfig()
{
    return system_config_settings_machine_.saveToSettings("../system_config.ini");
}



bool findMaskBaseConfidence(cv::Mat confidence_map, int threshold, cv::Mat& mask)
{
	if (confidence_map.empty())
	{
		return true;
	}

	int nr = confidence_map.rows;
	int nc = confidence_map.cols;


	cv::Mat bin_map;

	cv::threshold(confidence_map, bin_map, threshold, 255, cv::THRESH_BINARY);
	bin_map.convertTo(bin_map, CV_8UC1);

	std::vector<std::vector<cv::Point>> contours;

	cv::findContours(
		bin_map,
		contours,
		cv::noArray(),
		cv::RETR_EXTERNAL,
		cv::CHAIN_APPROX_SIMPLE
	);

	std::vector<cv::Point> max_contours;
	int max_contours_size = 0;

	for (int i = 0; i < contours.size(); i++)
	{
		if (contours[i].size() > max_contours_size)
		{
			max_contours_size = contours[i].size();
			max_contours = contours[i];
		}

	}

	contours.clear();
	contours.push_back(max_contours);

	cv::Mat show_contours(nr, nc, CV_8U, cv::Scalar(0));
	cv::drawContours(show_contours, contours, -1, cv::Scalar(255), -1);

	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::Mat result;
	cv::erode(show_contours, result, element);

	mask = result.clone();


	return true;
}


bool set_camera_version(int version)
{
    switch (version)
    {
    case DFX_800:
    {
        cuda_set_camera_version(DFX_800);
        max_camera_exposure_ = 60000;
        min_camera_exposure_ = 6000;
        return true;
    }
    break;

    case DFX_1800:
    {

        cuda_set_camera_version(DFX_1800);
        max_camera_exposure_ = 28000; 
        min_camera_exposure_ = 6000;
        return true;
    }
    break;

    default:
        break;
    }

    return false;
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

int handle_cmd_unknown(int client_sock)
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
        ret = send_command(client_sock, DF_CMD_UNKNOWN);
        return DF_SUCCESS;
    }
    else
    {
        std::cout<<"reject"<<std::endl;
        ret = send_command(client_sock, DF_CMD_REJECT);
        return DF_FAILED;
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



int handle_cmd_set_auto_exposure_base_board(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    int buffer_size = 1920 * 1200;
    char *buffer = new char[buffer_size];

    ConfigureAutoExposure auto_exposure_machine;
    float average_pixel = 0;
    float over_exposure_rate = 0;


    cv::Mat brightness_mat(1200,1920,CV_8U,cv::Scalar(0));

    float current_exposure = system_config_settings_machine_.Instance().config_param_.camera_exposure_time;

    //发光，自定义曝光时间
    lc3010.enable_solid_field();
    bool capture_one_ret = camera.captureSingleExposureImage(current_exposure, (char*)brightness_mat.data);

    auto_exposure_machine.evaluateBrightnessParam(brightness_mat,cv::Mat(),average_pixel,over_exposure_rate);

    lc3010.disable_solid_field();

    return DF_SUCCESS;
}

int handle_cmd_set_auto_exposure_base_roi_half(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    int buffer_size = 1920 * 1200;
    char *buffer = new char[buffer_size];

    ConfigureAutoExposure auto_exposure_machine;
    float average_pixel = 0;
    float over_exposure_rate = 0;

    float high_max_over_rate = 1.0;
    float high_min_over_rate = 0.7;

    float low_max_over_rate = 0.3;
    float low_min_over_rate = 0.2;
 
    cv::Mat brightness_mat(1200,1920,CV_8U,cv::Scalar(0));


    int current_exposure = (min_camera_exposure_ + max_camera_exposure_)/2;

    int adjust_exposure_val = current_exposure;

    int low_limit_exposure = min_camera_exposure_;
    int high_limit_exposure = max_camera_exposure_;

    //电流值设置到最大
    if (brightness_current < 1023 && current_exposure != min_camera_exposure_)
    {
        brightness_current = 1023;
        lc3010.SetLedCurrent(brightness_current, brightness_current, brightness_current);
        system_config_settings_machine_.Instance().config_param_.led_current = brightness_current;
    }

    //发光，自定义曝光时间
    lc3010.enable_solid_field();
    bool capture_one_ret = camera.captureSingleExposureImage(current_exposure, (char*)brightness_mat.data); 
    auto_exposure_machine.evaluateBrightnessParam(brightness_mat,cv::Mat(),average_pixel,over_exposure_rate);


    int iterations_num = 0;
    while(iterations_num< 16)
    {
        if(average_pixel> 127.5 && average_pixel < 128.5)
        {
            break;
        }

        if(average_pixel<= 127.5)
        {
            //偏暗
            low_limit_exposure = current_exposure;
            adjust_exposure_val /= 2;
            current_exposure = current_exposure + adjust_exposure_val;
        }
        else if(average_pixel >= 128.5)
        {
            //偏亮
            high_limit_exposure = current_exposure;
            adjust_exposure_val /= 2;
            current_exposure = current_exposure - adjust_exposure_val;

        }

        capture_one_ret = camera.captureSingleExposureImage(current_exposure, (char*)brightness_mat.data); 
        auto_exposure_machine.evaluateBrightnessParam(brightness_mat,cv::Mat(),average_pixel,over_exposure_rate);

        iterations_num++;

        LOG(INFO) << "adjust_exposure_val: " << adjust_exposure_val;
        LOG(INFO) << "current_exposure: " << current_exposure;
        LOG(INFO) << "low_limit_exposure: " << low_limit_exposure;
        LOG(INFO) << "high_limit_exposure: " << high_limit_exposure;
        LOG(INFO) << "iterations_num: " << iterations_num;
        LOG(INFO) << "over_exposure_rate: " << over_exposure_rate;
        LOG(INFO) << "average_pixel: " << average_pixel;
        LOG(INFO) << "";
    }
    
    /**************************************************************************************************/

    if(average_pixel> 127.5 && average_pixel < 128.5)
    {

        //根据过曝光情况调整
        if (over_exposure_rate < low_min_over_rate)
        {
            //需要加亮
            low_limit_exposure = current_exposure;
            high_limit_exposure = max_camera_exposure_;
            current_exposure = (high_limit_exposure - low_limit_exposure) / 2;
            adjust_exposure_val = current_exposure;
        }
        else if (over_exposure_rate > high_max_over_rate)
        {
            //需要变暗
            low_limit_exposure = min_camera_exposure_;
            high_limit_exposure = current_exposure;
            current_exposure = (high_limit_exposure - low_limit_exposure) / 2;
            adjust_exposure_val = current_exposure;
        }

        capture_one_ret = camera.captureSingleExposureImage(current_exposure, (char *)brightness_mat.data);
        auto_exposure_machine.evaluateBrightnessParam(brightness_mat, cv::Mat(), average_pixel, over_exposure_rate);

        
        iterations_num = 0;
        while (iterations_num < 16)
        {
            if (over_exposure_rate >= low_min_over_rate && over_exposure_rate < high_max_over_rate)
            {
                break;
            }

            if (over_exposure_rate < low_min_over_rate)
            {
                //偏暗
                low_limit_exposure = current_exposure;
                adjust_exposure_val /= 2;
                current_exposure = current_exposure + adjust_exposure_val;
            }
            else if (over_exposure_rate >= high_max_over_rate)
            {
                //偏亮
                high_limit_exposure = current_exposure;
                adjust_exposure_val /= 2;
                current_exposure = current_exposure - adjust_exposure_val;
            }

            capture_one_ret = camera.captureSingleExposureImage(current_exposure, (char *)brightness_mat.data);
            auto_exposure_machine.evaluateBrightnessParam(brightness_mat, cv::Mat(), average_pixel, over_exposure_rate);

            iterations_num++;

            LOG(INFO) << "adjust_exposure_val: " << adjust_exposure_val;
            LOG(INFO) << "current_exposure: " << current_exposure;
            LOG(INFO) << "low_limit_exposure: " << low_limit_exposure;
            LOG(INFO) << "high_limit_exposure: " << high_limit_exposure;
            LOG(INFO) << "iterations_num: " << iterations_num;
            LOG(INFO) << "over_exposure_rate: " << over_exposure_rate;
            LOG(INFO) << "average_pixel: " << average_pixel;
            LOG(INFO) << "";
        }
    }
    else if(average_pixel>= 128.5)
    {
        //太亮、曝光时间设置成最小值、调整led值
        current_exposure = min_camera_exposure_;

        int low_limit_led = 0;
        int high_limit_led = 1023;

        int current_led = (high_limit_led - low_limit_led) / 2;
        int adjust_led_val = current_led;

        brightness_current = current_led;
        lc3010.SetLedCurrent(brightness_current, brightness_current, brightness_current); 

        capture_one_ret = camera.captureSingleExposureImage(current_exposure, (char *)brightness_mat.data);
        auto_exposure_machine.evaluateBrightnessParam(brightness_mat, cv::Mat(), average_pixel, over_exposure_rate);

        iterations_num = 0;
        while (iterations_num < 10)
        {
            if (average_pixel > 127.5 && average_pixel < 128.5)
            {
                break;
            }

        if(average_pixel<= 127.5)
        {
            //偏暗
            low_limit_led = current_led;
            adjust_led_val /= 2;
            current_led = current_led + adjust_led_val;
        }
        else if(average_pixel >= 128.5)
        {
            //偏亮
            high_limit_led = current_led;
            adjust_led_val /= 2;
            current_led = current_led - adjust_led_val;

        }

        brightness_current = current_led;
        lc3010.SetLedCurrent(brightness_current, brightness_current, brightness_current); 
        capture_one_ret = camera.captureSingleExposureImage(current_exposure, (char*)brightness_mat.data); 
        auto_exposure_machine.evaluateBrightnessParam(brightness_mat,cv::Mat(),average_pixel,over_exposure_rate);

        iterations_num++;

        LOG(INFO) << "adjust_led_val: " << adjust_led_val;
        LOG(INFO) << "current_exposure: " << current_exposure;
        LOG(INFO) << "low_limit_led: " << low_limit_exposure;
        LOG(INFO) << "high_limit_led: " << high_limit_led;
        LOG(INFO) << "iterations_num: " << iterations_num;
        LOG(INFO) << "over_exposure_rate: " << over_exposure_rate;
        LOG(INFO) << "average_pixel: " << average_pixel;
        LOG(INFO) << "";
        }

    }
    else if(average_pixel< 127.5)
    {
        //太暗，曝光设置成最大值
        current_exposure = max_camera_exposure_;
        capture_one_ret = camera.captureSingleExposureImage(current_exposure, (char *)brightness_mat.data);
    }

    /***************************************************************************************************/

    
    lc3010.disable_solid_field();

    system_config_settings_machine_.Instance().config_param_.led_current = brightness_current;

    int auto_exposure = current_exposure;
    int auto_led = brightness_current;

    int ret = send_buffer(client_sock, (char*)(&auto_exposure), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
 
    ret = send_buffer(client_sock, (char*)(&auto_led), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }


    return DF_SUCCESS;

}

int handle_cmd_set_auto_exposure_base_roi_pid(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    int buffer_size = 1920 * 1200;
    char *buffer = new char[buffer_size];

    ConfigureAutoExposure auto_exposure_machine;
    float average_pixel = 0;
    float over_exposure_rate = 0;


    cv::Mat brightness_mat(1200,1920,CV_8U,cv::Scalar(0));

    float current_exposure = system_config_settings_machine_.Instance().config_param_.camera_exposure_time;

    //发光，自定义曝光时间
    lc3010.enable_solid_field();
    bool capture_one_ret = camera.captureSingleExposureImage(current_exposure, (char*)brightness_mat.data); 
    auto_exposure_machine.evaluateBrightnessParam(brightness_mat,cv::Mat(),average_pixel,over_exposure_rate);
 
    int adjust_exposure_val = 1000;
    int adjust_led_= 128;

    int adjust_led_val = 1000; 
    int current_led = 0;
    
    float high_max_over = 1.0;
    float high_min_over = 0.7;

    float low_max_over = 0.3;
    float low_min_over = 0.2;
 /*******************************************************************************************************************/
   
    /*******************************************************************************************************************/
    //pid 调节到128
    float Kp = 200;
    float Ki = 0.005;
    float Kd = 0.1;

    float error_p =0;
    float error_i =0;
    float error_d =0;
    float error_dp =0; 
    int iterations_num = 0;

	while (iterations_num< 30) {
        error_p = 128 - average_pixel;
        error_i += error_p;
        error_d = error_p - error_dp;
        error_dp = error_p;

        adjust_exposure_val = Kp * error_p + Ki * error_i + Kd * error_d;
        current_exposure += Kp * error_p + Ki * error_i + Kd * error_d;

        if (brightness_current < 1023 && current_exposure != min_camera_exposure_)
        {
            brightness_current = 1023;
            lc3010.SetLedCurrent(brightness_current, brightness_current, brightness_current);
            system_config_settings_machine_.Instance().config_param_.led_current = brightness_current;
        }

        if (current_exposure > max_camera_exposure_)
        {
            current_exposure = max_camera_exposure_;
        }

        if(current_exposure< min_camera_exposure_)
        {
            current_exposure = min_camera_exposure_;
        }


        iterations_num++;
        capture_one_ret = camera.captureSingleExposureImage(current_exposure, (char *)brightness_mat.data);
        auto_exposure_machine.evaluateBrightnessParam(brightness_mat, cv::Mat(), average_pixel, over_exposure_rate);

        LOG(INFO) << "adjust_exposure_val: " << adjust_exposure_val;
        LOG(INFO) << "current_exposure: " << current_exposure;
        LOG(INFO) << "current_led: " << brightness_current;
        LOG(INFO) << "iterations_num: " << iterations_num;
        LOG(INFO) << "over_exposure_rate: " << over_exposure_rate;
        LOG(INFO) << "average_pixel: " << average_pixel;
        LOG(INFO) << "";

        if(average_pixel< 128.5 && average_pixel> 127.5)
        {
            break;
        }

        //最大亮度还不够
        if(average_pixel< 127.5 && current_exposure == max_camera_exposure_)
        {
            break;
        }

        //最小还是过曝光
        if(average_pixel> 127.5 && current_exposure == min_camera_exposure_ && over_exposure_rate > high_max_over)
        {
            break;
        } 
    } 

    /********************************************************************************************************************/
    //过调led
    //调LED
    if (current_exposure == min_camera_exposure_ && over_exposure_rate > high_max_over)
    {
        //pid 调节到128
        float led_Kp = 6;
        float led_Ki = 0.5;
        float led_Kd = 1;

        float led_error_p = 0;
        float led_error_i = 0;
        float led_error_d = 0;
        float led_error_dp = 0;
        int led_iterations_num = 0;

        while (led_iterations_num < 30)
        {
            led_error_p = 128 - average_pixel;
            led_error_i += led_error_p;
            led_error_d = led_error_p - led_error_dp;
            led_error_dp = led_error_p;

            adjust_led_val = led_Kp * led_error_p + led_Ki * led_error_i + led_Kd * led_error_d;
            current_led += led_Kp * led_error_p + led_Ki * led_error_i + led_Kd * led_error_d;
 
            if (current_led > 1023)
            {
                current_led = 1023;
            }

            if(current_led< 0)
            {
                current_led = 0;
            }


            led_iterations_num++;
            lc3010.SetLedCurrent(current_led, current_led, current_led);
            capture_one_ret = camera.captureSingleExposureImage(current_exposure, (char *)brightness_mat.data);
            auto_exposure_machine.evaluateBrightnessParam(brightness_mat, cv::Mat(), average_pixel, over_exposure_rate);

            LOG(INFO) <<""; 
            LOG(INFO) << "adjust_led_val: " << adjust_led_val;
            LOG(INFO) << "current_exposure: " << current_exposure;
            LOG(INFO) << "current_led: " << current_led;
            LOG(INFO) << "iterations_num: " << led_iterations_num;
            LOG(INFO) << "over_exposure_rate: " << over_exposure_rate;
            LOG(INFO) << "average_pixel: " << average_pixel;
            LOG(INFO) <<""; 

            if (average_pixel < 128.5 && average_pixel > 127.5)
            {
                break;
            }

            //最大亮度还不够
            if (average_pixel < 127.5 && current_led == 1023)
            {
                break;
            }

            //最小还是过曝光
            if (average_pixel > 127.5 && current_led == 0 && over_exposure_rate > high_max_over)
            {
                break;
            }
        }
    }

   
    /********************************************************************************************************************/
    //微调
    if (average_pixel > 128 && over_exposure_rate < low_min_over)
    {
        //调节到low_min_over

        float Kp = 200;
        float Ki = 0.5;
        float Kd = 1;

        float error_p = 0;
        float error_i = 0;
        float error_d = 0;
        float error_dp = 0;
        int iterations_num = 0;

        while (iterations_num < 30)
        {
            error_p = over_exposure_rate - (low_min_over + low_max_over) / 2.0;
            error_i += error_p;
            error_d = error_p - error_dp;
            error_dp = error_p;

            adjust_exposure_val = Kp * error_p + Ki * error_i + Kd * error_d;
            current_exposure += Kp * error_p + Ki * error_i + Kd * error_d;

            if (current_exposure > max_camera_exposure_)
            {
                current_exposure = max_camera_exposure_;
            }

            if(current_exposure< min_camera_exposure_)
            {
                current_exposure = min_camera_exposure_;
            }
            iterations_num++;
            capture_one_ret = camera.captureSingleExposureImage(current_exposure, (char *)brightness_mat.data);
            auto_exposure_machine.evaluateBrightnessParam(brightness_mat, cv::Mat(), average_pixel, over_exposure_rate);

            LOG(INFO) << "adjust_exposure_val: " << adjust_exposure_val;
            LOG(INFO) << "current_exposure: " << current_exposure;
            LOG(INFO) << "current_led: " << brightness_current;
            LOG(INFO) << "iterations_num: " << iterations_num;
            LOG(INFO) << "over_exposure_rate: " << over_exposure_rate;
            LOG(INFO) << "average_pixel: " << average_pixel;
            LOG(INFO) << "";

            //最小还是过曝光
            if (over_exposure_rate > low_min_over && over_exposure_rate < low_max_over)
            {
                break;
            }
        }
    }
    else
    {
        //调节到high_min_over

        float Kp = 200;
        float Ki = 0.5;
        float Kd = 1;

        float error_p = 0;
        float error_i = 0;
        float error_d = 0;
        float error_dp = 0;
        int iterations_num = 0;

        while (iterations_num < 30)
        {
            error_p = over_exposure_rate - (high_min_over + high_max_over) / 2.0;
            error_i += error_p;
            error_d = error_p - error_dp;
            error_dp = error_p;

            adjust_exposure_val = Kp * error_p + Ki * error_i + Kd * error_d;
            current_exposure += Kp * error_p + Ki * error_i + Kd * error_d;

            if (current_exposure > max_camera_exposure_)
            {
                current_exposure = max_camera_exposure_;
            }

            if(current_exposure< min_camera_exposure_)
            {
                current_exposure = min_camera_exposure_;
            }

            iterations_num++;
            capture_one_ret = camera.captureSingleExposureImage(current_exposure, (char *)brightness_mat.data);
            auto_exposure_machine.evaluateBrightnessParam(brightness_mat, cv::Mat(), average_pixel, over_exposure_rate);

            LOG(INFO) << "adjust_exposure_val: " << adjust_exposure_val;
            LOG(INFO) << "current_exposure: " << current_exposure;
            LOG(INFO) << "current_led: " << brightness_current;
            LOG(INFO) << "iterations_num: " << iterations_num;
            LOG(INFO) << "over_exposure_rate: " << over_exposure_rate;
            LOG(INFO) << "average_pixel: " << average_pixel;
            LOG(INFO) << "";

            //最小还是过曝光
            if (over_exposure_rate > high_min_over && over_exposure_rate < high_max_over)
            {
                break;
            }
        }

    }

    lc3010.disable_solid_field();

    int auto_exposure = current_exposure;
    int auto_led = brightness_current;

    int ret = send_buffer(client_sock, (char*)(&auto_exposure), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
 
    ret = send_buffer(client_sock, (char*)(&auto_led), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }


    return DF_SUCCESS;
}

int handle_cmd_get_brightness(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }
    LOG(INFO)<<"capture single image";

    int buffer_size = 1920*1200;
    char* buffer = new char[buffer_size];

    switch (generate_brightness_model)
    {
        case 1:
            {
                //同步扫描曝光
                lc3010.pattern_mode_brightness();
                int image_num= 1;  
                camera.captureRawTest(image_num,buffer);
            }
        break;
        case 2:
            {
                //发光，自定义曝光时间 
                lc3010.enable_solid_field();
                bool capture_one_ret = camera.captureSingleExposureImage(generate_brightness_exposure_time,buffer);
                lc3010.disable_solid_field();
            }
        break;
        case 3:
            {
                //不发光，自定义曝光时间 
                bool capture_one_ret = camera.captureSingleExposureImage(generate_brightness_exposure_time,buffer);
            }
        break;
        
        default:
            break;
    }

 

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



int handle_cmd_get_raw_04_repetition(int client_sock)
{

    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    int repetition_count = 1;
    //接收重复数
    int ret = recv_buffer(client_sock, (char*)(&repetition_count), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }


    lc3010.pattern_mode04_repetition(repetition_count);

    int image_num= 19 + 6*(repetition_count-1);

    //cv::Mat image = get_mat();
    //lc3010.start_pattern_sequence();
    int buffer_size = 1920*1200*image_num;
    char* buffer = new char[buffer_size];
    camera.captureRawTest(image_num,buffer);

    printf("start send image, buffer_size=%d\n", buffer_size);
    ret = send_buffer(client_sock, buffer, buffer_size);
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


int handle_cmd_get_raw_04(int client_sock)
{
    lc3010.pattern_mode04();

    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }


    int image_num= 19;

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

/*******************************************************************************************************************/

int handle_cmd_get_frame_04_hdr_parallel_mixed_led_and_exposure(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    LOG(INFO)<<"Mixed HDR Exposure:"; 
  
    std::vector<int> led_current_list; 
    std::vector<int> camera_exposure_list; 
    
    for(int i= 0;i< system_config_settings_machine_.Instance().firwmare_param_.mixed_exposure_num;i++)
    {
        led_current_list.push_back(system_config_settings_machine_.Instance().firwmare_param_.mixed_led_param_list[i]);
        camera_exposure_list.push_back(system_config_settings_machine_.Instance().firwmare_param_.mixed_exposure_param_list[i]);
 
    }

    int depth_buf_size = 1920*1200*4;  
    int brightness_buf_size = 1920*1200*1;

    float* depth_map = new float[depth_buf_size]; 
    unsigned char* brightness = new unsigned char[brightness_buf_size];


//    std::sort(led_current_list.begin(),led_current_list.end(),std::greater<int>());

    //关闭额外拍摄亮度图
    camera.setGenerateBrightnessParam(1,generate_brightness_exposure_time);

    for(int i= 0;i< led_current_list.size();i++)
    {
        int led_current = led_current_list[i];
        lc3010.SetLedCurrent(led_current,led_current,led_current); 
        
        std::cout << "set led: " << led_current << std::endl;
 
        float exposure = camera_exposure_list[i];

        if (exposure > max_camera_exposure_)
        {
            exposure = max_camera_exposure_;
            LOG(INFO) << "Set Camera Exposure Time Error!"
                      << "\n";
        }
        else if (exposure < min_camera_exposure_)
        {
            exposure = min_camera_exposure_;
            LOG(INFO) << "Set Camera Exposure Time Error!"
                      << "\n";
        }

        LOG(INFO) << "Set Camera Exposure Time: " << exposure << "\n";

        if(camera.setScanExposure(exposure))
        {
            lc3010.set_camera_exposure(exposure);
        } 

        lc3010.pattern_mode04();
    
        camera.captureFrame04ToGpu();   
        parallel_cuda_copy_result_to_hdr(i,18); 
    }

    
    camera.setGenerateBrightnessParam(generate_brightness_model,generate_brightness_exposure_time);


    lc3010.SetLedCurrent(brightness_current, brightness_current, brightness_current); 
    LOG(INFO) << "Set Camera Exposure Time: " << system_config_settings_machine_.Instance().config_param_.camera_exposure_time << "\n"; 
    if (camera.setScanExposure(system_config_settings_machine_.Instance().config_param_.camera_exposure_time))
    {
        lc3010.set_camera_exposure(system_config_settings_machine_.Instance().config_param_.camera_exposure_time);
    }

    cudaDeviceSynchronize();


    parallel_cuda_merge_hdr_data(led_current_list.size(), depth_map, brightness); 

    /******************************************************************************************************/


    switch (generate_brightness_model)
    { 
        case 2:
            {
                //发光，自定义曝光时间 
                lc3010.enable_solid_field();
                bool capture_one_ret = camera.captureSingleExposureImage(generate_brightness_exposure_time,(char*)brightness);
                lc3010.disable_solid_field();
            }
        break;
        case 3:
            {
                //不发光，自定义曝光时间 
                bool capture_one_ret = camera.captureSingleExposureImage(generate_brightness_exposure_time,(char*)brightness);
            }
        break;
        
        default:
            break;
    }
  

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(1200, 1920, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(1200, 1920, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral";
    }

    /***************************************************************************************************/
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


/********************************************************************************************************************/

/*******************************************************************************************************************/

int handle_cmd_get_frame_04_hdr_parallel(int client_sock)
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

 

//    std::sort(led_current_list.begin(),led_current_list.end(),std::greater<int>());
    //关闭额外拍摄亮度图
    camera.setGenerateBrightnessParam(1,generate_brightness_exposure_time);
    for(int i= 0;i< led_current_list.size();i++)
    {
        int led_current = led_current_list[i];
        lc3010.SetLedCurrent(led_current,led_current,led_current);	
        
        std::cout << "set led: " << led_current << std::endl;

        lc3010.pattern_mode04();
    
        camera.captureFrame04ToGpu();   
        parallel_cuda_copy_result_to_hdr(i,18); 
    }

    
    camera.setGenerateBrightnessParam(generate_brightness_model,generate_brightness_exposure_time);
    lc3010.SetLedCurrent(brightness_current,brightness_current,brightness_current);
 
	cudaDeviceSynchronize();
    parallel_cuda_merge_hdr_data(led_current_list.size(), depth_map, brightness); 

    /************************************************************************************/

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(1200, 1920, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(1200, 1920, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral";
    }

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


/********************************************************************************************************************/
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
        parallel_cuda_copy_result_to_hdr(i,30); 
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

int handle_cmd_get_phase_02_repetition_02_parallel(int client_sock)
{
    /**************************************************************************************/

    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }
	
    
    int repetition_count = 1;

    int ret = recv_buffer(client_sock, (char*)(&repetition_count), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }
    LOG(INFO)<<"repetition_count: "<<repetition_count<<"\n";
    /***************************************************************************************/

    int phase_buf_size = 1920 * 1200 * 4;
    float *phase_map_x = new float[phase_buf_size];
    float *phase_map_y = new float[phase_buf_size];

    int brightness_buf_size = 1920 * 1200 * 1;
    unsigned char *brightness = new unsigned char[brightness_buf_size];

    if (repetition_count < 1)
    {
        repetition_count = 1;
    }

    if (repetition_count > 10)
    {
        repetition_count = 10;
    }

    // lc3010.pattern_mode04_repetition(repetition_count);
    camera.capturePhase02Repetition02ToGpu(repetition_count);

    copy_phase_from_cuda_memory(phase_map_x, phase_map_y);
    copy_merge_brightness_from_cuda_memory(brightness);

    LOG(INFO) << "start send depth, buffer_size= " << phase_buf_size;
    ret = send_buffer(client_sock, (const char *)phase_map_x, phase_buf_size);
    LOG(INFO) << "depth ret= " << ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] phase_map_x;
        delete[] phase_map_y;
        delete[] brightness;

        return DF_FAILED;
    }

    LOG(INFO) << "start send depth, buffer_size=" << phase_buf_size;
    ret = send_buffer(client_sock, (const char *)phase_map_y, phase_buf_size);
    LOG(INFO) << "depth ret= " << ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] phase_map_x;
        delete[] phase_map_y;
        delete[] brightness;

        return DF_FAILED;
    }

    LOG(INFO) << "start send brightness, buffer_size= " << brightness_buf_size;
    ret = send_buffer(client_sock, (const char *)brightness, brightness_buf_size);
    LOG(INFO) << "brightness ret= " << ret;

    LOG(INFO) << "Send Frame04";

    float temperature = read_temperature(0);

    LOG(INFO) << "temperature: " << temperature << " deg";

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";

        delete[] phase_map_x;
        delete[] phase_map_y;
        delete[] brightness;

        return DF_FAILED;
    }
    LOG(INFO) << "frame sent!";
    delete[] phase_map_x;
    delete[] phase_map_y;
    delete[] brightness;
    return DF_SUCCESS;
}


int handle_cmd_get_frame_04_repetition_02_parallel(int client_sock)
{
    /**************************************************************************************/

    if(check_token(client_sock) == DF_FAILED)
    {
	return DF_FAILED;
    }
	
    
    int repetition_count = 1;

    int ret = recv_buffer(client_sock, (char*)(&repetition_count), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }
    LOG(INFO)<<"repetition_count: "<<repetition_count<<"\n";
    /***************************************************************************************/


    int depth_buf_size = 1920*1200*4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = 1920*1200*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 

    if(repetition_count< 1)
    {
      repetition_count = 1;
    }
    
    if(repetition_count> 10)
    {
      repetition_count = 10;
    }

    
    camera.setGenerateBrightnessParam(generate_brightness_model,generate_brightness_exposure_time);

    // lc3010.pattern_mode04_repetition(repetition_count); 
    camera.captureFrame04Repetition02ToGpu(repetition_count);
   
  
    // camera.copyBrightness((char*)brightness);
    reconstruct_copy_brightness_from_cuda_memory(brightness); 
    LOG(INFO)<<"copy depth";
    reconstruct_copy_depth_from_cuda_memory((float*)depth_map);
 
    LOG(INFO)<<"Reconstruct Frame04 Finished!";

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(1200, 1920, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(1200, 1920, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral";


    }

    //基于置信图最大轮廓滤波
    if(false)
    {
        int nr = 1200;
        int nc = 1920;
        cv::Mat confidence_mat(nr,nc,CV_32FC1,cv::Scalar(0.));
        reconstruct_copy_confidence_from_cuda_memory((float*)confidence_mat.data);
        cv::Mat confidence_mask;
        findMaskBaseConfidence(confidence_mat,15,confidence_mask);

        for(int r= 0;r< nr;r++)
        {
            uchar* ptr_m = confidence_mask.ptr<uchar>(r);
            for(int c = 0;c< nc;c++)
            {
                if(0 == ptr_m[c])
                {
                    depth_map[r*nc+c] = 0.0;
                }
            }

        }
        
        LOG(INFO) << "filter base confidence";
    }

 

    printf("start send depth, buffer_size=%d\n", depth_buf_size);
    ret = send_buffer(client_sock, (const char *)depth_map, depth_buf_size);
    printf("depth ret=%d\n", ret);

    if (ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        return DF_FAILED;
    }

    printf("start send brightness, buffer_size=%d\n", brightness_buf_size);
    ret = send_buffer(client_sock, (const char *)brightness, brightness_buf_size);
    printf("brightness ret=%d\n", ret);

    LOG(INFO) << "Send Frame04";

    float temperature = read_temperature(0);

    LOG(INFO) << "temperature: " << temperature << " deg";

    if (ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        return DF_FAILED;
    }
    printf("frame sent!\n");
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;
    return DF_SUCCESS;

    

}



int handle_cmd_get_frame_04_repetition_01_parallel(int client_sock)
{
    /**************************************************************************************/

    if(check_token(client_sock) == DF_FAILED)
    {
	return DF_FAILED;
    }
	
    
    int repetition_count = 1;

    int ret = recv_buffer(client_sock, (char*)(&repetition_count), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }
    LOG(INFO)<<"repetition_count: "<<repetition_count<<"\n";
    /***************************************************************************************/


    int depth_buf_size = 1920*1200*4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = 1920*1200*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 

    if(repetition_count< 1)
    {
      repetition_count = 1;
    }
    
    if(repetition_count> 10)
    {
      repetition_count = 10;
    }

    
    camera.setGenerateBrightnessParam(generate_brightness_model,generate_brightness_exposure_time);

    lc3010.pattern_mode04_repetition(repetition_count); 
    camera.captureFrame04RepetitionToGpu(repetition_count);
   
  
    camera.copyBrightness((char*)brightness);
    // reconstruct_copy_brightness_from_cuda_memory(brightness); 
    LOG(INFO)<<"copy depth";
    reconstruct_copy_depth_from_cuda_memory((float*)depth_map);
 
    LOG(INFO)<<"Reconstruct Frame04 Finished!";

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(1200, 1920, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(1200, 1920, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral";


    }

    //基于置信图最大轮廓滤波
    if(false)
    {
        int nr = 1200;
        int nc = 1920;
        cv::Mat confidence_mat(nr,nc,CV_32FC1,cv::Scalar(0.));
        reconstruct_copy_confidence_from_cuda_memory((float*)confidence_mat.data);
        cv::Mat confidence_mask;
        findMaskBaseConfidence(confidence_mat,15,confidence_mask);

        for(int r= 0;r< nr;r++)
        {
            uchar* ptr_m = confidence_mask.ptr<uchar>(r);
            for(int c = 0;c< nc;c++)
            {
                if(0 == ptr_m[c])
                {
                    depth_map[r*nc+c] = 0.0;
                }
            }

        }
        
        LOG(INFO) << "filter base confidence";
    }

 

    printf("start send depth, buffer_size=%d\n", depth_buf_size);
    ret = send_buffer(client_sock, (const char *)depth_map, depth_buf_size);
    printf("depth ret=%d\n", ret);

    if (ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        return DF_FAILED;
    }

    printf("start send brightness, buffer_size=%d\n", brightness_buf_size);
    ret = send_buffer(client_sock, (const char *)brightness, brightness_buf_size);
    printf("brightness ret=%d\n", ret);

    LOG(INFO) << "Send Frame04";

    float temperature = read_temperature(0);

    LOG(INFO) << "temperature: " << temperature << " deg";

    if (ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        return DF_FAILED;
    }
    printf("frame sent!\n");
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;
    return DF_SUCCESS;

    

}



int handle_cmd_get_frame_03_repetition_parallel(int client_sock)
{
    /**************************************************************************************/

    if(check_token(client_sock) == DF_FAILED)
    {
	return DF_FAILED;
    }
	
    
    int repetition_count = 1;

    int ret = recv_buffer(client_sock, (char*)(&repetition_count), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }
    LOG(INFO)<<"repetition_count: "<<repetition_count<<"\n";
    /***************************************************************************************/


    int depth_buf_size = 1920*1200*4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = 1920*1200*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 

    if(repetition_count< 1)
    {
      repetition_count = 1;
    }
    
    if(repetition_count> 10)
    {
      repetition_count = 10;
    }

    lc3010.pattern_mode03_repetition(repetition_count); 
    camera.captureFrame03RepetitionToGpu(repetition_count);
  
    ret= parallel_cuda_copy_result_from_gpu((float*)depth_map,brightness);

    
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

    LOG(INFO)<<"Send Frame03 Repetition";

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


int handle_cmd_get_standard_plane_param_parallel(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    int pointcloud_buf_size = 1920*1200*4*3;
    float* pointcloud_map = new float[pointcloud_buf_size];

    int brightness_buf_size = 1920*1200*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 


    lc3010.pattern_mode03(); 
    camera.captureFrame03ToGpu();
  
    int ret= parallel_cuda_copy_pointcloud_from_gpu((float*)pointcloud_map,brightness);


    int plane_buf_size = 12*4;
    float* plane_param = new float[plane_buf_size];

    memset(plane_param, 0, sizeof(float) * 12);

    float* R = new float[9];
    float* T = new float[3];
    
    memset(R, 0, sizeof(float) * 9);
    memset(T, 0, sizeof(float) * 3);

 

    ConfigureStandardPlane plane_machine;
    plane_machine.setCalibrateParam(param);
    bool found = plane_machine.getStandardPlaneParam(pointcloud_map,brightness,R,T);

    if(found)
    {
        memcpy(plane_param, R, sizeof(float) * 9);
        memcpy(plane_param+9, T, sizeof(float) * 3);
    }



    printf("start send plane param, buffer_size=%d\n", plane_buf_size);
    ret = send_buffer(client_sock, (const char*)plane_param, plane_buf_size);
    printf("depth ret=%d\n", ret);

    if(ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
	// delete [] buffer;
	delete [] pointcloud_map;
	delete [] brightness;
	
	return DF_FAILED;
    }
     
    printf("plane param sent!\n");
    // delete [] buffer;
    delete [] pointcloud_map;
    delete [] brightness;
    return DF_SUCCESS;
     
}



int handle_cmd_get_frame_04_parallel(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }
    int depth_buf_size = 1920 * 1200 * 4;
    int brightness_buf_size = 1920 * 1200 * 1;
    unsigned char *brightness = NULL;
    float *depth_map = NULL;

    LOG(INFO)<<"captureFrame04";
    scan3d_machine_.captureFrame04();
     
    scan3d_machine_.getPtrBrightness(brightness);
    scan3d_machine_.getPtrDepth(depth_map);

    // int depth_buf_size = 1920*1200*4;
    // float* depth_map = new float[depth_buf_size];

    // int brightness_buf_size = 1920*1200*1;
    // unsigned char* brightness = new unsigned char[brightness_buf_size]; 

    // camera.setGenerateBrightnessParam(generate_brightness_model,generate_brightness_exposure_time);

    // lc3010.pattern_mode04(); 
    // camera.captureFrame04ToGpu();
  
    // camera.copyBrightness((char*)brightness);
    // reconstruct_copy_brightness_from_cuda_memory(brightness); 
    LOG(INFO)<<"copy depth";
    // reconstruct_copy_depth_from_cuda_memory((float*)depth_map);
 
    LOG(INFO)<<"Reconstruct Frame04 Finished!";

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(1200, 1920, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(1200, 1920, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral";


    }
  

    printf("start send depth, buffer_size=%d\n", depth_buf_size);
    int ret = send_buffer(client_sock, (const char *)depth_map, depth_buf_size);
    printf("depth ret=%d\n", ret);

    if (ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        return DF_FAILED;
    }

    printf("start send brightness, buffer_size=%d\n", brightness_buf_size);
    ret = send_buffer(client_sock, (const char *)brightness, brightness_buf_size);
    printf("brightness ret=%d\n", ret);

    LOG(INFO) << "Send Frame04";

    float temperature = read_temperature(0);

    LOG(INFO) << "temperature: " << temperature << " deg";

    if (ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        return DF_FAILED;
    }
    printf("frame sent!\n");
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;
    return DF_SUCCESS;
}


int handle_cmd_get_frame_05_parallel(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    int depth_buf_size = 1920 * 1200 * 4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = 1920 * 1200 * 1;
    unsigned char* brightness = new unsigned char[brightness_buf_size];


    lc3010.pattern_mode04();
    camera.captureFrame05ToGpu();

    reconstruct_copy_brightness_from_cuda_memory(brightness);
    reconstruct_copy_depth_from_cuda_memory((float*)depth_map);

    LOG(INFO) << "Reconstruct Frame05 Finished!";


    // cv::Mat pointcloud_map(1200,1920,CV_32FC3,cv::Scalar(-2));
    // reconstruct_copy_pointcloud_from_cuda_memory((float*)pointcloud_map.data);  
    // cv::Mat depth_mat(1200,1920,CV_32FC1,depth_map);

    // cv::imwrite("./depth_map.tiff",depth_mat);
    // cv::imwrite("./pointcloud_map.tiff",pointcloud_map);

    printf("start send depth, buffer_size=%d\n", depth_buf_size);
    int ret = send_buffer(client_sock, (const char*)depth_map, depth_buf_size);
    printf("depth ret=%d\n", ret);

    if (ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        return DF_FAILED;
    }

    printf("start send brightness, buffer_size=%d\n", brightness_buf_size);
    ret = send_buffer(client_sock, (const char*)brightness, brightness_buf_size);
    printf("brightness ret=%d\n", ret);

    LOG(INFO) << "Send Frame05";

    float temperature = read_temperature(0);

    LOG(INFO) << "temperature: " << temperature << " deg";

    if (ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        return DF_FAILED;
    }
    printf("frame sent!\n");
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;
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

    
    LOG(INFO) << "Reconstruct Frame03 Finished!";   
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

    if(!ifile.is_open())
    {
        return DF_FAILED;
    }

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
 
 
    //set external param
    
    std::memcpy(system_config_settings_machine_.Instance().config_param_.standard_plane_external_param , rect_config_param.standard_plane_external_param,sizeof(rect_config_param.standard_plane_external_param));

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

/**********************************************************************************************************************/
//设置基准平面外参
int handle_cmd_set_param_standard_param_external(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	  
    float plane_param[12]; 

    int ret = recv_buffer(client_sock, (char*)(plane_param), sizeof(float)*12);
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }


	memcpy(system_config_settings_machine_.Instance().config_param_.standard_plane_external_param, plane_param, sizeof(float)*12);
 
 
    return DF_SUCCESS;
 
}


//获取基准平面外参
int handle_cmd_get_param_standard_param_external(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
   
	
    int ret = send_buffer(client_sock, (char*)(system_config_settings_machine_.Instance().config_param_.standard_plane_external_param), sizeof(float)*12);
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	return DF_FAILED;
    }
    return DF_SUCCESS;
 
       
}

//获取相机增益参数
int handle_cmd_get_param_camera_gain(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    } 
	
    int ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().config_param_.camera_gain), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
 
    return DF_SUCCESS;
  
}

//获取相机曝光参数
int handle_cmd_get_param_camera_exposure(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    } 
	
    int ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().config_param_.camera_exposure_time), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
 
    return DF_SUCCESS;
  
}

//设置补偿参数
int handle_cmd_set_param_offset(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	  

    float offset = 0;

    int ret = recv_buffer(client_sock, (char*)(&offset), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }
 

    if(offset>= 0)
    {    
        camera.setOffsetParam(offset);
        LOG(INFO)<<"Set Offset: "<<offset<<"\n";
    }
    else
    {
        LOG(INFO)<<"Set Camera Exposure Time Error!"<<"\n";
    }
 
  
    return DF_SUCCESS;
}


//获取补偿参数
int handle_cmd_get_param_offset(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	
    float offset = 0;
 
    camera.getOffsetParam(offset);

    int ret = send_buffer(client_sock, (char*)(&offset), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    } 
  
    return DF_SUCCESS;
}


//设置相机增益参数
int handle_cmd_set_param_camera_gain(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	  

    float gain = 0;

    int ret = recv_buffer(client_sock, (char*)(&gain), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }

    if(gain< 0)
    {
        gain = 0;
    }
    else if(gain > 10)
    {
        gain = 10;
    }

 
    system_config_settings_machine_.Instance().config_param_.camera_gain = gain;

    if (camera.setScanGain(system_config_settings_machine_.Instance().config_param_.camera_gain))
    {
     LOG(INFO) << "Set Camera Gain: " << gain << "\n";  
    }

 
  
    return DF_SUCCESS;
}

//设置相机曝光参数
int handle_cmd_set_param_camera_exposure(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	  

    float exposure = 0;

    int ret = recv_buffer(client_sock, (char*)(&exposure), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }

    if(exposure> max_camera_exposure_)
    {
        exposure = max_camera_exposure_;
        LOG(INFO) << "Set Camera Exposure Time Error!"
                  << "\n";
    }
    else if (exposure < min_camera_exposure_)
    {
        exposure = min_camera_exposure_;
        LOG(INFO) << "Set Camera Exposure Time Error!"
                  << "\n";
    }

    // if(exposure>= 6000 && exposure<= 60000)
    // {
    system_config_settings_machine_.Instance().config_param_.camera_exposure_time = exposure;

    LOG(INFO) << "Set Camera Exposure Time: " << exposure << "\n";

    if (camera.setScanExposure(exposure))
    {
        lc3010.set_camera_exposure(exposure);
    }

    // }
    // else
    // {
    //     LOG(INFO)<<"Set Camera Exposure Time Error!"<<"\n";
    // }
 
  
    return DF_SUCCESS;
}

//获取生成亮度参数
int handle_cmd_get_param_generate_brightness(int client_sock)
{
   if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
   
	
    int ret = send_buffer(client_sock, (char*)(&generate_brightness_model), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }

    ret = send_buffer(client_sock, (char*)(&generate_brightness_exposure_time), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }

    return DF_SUCCESS;
}

//设置生成亮度参数
int handle_cmd_set_param_generate_brightness(int client_sock)
{
 if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	  
    int flag = 1 ; 

    int ret = recv_buffer(client_sock, (char*)(&flag), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }

    float exposure = 0;

    ret = recv_buffer(client_sock, (char*)(&exposure), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }

    generate_brightness_model = flag;
    generate_brightness_exposure_time = exposure;


    LOG(INFO)<<"generate_brightness_model: "<<generate_brightness_model<<"\n";
    LOG(INFO)<<"generate_brightness_exposure_time: "<<generate_brightness_exposure_time<<"\n";

    
    camera.setGenerateBrightnessParam(generate_brightness_model,generate_brightness_exposure_time);

  
    return DF_SUCCESS;
}


//设置双边滤波参数
int handle_cmd_set_param_bilateral_filter(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }


    int param[2];

    int ret = recv_buffer(client_sock, (char*)(&param[0]), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }
 

    ret = recv_buffer(client_sock, (char*)(&param[1]), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }

    system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter = param[0];


    if (1 == param[0])
    {   
        if (3 == param[1] || 5 == param[1] || 7 == param[1] || 9 == param[1] || 11 == param[1])
        { 
        system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d = param[1];
        }
    }

    
    LOG(INFO)<<"Use Bilateral Filter: "<<system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter;
    LOG(INFO)<<"Bilateral Filter param: "<<system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d;
         

    return DF_SUCCESS;
}

//获取双边滤波参数
int handle_cmd_get_param_bilateral_filter(int client_sock)
{
   if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
     
    int ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter), sizeof(int) );
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
 
    ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
 

    return DF_SUCCESS;
}

//设置混合多曝光参数
int handle_cmd_set_param_mixed_hdr(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	  
    int param[13]; 

    int ret = recv_buffer(client_sock, (char*)(&param), sizeof(int)*13);
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }

    int num = param[0];  
      //set led current

    for (int i = 0; i < num; i++)
    {
        int exposure = param[1 + i];

        if (exposure > max_camera_exposure_)
        {
            exposure = max_camera_exposure_;
        }
        else if (exposure < min_camera_exposure_)
        {
            exposure = min_camera_exposure_;
        }
        param[1 + i] = exposure;
    }

        if(0< num && num<= 6)
        {
 
            system_config_settings_machine_.Instance().firwmare_param_.mixed_exposure_num = num;
            memcpy(system_config_settings_machine_.Instance().firwmare_param_.mixed_exposure_param_list, param+1, sizeof(int) * 6);
            memcpy(system_config_settings_machine_.Instance().firwmare_param_.mixed_led_param_list, param+7, sizeof(int) * 6);
            system_config_settings_machine_.Instance().firwmare_param_.hdr_model = 2;
            return DF_SUCCESS;
        }
  
        return DF_FAILED;
}

//获取相机版本参数
int handle_cmd_get_param_camera_version(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    int version = 0;

    lc3010.read_dmd_device_id(version); 

    int ret = send_buffer(client_sock, (char *)(&version), sizeof(int) * 1);
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }

    LOG(INFO)<<"camera version: "<<version << "\n";

    return DF_SUCCESS;

}


//设置置信度参数
int handle_cmd_set_param_confidence(int client_sock)
{

 if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	  

    float val = 0; 
    int ret = recv_buffer(client_sock, (char*)(&val), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }
    LOG(INFO) << "Set Confidence: "<<val;
    system_config_settings_machine_.Instance().firwmare_param_.confidence = val;
    cuda_set_config(system_config_settings_machine_);

    return DF_SUCCESS;
}

//获取置信度参数
int handle_cmd_get_param_confidence(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }
 
    float confidence = system_config_settings_machine_.Instance().firwmare_param_.confidence;
 

    int ret = send_buffer(client_sock, (char *)(&confidence), sizeof(float) * 1);
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }
    return DF_SUCCESS;
}

//获取混合多曝光参数
int handle_cmd_get_param_mixed_hdr(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    int param[13];
    param[0] = system_config_settings_machine_.Instance().firwmare_param_.mixed_exposure_num;

    memcpy(param + 1, system_config_settings_machine_.Instance().firwmare_param_.mixed_exposure_param_list, sizeof(int) * 6);
    memcpy(param + 7, system_config_settings_machine_.Instance().firwmare_param_.mixed_led_param_list, sizeof(int) * 6);

    int ret = send_buffer(client_sock, (char *)(&param), sizeof(int) * 13);
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }
    return DF_SUCCESS;
}

//设置多曝光参数
int handle_cmd_set_param_hdr(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	  
    int param[7]; 

    int ret = recv_buffer(client_sock, (char*)(&param), sizeof(int)*7);
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }

    int num = param[0];  
      //set led current
    
        if(0< num && num<= 6)
        {
 
            system_config_settings_machine_.Instance().config_param_.exposure_num = num;
            memcpy(system_config_settings_machine_.Instance().config_param_.exposure_param, param+1, sizeof(int) * 6);
            system_config_settings_machine_.Instance().firwmare_param_.hdr_model = 1;
            return DF_SUCCESS;
        }
  
        return DF_FAILED; 
}

//获取多曝光参数
int handle_cmd_get_param_hdr(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
  
  		int param[7];
		param[0] = system_config_settings_machine_.Instance().config_param_.exposure_num;

		memcpy(param+1, system_config_settings_machine_.Instance().config_param_.exposure_param, sizeof(int)*6);
	
    int ret = send_buffer(client_sock, (char*)(&param), sizeof(int) * 7);
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	return DF_FAILED;
    }
    return DF_SUCCESS;
 
       
}

//设置光机投影亮度
int handle_cmd_set_param_led_current(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	  

    int led= -1;

    int ret = recv_buffer(client_sock, (char*)(&led), sizeof(led));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }


      //set led current
    
        if(0<= led && led< 1024)
        {
            brightness_current = led;
            lc3010.SetLedCurrent(brightness_current,brightness_current,brightness_current); 
            system_config_settings_machine_.Instance().config_param_.led_current = brightness_current;
             return DF_SUCCESS;
        }
 
     
 
        return DF_FAILED; 
}

//获取光机投影亮度
int handle_cmd_get_param_led_current(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
  
	
    int ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().config_param_.led_current), sizeof(system_config_settings_machine_.Instance().config_param_.led_current));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	return DF_FAILED;
    }
    return DF_SUCCESS;
 
       
}
/*************************************************************************************************************************/

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
    
    
int handle_set_camera_looktable(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	return DF_FAILED;
    }
	
    int ret = -1;

    ret = recv_buffer(client_sock, (char*)(&param), sizeof(param));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }

     LOG(INFO)<<"recv param\n";
    /**************************************************************************************/
    cv::Mat xL_rotate_x(1200,1920,CV_32FC1,cv::Scalar(-2));
    cv::Mat xL_rotate_y(1200,1920,CV_32FC1,cv::Scalar(-2));
    cv::Mat rectify_R1(3,3,CV_32FC1,cv::Scalar(-2));
    cv::Mat pattern_mapping(4000,2000,CV_32FC1,cv::Scalar(-2));

     ret = recv_buffer(client_sock, (char*)(xL_rotate_x.data), 1200*1920 *sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    LOG(INFO)<<"recv xL_rotate_x\n";

     ret = recv_buffer(client_sock, (char*)(xL_rotate_y.data), 1200*1920 *sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    LOG(INFO)<<"recv xL_rotate_y\n";

     ret = recv_buffer(client_sock, (char*)(rectify_R1.data), 3*3 *sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    LOG(INFO)<<"recv rectify_R1\n";

     ret = recv_buffer(client_sock, (char*)(pattern_mapping.data), 4000*2000 *sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    LOG(INFO)<<"recv pattern_mapping\n";

  
	    cv::Mat R1_t = rectify_R1.t(); 

        LOG(INFO)<<"start copy table:";
        reconstruct_copy_talbe_to_cuda_memory((float*)pattern_mapping.data,(float*)xL_rotate_x.data,(float*)xL_rotate_y.data,(float*)R1_t.data);
        LOG(INFO)<<"copy finished!";

        float b = sqrt(pow(param.translation_matrix[0], 2) + pow(param.translation_matrix[1], 2) + pow(param.translation_matrix[2], 2));
        reconstruct_set_baseline(b);
 
    LOG(INFO)<<"copy looktable\n"; 
    
    write_calib_param();
 
	LookupTableFunction lookup_table_machine_; 
    lookup_table_machine_.saveBinMappingFloat("./combine_xL_rotate_x_cam1_iter.bin",xL_rotate_x);
    lookup_table_machine_.saveBinMappingFloat("./combine_xL_rotate_y_cam1_iter.bin",xL_rotate_y);
    lookup_table_machine_.saveBinMappingFloat("./R1.bin",rectify_R1);
    lookup_table_machine_.saveBinMappingFloat("./single_pattern_mapping.bin",pattern_mapping);

    LOG(INFO)<<"save looktable\n";

    /***************************************************************************************/

    return DF_SUCCESS;

}


int handle_set_camera_minilooktable(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	return DF_FAILED;
    }
	
    int ret = -1;

    ret = recv_buffer(client_sock, (char*)(&param), sizeof(param));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }

     LOG(INFO)<<"recv param\n";
    /**************************************************************************************/
    cv::Mat xL_rotate_x(1200,1920,CV_32FC1,cv::Scalar(-2));
    cv::Mat xL_rotate_y(1200,1920,CV_32FC1,cv::Scalar(-2));
    cv::Mat rectify_R1(3,3,CV_32FC1,cv::Scalar(-2));
    cv::Mat pattern_minimapping(128,128,CV_32FC1,cv::Scalar(-2));

    ret = recv_buffer(client_sock, (char*)(xL_rotate_x.data), 1200*1920 *sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    LOG(INFO)<<"recv xL_rotate_x\n";

     ret = recv_buffer(client_sock, (char*)(xL_rotate_y.data), 1200*1920 *sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    LOG(INFO)<<"recv xL_rotate_y\n";

     ret = recv_buffer(client_sock, (char*)(rectify_R1.data), 3*3 *sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    LOG(INFO)<<"recv rectify_R1\n";

     ret = recv_buffer(client_sock, (char*)(pattern_minimapping.data), 128*128 *sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send minimap error, close this connection!\n";
	    return DF_FAILED;
    }
    LOG(INFO)<<"recv pattern_mini_mapping\n";

  
	    cv::Mat R1_t = rectify_R1.t(); 

        LOG(INFO)<<"start copy table:";
        reconstruct_copy_minitalbe_to_cuda_memory((float*)pattern_minimapping.data,(float*)xL_rotate_x.data,(float*)xL_rotate_y.data,(float*)R1_t.data);
        LOG(INFO)<<"copy finished!";

        float b = sqrt(pow(param.translation_matrix[0], 2) + pow(param.translation_matrix[1], 2) + pow(param.translation_matrix[2], 2));
        reconstruct_set_baseline(b);
 
    LOG(INFO)<<"copy looktable\n"; 
    
    write_calib_param();
 
	LookupTableFunction lookup_table_machine_; 
    lookup_table_machine_.saveBinMappingFloat("./combine_xL_rotate_x_cam1_iter.bin",xL_rotate_x);
    lookup_table_machine_.saveBinMappingFloat("./combine_xL_rotate_y_cam1_iter.bin",xL_rotate_y);
    lookup_table_machine_.saveBinMappingFloat("./R1.bin",rectify_R1);
    lookup_table_machine_.saveBinMappingFloat("./single_pattern_minimapping.bin",pattern_minimapping);

    LOG(INFO)<<"save minilooktable\n";

    /***************************************************************************************/

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
int handle_get_firmware_version(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }

    LOG(INFO)<<"get firmware version!";

    char version[_VERSION_LENGTH_] = _VERSION_;
    int ret = send_buffer(client_sock, version, _VERSION_LENGTH_);
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    
    return DF_SUCCESS;
}

bool check_trigger_line()
{
    bool ret = false;
    char* buffer = new char[1920*1200];

    lc3010.pattern_mode_brightness();
    ret = camera.CaptureSelfTest();

    delete [] buffer;

    return ret;
}

void self_test(char *test_out)
{
    // check the network
    if( read_bandwidth() < 1000) {
        sprintf(test_out, "The network failure -- bandwidth less than 1000Mb.");
        return;
    }

    // check usb camera
    if (GXInitLib() != GX_STATUS_SUCCESS)
    {
        sprintf(test_out, "The camera failure -- driver installed not porperly.");
        return;
    }

    uint32_t nDeviceNum = 0;
    GX_STATUS status = GXUpdateDeviceList(&nDeviceNum, 1000);
    if ((status != GX_STATUS_SUCCESS) || (nDeviceNum <= 0))
    {
        sprintf(test_out, "The camera failure -- device not connected.");
        return;
    }

    // check projector i2c
    int version= 0;
    lc3010.read_dmd_device_id(version);
    if ((version != 800) && (version != 1800))
    {
        sprintf(test_out, "The projector failure -- communication error.");
        return;
    }

    // check trigger-line
    if (check_trigger_line() == false) {
        sprintf(test_out, "The camera failure -- trigger-line not connected.");
        return;
    }

    sprintf(test_out, "Self-test OK.");
}

int handle_cmd_self_test(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }

    LOG(INFO)<<"self test!";

    char test[500] = {'\0'};
    self_test(test);
    int ret = send_buffer(client_sock, test, sizeof(test));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    
    return DF_SUCCESS;
}

int handle_get_projector_temperature(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }

    LOG(INFO)<<"get projector temperature!";

    float temperature = lc3010.get_projector_temperature();

    int ret = send_buffer(client_sock, (char*)(&temperature), sizeof(temperature));
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

    // set led indicator
	GPIO::output(OUTPUT_PIN, GPIO::HIGH); 

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
	    handle_cmd_get_raw(client_sock);
	    break;
	case DF_CMD_GET_RAW_TEST:
	    LOG(INFO)<<"DF_CMD_GET_RAW_TEST"; 
	    handle_cmd_get_raw_02(client_sock);
	    break;
	case DF_CMD_GET_RAW_03:
	    LOG(INFO)<<"DF_CMD_GET_RAW_03"; 
	    handle_cmd_get_raw_03(client_sock);
	    break;
    case DF_CMD_GET_RAW_04:
	    LOG(INFO)<<"DF_CMD_GET_RAW_04"; 
	    handle_cmd_get_raw_04(client_sock);
	    break;
    case DF_CMD_GET_RAW_04_REPETITION:
	    LOG(INFO)<<"DF_CMD_GET_RAW_04_REPETITION"; 
	    handle_cmd_get_raw_04_repetition(client_sock);
	    break;
        
	case DF_CMD_GET_FRAME_01:
	    LOG(INFO)<<"DF_CMD_GET_FRAME_01"; 
	    handle_cmd_get_frame_01(client_sock); 
	    break;
    case DF_CMD_GET_FRAME_HDR:
	    LOG(INFO)<<"DF_CMD_GET_FRAME_HDR";  
        if(1 == system_config_settings_machine_.Instance().firwmare_param_.hdr_model)
        {
            handle_cmd_get_frame_04_hdr_parallel(client_sock);
        }
        else if (2 == system_config_settings_machine_.Instance().firwmare_param_.hdr_model)
        {
            handle_cmd_get_frame_04_hdr_parallel_mixed_led_and_exposure(client_sock);
        } 
        break;
 
	case DF_CMD_GET_FRAME_03:
	    LOG(INFO)<<"DF_CMD_GET_FRAME_03";   
    	handle_cmd_get_frame_03_parallel(client_sock); 
	    break;
	case DF_CMD_GET_REPETITION_FRAME_03:
	    LOG(INFO)<<"DF_CMD_GET_REPETITION_FRAME_03";   
        handle_cmd_get_frame_03_repetition_parallel(client_sock);
	    break; 
    case DF_CMD_GET_REPETITION_FRAME_04:
	    LOG(INFO)<<"DF_CMD_GET_REPETITION_FRAME_04";   
        handle_cmd_get_frame_04_repetition_02_parallel(client_sock);
	    break; 
	case DF_CMD_GET_FRAME_04:
	    LOG(INFO)<<"DF_CMD_GET_FRAME_04";   
    	handle_cmd_get_frame_04_parallel(client_sock); 
	    break;
    case DF_CMD_GET_FRAME_05:
        LOG(INFO) << "DF_CMD_GET_FRAME_05";
        handle_cmd_get_frame_05_parallel(client_sock);
        break;
	case DF_CMD_GET_POINTCLOUD:
	    LOG(INFO)<<"DF_CMD_GET_POINTCLOUD"; 
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
	case DF_CMD_SET_CAMERA_LOOKTABLE:
	    LOG(INFO)<<"DF_CMD_SET_CAMERA_LOOKTABLE";
	    handle_set_camera_looktable(client_sock);
        read_calib_param();
        cuda_copy_calib_data(param.camera_intrinsic, 
		         param.projector_intrinsic, 
			 param.camera_distortion,
	                 param.projector_distortion, 
			 param.rotation_matrix, 
			 param.translation_matrix);
	    break;
	case DF_CMD_SET_CAMERA_MINILOOKTABLE:
	    LOG(INFO)<<"DF_CMD_SET_CAMERA_MINILOOKTABLE";
	    handle_set_camera_minilooktable(client_sock);
        read_calib_param();
        cuda_copy_calib_data(param.camera_intrinsic, 
		         param.projector_intrinsic, 
			 param.camera_distortion,
	                 param.projector_distortion, 
			 param.rotation_matrix, 
			 param.translation_matrix);
	    break;
	case DF_CMD_ENABLE_CHECKER_BOARD:
	    LOG(INFO)<<"DF_CMD_ENABLE_CHECKER_BOARD";
	    handle_enable_checkerboard(client_sock);
	    break;

	case DF_CMD_DISABLE_CHECKER_BOARD:
	    LOG(INFO)<<"DF_CMD_DISABLE_CHECKER_BOARD";
	    handle_disable_checkerboard(client_sock);
	    break;

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

	case DF_CMD_GET_FIRMWARE_VERSION:
	    LOG(INFO)<<"DF_CMD_GET_FIRMWARE_VERSION";
	    handle_get_firmware_version(client_sock);
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
	case DF_CMD_GET_STANDARD_PLANE_PARAM:
	    LOG(INFO)<<"DF_CMD_GET_STANDARD_PLANE_PARAM";   
    	handle_cmd_get_standard_plane_param_parallel(client_sock); 
	    break;
	case DF_CMD_GET_PARAM_LED_CURRENT:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_LED_CURRENT";   
    	handle_cmd_get_param_led_current(client_sock);  
	    break;
	case DF_CMD_SET_PARAM_LED_CURRENT:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_LED_CURRENT";   
    	handle_cmd_set_param_led_current(client_sock);  
	    break;
    case DF_CMD_GET_PARAM_HDR:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_HDR";   
    	handle_cmd_get_param_hdr(client_sock);  
	    break;
	case DF_CMD_SET_PARAM_HDR:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_HDR";   
    	handle_cmd_set_param_hdr(client_sock);  
	    break;
    case DF_CMD_GET_PARAM_STANDARD_PLANE_EXTERNAL_PARAM:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_STANDARD_PLANE_EXTERNAL_PARAM";   
    	handle_cmd_get_param_standard_param_external(client_sock);  
	    break;
	case DF_CMD_SET_PARAM_STANDARD_PLANE_EXTERNAL_PARAM:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_STANDARD_PLANE_EXTERNAL_PARAM";   
    	handle_cmd_set_param_standard_param_external(client_sock);  
	    break;
	case DF_CMD_SET_PARAM_GENERATE_BRIGHTNESS:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_GENERATE_BRIGHTNESS";   
    	handle_cmd_set_param_generate_brightness(client_sock);  
	    break;
    case DF_CMD_GET_PARAM_GENERATE_BRIGHTNESS:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_GENERATE_BRIGHTNESS";   
    	handle_cmd_get_param_generate_brightness(client_sock);  
	    break;
	case DF_CMD_SET_PARAM_CAMERA_EXPOSURE_TIME:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_CAMERA_EXPOSURE_TIME";   
    	handle_cmd_set_param_camera_exposure(client_sock);
	    break;
	case DF_CMD_GET_PARAM_CAMERA_EXPOSURE_TIME:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_CAMERA_EXPOSURE_TIME";   
    	handle_cmd_get_param_camera_exposure(client_sock);
	    break;
    case DF_CMD_SET_PARAM_CAMERA_GAIN:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_CAMERA_GAIN";   
    	handle_cmd_set_param_camera_gain(client_sock);
	    break;
	case DF_CMD_GET_PARAM_CAMERA_GAIN:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_CAMERA_GAIN";   
    	handle_cmd_get_param_camera_gain(client_sock);
	    break;
	case DF_CMD_SET_PARAM_OFFSET:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_OFFSET";   
    	handle_cmd_set_param_offset(client_sock);
	    break;
	case DF_CMD_GET_PARAM_OFFSET:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_OFFSET";   
    	handle_cmd_get_param_offset(client_sock);
	    break;
	case DF_CMD_SET_PARAM_MIXED_HDR:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_MIXED_HDR";   
    	handle_cmd_set_param_mixed_hdr(client_sock);
	    break;
	case DF_CMD_GET_PARAM_MIXED_HDR:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_MIXED_HDR";   
    	handle_cmd_get_param_mixed_hdr(client_sock);
	    break;
    case DF_CMD_GET_PARAM_CAMERA_CONFIDENCE:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_CAMERA_CONFIDENCE";   
    	handle_cmd_get_param_confidence(client_sock);
	    break;
    case DF_CMD_SET_PARAM_CAMERA_CONFIDENCE:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_CAMERA_CONFIDENCE";   
    	handle_cmd_set_param_confidence(client_sock);
	    break;
	case DF_CMD_GET_PARAM_CAMERA_VERSION:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_CAMERA_VERSION";   
    	handle_cmd_get_param_camera_version(client_sock);
	    break;
	case DF_CMD_SET_PARAM_BILATERAL_FILTER:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_BILATERAL_FILTER";   
    	handle_cmd_set_param_bilateral_filter(client_sock);
	    break;
	case DF_CMD_GET_PARAM_BILATERAL_FILTER:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_BILATERAL_FILTER";   
    	handle_cmd_get_param_bilateral_filter(client_sock);
	    break;
	case DF_CMD_SET_AUTO_EXPOSURE_BASE_ROI:
	    LOG(INFO)<<"DF_CMD_SET_AUTO_EXPOSURE_BASE_ROI";   
    	handle_cmd_set_auto_exposure_base_roi_half(client_sock);
	    break;
	case DF_CMD_SET_AUTO_EXPOSURE_BASE_BOARD:
	    LOG(INFO)<<"DF_CMD_SET_AUTO_EXPOSURE_BASE_BOARD";   
    	handle_cmd_set_auto_exposure_base_board(client_sock);
	case DF_CMD_SELF_TEST:
	    LOG(INFO)<<"DF_CMD_SELF_TEST";   
    	handle_cmd_self_test(client_sock);
	    break;
	case DF_CMD_GET_PROJECTOR_TEMPERATURE:
	    LOG(INFO)<<"DF_CMD_GET_PROJECTOR_TEMPERATURE";
	    handle_get_projector_temperature(client_sock);
	    break;
    case DF_CMD_GET_PHASE_02_REPETITION:
    	LOG(INFO)<<"DF_CMD_GET_PHASE_02_REPETITION";
	    handle_cmd_get_phase_02_repetition_02_parallel(client_sock);
	    break;
	default:
	    LOG(INFO)<<"DF_CMD_UNKNOWN";
        handle_cmd_unknown(client_sock);
	    break;
    }

    // close led indicator
	GPIO::output(OUTPUT_PIN, GPIO::LOW); 

    close(client_sock);
    return DF_SUCCESS;
}

int init()
{
    // readSystemConfig();

    // init led indicator
	GPIO::setmode(GPIO::BCM);                       // BCM mode
	GPIO::setup(OUTPUT_PIN, GPIO::OUT, GPIO::LOW); // output pin, set to HIGH level

    brightness_current = system_config_settings_machine_.Instance().config_param_.led_current;

    // if(!camera.openCamera())
    // { 
    //     LOG(INFO)<<"Open Camera Error!";
    // }
    

    // camera.switchToScanMode();
    // lc3010.SetLedCurrent(brightness_current,brightness_current,brightness_current);
    // cuda_malloc_memory();
    // int ret = read_calib_param();

    // if(DF_FAILED == ret)
    // { 
    //     LOG(INFO)<<"Read Calib Param Error!";
    // }

    // cuda_copy_calib_data(param.camera_intrinsic, 
	// 	         param.projector_intrinsic, 
	// 		 param.camera_distortion,
	//                  param.projector_distortion, 
	// 		 param.rotation_matrix, 
	// 		 param.translation_matrix); 

	// LookupTableFunction lookup_table_machine_; 
    // MiniLookupTableFunction minilookup_table_machine_;

	// lookup_table_machine_.setCalibData(param);
    // minilookup_table_machine_.setCalibData(param);

    // LOG(INFO)<<"start read table:";
    
    // cv::Mat xL_rotate_x;
    // cv::Mat xL_rotate_y;
    // cv::Mat rectify_R1;
    // cv::Mat pattern_mapping;
    // cv::Mat pattern_minimapping;

    // bool read_map_ok = lookup_table_machine_.readTableFloat("./", xL_rotate_x, xL_rotate_y, rectify_R1, pattern_mapping);
    // bool read_minimap_ok = minilookup_table_machine_.readTableFloat("./", xL_rotate_x, xL_rotate_y, rectify_R1, pattern_minimapping);
  
    // if(read_map_ok)
    // {  
    //     LOG(INFO)<<"read table finished!";
	//     cv::Mat R1_t = rectify_R1.t();
    //     xL_rotate_x.convertTo(xL_rotate_x, CV_32F);
    //     xL_rotate_y.convertTo(xL_rotate_y, CV_32F);
    //     R1_t.convertTo(R1_t, CV_32F);
    //     pattern_mapping.convertTo(pattern_mapping, CV_32F);

    //     LOG(INFO)<<"start copy table:";
    //     reconstruct_copy_talbe_to_cuda_memory((float*)pattern_mapping.data,(float*)xL_rotate_x.data,(float*)xL_rotate_y.data,(float*)R1_t.data);
    //     LOG(INFO)<<"copy finished!";

    //     float b = sqrt(pow(param.translation_matrix[0], 2) + pow(param.translation_matrix[1], 2) + pow(param.translation_matrix[2], 2));
    //     reconstruct_set_baseline(b);
    // }
    // if (read_minimap_ok)
    // {
    //     cv::Mat R1_t = rectify_R1.t();
    //     xL_rotate_x.convertTo(xL_rotate_x, CV_32F);
    //     xL_rotate_y.convertTo(xL_rotate_y, CV_32F);
    //     R1_t.convertTo(R1_t, CV_32F);
    //     pattern_minimapping.convertTo(pattern_minimapping, CV_32F);

    //     LOG(INFO) << "start copy minitable:";
    //     reconstruct_copy_minitalbe_to_cuda_memory((float*)pattern_minimapping.data, (float*)xL_rotate_x.data, (float*)xL_rotate_y.data, (float*)R1_t.data);
    //     LOG(INFO) << "copy minitable finished!";

    //     float b = sqrt(pow(param.translation_matrix[0], 2) + pow(param.translation_matrix[1], 2) + pow(param.translation_matrix[2], 2));
    //     reconstruct_set_baseline(b);
    // }

    // float temperature_val = read_temperature(0); 
    // LOG(INFO)<<"temperature: "<<temperature_val<<" deg";

    scan3d_machine_.init();

    int version= 0;
    lc3010.read_dmd_device_id(version);
    LOG(INFO)<<"read camera version: "<<version;

    cuda_set_config(system_config_settings_machine_);

    set_camera_version(version);
    // LOG(INFO)<<"camera version: "<<DFX_800;

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
	GPIO::cleanup();

    return 0;
}
