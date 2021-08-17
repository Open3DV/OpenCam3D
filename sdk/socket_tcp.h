#pragma once
#include "../Firmware/protocol.h"



#ifdef _MSC_VER

#include <winsock2.h>
int setup_socket(const char* camera_ip, int port, SOCKET& sock);
int close_socket(SOCKET& sock);
int send_command(int command, SOCKET& sock);
int recv_command(int* command, SOCKET& sock);

int send_buffer(const char* buffer, int buffer_size, SOCKET& sock);
int recv_buffer(char* buffer, int buffer_size, SOCKET& sock);

#endif

