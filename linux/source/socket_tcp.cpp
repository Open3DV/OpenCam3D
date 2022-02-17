#define _CRT_SECURE_NO_WARNINGS

#ifdef _WIN32 
	#include <winsock2.h>
	#pragma comment(lib, "ws2_32.lib")
#elif __linux


	#include <sys/socket.h>
	#include <arpa/inet.h>
	#include <netinet/in.h>
	#include <netinet/tcp.h>

	#include <unistd.h>

	//#define SOCKET int 
	#define INVALID_SOCKET (~0)
	#define SOCKET_ERROR -1
	//#define AF_INET 2
	//#define SOCK_STREAM 1
	
	//#define WORD unsigned short 
	//#define MAKEWORD(a, b)      ((WORD)(((BYTE)(((DWORD_PTR)(a)) & 0xff)) | ((WORD)((BYTE)(((DWORD_PTR)(b)) & 0xff))) << 8))
#endif


#include "socket_tcp.h"
#include <iostream>
#include <assert.h>
#include "easylogging++.h"

SOCKET g_sock = INVALID_SOCKET;
SOCKET g_sock_heartbeat = INVALID_SOCKET;

int setup_socket(const char* camera_ip, int port, SOCKET& sock)
{
	if (sock == INVALID_SOCKET)
	{
		/*WORD sockVersion = MAKEWORD(2, 2);
		WSADATA data;
		if (WSAStartup(sockVersion, &data) != 0)
		{
			LOG(ERROR) << "WSAStartup error!";
			return DF_FAILED;
		}
		*/
		sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (sock == INVALID_SOCKET)
		{
			LOG(ERROR) << "Invalid socket!";
			return DF_FAILED;
		}

		struct sockaddr_in host_addr;
		host_addr.sin_family = AF_INET;
		host_addr.sin_port = htons(port);
		host_addr.sin_addr.s_addr = inet_addr(camera_ip);

		if (connect(sock, (struct sockaddr*)&host_addr, sizeof(host_addr)) == SOCKET_ERROR)
		{ 
			LOG(ERROR) << "Connect error !";
			close(sock);
			sock = INVALID_SOCKET;
			return DF_FAILED;
		}
		LOG(TRACE) << "socket setup ok";
		return DF_SUCCESS;
	}
	else
	{
		LOG(WARNING) << "Socket alrealy setup!";
		return DF_FAILED;
	}
}

int close_socket(SOCKET& sock)
{
	if (sock != INVALID_SOCKET)
	{
		close(sock);
		sock = INVALID_SOCKET;
		//WSACleanup();
	}
	return DF_SUCCESS;
}


int send_command(int command, SOCKET& sock)
{
	return send_buffer((const char*)&command, sizeof(int), sock);
}

int recv_command(int* command, SOCKET& sock)
{
	return recv_buffer((char*)command, sizeof(int),sock);
}

int send_buffer(const char* buffer, int buffer_size, SOCKET& sock)
{
	int size = 0;
	int ret = send(sock, (char*)&buffer_size, sizeof(buffer_size), 0);
	if (ret == -1)
	{
		return DF_FAILED;
	}

	ret = send(sock, buffer, buffer_size, 0);
	if (ret == -1)
	{
		return DF_FAILED;
	}

	return DF_SUCCESS;
}

int recv_buffer(char* buffer, int buffer_size, SOCKET& sock)
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
