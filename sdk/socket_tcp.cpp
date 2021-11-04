#ifdef _MSC_VER
#include "socket_tcp.h"
#include <winsock2.h>
#include <iostream>
#include <assert.h>
#pragma comment(lib, "ws2_32.lib")
#include "../Firmware/easylogging++.h"

SOCKET g_sock = INVALID_SOCKET;
SOCKET g_sock_heartbeat = INVALID_SOCKET;

int setup_socket(const char* camera_ip, int port, SOCKET& sock)
{
	if (sock == INVALID_SOCKET)
	{
		WORD sockVersion = MAKEWORD(2, 2);
		WSADATA data;
		if (WSAStartup(sockVersion, &data) != 0)
		{
			LOG(ERROR) << "WSAStartup error!";
			return DF_FAILED;
		}
		sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (sock == INVALID_SOCKET)
		{
			LOG(ERROR) << "Invalid socket!";
			return DF_FAILED;
		}

		struct sockaddr_in host_addr;
		host_addr.sin_family = AF_INET;
		host_addr.sin_port = htons(port);
		host_addr.sin_addr.S_un.S_addr = inet_addr(camera_ip);

		if (connect(sock, (struct sockaddr*)&host_addr, sizeof(host_addr)) == SOCKET_ERROR)
		{ 
			LOG(ERROR) << "Connect error !";
			closesocket(sock);
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
		closesocket(sock);
		sock = INVALID_SOCKET;
		WSACleanup();
	}
	return DF_SUCCESS;
}
#else

#endif

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

	int null_flag = 0;

	while (ret != -1)
	{
		ret = recv(sock, buffer, buffer_size, 0); 
		//LOG(INFO) << "recv£º " << "ret=" << ret << std::endl;
		if (ret > 0)
		{
			buffer_size -= ret;
			n_recv += ret;
			buffer += ret;
		}
		else if (0 == ret)
		{
			null_flag++;
		}

		if (null_flag > 10)
		{ 
			return DF_FAILED;
		}

		if (buffer_size == 0)
		{
			assert(n_recv == size);
			return DF_SUCCESS;
		}
	}
	return DF_FAILED;
}