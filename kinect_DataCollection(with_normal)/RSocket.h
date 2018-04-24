#pragma once

#include <stdio.h>
#include <Windows.h>

#pragma comment(lib,"ws2_32.lib")
#define  PORT 5000
#define  IP_ADDRESS "192.168.143.101"

class RSocket
{
public:
	RSocket();
	~RSocket();

private:
	
public:
	SOCKET InitSocket(int Sport);
	void Rsend(SOCKET s, const char *SendBuffer);
	void close(SOCKET ClientSocket);
};

