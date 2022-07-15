#include "help.h"
#include "case.h"
#include "status.h"

int NetDropInfo(void* param)
{
	std::cout << "Network dropped!" << std::endl;
	return 0;
}

bool NetLink(const char* ip)
{
	DfRegisterOnDropped(NetDropInfo);

	if (DfConnectNet(ip) == DF_SUCCESS) {
		return true;
	} else {
		return false;
	}
}

bool NetDisLink()
{
	if (DfDisconnectNet() == DF_SUCCESS) {
		return true;
	} else {
		return false;
	}
}

void Case(int cmd, const char *camera)
{
//	if (NetLink(camera) == false) { 
//		return; 
//	}

	switch (cmd)
	{
	case GET_POINT_CLOUD:
		get_point_cloud(camera);
		break;
	default:
		break;
	}

//	if (NetDisLink() == false) { 
//		printf("Network DisLink Error."); 
//	}
}