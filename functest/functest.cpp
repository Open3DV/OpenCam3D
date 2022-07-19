#pragma once
#include "help.h"
#include "status.h"
#include "case.h"

extern int optopt;
extern char* optarg;

const char* camera_id;
const char* path;
int command = HELP;

int main(int argc, char* argv[])
{
	int c = 0;

	while (EOF != (c = getopt_long(argc, argv, "i:h", long_options, NULL)))
	{
		switch (c)
		{
		case IP:
			camera_id = optarg;
			break;
		case PATH:
			path = optarg;
			break;
		case HELP:
			Help();
			break;
		case '?':
			printf("unknow option:%c\n", optopt);
			break;
		default:
			command = c;
			break;
		}
	}

	Case(command, camera_id);

	return 0;
}

