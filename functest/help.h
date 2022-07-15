#pragma once

#include "../cmd/getopt.h"

enum opt_set
{
	IP,
	PATH,
	HELP,
	GET_POINT_CLOUD
};

static struct option long_options[] =
{
	{"ip", required_argument, NULL, IP},
	{"path", required_argument, NULL, PATH},
	{"help", no_argument,NULL, HELP},
	{"get-point-cloud", no_argument, NULL, GET_POINT_CLOUD},
};

void Help();

