#pragma once

#include "../cmd/getopt.h"

enum opt_set
{
	IP,
	PATH,
	HELP,
	GET_FRAME_01
};

static struct option long_options[] =
{
	{"ip", required_argument, NULL, IP},
	{"path", required_argument, NULL, PATH},
	{"help", no_argument,NULL, HELP},
	{"get-frame-01", no_argument, NULL, GET_FRAME_01},
};

void Help();

