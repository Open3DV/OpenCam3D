#pragma once

#define DF_SUCCESS 0
#define DF_FAILED 1
#define DF_UNKNOWN 2

bool NetLink(const char* ip);
bool NetDisLink();

void Case(int cmd, const char* camera);