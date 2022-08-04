#include "../firmware/version.h"
#include "case.h"
#include "help.h"

const char* help_info =
"--Help:\n\
\n\
1.Get Frame 01 Data:\n\
functest.exe --get-frame-01 --ip 192.168.x.x\n\
\n\
";

void Help()
{
	char info[100 * 1024] = { '\0' };
	char version[] = _VERSION_;
	char enter[] = "\n";

	strcpy_s(info, sizeof(enter), enter);
	strcat_s(info, sizeof(info), version);
	strcat_s(info, sizeof(info), enter);
	strcat_s(info, sizeof(info), enter);
	strcat_s(info, sizeof(info), help_info);

	printf(info);
}

