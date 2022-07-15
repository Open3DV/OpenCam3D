#include <iostream>
#include <fstream>
#include "case.h"

using namespace std;

void get_point_cloud(const char* ip)
{
	printf("---- get point cloud ---- \n");

	for (int i = 0; i < 24; i++) 
	{
		char string[50] = { '\0' };
		sprintf(string, ".\\capture\\01\\phase%02d.bmp", i);
		cout << string << endl;

		// read bmp image
		ifstream is(string, ifstream::in | ios::binary);

		// get image size
		is.seekg(0, is.end);
		int length = is.tellg();
		is.seekg(0, is.beg);

		char* buffer = new char[length];

		// read image
		is.read(buffer, length);
/*
		std::string strFile;
		for (size_t i = 0; i < length; i++) {
			strFile += buffer[i];
		}

		sprintf(string, "%02d_temp.bmp", i);
		cout << string << endl;
		ofstream fout(string, ios::binary);
		if (!fout) {
			cout << "File can not open" << endl;
		}
		else {
			fout.write(strFile.c_str(), strFile.size());
			fout.close();
		}
*/
		delete[] buffer;
		buffer = NULL;
	}
}
