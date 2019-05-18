#pragma once
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>
#include "Utils.h"
using namespace std;


//这个类实现一些asc文件转pcl点云数据的操作.
class  FileOperaterClass
{

public:

	void WriteStructAsc(string &souce, AscDataD3Struct &vc, int id, int num);

	void SplitString(const string& s, AscDataD3Struct& v, int id, const string& c);

	void SplitString(const string& s, vector<string>& v, const string& c);

	//int ReadAscFile(const string ascfile, pcl::PointCloud<pcl::PointXYZ>& P3Vectors);

	int ReadAscFile(const string ascfile, vector<AscDataD3Struct>& P3Vectors);

	FileOperaterClass();

	~FileOperaterClass();
};

