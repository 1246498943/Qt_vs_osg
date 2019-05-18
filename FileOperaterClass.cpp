#include "FileOperaterClass.h"

#pragma region 文件操作的一些类.


FileOperaterClass::FileOperaterClass()
{

}

FileOperaterClass::~FileOperaterClass()
{

}

void FileOperaterClass::WriteStructAsc(string &souce, AscDataD3Struct &vc, int id, int num)
{
	switch (num)
	{
	case 0:
	{
		vc.x = atof(souce.c_str());
		break;
	}
	case 1:
	{
		vc.y = atof(souce.c_str());
		break;
	}
	case 2:
	{
		vc.z = atof(souce.c_str());
		break;
	}
	case 3:
	{
		vc.normalX = atof(souce.c_str());
		break;
	}
	case 4:
	{
		vc.normalY = atof(souce.c_str());
		break;
	}
	case 5:
	{
		vc.normalZ = atof(souce.c_str());
		break;
	}
	case 6:
	{
		vc.R = atoi(souce.c_str());
		break;
	}
	case 7:
	{
		vc.G = atoi(souce.c_str());
		break;
	}
	case 8:
	{
		vc.B = atoi(souce.c_str());
		break;
	}
	case 9:
	{
		vc.ID = id;
		break;
	}
	default:
		break;
	}
}

void FileOperaterClass::SplitString(const string& s, AscDataD3Struct& v, int id, const string& c)
{
	string::size_type pos1, pos2;
	pos2 = s.find(c);
	pos1 = 0;
	string temp;
	int EnumNum = 0;
	vector<string> vc;
	while (string::npos != pos2)
	{
		vc.push_back(s.substr(pos1, pos2 - pos1));

		pos1 = pos2 + c.size();
		pos2 = s.find(c, pos1);

	}
	if (pos1 != s.length())
	{
		vc.push_back(s.substr(pos1));

	}

	for (int i = 0; i < vc.size(); i++)
	{

		WriteStructAsc(vc[i], v, id, i);
	}
}


void FileOperaterClass::SplitString(const string& s, vector<string>& v, const string& c)
{
	string::size_type pos1, pos2;
	pos2 = s.find(c);
	pos1 = 0;

	while (string::npos != pos2)
	{
		v.push_back(s.substr(pos1, pos2 - pos1));
		cout << s.substr(pos1, pos2 - pos1) << "---" << endl;
		pos1 = pos2 + c.size();
		pos2 = s.find(c, pos1);
	}
	if (pos1 != s.length())
	{
		v.push_back(s.substr(pos1));
		cout << s.substr(pos1) << "+++" << endl;
	}
}


int FileOperaterClass::ReadAscFile(const string ascfile, vector<AscDataD3Struct>& P3Vectors)
{
	std::ifstream fs(ascfile);
	std::string line, res;
	int longLines = 0;
	while (!fs)
	{
		fs.close();
		fs.open(ascfile, std::ios::in);
		_sleep(100);

	}

	long double ControlX = 0, ContorlY = 0, ControlZ = 0;
	double MidX = 0, MidY = 0, MidZ = 0;
	vector<AscDataD3Struct> vecT;
	while (getline(fs, line))
	{

		AscDataD3Struct temp;
		SplitString(line, temp, longLines, " ");
		vecT.push_back(temp);
		ControlX += temp.x;
		ContorlY += temp.y;
		ControlZ += temp.z;

		longLines++;

	}
	fs.close();



	P3Vectors.resize(vecT.size() + 1);
	if (longLines == 0)
	{
		longLines = 1;
	}

	MidX = ControlX / longLines;
	MidY = ContorlY / longLines;
	MidZ = ControlZ / longLines;
	for (int i = 0; i < vecT.size(); i++)
	{
		P3Vectors[i].x = (-vecT[i].x + MidX) / 2;
		P3Vectors[i].y = (-vecT[i].y + MidY) / 2;
		P3Vectors[i].z = (-vecT[i].z + MidZ) / 2;

		P3Vectors[i].normalX = -vecT[i].normalX;
		P3Vectors[i].normalY = -vecT[i].normalY;
		P3Vectors[i].normalZ = -vecT[i].normalZ;

		P3Vectors[i].R = vecT[i].R;
		P3Vectors[i].G = vecT[i].G;
		P3Vectors[i].B = vecT[i].B;
	}

	cout << "读取完毕" << endl;
	return 0;
}


//int FileOperaterClass::ReadAscFile(const string ascfile, pcl::PointCloud<pcl::PointXYZ>& P3Vectors)
//{
//	std::ifstream fs(ascfile);
//	std::string line, res;
//	int longLines = 0;
//	while (!fs)
//	{
//		fs.close();
//		fs.open(ascfile, std::ios::in);
//		Sleep(100);
//	}
//
//	long double ControlX = 0, ContorlY = 0, ControlZ = 0;
//	double MidX = 0, MidY = 0, MidZ = 0;
//	vector<AscDataD3Struct> vecT;
//	while (getline(fs, line))
//	{
//		//std::cout << line << endl;
//
//		AscDataD3Struct temp;
//		SplitString(line, temp, longLines, " ");
//		vecT.push_back(temp);
//		ControlX += temp.x;
//		ContorlY += temp.y;
//		ControlZ += temp.z;
//
//		longLines++;
//		/*if (longLines == 85000)
//		{
//			break;
//		}*/
//	}
//	fs.close();
//
//
//
//	P3Vectors.resize(vecT.size() + 1);
//	if (longLines == 0)
//	{
//		longLines = 1;
//	}
//
//	MidX = ControlX / longLines;
//	MidY = ContorlY / longLines;
//	MidZ = ControlZ / longLines;
//	for (int i = 0; i < vecT.size(); i++)
//	{
//		P3Vectors.points[i].x = (-vecT[i].x + MidX) / 2;
//		P3Vectors.points[i].y = (-vecT[i].y + MidY) / 2;
//		P3Vectors.points[i].z = (-vecT[i].z + MidZ) / 2;
//	}
//
//
//
//	cout << "读取完毕" << endl;
//	return 0;
//}

#pragma endregion
