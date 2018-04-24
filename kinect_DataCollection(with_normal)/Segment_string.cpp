#include "stdafx.h"
#include "Segment_string.h"


Segment_string::Segment_string()
{
}


Segment_string::~Segment_string()
{
}

static bool myCompareL(const Eigen::Vector4f& a1, const Eigen::Vector4f& a2)
{
	cout << a1[0] << ", " << a2[0] << endl;
	return a1[0] <= a2[0];
}

static bool myCompareR(const Eigen::Vector4f& a1, const Eigen::Vector4f& a2)
{
	cout << a1[0] << ", " << a2[0] << endl;
	return a1[0] >= a2[0];
}

static bool myCompareF(const Eigen::Vector4f& a1, const Eigen::Vector4f& a2)
{
	cout << a1[1] << ", " << a2[1] << endl;
	return a1[1] <= a2[1];
}

static bool myCompareB(const Eigen::Vector4f& a1, const Eigen::Vector4f& a2)
{
	cout << a1[1] << ", " << a2[1] << endl;
	return a1[1] >= a2[1];
}

bool Segment_string::Segment(std::vector<std::string> &vec_string){
	std::ifstream fin("result.txt", ios::in);
	if (!fin){ cout << "result.txt file read error!" << endl; return -1; }
	string temptext, tempstr, text;
	while (!fin.eof())
	{
		getline(fin, tempstr);
		temptext += tempstr + "\r\n";
		cout << temptext << endl;
	}
	int index = 0;
	index = temptext.find("input=");
	text = temptext.substr(index + 6);
	if (!NLPIR_Init())
	{
		cout << "NLPIR_Init() error!" << endl;
		return -1;
	}
	string resulttext = NLPIR_ParagraphProcess(text.c_str(), 0);
	NLPIR_Exit();
	fin.close();

	int iBegin = 0;
	string::size_type iLatter = 0;
	string::size_type iFormer = string::npos;
	//vector<string> vec_string;
	while (1)
	{
		iLatter = resulttext.find_first_not_of(' ', iLatter);
		if (string::npos == iLatter)
		{
			break;
		}

		iFormer = resulttext.find_first_of(' ', iLatter + 1);
		if (string::npos == iFormer)
		{
			iFormer = resulttext.length();
		}

		string strNew(resulttext, iLatter, iFormer - iLatter);
		if (strNew != "\r" && strNew != "\n")
		{
			vec_string.push_back(strNew);
		}

		iLatter = iFormer + 1;
	}

	return true;

}

int Segment_string::PositionNumber(string str){
	string word(str, 0, 2);
	if (word == "��")
		return 0;
	else if (word == "��")
		return 1;
	else if (word == "ǰ")
		return 2;
	else if (word == "��")
		return 3;
	else
		return -1;
}

int Segment_string::ChineseNoMapping(string str){
	if (str == "һ")
		return 1;
	if (str == "��")
		return 2;
	if (str == "��")
		return 3;
	if (str == "��")
		return 4;
	if (str == "��")
		return 5;
	if (str == "��")
		return 6;
	if (str == "��")
		return 7;
	if (str == "��")
		return 8;
	if (str == "��")
		return 9;

}

Eigen::Vector4f Segment_string::SemanticsMap(vector<string> vec_str, std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> vec){
	Eigen::Vector4f res(0., 0., 0., 0.);
	for (int i = 0; i < vec_str.size(); i++)
	{
		switch (PositionNumber(vec_str[i]))
		{
		case 0:
			sort(vec.begin(), vec.end(), myCompareL);
			cout << "sort:" << endl;
			for (int xx = 0; xx < vec.size(); xx++)
			{
				cout << vec[xx] << endl;
			}
			break;
		case 1:
			sort(vec.begin(), vec.end(), myCompareR);
			cout << "sort:" << endl;
			for (int xx = 0; xx < vec.size(); xx++)
			{
				cout << vec[xx] << endl;
			}
			break;
		case 2:
			sort(vec.begin(), vec.end(), myCompareF);
			break;
		case 3:
			sort(vec.begin(), vec.end(), myCompareB);
			break;
		default:
			break;
		}

		string word(vec_str[i], 0, 2);
		if (word == "��")
		{
			string num(vec_str[i], 2, 4);
			int x = ChineseNoMapping(num) - 1;
			if (vec.size() < x)
			{
				cout << "vec size error" << endl;
				return res;
				//exit(1);
			}
			else
			{
				cout << vec[x] << endl;
				cout << "NO." << x << endl;
				return vec[x];
			}
		}
	}
	return res;
}
