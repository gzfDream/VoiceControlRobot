#pragma once
#include "NLPIR.h"
#pragma comment(lib,"libs/NLPIR.lib")
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/StdVector>
using namespace std;
class Segment_string
{
public:
	Segment_string();
	~Segment_string();

public:
	bool Segment(std::vector<std::string> &vec_string);
	Eigen::Vector4f SemanticsMap(vector<string> vec_str,
		std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> vec);

private:
	int PositionNumber(string str);
	int ChineseNoMapping(string str);
};

