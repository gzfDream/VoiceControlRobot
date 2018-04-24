// kinect_DataCollection.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "Pointcloud_processing.h"
#include "Voice_process.h"
#include "Segment_string.h"
#include "RSocket.h"
int _tmain(int argc, _TCHAR* argv[])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
	Pointcloud_processing pp;
	HRESULT hr = pp.init_kinect();
	Voice_process voice;
	Segment_string strSge;
	
	const char* SendBuffer;// [MAX_PATH];
	RSocket Rsocket;
	SOCKET sclient;
	sclient = Rsocket.InitSocket(5000);

	if (SUCCEEDED(hr))
	{
		if (voice.init())
		{
			while (true)
			{
				//Eigen::Vector4f a1(1,1,1,1), a2(2,2,2,1), a3(3,3,3,1), a4(4,4,4,1);
				std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> vec;
				//vec.push_back(a1);
				//vec.push_back(a2);
				//vec.push_back(a3);
				//vec.push_back(a4);
				Eigen::Vector4f result_seg;
				bool m_stop = pp.show_pointCloud(m_pointcloud, vec);
				if (m_stop)
				{
					voice.run();

					vector<string> vec_string;
					strSge.Segment(vec_string);
					result_seg = strSge.SemanticsMap(vec_string, vec);

					std::cout << result_seg[0] << " " << result_seg[1] << " " << result_seg[2] << endl;

					ostringstream fbuffer;
					string str_buf;
					fbuffer << 'p' << " ";
					for (int xn = 0; xn < 3; xn++)
					{
						fbuffer << result_seg[xn];
						fbuffer << " ";
					}
					str_buf = fbuffer.str();
					SendBuffer = str_buf.c_str();
					cout << SendBuffer << endl;
					/*Êý¾Ý´«Êä*/
					Rsocket.Rsend(sclient, SendBuffer);
				}
				else
				{
					SendBuffer = "Q";
					Rsocket.Rsend(sclient, SendBuffer);
					break;
				}
			}
		}
	}
	closesocket(sclient);
	return 0;
}

