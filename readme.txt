                    ---EyeInHand（机器人手眼标定）
VoiceControlRobot---
                    ---kinect_DataCollection（语音控制机械臂）

    
	1. EyeInHand（机器人手眼标定）（棋盘格：10x7,30mm）
		Eye_To_Hand  手在眼外标定
		EyeInHand_AsusCamera	眼在手上，使用体感摄像头
		EyeInHand_kinect2.0		眼在手上，使用Kinect2.0
	2. kinect_DataCollection（语音控制机械臂）			
		①语音：讯飞离线命令识别系统（免费试用35天，之后需要下载新的重新替换文件夹msc中的文件）
		②语句分词：nlpir，授权过期，重新授权https://github.com/NLPIR-team/NLPIR
		③./data/cal.txt  桌面上棋盘格与相机的标定矩阵，将相机坐标系转到棋盘格
		
					




