#include "stdafx.h"
#include "Pointcloud_processing.h"
#include "PCSegment.h"

Pointcloud_processing::Pointcloud_processing()
{
}


Pointcloud_processing::~Pointcloud_processing()
{
	SafeRelease(pColorFrameSource);
	SafeRelease(pDepthFrameSource);
	SafeRelease(pColorReader);
	SafeRelease(pDepthReader);
	SafeRelease(pColorDescription);
	SafeRelease(pDepthDescription);
	SafeRelease(pCoordinateMapper);
	if (pSensor) {
		pSensor->Close();
	}
	SafeRelease(pSensor);
}

HRESULT Pointcloud_processing::init_kinect(){
	// 查找当前默认Kinect
	HRESULT hr = ::GetDefaultKinectSensor(&pSensor);
	// 绅士地打开Kinect
	if (SUCCEEDED(hr)){
		hr = pSensor->Open();
	}
	// Retrieved Coordinate Mapper
	if (SUCCEEDED(hr)){
		hr = pSensor->get_CoordinateMapper(&pCoordinateMapper);
	}
	// 获取深度帧源(DepthFrameSource)
	if (SUCCEEDED(hr)){
		hr = pSensor->get_DepthFrameSource(&pDepthFrameSource);
	}
	// 获取颜色帧源(ColorFrameSource)
	if (SUCCEEDED(hr)){
		hr = pSensor->get_ColorFrameSource(&pColorFrameSource);
	}
	// 再获取深度帧读取器
	if (SUCCEEDED(hr)){
		hr = pDepthFrameSource->OpenReader(&pDepthReader);
	}
	// 再获取颜色帧读取器
	if (SUCCEEDED(hr)){
		hr = pColorFrameSource->OpenReader(&pColorReader);
	}
	// Retrieved Color Frame Size
	if (SUCCEEDED(hr)){
		hr = pColorFrameSource->get_FrameDescription(&pColorDescription);
	}
	//pColorDescription->get_Width(&ColorWidth); // 1920
	//pColorDescription->get_Height(&ColorHeight); // 1080
	// Retrieved Depth Frame Size
	if (SUCCEEDED(hr)){
		hr = pDepthFrameSource->get_FrameDescription(&pDepthDescription);
	}
	//pDepthDescription->get_Width(&DepthWidth); //512
	//pDepthDescription->get_Height(&DepthHeight); //424

	camera_ins = getCameIns(pCoordinateMapper);
	return hr;
}

bool Pointcloud_processing::show_pointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &m_pointcloud, std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> &vec){
	std::vector<RGBQUAD> colorBuffer(ColorWidth * ColorHeight);
	std::vector<UINT16> depthBuffer(DepthWidth * DepthHeight);

	// Create Cloud Viewer
	pcl::visualization::CloudViewer viewer("Point Cloud Viewer");
	//存储颜色图
	RGBQUAD *sav_color = new RGBQUAD[DepthWidth * DepthHeight];

	UINT16 *bufdepth = NULL;
	UINT nBufferSize_depth = 0;
	cv::Mat mat_depth;

	HRESULT hResult = S_OK;
	int t1 = 0, t2 = 0;
	char key = 0;
	bool loop = true;
	while (!viewer.wasStopped() && loop)
	{
		if (!pColorReader)
		{
			break;
		}

		//ColorFrame
		IColorFrame* pColorFrame = nullptr;
		//DepthFrame
		IDepthFrame* pDepthFrame = nullptr;
		
		hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
		if (SUCCEEDED(hResult)) {
			// Retrieved Color Data
			hResult = pColorFrame->CopyConvertedFrameDataToArray(colorBuffer.size() * sizeof(RGBQUAD), reinterpret_cast<BYTE*>(&colorBuffer[0]), ColorImageFormat::ColorImageFormat_Bgra);
			if (FAILED(hResult)) {
				std::cerr << "Error : IColorFrame::CopyConvertedFrameDataToArray()" << std::endl;
			}
		}
		SafeRelease(pColorFrame);

		hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame);
		if (SUCCEEDED(hResult)) {
			// Retrieved Depth Data
			hResult = pDepthFrame->CopyFrameDataToArray(depthBuffer.size(), &depthBuffer[0]);
			if (FAILED(hResult)) {
				std::cerr << "Error : IDepthFrame::CopyFrameDataToArray()" << std::endl;
			}
		}

		if (SUCCEEDED(hResult))
		{
			hResult = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize_depth, &bufdepth);
			//save depth image to vector
			mat_depth = ConvertMat16x(bufdepth, DepthWidth, DepthHeight);
		}
		SafeRelease(pDepthFrame);

		// Create Point Cloud
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>());

		pointcloud->width = static_cast<uint32_t>(DepthWidth);
		pointcloud->height = static_cast<uint32_t>(DepthHeight);
		pointcloud->is_dense = false;

		for (int y = 0; y < DepthHeight; y++) {
			for (int x = 0; x < DepthWidth; x++) {
				pcl::PointXYZRGB point;

				DepthSpacePoint depthSpacePoint = { static_cast<float>(x), static_cast<float>(y) };
				UINT16 depth = depthBuffer[y * DepthWidth + x];

				// Coordinate Mapping Depth to Color Space, and Setting PointCloud RGB
				ColorSpacePoint colorSpacePoint = { 0.0f, 0.0f };
				pCoordinateMapper->MapDepthPointToColorSpace(depthSpacePoint, depth, &colorSpacePoint);
				int colorX = static_cast<int>(std::floor(colorSpacePoint.X + 0.5f));
				int colorY = static_cast<int>(std::floor(colorSpacePoint.Y + 0.5f));
				if ((0 <= colorX) && (colorX < ColorWidth) && (0 <= colorY) && (colorY < ColorHeight)) {
					RGBQUAD color = colorBuffer[colorY * ColorWidth + colorX];
					point.b = color.rgbBlue;
					point.g = color.rgbGreen;
					point.r = color.rgbRed;

					//save color of one point
					sav_color[y * DepthWidth + x] = color;
				}

				// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
				CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
				pCoordinateMapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);
				if ((0 <= colorX) && (colorX < ColorWidth) && (0 <= colorY) && (colorY < ColorHeight)) {
					point.x = cameraSpacePoint.X;
					point.y = cameraSpacePoint.Y;
					point.z = cameraSpacePoint.Z;
				}

				pointcloud->push_back(point);

			}
		}

		// show img & pcd
		if (SUCCEEDED(hResult))
		{
			cv::Mat eximg1 = ConvertMatx(sav_color, DepthWidth, DepthHeight);
			medianBlur(mat_depth, mat_depth, 5);

			//cv::imwrite("../data/color2/img_color_" + std::to_string(t2) + ".png", eximg);
			//t2++;
			cv::Mat eximg, mat_depth1;
			flip(eximg1, eximg, 1);
			flip(mat_depth, mat_depth1, 1);
			cv::imshow("depth", mat_depth1);
			cv::imshow("color", eximg);
			//show points cloud
			viewer.showCloud(pointcloud);

			key = cv::waitKey(1);
			if (key == '1')
			{
				PointCloud_Synthesis(eximg, mat_depth1, m_pointcloud);
				PointCloud_Process(m_pointcloud, vec);

				cv::destroyAllWindows();
				loop = false;
				return true;
			}
		}
	}
	return false;
}

// convert color image into cv::Mat
cv::Mat Pointcloud_processing::ConvertMatx(const RGBQUAD* pBuffer, int nWidth, int nHeight)
{
	cv::Mat img(nHeight, nWidth, CV_8UC3);
	uchar* p_mat = img.data;

	const RGBQUAD* pBufferEnd = pBuffer + (nWidth * nHeight);

	while (pBuffer < pBufferEnd)
	{
		*p_mat = pBuffer->rgbBlue;
		p_mat++;
		*p_mat = pBuffer->rgbGreen;
		p_mat++;
		*p_mat = pBuffer->rgbRed;
		p_mat++;

		++pBuffer;
	}
	return img;
}

//  convert depth image into cv::Mat 16bit
cv::Mat Pointcloud_processing::ConvertMat16x(const UINT16* pBuffer, int nWidth, int nHeight)
{
	cv::Mat img(nHeight, nWidth, CV_16UC1);
	UINT16* p_mat = (UINT16*)img.data;

	const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

	while (pBuffer < pBufferEnd)
	{
		/*
		之所以需要“*5”，是为了更好的存储，保证数据的有效性，记得在读取时除掉
		*/
		*p_mat = (*pBuffer) * 5;
		p_mat++;
		++pBuffer;
	}
	return img;
}

bool Pointcloud_processing::PointCloud_Synthesis(cv::Mat color_mat, cv::Mat depth_mat, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
	double u0 = 262.151;
	double v0 = 205.671;
	double fx = 364.99;
	double fy = 364.99;

	int rowNumber = color_mat.rows;
	int colNumber = color_mat.cols;

	cloud->height = 1;
	cloud->width = depth_mat.cols * depth_mat.rows;
	cloud->points.resize(depth_mat.rows * depth_mat.cols);

	for (unsigned int v = 0; v < depth_mat.rows; ++v)
	{
		for (unsigned int u = 0; u < depth_mat.cols; ++u)
		{
			unsigned int num = v*colNumber + u;
			double Xw = 0, Yw = 0, Zw = 0;
			if (depth_mat.at<ushort>(v, u)>0)
			{
				Zw = depth_mat.at<ushort>(v, u) / 5000.0;
				Xw = ((double)u - u0) * Zw / fx;
				Yw = ((double)v - v0) * Zw / fy;
			}

			//cloud->points[num].b = color_mat.at<cv::Vec3b>(v, u)[0];
			//cloud->points[num].g = color_mat.at<cv::Vec3b>(v, u)[1];
			//cloud->points[num].r = color_mat.at<cv::Vec3b>(v, u)[2];

			cloud->points[num].x = Xw;
			cloud->points[num].y = Yw;
			cloud->points[num].z = Zw;
		}
	}
	return true;
}

bool Pointcloud_processing::PointCloud_Process(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> &vec){
	PCSegment seg;

	pcl::PointCloud<pcl::PointXYZ>::Ptr second_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr third_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr fouth_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> vec;
	if (cloud->points.size() > 100)
	{
		if (seg.PassThrough_filter(cloud, second_cloud) &&
			seg.PlaneModel_segmentation(second_cloud, 0.005, third_cloud) &&
			seg.EuclideanCluster(third_cloud, vec, fouth_cloud)){
			cout << "SUCCESS!" << endl;
		}
	}

	return true;
}

Camera_Intrinsics Pointcloud_processing::getCameIns(ICoordinateMapper* pCoordinateMapper){
	CameraIntrinsics *cameraIntrinsics = new CameraIntrinsics;
	pCoordinateMapper->GetDepthCameraIntrinsics(cameraIntrinsics);
	Camera_Intrinsics camera_ins;

	camera_ins.FLX = cameraIntrinsics->FocalLengthX;
	camera_ins.FLY = cameraIntrinsics->FocalLengthY;
	camera_ins.PPX = cameraIntrinsics->PrincipalPointX;
	camera_ins.PPY = cameraIntrinsics->PrincipalPointY;
	return camera_ins;
}