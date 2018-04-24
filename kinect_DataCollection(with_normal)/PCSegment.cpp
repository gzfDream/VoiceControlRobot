#include "stdafx.h"
#include "PCSegment.h"
#define MAX 10
PCSegment::PCSegment()
{
	color_segment = new double[33]{
		127.0, 76.0, 51.0,   // color - brown
		241.0, 94.0, 13.0,   // color - orange
		0.0, 167.0, 30.0,      // color green
		51.0, 51.0, 229.0,  // color - blue
		229.0, 229.0, 127.0,  // color - yellow
		178.0, 51.0, 229.0,  // color - purple
		51.0, 229.0, 229.0,  // color - light blue
		155.0, 162.0, 237.0, // color - purple
		229.0, 51.0, 153.0,  // color - pink
		153.0, 153.0, 178.0,  // color - gray
		229.0, 153.0, 127.0   // color - light pink
	};
}


PCSegment::~PCSegment()
{
}

double PCSegment::Cofactor(double* matrix[], int jie, int row, int column)

{

	double result;

	int i, j;

	double*smallmatr[MAX - 1];

	for (i = 0; i < jie - 1; i++)

		smallmatr[i] = new double[jie - 1];

	for (i = 0; i < row; i++)

		for (j = 0; j < column; j++)

			*(smallmatr[i] + j) = *(matrix[i] + j);

	for (i = row; i < jie - 1; i++)

		for (j = 0; j < column; j++)

			*(smallmatr[i] + j) = *(matrix[i + 1] + j);

	for (i = 0; i < row; i++)

		for (j = column; j < jie - 1; j++)

			*(smallmatr[i] + j) = *(matrix[i] + j + 1);

	for (i = row; i < jie - 1; i++)

		for (j = column; j < jie - 1; j++)

			*(smallmatr[i] + j) = *(matrix[i + 1] + j + 1);

	result = Determinant(smallmatr, jie - 1);

	for (i = 0; i < jie - 1; i++)

		delete[] smallmatr[i];

	return result;

}

double PCSegment::Alco(double* matrix[], int jie, int row, int column){
	double result;

	if ((row + column) % 2 == 0)

		result = Cofactor(matrix, jie, row, column);

	else result = (-1)*Cofactor(matrix, jie, row, column);

	return result;

}

void PCSegment::Inverse(double *matrix1[], double*matrix2[], int n, double d)

{

	int i, j;

	for (i = 0; i < n; i++)

		matrix2[i] = (double *)malloc(n*sizeof(double));

	for (i = 0; i < n; i++)

		for (j = 0; j < n; j++)

			*(matrix2[j] + i) = (Alco(matrix1, n, i, j) / d);

}

double PCSegment::Determinant(double* matrix[], int n)

{

	double result = 0, temp;

	int i;

	if (n == 1)

		result = (*matrix[0]);

	else

	{

		for (i = 0; i < n; i++)

		{

			temp = Alco(matrix, n, n - 1, i);

			result += (*(matrix[n - 1] + i))*temp;

		}

	}

	return result;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr PCSegment::Distance(double A, double B, double C, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::PointXYZ> point;
	double t = 0.0, j = 0.0;
	for (int i = 0; i < cloud->size(); i++)
	{
		//t = fabs(A*cloud->points[i].x + B*cloud->points[i].y + C*cloud->points[i].z + 1) / sqrt(pow(A, 2) + pow(B, 2) + pow(C, 2));
		t = (A*cloud->points[i].x + B*cloud->points[i].y + C*cloud->points[i].z + 1) / sqrt(pow(A, 2) + pow(B, 2) + pow(C, 2));
		if (t > 0.01){
			point.push_back(cloud->points[i]);
			j++;
		}
	}
	std::cout << point.size() << std::endl;
	cloud_f->width = point.size(); cloud_f->height = 1;
	cloud_f->resize(cloud_f->width*cloud_f->height);
	for (int i = 0; i < point.size(); i++)
	{
		cloud_f->points[i].x = point[i].x;
		cloud_f->points[i].y = point[i].y;
		cloud_f->points[i].z = point[i].z;
		//cloud_f->points[i].normal_x = point[i].normal_x;
		//cloud_f->points[i].normal_y = point[i].normal_y;
		//cloud_f->points[i].normal_z = point[i].normal_z;
	}

	/*pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
	int M = cloud_f->points.size();
	m_cloud->width = 1;
	m_cloud->height = M;
	m_cloud->resize(M*1);
	for (int i = 0; i < M; i++)
	{
	pcl::PointXYZ p;
	p.x = cloud_f->points[i].x;
	p.y = cloud_f->points[i].y;
	p.z = cloud_f->points[i].z;
	m_cloud->points.push_back(p);
	}*/

	//pcl::copyPointCloud(cloud_f, m_cloud);	
	//pcl::PCDWriter writer;
	//writer.write<pcl::PointXYZ>("../PointCloud/clip.pcd", *cloud_f, false);
	//std::cout << j << std::endl;
	return cloud_f;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PCSegment::process(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud){
	double array[30000][3], Y[3];

	double A, B, C;

	A = B = C = 0.0;

	ZeroMemory(array, sizeof(array));

	ZeroMemory(Y, sizeof(Y));

	for (int i = 0; i < plane_cloud->size(); i++)
	{
		array[i][0] = plane_cloud->points[i].x;
		array[i][1] = plane_cloud->points[i].y;
		array[i][2] = plane_cloud->points[i].z;
	}

	double *Matrix[3], *IMatrix[3];

	for (int i = 0; i < 3; i++)
	{
		Matrix[i] = new double[3];
		IMatrix[i] = new double[3];
	}

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			*(Matrix[i] + j) = 0.0;
		}
	}

	for (int j = 0; j < 3; j++)
	{
		for (int i = 0; i < plane_cloud->size(); i++)
		{
			*(Matrix[0] + j) += array[i][0] * array[i][j];
			*(Matrix[1] + j) += array[i][1] * array[i][j];
			*(Matrix[2] + j) += array[i][2] * array[i][j];
			Y[j] -= array[i][j];
		}
	}

	double d = Determinant(Matrix, 3);

	if (abs(d) < 0.0001)
	{
		printf("\n矩阵奇异");
		getchar();
		//return -1;
	}

	Inverse(Matrix, IMatrix, 3, d);

	for (int i = 0; i < 3; i++)
	{
		A += *(IMatrix[0] + i)*Y[i];
		B += *(IMatrix[1] + i)*Y[i];
		C += *(IMatrix[2] + i)*Y[i];
	}

	printf("\n A = %5.3f,B = %5.3f,C= %5.3f", A, B, C);

	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
	m_cloud = Distance(A, B, C, cloud);

	for (int i = 0; i < 3; i++)
	{
		delete[] Matrix[i];
		delete[] IMatrix[i];
	}

	return m_cloud;
}

//////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f readf(string str){
	Eigen::Matrix4f mat;

	ifstream  file(str);
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			file >> mat(i, j);
		}
	}

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			cout << mat(i, j) << ' ';
		}
		cout << endl;
	}
	file.close();

	return mat;
}
//////////////////////////////////////////////////////////////////////////

bool PCSegment::PassThrough_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr &m_cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZ>);

	if (cloud->points.size() <= 100)
	{
		std::cerr << "ERROR pass." << std::endl;
		return false;
	}
	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	//pcl::VoxelGrid<pcl::PointXYZ> sor;
	//sor.setInputCloud(cloud_t1);
	//sor.setLeafSize(0.005f, 0.005f, 0.005f);
	//sor.filter(*cloud_filtered);
	
	Eigen::Matrix4f mat1, mat11;
	mat1 = readf("../data/cal.txt");
	//mat1 = mat11.inverse();
	pcl::transformPointCloud(*cloud, *cloud_trans, mat1);

	pcl::PCDWriter writer;
	//writer.write<pcl::PointXYZ>("../data/pointCloud/Ori_cloud.pcd", *cloud);
	//writer.write<pcl::PointXYZ>("../data/pointCloud/Ori_cloud.pcd", *cloud_trans);

	if (cloud_trans->points.size() <= 100)
	{
		std::cerr << "ERROR pass." << std::endl;
		return false;
	}

	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_trans);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-0.2, 0.5);
	//pass.setFilterLimitsNegative (true);
	pass.filter(*cloud_result1);

	pass.setInputCloud(cloud_result1);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-0.15, 0.25);
	//pass.setFilterLimitsNegative (true);
	pass.filter(*cloud_result);

	if (cloud_result->points.size() <= 100)
	{
		std::cerr << "ERROR pass." << std::endl;
		return false;
	}

	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud_result, *cloud_result, indices);

	//滤掉离群点
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_result);
	sor.setMeanK(500);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_filtered1);

	m_cloud = cloud_filtered1;

	//writer.write<pcl::PointXYZ>("../data/pointCloud/pass.pcd", *m_cloud);
	
	// Normal estimation*
	//pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;      //法线估计对象
	//pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);   //存储估计的法线
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);  //定义kd树指针
	//tree->setInputCloud(cloud_result);   ///用cloud构建tree对象
	//n.setInputCloud(cloud_result);
	//n.setSearchMethod(tree);
	//n.setKSearch(50);
	//n.compute(*normals);       ////估计法线存储到其中
	//// Concatenate the XYZ and normal fields*
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	//pcl::concatenateFields(*cloud_result, *normals, *m_cloud);    //连接字段

	return true;
}

bool PCSegment::PlaneModel_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float distance,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &m_cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	//seg.setMaxIterations(1000);
	seg.setDistanceThreshold(distance);
	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PCDWriter writer;

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() == 0)
	{
		std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
		return false;
	}
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	//非平面
	extract.setNegative(true);
	extract.filter(*cloud_filtered);
	
	//writer.write<pcl::PointXYZ>("../data/pointCloud/NOplane.pcd", *cloud_filtered);
	//平面
	extract.setNegative(false);
	extract.filter(*cloud_plane);
	//writer.write<pcl::PointXYZ>("../data/pointCloud/plane.pcd", *cloud_plane);
	if (cloud_plane->points.size() == 0){
		std::cerr << "ERROR plane" << std::endl;
		return false;
	}
	if (cloud_filtered->size() != 0 && cloud_plane->size() != 0)
	{
		std::vector<pcl::PointXYZ> point;
		//拟合平面
		//m_cloud = process(cloud_filtered, cloud_plane);

		for (int i = 0; i < cloud_filtered->size(); i++)
		{
			if (cloud_filtered->points[i].z > 0.015){
				point.push_back(cloud_filtered->points[i]);
			}
		}
		std::cout << point.size() << std::endl;
		m_cloud->width = point.size(); m_cloud->height = 1;
		m_cloud->resize(m_cloud->width*m_cloud->height);
		for (int i = 0; i < point.size(); i++)
		{
			m_cloud->points[i].x = point[i].x;
			m_cloud->points[i].y = point[i].y;
			m_cloud->points[i].z = point[i].z;
		}
	}
	
	if (m_cloud->points.size() == 0){
		std::cerr << "ERROR plane" << std::endl;
		return false;
	}

	//writer.write<pcl::PointXYZ>("../data/pointCloud/cloud_plane.pcd", *m_cloud);
	return true;
}

//法线计算和快速网格化
void PCSegment::Normal_Triangles(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int x){

	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;      //法线估计对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);   //存储估计的法线
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);  //定义kd树指针
	tree->setInputCloud(cloud);   ///用cloud构建tree对象
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(50);
	n.compute(*normals);       ////估计法线存储到其中
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);    //连接字段
	//* cloud_with_normals = cloud + normals

	/*pcl::PCDWriter w;
	w.write<pcl::PointNormal>("test.pcd", *cloud_with_normals);*/
	//定义搜索树对象
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);   //点云构建搜索树

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   //定义三角化对象
	pcl::PolygonMesh triangles;                //存储最终三角化的网络模型

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);  //设置连接点之间的最大距离，（即是三角形最大边长）

	// 设置各参数值
	gp3.setMu(2.5);  //设置被样本点搜索其近邻点的最远距离为2.5，为了使用点云密度的变化
	gp3.setMaximumNearestNeighbors(1000);    //设置样本点可搜索的邻域个数
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 设置某点法线方向偏离样本点法线的最大角度45
	gp3.setMinimumAngle(M_PI / 18); // 设置三角化后得到的三角形内角的最小的角度为10
	gp3.setMaximumAngle(2 * M_PI / 3); // 设置三角化后得到的三角形内角的最大角度为120
	gp3.setNormalConsistency(false);  //设置该参数保证法线朝向一致

	// Get result
	gp3.setInputCloud(cloud_with_normals);     //设置输入点云为有向点云
	gp3.setSearchMethod(tree2);   //设置搜索方式
	gp3.reconstruct(triangles);  //重建提取三角化

	// 附加顶点信息
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	//pcl::io::saveVTKFile("mesh.vtk", triangles);
	pcl::io::savePLYFile("../PointCloud/mesh_"+std::to_string(x)+".ply", triangles);

}

//////////////////////////////////////////////////////////////////////////
bool PCSegment::EuclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> &vec,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &m_cloud){
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>);

	int M = cloud->points.size();
	cloud_copy->width = 1;
	cloud_copy->height = M;
	cloud_copy->resize(1*M);
	for (int i = 0; i < M; i++)
	{
	pcl::PointXYZ p;
	p.x = cloud->points[i].x;
	p.y = cloud->points[i].y;
	p.z = cloud->points[i].z;
	cloud_copy->points.push_back(p);
	}*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_part(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_part1(new pcl::PointCloud<pcl::PointXYZ>);
	
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_PointNormal(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PCDWriter writer;

	//滤掉离群点
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_part1);

	tree->setInputCloud(cloud_part1);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.05); 
	ec.setMinClusterSize(250);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	Eigen::Vector4f centroid;
	centroid.Identity();
	vec.clear();

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
			pcl::PointXYZ p;
			p.x = cloud->points[*pit].x;
			p.y = cloud->points[*pit].y;
			p.z = cloud->points[*pit].z;

			cloud_cluster->points.push_back(p);
			cloud_part->points.push_back(p);
		}

		pcl::compute3DCentroid(*cloud_part, centroid);
		//注意12.26改
		//centroid[2] = -centroid[2];
		vec.push_back(centroid);

		cloud_part->width = cloud_part->size();
		cloud_part->height = 1;
		cloud_part->resize(cloud_part->width * cloud_part->height);

		writer.write<pcl::PointXYZ>("../data/pointCloud/cloud_" + to_string(j) + ".pcd", *cloud_part);

		//Normal_Triangles(cloud_part, j);

		cloud_part->clear();
		j++;
	}
	cloud_cluster->height = 1;
	cloud_cluster->width = cloud->points.size();

	m_cloud = cloud_cluster;
	return true;
}

bool PCSegment::EuclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZ>::Ptr &m_cloud){
	
	if (cloud->points.size() == 0)
	{
		return false;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.1);
	ec.setMinClusterSize(30);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	if (cluster_indices.size() == 0)
	{
		std::cout << "Euclidean" << std::endl;
		return false;
	}

	//pcl::PCDWriter writer;
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
			pcl::PointXYZ p;
			p.x = cloud->points[*pit].x;
			p.y = cloud->points[*pit].y;
			p.z = cloud->points[*pit].z;

			cloud_cluster->points.push_back(p);
		}
		j++;
	}
	cloud_cluster->height = 1;
	cloud_cluster->width = cloud_cluster->points.size();

	if (cloud_cluster->points.size() == 0)
	{
		std::cout << "Euclidean" << std::endl;
		return false;
	}

	m_cloud = cloud_cluster;
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("../PointCloud/cloud_E.pcd", *cloud_cluster);
	return true;
}

//////////////////////////////////////////////////////////////////////////
