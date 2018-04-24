#pragma once
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree.h>
#include <Eigen/StdVector>
#include <pcl/common/transforms.h> 
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/gp3.h>      //贪婪投影三角化算法
using namespace std;

class PCSegment
{
public:
	PCSegment();
	~PCSegment();

public:
	bool PassThrough_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr &m_cloud);
	bool PlaneModel_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float distance,
		pcl::PointCloud<pcl::PointXYZ>::Ptr &m_cloud);
	bool EuclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> &vec,
		pcl::PointCloud<pcl::PointXYZ>::Ptr &m_cloud);
	bool EuclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr &m_cloud);


private:
	double Determinant(double* matrix[], int n);//方阵，行列式
	double Alco(double* matrix[], int jie, int row, int column); //代数余子式A = (-1)I + J(M)
	double Cofactor(double* matrix[], int jie, int row, int column); //余子式
	void Inverse(double *matrix1[], double*matrix2[], int n, double d);//矩阵求拟
	pcl::PointCloud<pcl::PointXYZ>::Ptr Distance(double A, double B, double C, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr process(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud);
	void Normal_Triangles(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int x);

private:
	double* color_segment;
};

