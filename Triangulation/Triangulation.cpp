// Triangulation.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "pch.h"
#include <iostream>
#include<pcl/point_types.h>
#include<pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include<pcl/kdtree/kdtree_flann.h>
#include<pcl/features/normal_3d.h>
#include<pcl/surface/gp3.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<boost/math/special_functions/round.hpp>
#include<fstream>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/filters/radius_outlier_removal.h>
#include<pcl/surface/mls.h>

using namespace std;

//文件读取
pcl::PointCloud<pcl::PointXYZ> readFile() {
	ifstream filePath;
	filePath.open("all_overlap.txt", ios::in);

	//求点云总数目
	int rows = -1; //eof会多读一个
	char buf[150];
	while (!filePath.eof()) {
		rows++;
		filePath.getline(buf, sizeof(buf));
	}
	filePath.clear();
	filePath.seekg(0, ios::beg); //移到最开始的位置

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = rows;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width*cloud->height);

	double tem[3];
	for (size_t i = 0; i < rows; i++) {
		filePath >> tem[0];
		filePath >> tem[1];
		filePath >> tem[2];
		
		cloud->points[i].x = tem[0];
		cloud->points[i].y = tem[1];
		cloud->points[i].z = tem[2];
	}
	return *cloud;
}

//点云简化(滤波+精简)
pcl::PointCloud<pcl::PointXYZ> cloudSimplication(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

	// Scheme1：半径滤波（无法分割大块背景点云效果，可以去除整体点云附加的一些离群点）  
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	outrem.setInputCloud(cloud);
	outrem.setRadiusSearch(3);  //搜索邻近点的范围大小
	outrem.setMinNeighborsInRadius(70); //设置查询点的邻近点集数小于X的删除

	pcl::PointCloud<pcl::PointXYZ>::Ptr filter_radius(new pcl::PointCloud<pcl::PointXYZ>);
	outrem.filter(*filter_radius);

	//下采样
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud(filter_radius);
	vg.setLeafSize(1.0f, 1.0f, 1.0f); //使用Xcm长的叶子节点大小进行下采样
	vg.filter(*cloud_down);

	//pcl::io::savePCDFile("cloud_down.pcd", *cloud_down);
	
	//// 显示结果图
	//pcl::visualization::PCLVisualizer viewer("viewer1");
	//viewer.addPointCloud(cloud_down);
	//viewer.spin();
	return *cloud_down;
}

//三角化
void triangulationPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	
	//Normal Estimation
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>n; //法线估计对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //存储法线的向量
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(30);
	n.compute(*normals);  //估计法线存储位置

	//Concatenate the XYZ and normal field
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);  //连接字段
	//point_with_normals = cloud + normals

	//定义搜索树对象
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals); //点云搜索树

	//Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal>gp3;  //定义三角化对象
	pcl::PolygonMesh triangles; //定义最终三角化的网络模型

	gp3.setSearchRadius(2.5);  //设置连接点之间的最大距离（即为三角形的最大边长）

	//设置各参数值
	gp3.setMu(3);    //设置被样本点搜索其最近邻点的最远距离，为了使用点云密度的变化
	gp3.setMaximumNearestNeighbors(100); //样本点可搜索的领域个数
	gp3.setMaximumSurfaceAngle(M_PI / 4);  //某点法向量方向偏离样本点法线的最大角度45°
	gp3.setMinimumAngle(M_PI / 18);  //设置三角化后得到的三角形内角最小角度为10°
	gp3.setMaximumAngle(2 * M_PI / 3); //设置三角化后得到的三角形内角的最大角度为120°
	gp3.setNormalConsistency(false); //设置该参数保证法线朝向一致

	//Get Result
	gp3.setInputCloud(cloud_with_normals);  //设置输入点云为有向点云
	gp3.setSearchMethod(tree2); //设置搜索方式
	gp3.reconstruct(triangles); //重建提取三角化

	pcl::io::savePLYFile("triangulation.ply", triangles);
	
	// 显示结果图
	pcl::visualization::PCLVisualizer viewer("viewer");
	viewer.addPolygonMesh(triangles);
	viewer.spin();
}

int main()
{
	pcl::PointCloud<pcl::PointXYZ>cloud,cloud_down;
	cloud = readFile();
	cloud_down=cloudSimplication(cloud.makeShared());
	triangulationPoint(cloud_down.makeShared());
}

