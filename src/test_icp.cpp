/**
Software License Agreement (proprietary)

\file      test_icp.cpp
\authors   Vaibhav Mehta <vmehta@clearpathrobotics.com>
\copyright Copyright (c) 2017, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/

#include <iostream>
#include <cstdio>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <algorithm>

#include <helper_string.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <vtkVersion.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

#include "emicp.h"

using namespace std;

void loadFile(const char* fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  if (pcl::io::loadPCDFile(fileName, *cloud) == -1)
  {
    std::cerr << "Failed to read PCD file: " << fileName << std::endl;
	return;
  }

  // Remove NaN values.
  std::vector<int> index;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, index);
}

void loadFile(const char* fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  if (pcl::io::loadPCDFile(fileName, *cloud) == -1)
  {
    std::cerr << "Failed to read PCD file: " << fileName << std::endl;
	return;
  }

  // Remove NaN values.
  std::vector<int> index;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, index);
}

void voxelGridFilterCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float leaf_size)
{
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_filter.filter(*cloud);
}

void init_RT(float *h_R, float *h_t)
{
  // Set initial rotation to identity.
  h_R[0] = 1.0f;
  h_R[1] = 0.0f;
  h_R[2] = 0.0f;
  h_R[3] = 0.0f;
  h_R[4] = 1.0f;
  h_R[5] = 0.0f;
  h_R[6] = 0.0f;
  h_R[7] = 0.0f;
  h_R[8] = 1.0f;

  // Set initial translation to zeros.
  h_t[0] = 0.0f;
  h_t[1] = 0.0f;
  h_t[2] = 0.0f;
}

Eigen::Vector4f min_pt, max_pt;

void viewPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                    const std::string& viewer_window_name)
{
  pcl::visualization::PCLVisualizer visualizer(viewer_window_name);

  visualizer.addCoordinateSystem (1.0);
  visualizer.addCube(min_pt[0], max_pt[0],
                     min_pt[1], max_pt[1],
                     min_pt[2], max_pt[2],
                     1.0, 0.0, 0.0);
  visualizer.setRepresentationToWireframeForAllActors();

  visualizer.addPointCloud<pcl::PointXYZRGB> (cloud);

  visualizer.spin();
}

void viewPointCloudRaw(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                    const std::string& viewer_window_name)
{
  pcl::visualization::PCLVisualizer visualizer(viewer_window_name);
  visualizer.addCoordinateSystem (1.0);
  visualizer.addPointCloud<pcl::PointXYZRGB> (cloud);
  visualizer.spin();
}

Eigen::Quaternionf euler2Quaternion(const float roll, const float pitch, const float yaw)
{
  Eigen::AngleAxisf rollAngle((roll*M_PI) / 180, Eigen::Vector3f::UnitZ());
  Eigen::AngleAxisf yawAngle((yaw*M_PI) / 180, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf pitchAngle((pitch*M_PI) / 180, Eigen::Vector3f::UnitX());

  Eigen::Quaternionf q =  rollAngle * pitchAngle * yawAngle;
  return q;
}

void printRT(const float* R, const float* t)
{
  printf("R\n");
  for(int r=0; r<9; r++){
		printf("%f ", R[r]);
		if((r+1)%3==0) printf("\n");
	}
	printf("t\n");
	for(int r=0; r<3; r++)
		printf("%f ", t[r]);
	printf("\n");
}

int main(int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_stereo = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_template = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();

  // Load the pointclouds from specified files.
  loadFile("/home/vmehta/bags_and_data/stereo_pcd_floor.pcd", cloud_stereo);
  loadFile("/home/vmehta/bags_and_data/out_sampling.pcd", cloud_template);

  // viewPointCloudRaw(cloud_stereo, "Input unfiltered stereo cloud");
  // viewPointCloudRaw(cloud_template, "Input unfiltered template cloud");
  // loadFile(pointFileY, cloud_template);
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  Eigen::Matrix3f rot_mat = euler2Quaternion(0, 180, 0).normalized().toRotationMatrix();
  transformation.block(0,0,3,3) = rot_mat;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::transformPointCloud(*cloud_stereo, *transformed_cloud, transformation);

  min_pt[0] = -1.0; min_pt[1] = -1.0; min_pt[2] = -4.5;
  max_pt[0] =  1.0; max_pt[1] = 0.0; max_pt[2] = -2.5;

  // viewPointCloud(cleaned_up_cloud, "cleaned_up_cloud");

  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ> ());
  // pcl::CropBox<pcl::PointXYZ> crop_box;
  // crop_box.setInputCloud(cleaned_up_cloud);
  // crop_box.setMin(min_pt);
  // crop_box.setMax(max_pt);
  // crop_box.setTransform(Eigen::Affine3f::Identity());
  // crop_box.filter(*cloud_filtered);
  // std::cout<<"Min pt : "<<crop_box.getMin()<<std::endl;
  // std::cout<<"Max pt : "<<crop_box.getMax()<<std::endl;
  // std::cout<<"Size input : "<<cleaned_up_cloud->size()<<std::endl;
  // std::cout<<"Size output : "<<cloud_filtered->size()<<std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_x_filtered(new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_y_filtered(new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_z_filtered(new pcl::PointCloud<pcl::PointXYZRGB> ());

  pcl::PassThrough<pcl::PointXYZRGB> passthrough_x;
  passthrough_x.setInputCloud(transformed_cloud);
  passthrough_x.setFilterFieldName("x");
  passthrough_x.setFilterLimits(min_pt[0], max_pt[0]);
  passthrough_x.filter(*cloud_x_filtered);

  pcl::PassThrough<pcl::PointXYZRGB> passthrough_y;
  passthrough_y.setInputCloud(cloud_x_filtered);
  passthrough_y.setFilterFieldName("y");
  passthrough_y.setFilterLimits(min_pt[1], max_pt[1]);
  passthrough_y.filter(*cloud_y_filtered);

  pcl::PassThrough<pcl::PointXYZRGB> passthrough_z;
  passthrough_z.setInputCloud(cloud_y_filtered);
  passthrough_z.setFilterFieldName("z");
  passthrough_z.setFilterLimits(min_pt[2], max_pt[2]);
  passthrough_z.filter(*cloud_z_filtered);
  viewPointCloud(cloud_z_filtered, "cloud_filtered");

  // std::cout<<"Size input : "<<cloud_z_filtered->size()<<std::endl;
  // voxelGridFilterCloud(cloud_z_filtered, 0.02);
  // std::cout<<"Size output : "<<cloud_z_filtered->size()<<std::endl;
  viewPointCloud(cloud_z_filtered, "cloud voxel filtered");

  pcl::PointCloud<pcl::PointXYZ>::Ptr stereo_pcl_filtered(new pcl::PointCloud<pcl::PointXYZ> ());
  copyPointCloud(*cloud_z_filtered, *stereo_pcl_filtered);

  float* h_R = new float [9]; // rotation matrix
  float* h_t = new float [3]; // translation vector
  init_RT(h_R, h_t); // set R to Identity matrix, t to zero vector

  // initialize parameters
  registrationParameters param;

  clock_t start, end;
  start = clock();

  // Call the function that executes the algorithm.
  emicp(stereo_pcl_filtered, cloud_template, h_R, h_t, param);

  end = clock();
  printf("elapsed time: %f\n", (double)(end - start) / CLOCKS_PER_SEC);

  // Print the results and write it to file.
  printRT(h_R, h_t);
}
