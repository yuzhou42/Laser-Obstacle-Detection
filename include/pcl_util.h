#pragma once

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>

// Filters
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

typedef pcl::PointXYZ    PointT;
typedef pcl::PointXYZRGB PointCT;
typedef pcl::PointCloud<PointT> Cloud;
typedef pcl::PointCloud<PointCT> ColorCloud;

#include "pcl_util.h"


//! Message transform
void ROS2PCL(const sensor_msgs::PointCloudConstPtr packet,
             Cloud::Ptr cloud_out);

//! Plane Extraction
void subPlane(Cloud::Ptr cloud_in, Cloud::Ptr cloud_out,
              double cut_max, double cut_min, double cut_height);

void subrectangle(Cloud::Ptr cloud_in, Cloud::Ptr cloud_out,
                 double cut_max_x, double cut_min_x,double cut_max_y, double cut_min_y);

void GetPlane(Cloud::Ptr cloud_in,
              pcl::ModelCoefficients::Ptr coeffs,
              pcl::PointIndices::Ptr indices);

//! Filter
void RadiusFilter(Cloud::Ptr cloud_in, Cloud::Ptr cloud_out, double radius,
                  int neighbor);
void VoxelFilter(Cloud::Ptr cloud_in, Cloud::Ptr cloud_out, double leaf_size);

//! Function
void Project(Cloud::Ptr cloud_in,
             Cloud::Ptr cloudRoad,
             Cloud::Ptr cloudObject,
             ColorCloud::Ptr cloudSeg,
             ColorCloud::Ptr cloudProject,
             pcl::ModelCoefficients::Ptr coeffs);

void FillIn(ColorCloud::Ptr cloudSeg, ColorCloud::Ptr cloudProject, Cloud::Ptr cloudRoad,
            pcl::ModelCoefficients::Ptr coeffs);

void RoadEstimation(ColorCloud::Ptr cloudProject, Cloud::Ptr outCloud);
