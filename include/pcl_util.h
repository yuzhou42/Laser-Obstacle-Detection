#pragma once

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

// Filters
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

//MSgs
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <ugv_msgs/VelodynePacket.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

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

void subrectangle(Cloud::Ptr cloud_in,
                  Cloud::Ptr cloud_out,
                  double cut_max_x,
                  double cut_min_x,
                  double cut_max_y,
                  double cut_min_y,
                  double cut_height);

void cutPC64(Cloud::Ptr cloud_in, Cloud::Ptr cloud_out);
void cutPC32(Cloud::Ptr cloud_in, Cloud::Ptr cloud_out);



bool GetPlane(Cloud::Ptr cloud_in,
              pcl::ModelCoefficients::Ptr coeffs,
              pcl::PointIndices::Ptr indices, double threshold);

void RemoveGround(Cloud::Ptr cloudin , pcl::PointIndices::Ptr indices ,Cloud::Ptr cloud_filtered_ground ,Cloud::Ptr cloud_ground,ColorCloud::Ptr cloud_filtered_ground_rgb, ColorCloud::Ptr cloud_ground_rgb );
void RemoveGround1(Cloud::Ptr cloud_in, Cloud::Ptr cloud_out, pcl::ModelCoefficients::Ptr coeffs, double filter_ground_range);
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
             pcl::ModelCoefficients::Ptr coeffs,
	     double filterGroundRange);

void FillIn(ColorCloud::Ptr cloudSeg, ColorCloud::Ptr cloudProject, Cloud::Ptr cloudRoad,
            pcl::ModelCoefficients::Ptr coeffs);

void RoadEstimation(ColorCloud::Ptr cloudProject, Cloud::Ptr outCloud);
//! obstacle detection
void euclidenCluster(Cloud::Ptr cloudin, ColorCloud::Ptr cloudout, double ClusterTolerance, int MinClusterSize, int MaxClusterSize, visualization_msgs::Marker::Ptr marker, geometry_msgs::PoseArray::Ptr centroids, pcl::ModelCoefficients::Ptr coeffs);
