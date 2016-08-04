#include "pcl_util.h"

//----------------------------------------------------------------------------//
//------------------------------message transform-----------------------------//
//----------------------------------------------------------------------------//
void ROS2PCL(const sensor_msgs::PointCloudConstPtr packet,
             Cloud::Ptr cloud_out)
{
    //------------------Transform into pcl::pointcloud-------------------//
    sensor_msgs::PointCloud2 cloud_2;
    sensor_msgs::convertPointCloudToPointCloud2(*packet, cloud_2);

    pcl::PCLPointCloud2 cloud_pc2;
    pcl_conversions::toPCL(cloud_2, cloud_pc2);

    pcl::fromPCLPointCloud2(cloud_pc2, *cloud_out);
}

void subPlane(Cloud::Ptr cloud_in, Cloud::Ptr cloud_out,
              double cut_max, double cut_min, double cut_height)
{
    Cloud::Ptr cloud (new Cloud);
    for (size_t i = 0; i < cloud_in->points.size(); i++)
    {
        float distance = sqrt(pow(cloud_in->points[i].x, 2) + pow(cloud_in->points[i].y, 2));

        if(distance <= cut_min && cloud_in->points[i].z < cut_height)
        {
            cloud->push_back(cloud_in->points[i]);
            continue;
        }

        if (distance <= cut_max && distance > cut_min && cloud_in->points[i].z < cut_height)
        {
            cloud->push_back(cloud_in->points[i]);
        }
    }

    *cloud_out = *cloud;
    return;
}
void subrectangle(Cloud::Ptr cloud_in, Cloud::Ptr cloud_out,
                 double cut_max_x, double cut_min_x,double cut_max_y, double cut_min_y)
{
    Cloud::Ptr cloud(new Cloud);
    for (size_t i = 0; i < cloud_in->points.size(); i++)
    {
        if(cloud_in->points[i].x >= cut_min_x&&cloud_in->points[i].x <= cut_max_x&&
                cloud_in->points[i].y >= cut_min_y&&cloud_in->points[i].y <= cut_max_y)
        {
            cloud->push_back(cloud_in->points[i]);
        }
    }
    *cloud_out = *cloud;
    return;
}

void GetPlane(Cloud::Ptr cloud_in,
              pcl::ModelCoefficients::Ptr coeffs,
              pcl::PointIndices::Ptr indices)
{

    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    segmentation.setInputCloud(cloud_in);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(0.1);
    segmentation.setOptimizeCoefficients(true);
    segmentation.segment(*indices,*coeffs);

    return;
}

//----------------------------------------------------------------------------//
//----------------------------------Filters-----------------------------------//
//----------------------------------------------------------------------------//
void RadiusFilter(Cloud::Ptr cloud_in, Cloud::Ptr cloud_out,
                  double radius, int neighbor)
{
    // Filter object
    Cloud::Ptr cloud (new Cloud);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud_in);

    // Every point must have 10 neighbors within 15cm, or it will be removed
    filter.setRadiusSearch(radius);
    filter.setMinNeighborsInRadius(neighbor);
    filter.filter(*cloud);

    *cloud_out = *cloud;

    return;
}
void VoxelFilter(Cloud::Ptr cloud_in, Cloud::Ptr cloud_out, double leaf_size)
{
    Cloud::Ptr cloud (new Cloud);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud_in);
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.filter(*cloud);

    *cloud_out = *cloud;

    return;
}
void Project(Cloud::Ptr cloud_in,
             Cloud::Ptr cloudRoad,
             Cloud::Ptr cloudObject,
             ColorCloud::Ptr cloudSeg,
             ColorCloud::Ptr cloudProject,
             pcl::ModelCoefficients::Ptr coeffs)
{

    for (size_t i = 0; i < cloud_in->size(); ++i)
    {
        PointCT rgb_point, project_point;
        PointT point;
        rgb_point.x = cloud_in->points[i].x;
        rgb_point.y = cloud_in->points[i].y;
        rgb_point.z = cloud_in->points[i].z;

        point.x = rgb_point.x;
        point.y = rgb_point.y;
        point.z = rgb_point.z;

        // cut the distance that we don't need
        double distance = sqrt(pow(rgb_point.x,2) + pow(rgb_point.y,2));
        if( distance > 30 || (distance < 2 && cloud_in->points[i].z > -2)) continue;

        double height = cloud_in->points[i].x * coeffs->values[0] +
                      cloud_in->points[i].y * coeffs->values[1] +
                      cloud_in->points[i].z * coeffs->values[2] +
                      coeffs->values[3];
        project_point.x = rgb_point.x;
        project_point.y = rgb_point.y;
        project_point.z = rgb_point.z;


        if (height <= 0.5)
        {
            rgb_point.r = 0;
            rgb_point.g = 200;
            rgb_point.b = 0;
            project_point.r = 0;
            project_point.g = 200;
            project_point.b = 0;
            cloudRoad->push_back(point);
        }
        else
        {
            rgb_point.r = 200;
            rgb_point.g = 0;
            rgb_point.b = 0;
            project_point.r = 200;
            project_point.g = 0;
            project_point.b = 0;
            if (height > 1) cloudObject->push_back(point);
        }
        cloudSeg->push_back(rgb_point);
        cloudProject->push_back(project_point);
    }

    return;
}


void FillIn(ColorCloud::Ptr cloudSeg, ColorCloud::Ptr cloudProject,
            Cloud::Ptr cloudRoad,
            pcl::ModelCoefficients::Ptr coeffs)
{
    Cloud::Ptr cloud (new Cloud);
    for (size_t i = 0; i <5000; ++i)
    {
       pcl::PointXYZ point;
       point.x = 8 * (rand () / (RAND_MAX + 1.0f)-0.5);
       point.y = 8 * (rand () / (RAND_MAX + 1.0f)-0.5);

       double distance = sqrt(pow(point.x, 2) + pow(point.y, 2));
       if (distance > 4 || point.x > 2) continue;

       point.z = -(point.x * coeffs->values[0] +
                          point.y * coeffs->values[1] +
                          coeffs->values[3])/coeffs->values[2];
       cloud->push_back(point);
    }

    VoxelFilter(cloud, cloud, 0.4);

    for(size_t i =0; i < cloud->size(); i++)
    {
       cloudRoad->push_back(cloud->points[i]);
       PointCT rgb_point;
       rgb_point.x = cloud->points[i].x;
       rgb_point.y = cloud->points[i].y;
       rgb_point.z = cloud->points[i].z;
       rgb_point.r = 0;
       rgb_point.g = 200;
       rgb_point.b = 0;
       cloudSeg->push_back(rgb_point);
       cloudProject->push_back(rgb_point);
    }

    return;
}


void RoadEstimation(ColorCloud::Ptr cloudProject, Cloud::Ptr outCloud)
{
    Cloud::Ptr cloud (new Cloud);
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree_search;
    kdtree_search.setInputCloud(cloudProject);
    int K_project = 20;
    for (size_t i=0; i < cloudProject->size(); i++)
    {
        std::vector<int> pointIdxNKNSearch(K_project);
        std::vector<float> pointNKNSquareDistance(K_project);

        if (cloudProject->points[i].g < 200) continue;

        if (kdtree_search.nearestKSearch(cloudProject->points[i],
                                     K_project,
                                     pointIdxNKNSearch,
                                     pointNKNSquareDistance) > 0)
        {
            double sum_plane = 0.0;
            for(size_t j = 0; j < pointIdxNKNSearch.size(); j++)
            {
                if (cloudProject->points[pointIdxNKNSearch[j] ].g > 100)
                    sum_plane += 1.0;
            }
            sum_plane /= pointIdxNKNSearch.size();

            if (sum_plane > 0.95)
            {
                pcl::PointXYZ point;
                point.x = cloudProject->points[i].x;
                point.y = cloudProject->points[i].y;
                point.z = cloudProject->points[i].z;

                cloud->push_back(point);
            }
        }
    }

    *outCloud = *cloud;

    return;
}

