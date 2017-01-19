#include "lidar_tf.h"
using namespace lidar_tf;
using namespace std;

Cloud_transform::Cloud_transform(ros::NodeHandle nodehandle)
    : nodehandle_(nodehandle),
      v32_cloud_(new Cloud),
      vscan_cloud_(new Cloud),
      vscan_inflated_cloud_(new Cloud),
      all_cloud_(new Cloud),
      all_cloud_filter_(new Cloud),
      cloud_cluster_(new ColorCloud),
      marker_(new visualization_msgs::Marker),
      centroids_(new geometry_msgs::PoseArray),
      v32_filter_high_(new Cloud),
      v32_filter_low_(new Cloud),
      v32_fit_plane_(new Cloud),
      coefficients_(new ModelCoefficients),
      inlierIndices_(new PointIndices)
{
    readParameters();
    //obstacle detection
    pub_cluster_    = nodehandle_.advertise<sensor_msgs::PointCloud2>("cluster",1);
    pub_marker_     = nodehandle_.advertise<visualization_msgs::Marker>("centroid_maker",1);
    pub_centroids_  = nodehandle_.advertise<geometry_msgs::PoseArray>("cluster_centroids",1);
    pub_plane_      = nodehandle_.advertise<sensor_msgs::PointCloud2> (strPub_Plane_, 1);
    pub_object_     = nodehandle_.advertise<sensor_msgs::PointCloud2> (strPub_Object_, 1);
    pub_inflated_scan_ = nodehandle_.advertise<sensor_msgs::PointCloud2>("inflated_scan",1);
 //   sub_cloud32_    = nodehandle_.subscribe(strSub_v32, 1, &Cloud_transform::v32Callback, this);
}

void Cloud_transform::readParameters()
{
    nodehandle_.param("sub32",      strSub_v32,     string("/velodyne32/velodyne_points"));
    nodehandle_.param("subscan",    strSub_lmscan,  string("/scan"));
    nodehandle_.param("plane",      strPub_Plane_,  string("/plane"));
    //nodehandle_.param("road",    strPub_Road_,   string("/road"));
    nodehandle_.param("object",    strPub_Object_, string("/object"));
    nodehandle_.param("voxel_grid_32_high",         voxel_grid_32_high_, double(0.2));
    nodehandle_.param("voxel_grid_32_low",         voxel_grid_32_low_, double(0.4));
    nodehandle_.param("cut_min_x",                  cut_min_x_,     double(0.0));
    nodehandle_.param("cut_max_x",                  cut_max_x_,     double(10.0));
    nodehandle_.param("cut_min_y",                  cut_min_y_,     double(-5.0));
    nodehandle_.param("cut_max_y",                  cut_max_y_,     double(5.0));
    nodehandle_.param("cut_height",                 cut_height_,    double(-1.5));


    nodehandle_.param("DistanceThreshold",          distance_threshold_,
                      double(0.1));                //ransac DistanceThreshold
    nodehandle_.param("filter_ground_range",        filter_ground_range_,
                      double(0.1));
    nodehandle_.param("ClusterTolerance",           ClusterTolerance_,
                      double(0.1));
    nodehandle_.param("MinClusterSize",             MinClusterSize_,
                      int(10));
    nodehandle_.param("MaxClusterSize",             MaxClusterSize_,
                      int(20000));
    nodehandle_.param("MaxIterationsRansac",        MaxIterationsRansac_,
                      int(1000));
    nodehandle_.param("scanObstacleHeight",         scanObstacleHeight_,    double(2.0));
    getMarkerParam();
}
void Cloud_transform::cfgCallback(const Parameter cfg)
{
    cfg_ = cfg;
}
void Cloud_transform::v32_vScan_Callback(const sensor_msgs::PointCloud2ConstPtr &cloud32,const sensor_msgs::LaserScanConstPtr &scan)
{
    v32Time_ = cloud32->header.stamp;
    vScanTime_ = scan->header.stamp;
    //std::cerr<<"!!!!!!!!!!!!I got the sync msgs!!!!!!!!!!!!!!!"<<std::endl;
    sensor_msgs::PointCloud cloud1;
    sensor_msgs::PointCloudPtr packet_tf (new sensor_msgs::PointCloud);
    sensor_msgs::convertPointCloud2ToPointCloud(*cloud32, cloud1);
    tf_listen_.transformPointCloud("vehicle", cloud1, *packet_tf);
    ROS2PCL(packet_tf, v32_cloud_);

    //-----------------------VoxelFilter --------------------------------------//
    VoxelFilter(v32_cloud_, v32_filter_high_, cfg_.voxel_grid_32_high);

    //-----------------------Cut Area --------------------------------------//
    subrectangle(v32_filter_high_, v32_fit_plane_,
                 cut_max_x_,cut_min_x_,cut_max_y_,cut_min_y_,cut_height_);

    //-----------------------Get the plane model------------------------------//
    if (GetPlane(v32_fit_plane_, coefficients_, inlierIndices_,cfg_.distance_threshold) == EXIT_FAILURE)
    {
        ROS_INFO("Could not get plane.");
        return;
    }

    transScan2PC2(scan);
    *all_cloud_ = *v32_filter_high_+*vscan_inflated_cloud_;
    object_detect();
    //-----------------------VoxelFilter --------------------------------------//
    VoxelFilter(all_cloud_, all_cloud_filter_, cfg_.voxel_grid_32_low);

    plane_detect();
}

void Cloud_transform::transScan2PC2(const sensor_msgs::LaserScanConstPtr &scan)
{
    sensor_msgs::LaserScan scanMsg = *scan;
    float inflate_dis = 2.0;
    laser_geometry::LaserProjection projector;
    for (size_t i = 0; i < scanMsg.ranges.size(); i++)
    {
        if (scanMsg.ranges[i]<0.01||scanMsg.ranges[i]>15)
            scanMsg.ranges[i] = 15.0;
    }
    sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
    projector.projectLaser(scanMsg,*cloud_msg);

    sensor_msgs::PointCloud cloud1;
    sensor_msgs::PointCloudPtr packet_tf (new sensor_msgs::PointCloud);
    sensor_msgs::convertPointCloud2ToPointCloud(*cloud_msg, cloud1);
    tf_listen_.waitForTransform("vehicle","laser",ros::Time(0),ros::Duration(2.0));
    tf_listen_.transformPointCloud("vehicle", cloud1, *packet_tf);
    ROS2PCL(packet_tf, vscan_cloud_);
    Cloud cloud;
    cloud = *vscan_cloud_;
    Cloud inflate_cloud;
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator item = cloud.begin(); item != cloud.end(); item++) {
        if ((item->x == 0 && item->y == 0)||(item->x>-1.35))
            continue;
        pcl::PointXYZ p;
        //push back bottom pointcloud
        p.x = item->x;
        p.y = item->y;
        p.z = item->z;
        inflate_cloud.points.push_back(p);
        //filling pointcloud
        float step = 5; //the number of points to insert
        float theta = atan2(p.x,p.y);
        float x_increment = inflate_dis*sin(theta);
        float y_increment = inflate_dis*cos(theta);
        float x_init = p.x;
        float y_init = p.y;
        float det_x=x_init,det_y=y_init;
        for(int i=0;i<step;i++)
        {
            det_x += x_increment/step;
            det_y += y_increment/step;
            p.x = det_x;
            p.y = det_y;
            p.z = -(p.x * coefficients_->values[0] +p.y * coefficients_->values[1] +coefficients_->values[3]-scanObstacleHeight_)/coefficients_->values[2];
            inflate_cloud.points.push_back(p);
        }
        double dis = sqrt(pow(x_init,2)+pow(y_init,2));
        step = dis/0.4;
        x_increment = dis*sin(theta);
        y_increment = dis*cos(theta);
        det_x = 0.0;
        det_y =0.0;
        for(int i = 0;i<step;i++)
        {
            det_x +=x_increment/step;
            det_y +=y_increment/step;
            p.x = det_x;
            p.y = det_y;
            p.z = -(p.x * coefficients_->values[0] +p.y * coefficients_->values[1] +coefficients_->values[3])/coefficients_->values[2];
            inflate_cloud.points.push_back(p);
        }
    }
    *vscan_inflated_cloud_ =  inflate_cloud;
    //-----------------------Transform back----------------------------//

    sensor_msgs::PointCloud2 cloud_s_p;
    pcl::PCLPointCloud2 cloud_pc2;

    pcl::toPCLPointCloud2(*vscan_inflated_cloud_,cloud_pc2);
    pcl_conversions::fromPCL(cloud_pc2,cloud_s_p);
    cloud_s_p.header.frame_id = "vehicle";
    cloud_s_p.header.stamp = vScanTime_;
    pub_inflated_scan_.publish(cloud_s_p);
}

void Cloud_transform::object_detect()
{
    //-----------------------Add different pointcloud-------------------------/cut_min_y_/
    Cloud::Ptr clouds (new Cloud);
    *clouds =*all_cloud_;
    if (clouds->size() == 0) return;
    //-----------------------Cut Area --------------------------------------//
    subrectangle(clouds, clouds,
                 10.0,0.0,10.0,-10.0,10);
    //----------------------filtered ground and non ground point cloud--------------//
    Cloud::Ptr cloud_filtered_ground (new Cloud);
    RemoveGround1(clouds,cloud_filtered_ground,coefficients_,cfg_.filter_ground_range);

    //~~~~~~~~~~~~~~~~~~~~~~Euclidean Cluster Extraction~~~~~~~~~~~~~~~~~~~~~~~~~~~//
    euclidenCluster(cloud_filtered_ground,
                    cloud_cluster_,
                    cfg_.ClusterTolerance,
                    cfg_.MinClusterSize,
                    cfg_.MaxClusterSize,
                    marker_,
                    centroids_,
                    coefficients_);

    pub_marker_.publish(marker_);
    marker_->points.clear();
    pub_centroids_.publish(centroids_);

    //-----------------------Transform back----------------------------//

    sensor_msgs::PointCloud2 cloud_s_p;
    pcl::PCLPointCloud2 cloud_pc2;

    pcl::toPCLPointCloud2(*cloud_cluster_,cloud_pc2);
    pcl_conversions::fromPCL(cloud_pc2,cloud_s_p);
    cloud_s_p.header.frame_id = "vehicle";
    cloud_s_p.header.stamp = v32Time_;
    pub_cluster_.publish(cloud_s_p);

}
void Cloud_transform::plane_detect()
{
    //-----------------------project the point cloud--------------------------//
    ColorCloud::Ptr seg_cloud (new ColorCloud);
    ColorCloud::Ptr project_cloud (new ColorCloud);//    // Step4 KMeans cluster
    Cloud::Ptr cloudRoad    (new Cloud);
    Cloud::Ptr cloudObject  (new Cloud);
    Project(all_cloud_filter_, cloudRoad, cloudObject, seg_cloud, project_cloud, coefficients_,cfg_.filter_ground_range);
    //-----------------------Fill in the R=3 circle---------------------------//
    FillIn(seg_cloud, project_cloud, cloudRoad, coefficients_);
    //-----------------------Kdtree search for road---------------------------//
    Cloud::Ptr out_cloud(new Cloud);
    RoadEstimation(project_cloud, out_cloud);
    RadiusFilter(out_cloud, out_cloud, 1, 3);

    ColorCloud::Ptr plane_cloud (new ColorCloud);
    for(size_t i = 0; i < out_cloud->size(); i ++)
    {
        PointCT point;
        point.x = out_cloud->points[i].x;
        point.y = out_cloud->points[i].y;
        point.z = out_cloud->points[i].z;

        point.r = 0;
        point.g = 200;
        point.b = 0;
        plane_cloud->push_back(point);
    }
    for(size_t i = 0; i < seg_cloud->size(); i ++)
    {
        if (seg_cloud->points[i].r > 100)
            plane_cloud->push_back(seg_cloud->points[i]);
    }

    //------------------------Transform back----------------------------------//
    sensor_msgs::PointCloud2 cloud_s_p;
    pcl::PCLPointCloud2 cloud_pc2;
    pcl::toPCLPointCloud2(*plane_cloud,cloud_pc2);
    pcl_conversions::fromPCL(cloud_pc2,cloud_s_p);
    cloud_s_p.header.frame_id = "vehicle";
    cloud_s_p.header.stamp = v32Time_;
    pub_plane_.publish(cloud_s_p);

    pcl::toPCLPointCloud2(*cloudObject,cloud_pc2);
    pcl_conversions::fromPCL(cloud_pc2,cloud_s_p);
    cloud_s_p.header.frame_id = "vehicle";
    cloud_s_p.header.stamp = v32Time_;
    pub_object_.publish(cloud_s_p);

}

void Cloud_transform::getMarkerParam()
{
    marker_->header.frame_id    =   "vehicle";
    marker_->header.stamp       =   ros::Time();

    marker_->ns                 =   "lidar";
    marker_->id                 =   0;
    marker_->type               =   visualization_msgs::Marker::SPHERE_LIST;
    marker_->action             =   visualization_msgs::Marker::ADD;

    marker_->scale.x = 1.0;
    marker_->scale.y = 1.0;
    marker_->scale.z = 1.0;
    marker_->color.a = 1.0;
    marker_->color.r = 1.0;
    marker_->color.g = 1.0;
    marker_->color.b = 0.0;
    marker_->frame_locked       =   true;
}
