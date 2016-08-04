#include "lidar_tf.h"


using namespace lidar_tf;
using namespace std;


Cloud_transform::Cloud_transform(ros::NodeHandle nodehandle)
    : nodehandle_(nodehandle),
      v64_cloud(new Cloud),
      v32_cloud(new Cloud),
      v64_cloud_cut_(new Cloud),
      v32_cloud_cut_(new Cloud),
      v32_cloud_trans_(new Cloud),
      tf2_listen_(buffer_),
      AffineBtwVelo_(Eigen::Affine3f::Identity())
{
    readParameters();

    pub_plane_  = nodehandle_.advertise<sensor_msgs::PointCloud2> (strPub_Plane_, 1);
    pub_road_   = nodehandle_.advertise<sensor_msgs::PointCloud2> (strPub_Road_, 1);
    pub_object_ = nodehandle_.advertise<sensor_msgs::PointCloud2> (strPub_Object_, 1);

    pub_v32_    = nodehandle_.advertise<sensor_msgs::PointCloud2>("v32",1);
    pub_v64_    = nodehandle_.advertise<sensor_msgs::PointCloud2>("v64",1);
    pub_v32_cut_    = nodehandle_.advertise<sensor_msgs::PointCloud2>("v32_cut",1);
    pub_v64_cut_    = nodehandle_.advertise<sensor_msgs::PointCloud2>("v64_cut",1);
    pub_trans_multi_    = nodehandle_.advertise<sensor_msgs::PointCloud2>("trans_multi",1);
    pub_icp_    = nodehandle_.advertise<sensor_msgs::PointCloud2>("icp",1);

    sub_cloud64_  = nodehandle_.subscribe(strSub_v64, 1, &Cloud_transform::v64Callback, this);
    sub_cloud32_  = nodehandle_.subscribe(strSub_v32, 1, &Cloud_transform::v32Callback, this);
}

void Cloud_transform::readParameters()
{
    nodehandle_.param("sub64",  strSub_v64,     string("/velodyne64_driver/velodyne64_packet"));
    nodehandle_.param("sub32",  strSub_v32,     string("/velodyne2Packet"));
    nodehandle_.param("plane",      strPub_Plane_,  string("/plane"));
    nodehandle_.param("noplane",    strPub_Road_,   string("/road"));
    nodehandle_.param("project",    strPub_Object_, string("/object"));

    nodehandle_.param("voxel_grid",voxel_grid_, double(0.05));
    nodehandle_.param("cut_min", cut_min_, double(2.0));
    nodehandle_.param("cut_max", cut_max_, double(30.0));
    nodehandle_.param("cut_height", cut_height_, double(-1.5));
}

void Cloud_transform::v64Callback(const sensor_msgs::PointCloudConstPtr &packet)
{
    ROS2PCL(packet, v64_cloud);
    v64Time_ = packet->header.stamp;
    process();
}

void Cloud_transform::v32Callback(const sensor_msgs::PointCloudConstPtr &packet)
{
    // transform to velodyne64 frame
    ROS2PCL(packet,v32_cloud);
    v32Time_ = packet->header.stamp;
//    sensor_msgs::PointCloudPtr packet_tf(new sensor_msgs::PointCloud);
//    //use tf2 to get the transformed pointcloud
//    geometry_msgs::TransformStamped trans_msg = buffer_.lookupTransform("velodyne64","velodyne32", ros::Time(0));
//    tf::StampedTransform trans_bt;
//    tf::transformStampedMsgToTF(trans_msg,trans_bt);
//    //need to change the /opt/indigo/include/tf/transform_listener.h file to change the private function transformPointCloud to public
//    tf_listen_.transformPointCloud("velodyne64",trans_bt,v32Time_,*packet, *packet_tf);
//    ROS2PCL(packet_tf, v32_cloud);
    process_32();
}

void Cloud_transform::process_32()
{
    //-----------------------Add different pointcloud-------------------------//
    Cloud::Ptr clouds (new Cloud);
    *clouds = *v32_cloud;
    if (clouds->size() == 0) return;
    //-----------------------PCL Filters--------------------------------------//
    Cloud::Ptr voxel_filtered (new Cloud);
    VoxelFilter(clouds, voxel_filtered, voxel_grid_);
    //-----------------------Cut Area --------------------------------------//
    subrectangle(voxel_filtered, v32_cloud_cut_,8,0,10,-10);
}


void Cloud_transform::process(void)
{
    //-----------------------Add different pointcloud-------------------------//
    Cloud::Ptr clouds (new Cloud);
    *clouds = *v64_cloud;

    if (clouds->size() == 0) return;

    //-----------------------PCL Filters--------------------------------------//
    Cloud::Ptr voxel_filtered (new Cloud);
    VoxelFilter(clouds, voxel_filtered, voxel_grid_);


    //-----------------------Plane Area --------------------------------------//
    Cloud::Ptr cloud_cut (new Cloud);
    subPlane(voxel_filtered, cloud_cut,cut_max_, cut_min_, cut_height_);
    subrectangle(voxel_filtered, v64_cloud_cut_,8,0,10,-10);

    //-----------------------ROR filter---------------------------------------//
    Cloud::Ptr Radiusfiltered(new Cloud);
    RadiusFilter(cloud_cut, Radiusfiltered, 1, 20);

    //-----------------------Get the plane model------------------------------//
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inlierIndices(new pcl::PointIndices);
    GetPlane(Radiusfiltered, coefficients, inlierIndices);


    //-----------------------project the point cloud--------------------------//
    ColorCloud::Ptr seg_cloud (new ColorCloud);
    ColorCloud::Ptr project_cloud (new ColorCloud);//    // Step4 KMeans cluster
    Cloud::Ptr cloudRoad    (new Cloud);
    Cloud::Ptr cloudObject  (new Cloud);
    Project(voxel_filtered, cloudRoad, cloudObject, seg_cloud, project_cloud, coefficients);

    //-----------------------Fill in the R=3 circle---------------------------//
    FillIn(seg_cloud, project_cloud, cloudRoad, coefficients);

    //-----------------------Kdtree search for road---------------------------//
    Cloud::Ptr out_cloud(new Cloud);
    RoadEstimation(project_cloud, out_cloud);
    RadiusFilter(out_cloud, out_cloud, 1, 5);

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
    cloud_s_p.header.stamp = v64Time_;
    pub_plane_.publish(cloud_s_p);

    pcl::toPCLPointCloud2(*out_cloud,cloud_pc2);
    pcl_conversions::fromPCL(cloud_pc2,cloud_s_p);
    cloud_s_p.header.frame_id = "vehicle";
    cloud_s_p.header.stamp = v64Time_;
    pub_road_.publish(cloud_s_p);

    pcl::toPCLPointCloud2(*cloudObject,cloud_pc2);
    pcl_conversions::fromPCL(cloud_pc2,cloud_s_p);
    cloud_s_p.header.frame_id = "vehicle";
    cloud_s_p.header.stamp = v64Time_;
    pub_object_.publish(cloud_s_p);

    pcl::toPCLPointCloud2(*v32_cloud,cloud_pc2);
    pcl_conversions::fromPCL(cloud_pc2,cloud_s_p);
    cloud_s_p.header.frame_id = "vehicle";
    cloud_s_p.header.stamp = v64Time_;
    pub_v32_.publish(cloud_s_p);

    pcl::toPCLPointCloud2(*v64_cloud,cloud_pc2);
    pcl_conversions::fromPCL(cloud_pc2,cloud_s_p);
    cloud_s_p.header.frame_id = "vehicle";
    cloud_s_p.header.stamp = v64Time_;
    pub_v64_.publish(cloud_s_p);

    pcl::toPCLPointCloud2(*v32_cloud_cut_,cloud_pc2);
    pcl_conversions::fromPCL(cloud_pc2,cloud_s_p);
    cloud_s_p.header.frame_id = "vehicle";
    cloud_s_p.header.stamp = v64Time_;
    pub_v32_cut_.publish(cloud_s_p);

    pcl::toPCLPointCloud2(*v64_cloud_cut_,cloud_pc2);
    pcl_conversions::fromPCL(cloud_pc2,cloud_s_p);
    cloud_s_p.header.frame_id = "vehicle";
    cloud_s_p.header.stamp = v64Time_;
    pub_v64_cut_.publish(cloud_s_p);

//    pcl::toPCLPointCloud2(Final,cloud_pc2);
//    pcl_conversions::fromPCL(cloud_pc2,cloud_s_p);
//    cloud_s_p.header.frame_id = "vehicle";
//    cloud_s_p.header.stamp = v64Time_;
//    pub_trans_multi_.publish(cloud_s_p);

    pcl::toPCLPointCloud2(*v32_cloud_trans_,cloud_pc2);
    pcl_conversions::fromPCL(cloud_pc2,cloud_s_p);
    cloud_s_p.header.frame_id = "vehicle";
    cloud_s_p.header.stamp = v64Time_;
    pub_trans_multi_.publish(cloud_s_p);


}

void Cloud_transform::getLidarTransform()
{
    if(v64_cloud_cut_->size()==0 || v32_cloud_cut_->size()==0)
    {
        std::cout<<"not ready to do the icp "<<std::endl;
        return;
    }
     Eigen::Matrix4f transBtwVeloBias;
     tf::Matrix3x3 trans;   //tf rotation matrix

     Eigen::Matrix3d temp_trans;//Eigen rotation matrix
     double roll,pitch,yaw;//Euler angles
     pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
     icp.setInputSource(v32_cloud_cut_);
     icp.setInputTarget(v64_cloud_cut_);
     //pcl::PointCloud<pcl::PointXYZ> Final;
     icp.align(Final);
     std::cout << "has converged:" << icp.hasConverged() << " score: " <<
     icp.getFitnessScore() << std::endl;
     transBtwVeloBias = icp.getFinalTransformation();
     pcl::transformPointCloud(*v32_cloud,*v32_cloud_trans_,transBtwVeloBias);
     //    ROS2PCL(packet_tf, v32_cloud);
     temp_trans<<transBtwVeloBias(0,0),transBtwVeloBias(0,1),transBtwVeloBias(0,2),
                 transBtwVeloBias(1,0),transBtwVeloBias(1,1),transBtwVeloBias(1,2),
                 transBtwVeloBias(2,0),transBtwVeloBias(2,1),transBtwVeloBias(2,2);
     tf::matrixEigenToTF(temp_trans,trans);
     trans.getRPY(roll,pitch,yaw);
     std::cout <<"icp_result:"<<std::endl;
     std::cout << transBtwVeloBias << std::endl;
     std::cout<<"translation: "<<" x: "<<transBtwVeloBias(0,3)<<" y: "<<transBtwVeloBias(1,3)<<" z: "<<transBtwVeloBias(2,3)<<std::endl;
     std::cout<<"roll: "<<roll<<" pitch: "<<pitch<<" yaw: "<<yaw<<std::endl;
}
