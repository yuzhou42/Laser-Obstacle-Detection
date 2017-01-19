#include <ros/ros.h>
#include "lidar_tf.h"
#include <dynamic_reconfigure/server.h>
#include <obstacle_detection/Cloud_transformConfig.h>
using namespace message_filters;
using namespace sensor_msgs;
Parameter param;

void Callback(obstacle_detection::Cloud_transformConfig &config, uint32_t level)
{
    param.voxel_grid_32_high           = config.voxel_grid_32_high;
    param.voxel_grid_32_low           = config.voxel_grid_32_low;
    param.filter_ground_range           = config.filter_ground_range;
    param.MinClusterSize           = config.MinClusterSize;
    param.MaxClusterSize           = config.MaxClusterSize;
    param.ClusterTolerance           = config.ClusterTolerance;
    param.distance_threshold        =config.distance_threshold;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_detection");
    ros::NodeHandle nh("~");

    lidar_tf::Cloud_transform cloud_transform(nh);
    message_filters::Subscriber<PointCloud2> lidar32(nh, "/velodyne32/velodyne_points", 1);
    message_filters::Subscriber<LaserScan> lmscan(nh,"/scan" , 1);
    typedef sync_policies::ApproximateTime<PointCloud2, LaserScan> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), lidar32, lmscan);
    sync.registerCallback(boost::bind(&lidar_tf::Cloud_transform::v32_vScan_Callback, &cloud_transform, _1, _2));
    //ros::spin();

    dynamic_reconfigure::Server<obstacle_detection::Cloud_transformConfig> server;
    dynamic_reconfigure::Server<obstacle_detection::Cloud_transformConfig>::CallbackType f;
    f = boost::bind(&Callback, _1, _2);
    server.setCallback(f);

    ros::Rate loop(10);
    while(ros::ok())
    {
        ros::spinOnce();
        cloud_transform.cfgCallback(param);  
    }
    return 0;
}
