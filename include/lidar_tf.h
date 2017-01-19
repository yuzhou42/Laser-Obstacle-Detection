#ifndef LIDAR_TF_H
#define LIDAR_TF_H

#include <ros/ros.h>
#include "pcl_util.h"

// Msg
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include "laser_geometry/laser_geometry.h"
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
// TF
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
//TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2/buffer_core.h>
#include <tf2/convert.h>

//Eigen
#include "Eigen/Geometry"
#include "Eigen/Core"

//opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>

//msg filter
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace pcl;
struct Parameter
{
    double voxel_grid_32_high;
    double voxel_grid_32_low;
    double distance_threshold;
    double filter_ground_range;
    int MinClusterSize;
    int MaxClusterSize;
    double ClusterTolerance;

 
};
namespace lidar_tf {

class Cloud_transform
{
public:

    /*!
     * Constructor.
     */
    Cloud_transform (ros::NodeHandle nodehandle);

    /*!
     * Deconstructor.
     */
    virtual ~Cloud_transform () {}

    /*!
     * Callback method for the input pointcloud.
     * @param packet the input pointcloud with message type
     * sensor_msgs::PointCloud.
     */

    void v32_vScan_Callback(const sensor_msgs::PointCloud2ConstPtr& cloudv32,const sensor_msgs::LaserScanConstPtr& scan);
    void plane_detect (void);
    void object_detect(void);
    void getMarkerParam(void);
    void transScan2PC2(const sensor_msgs::LaserScanConstPtr& scan);
    void cfgCallback(const Parameter cfg);

    string strSub_v32;
    string strSub_lmscan;
    string strPub_Plane_;
    string strPub_Road_;
    string strPub_Object_;

private:

    /*!
     * Read the parameters.
     */
    void readParameters();

    //! ROS nodehandle
    ros::NodeHandle nodehandle_;

    //! ROS Publisher


    ros::Publisher pub_v32_cut_;
    ros::Publisher pub_cluster_;
    ros::Publisher pub_marker_;
    ros::Publisher pub_cloud32_;
    ros::Publisher pub_centroids_;
    ros::Publisher pub_road_;
    ros::Publisher pub_object_;
    ros::Publisher pub_plane_;
    ros::Publisher pub_inflated_scan_;


    //! ROS Subs
    ros::Subscriber sub_cloud32_;
    ros::Time v32Time_;
    ros::Time vScanTime_;

    //! TF
    tf::TransformListener tf_listen_;

    //! Parameters
    double voxel_grid_32_high_;
    double voxel_grid_32_low_;
    double cut_min_;
    double cut_max_;
    double cut_height_;
    double cut_min_x_;
    double cut_max_x_;
    double cut_min_y_;
    double cut_max_y_;
    double cut_rad_;

    double distance_threshold_;
    double filter_ground_range_;
    double ClusterTolerance_;
    double scanObstacleHeight_;
    int    MinClusterSize_;
    int    MaxClusterSize_;
    int    MaxIterationsRansac_;


    //! pcl pointcloud
    Cloud::Ptr v32_cloud_;
    Cloud::Ptr vscan_cloud_;
    Cloud::Ptr vscan_inflated_cloud_;
    Cloud::Ptr v32_filter_high_;
    Cloud::Ptr v32_filter_low_;
    Cloud::Ptr v32_fit_plane_;
    Cloud::Ptr all_cloud_;
    Cloud::Ptr all_cloud_filter_;
    ColorCloud::Ptr cloud_cluster_;
    pcl::ModelCoefficients::Ptr coefficients_;
    pcl::PointIndices::Ptr inlierIndices_;

    //clustered color
    std::vector<cv::Scalar> colors_;
    visualization_msgs::Marker::Ptr marker_;
    geometry_msgs::PoseArray::Ptr centroids_;
    Parameter cfg_;
};

}
#endif // LIDAR_TF_H
