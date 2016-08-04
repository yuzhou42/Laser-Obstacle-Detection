#ifndef LIDAR_TF_H
#define LIDAR_TF_H

#include <ros/ros.h>
#include "pcl_util.h"

// Msg
#include <ugv_msgs/VelodynePacket.h>
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

using namespace std;

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
    void v64Callback(const sensor_msgs::PointCloudConstPtr& packet);
    void v32Callback(const sensor_msgs::PointCloudConstPtr& packet);

    void getLidarTransform();//use icp to get the the transform between velidyne64 and velodyne32
    void process(void);
    void process_32(void);

private:

    /*!
     * Read the parameters.
     */
    void readParameters();

    //! ROS nodehandle
    ros::NodeHandle nodehandle_;

    //! ROS Publisher
    ros::Publisher pub_road_;
    ros::Publisher pub_object_;
    ros::Publisher pub_plane_;

    ros::Publisher pub_v32_;
    ros::Publisher pub_v64_;
    ros::Publisher pub_v32_cut_;
    ros::Publisher pub_v64_cut_;
    ros::Publisher pub_trans_multi_;
    ros::Publisher pub_icp_;

    //! ROS Subs
    ros::Subscriber sub_cloud32_;
    ros::Subscriber sub_cloud64_;

    ros::Time v64Time_;
    ros::Time v32Time_;

    //! TF
    tf::TransformListener tf_listen_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf2_listen_;

    //transform
    Eigen::Affine3f AffineBtwVelo_;
    tf::Transform transBtwVelo_;

    //! Parameters
    double voxel_grid_;
    double cut_min_;
    double cut_max_;
    double cut_height_;

    string strSub_v64;
    string strSub_v32;
    string strPub_Plane_;
    string strPub_Road_;
    string strPub_Object_;

    //! pcl pointcloud
    Cloud::Ptr v64_cloud;
    Cloud::Ptr v32_cloud;
    Cloud::Ptr v64_cloud_cut_;
    Cloud::Ptr v32_cloud_cut_;
    Cloud::Ptr v32_cloud_trans_;
    Cloud Final;
};

}
#endif // LIDAR_TF_H
