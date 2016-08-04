#include <ros/ros.h>
#include "lidar_tf.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_tf");
    ros::NodeHandle nh("~");

    lidar_tf::Cloud_transform cloud_transform(nh);

    ros::Rate loop(1);
    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
        cloud_transform.getLidarTransform();
    }
    ros::spin();

    return 0;
}
