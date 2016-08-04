cmake_minimum_required(VERSION 2.8.3)
project(lidar_tf)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf_conversions
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

 add_executable(lidar_tf_node
        src/lidar_tf_node.cpp
        src/lidar_tf.cpp
        src/pcl_util.cpp

        include/lidar_tf.h
        include/pcl_util.h)

 add_dependencies(lidar_tf_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(lidar_tf_node
   ${catkin_LIBRARIES}
 )
