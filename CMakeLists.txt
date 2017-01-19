cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detection)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  ugv_msgs
  geometry_msgs
  message_filters
  tf_conversions
  tf
  laser_geometry
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
        cfg/obstacle_detection.cfg
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

 add_executable(obstacle_detection_node
        src/lidar_tf_node.cpp
        src/lidar_tf.cpp
        src/pcl_util.cpp

        include/lidar_tf.h
        include/pcl_util.h)

 add_dependencies(obstacle_detection_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(obstacle_detection_node
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
 )
