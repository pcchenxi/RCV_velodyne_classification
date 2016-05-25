cmake_minimum_required(VERSION 2.8.3)
project(terrain)

set (PCL_DIR ~/opencv/pcl/share/pcl-1.8)
set (OpenCV_DIR ~/opencv/opencv3.0.0/share/OpenCV)

find_package(OpenCV 3.0 EXACT REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  pcl_ros
  laser_geometry
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES terrain
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ~/opencv/pcl/include/pcl-1.8
  ~/opencv/opencv3.0.0/include
  ~/opencv/opencv3.0.0/build/opencv_contrib/modules/ximgproc/include
  ~/opencv/opencv3.0.0/build/opencv_contrib/modules/xfeatures2d/include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ~/opencv/pcl/build/lib
  ~/opencv/opencv3.0.0/build/lib
  ${OpenCV_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(velodyne_point_filter src/velodyne_point_filter.cpp)
add_executable(image_based_cloud_processor src/image_based_cloud_processor.cpp)
add_executable(pointshape_based_processor src/pointshape_based_processor.cpp)
add_executable(assambled_cloud_classifier src/assambled_cloud_classifier.cpp)

target_link_libraries(assambled_cloud_classifier
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(pointshape_based_processor
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(image_based_cloud_processor
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(velodyne_point_filter
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)




