cmake_minimum_required(VERSION 2.8.3)
project(stereo_obstacle_detection)

find_package(catkin REQUIRED  COMPONENTS tf_conversions tf roscpp message_filters sensor_msgs cv_bridge image_transport stereo_msgs pcl_conversions pcl_ros)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
catkin_package()
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
add_executable(stereo_obstacle_detection src/stereo_obstacle_detection.cpp)
target_link_libraries(stereo_obstacle_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES}  ${EIGEN3_INCLUDE_DIR} libvtkCommon.so libvtkFiltering.so)

#add_executable(stereo_obstacle_detection_imu src/stereo_obstacle_detection_imu.cpp)
#target_link_libraries(stereo_obstacle_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES}  ${EIGEN3_INCLUDE_DIR})


