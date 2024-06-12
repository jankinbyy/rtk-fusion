cmake_minimum_required(VERSION 3.0.2)
project(imu_wheel_localization)
include(cmake/yaml.cmake)
find_package(Eigen3 REQUIRED)
add_subdirectory(algorithm)
add_subdirectory(interface)
add_subdirectory(common)
find_package(nav_msgs REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(
  ${roscpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
set(ALL_TARGET_LIBRARIES  
    ${roscpp_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${nav_msgs_LIBRARIES}
)

include_directories(
  algorithm/dr_odom2/include
  algorithm/imu_wheel/include
  algorithm/rtk_odom_mio/include
  interface/ros1/include
  common/include
  ${roscpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(ros_wrapper_lib
  interface/ros1/src/localization_wrapper.cpp 
)

target_link_libraries(ros_wrapper_lib  
  ${EIGEN3_LIBS} 
  ${ALL_TARGET_LIBRARIES}
  dr_odom2_lib
  rtk_odo_mio_lib 
  imu_wheel_lib
  common_lib
  glog
  )
add_executable(${PROJECT_NAME}_node interface/ros1/src/localization_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ros_wrapper_lib
)