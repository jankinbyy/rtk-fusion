cmake_minimum_required(VERSION 3.0.2)
project(alg_location)
include_directories(
  include
  ../common/include
)
add_subdirectory(dr_odom2)
add_subdirectory(rtk_odom_mio)
add_subdirectory(imu_wheel)