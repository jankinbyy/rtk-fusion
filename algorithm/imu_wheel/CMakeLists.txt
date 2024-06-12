cmake_minimum_required(VERSION 2.8.3)
project(imu_wheel)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
find_package(Eigen3  REQUIRED)
include_directories(
  include
  ../../common/include
  ${EIGEN3_INCLUDE_DIR}
)
file(GLOB_RECURSE ALL_SRCS src/*)
add_library(imu_wheel_lib SHARED ${ALL_SRCS})
target_link_libraries(imu_wheel_lib PUBLIC ${ALL_TARGET_LIBRARIES})