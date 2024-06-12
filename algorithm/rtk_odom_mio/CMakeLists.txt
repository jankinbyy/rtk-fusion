cmake_minimum_required(VERSION 2.8.3)

SET(CMAKE_BUILD_TYPE "Release")

ADD_COMPILE_OPTIONS(-std=c++17)
set(CMAKE_CXX_FLAGS "-std=c++17 -O3")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -std=c++0x -std=c++17 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if (CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
    include(ProcessorCount)
    ProcessorCount(N)
    message("Processer number:  ${N}")
    if (N GREATER 4)
        add_definitions(-DMP_EN)
        add_definitions(-DMP_PROC_NUM=3)
        message("core for MP: 3")
    elseif (N GREATER 3)
        add_definitions(-DMP_EN)
        add_definitions(-DMP_PROC_NUM=2)
        message("core for MP: 2")
    else ()
        add_definitions(-DMP_PROC_NUM=1)
    endif ()
else ()
    add_definitions(-DMP_PROC_NUM=1)
endif ()

find_package(Eigen3 REQUIRED)

set(ALL_TARGET_LIBRARIES "")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/
${EIGEN3_INCLUDE_DIR}
../../common/include
)

message("============================" ${CMAKE_CURRENT_SOURCE_DIR}/include/)

file(GLOB_RECURSE ALL_SRCS "src/*")

message("ALL_SRCS: ${ALL_SRCS}")

# list(REMOVE_ITEM ALL_SRCS ${LASER_MAPPING_MAIN})
include(../../cmake/yaml.cmake)
add_library(rtk_odo_mio_lib SHARED ${ALL_SRCS})
target_link_libraries(rtk_odo_mio_lib PUBLIC ${ALL_TARGET_LIBRARIES} ${EIGEN3_LIBS} glog)
target_include_directories(rtk_odo_mio_lib PUBLIC 
                            ${CMAKE_CURRENT_SOURCE_DIR}/include/
                            ${EIGEN3_INCLUDE_DIR}
                            ${YAML_CPP_INCLUDEDIR}
                        )
