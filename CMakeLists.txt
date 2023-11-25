cmake_minimum_required(VERSION 3.5)

project(stereo)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)
include_directories("/usr/include/eigen3")
#include_directories(${EIGEN3_INCLUDE_DIR})

aux_source_directory (./src SRC_LIST)

add_executable (stereo ${SRC_LIST})
target_link_libraries(stereo ${OpenCV_LIBS})
set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
