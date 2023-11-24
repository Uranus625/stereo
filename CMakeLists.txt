cmake_minimum_required(VERSION 3.5)

project(stereo)

find_package(OpenCV REQUIRED)
include_directories(include)

aux_source_directory (./src SRC_LIST)

add_executable (stereo ${SRC_LIST})
target_link_libraries(stereo ${OpenCV_LIBS})
set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
