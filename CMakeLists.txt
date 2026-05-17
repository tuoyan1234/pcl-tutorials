cmake_minimum_required(VERSION 3.10)

project(pcl_tutorials)

find_package(PCL 1.10 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ./include)

add_executable(1_READ_WRITE_POINT ./src/1_read_write.cpp)
target_link_libraries(1_READ_WRITE_POINT ${PCL_LIBRARIES})

add_executable(2_point_normal ./src/2_point_normals.cpp)
target_link_libraries(2_point_normal ${PCL_LIBRARIES})