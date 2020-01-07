cmake_minimum_required(VERSION 2.8)

project(Source)
enable_testing()
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wextra -Wall -pg -std=c++11")

find_package(PCL 1.2 REQUIRED)
find_package(Boost REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(Vector3d SHARED vector3d.cpp)
add_library(Cloud SHARED cloud.cpp)
add_library(Plane SHARED plane.cpp)
add_library(Load_off_file SHARED load_off_file.cpp)
add_library(Consistent_orientation SHARED consistent_orientation.cpp)

#add_executable(main main.cpp)
#target_link_libraries(main Vector3d Plane Cloud Load_off_file Planes_orientation)
