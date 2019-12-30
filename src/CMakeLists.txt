cmake_minimum_required(VERSION 2.8)

project(Source)
enable_testing()
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wextra -Wall -pg -std=c++11")

add_library(Vector3d SHARED vector3d.cpp)
add_library(Vector3dList SHARED vector3dList.cpp)
add_library(Load_off_file SHARED load_off_file.cpp)

add_executable(main main.cpp)

target_link_libraries(main
                      Vector3d Vector3dList
)
