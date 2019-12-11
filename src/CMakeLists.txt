cmake_minimum_required(VERSION 2.8)

project(Source)
enable_testing()
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wextra -Wall -pg -std=c++11")

add_library(Point SHARED point.cpp)
add_library(PointList SHARED pointList.cpp)

add_executable(main main.cpp)

target_link_libraries(main
                      Point PointList
)
