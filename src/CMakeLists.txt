cmake_minimum_required(VERSION 2.8)

project(Source)
enable_testing()
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wextra -pg -std=c++11")

find_package(PCL REQUIRED) # used version 1.9
find_package(Boost REQUIRED) # version 1.72

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${GLM_INCLUDE_DIRS})
link_directories(${GLM_LIBRARY_DIRS})
add_definitions(${GLM_DEFINITIONS})

add_library(Vector3d SHARED vector3d.cpp)
add_library(Cloud SHARED cloud.cpp)
add_library(Plane SHARED plane.cpp)
add_library(Load_off_file SHARED load_off_file.cpp)
add_library(Consistent_orientation SHARED consistent_orientation.cpp)
add_library(Mesh SHARED Mesh.cpp)
add_library(ImplicitFunction SHARED ImplicitFunction.cpp)

add_executable(Test test.cpp)
target_link_libraries(Test ${PCL_LIBRARIES} Vector3d Plane Cloud Load_off_file Consistent_orientation Mesh ImplicitFunction)
#add_executable(main main.cpp)
#target_link_libraries(main Vector3d Plane Cloud Load_off_file Planes_orientation)
