cmake_minimum_required(VERSION 2.8)

project(Source)
enable_testing()
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wno-unknown-pragmas -pg -std=c++11")

find_package(PCL REQUIRED) # used version 1.9
find_package(Boost REQUIRED) # version 1.72
find_package(glm REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_definitions(
	-DTW_STATIC
	-DTW_NO_LIB_PRAGMA
	-DTW_NO_DIRECT3D
	-DGLEW_STATIC
	-D_CRT_SECURE_NO_WARNINGS

)

add_library(Vector3d SHARED vector3d.cpp)
add_library(Cloud SHARED cloud.cpp)
add_library(Plane SHARED plane.cpp)
add_library(Load_off_file SHARED load_off_file.cpp)
add_library(Consistent_orientation SHARED consistent_orientation.cpp)
add_library(ImplicitFunction SHARED ImplicitFunction.cpp)
add_library(AbstractMesh SHARED AbstractMesh.cpp)
add_library(Mesh SHARED Mesh.cpp)
add_library(MeshHE SHARED MeshHE.cpp)

#add_executable(main main.cpp)
#target_link_libraries(main Vector3d Plane Cloud Load_off_file Planes_orientation)
target_link_libraries(ImplicitFunction AbstractMesh Mesh MeshHE Consistent_orientation Vector3d Plane Cloud Load_off_file)
target_link_libraries(Cloud Plane Load_off_file)
