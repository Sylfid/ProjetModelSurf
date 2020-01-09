cmake_minimum_required(VERSION 2.8)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# Le projet est étiqueté tests
project(Cloud_tests)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wextra -Wall -std=c++11")

# On inclut la bibliothèque dans le processus de CMake
include_directories(${PROJECT_SOURCE_DIR}/src)
link_directories(${PROJECT_SOURCE_DIR}/src)

find_package(PCL REQUIRED)

add_executable(test_basics test_basics.cpp)
target_link_libraries(test_basics Vector3d Plane Load_off_file Cloud ${PCL_LIBRARIES})

add_executable(test_cloud test_cloud.cpp)
target_link_libraries(test_cloud Vector3d Plane Load_off_file Cloud ${PCL_LIBRARIES})
