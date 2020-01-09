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

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(CGAL REQUIRED)

include_directories(${BOOST_INCLUDE_DIRS})
link_directories(${BOOST_LIBRARY_DIRS})
add_definitions(${BOOST_DEFINITIONS})

include_directories(${CGAL_INCLUDE_DIRS})
link_directories(${CGAL_LIBRARY_DIRS})
add_definitions(${CGAL_DEFINITIONS})

add_executable(test_boost_example test_boost_example.cpp)
add_executable(test_boost_example2 test_boost_example2.cpp)
add_executable(test_boost_example3 test_boost_example3.cpp)

add_executable(test_construct_mst test_construct_mst.cpp)
target_link_libraries(test_construct_mst Load_off_file Vector3d Plane Cloud ${PCL_LIBRARIES})

#add_executable(test_boost_emst test_boost_emst.cpp)
#target_link_libraries(test_construct_emst ${CGAL_LIBRARIES})

add_executable(test_boost_example3_multipleEdges test_boost_example3_multipleEdges.cpp)
add_executable(test_boost_example3_singleEdge test_boost_example3_singleEdge.cpp)

add_executable(test_dfs test_dfs.cpp)
target_link_libraries(test_dfs Load_off_file Vector3d Plane Cloud ${PCL_LIBRARIES})

add_executable(test_dfs_complex test_dfs_complex.cpp)
target_link_libraries(test_dfs_complex Load_off_file Vector3d Plane Cloud ${PCL_LIBRARIES})
