cmake_minimum_required(VERSION 2.8)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# Le projet est étiqueté tests
project(tests)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wextra -Wall -std=c++11")

add_subdirectory(Vector3d_tests)
add_subdirectory(Cloud_tests)
add_subdirectory(BoostLib_tests)

# On inclut la bibliothèque dans le processus de CMake
include_directories(${PROJECT_SOURCE_DIR}/src/build)
link_directories(${PROJECT_SOURCE_DIR}/src/build)

find_package(PCL REQUIRED) #  used version 1.9
find_package(Boost REQUIRED) # version 1.72

include_directories(${EIGEN_INCLUDE_DIRS})
link_directories(${EIGEN_LIBRARY_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

#######

# ICI AJOUTER LES TESTS

add_executable(test_kNbhd test_kNbhd.cpp)
target_link_libraries(test_kNbhd Vector3d ${PCL_LIBRARIES})

add_executable(test_load_off_file test_load_off_file.cpp)
target_link_libraries(test_load_off_file Vector3d Load_off_file ${BOOST_LIBRARIES})

add_executable(test_perso test_perso.cpp)
target_link_libraries(test_perso Vector3d)

add_executable(test_eigenlib test_eigenlib.cpp)
target_link_libraries(test_eigenlib Vector3d Plane)

add_executable(test_planes test_planes.cpp)
target_link_libraries(test_planes Vector3d Plane)

add_executable(test_consistent_orientation test_consistent_orientation.cpp)
target_link_libraries(test_consistent_orientation Consistent_orientation Vector3d Plane Cloud Load_off_file ${PCL_LIBRARIES})

add_executable(test_ImplicitFunction test_ImplicitFunction.cpp)
target_link_libraries(test_ImplicitFunction Consistent_orientation Vector3d Plane Cloud ImplicitFunction Load_off_file ${PCL_LIBRARIES})
#######
