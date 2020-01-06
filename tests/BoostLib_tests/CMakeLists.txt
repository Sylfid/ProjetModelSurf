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

add_executable(test_boost_example test_boost_example.cpp)
add_executable(test_boost_example2 test_boost_example2.cpp)
add_executable(test_boost_example3 test_boost_example3.cpp)
#add_executable(test_construct_EMST test_construct_EMST.cpp)
