cmake_minimum_required(VERSION 2.8)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# Le projet est étiqueté tests
project(Vector_tests)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wextra -Wall -std=c++11")

# On inclut la bibliothèque dans le processus de CMake
include_directories(${PROJECT_SOURCE_DIR}/src)
link_directories(${PROJECT_SOURCE_DIR}/src)

add_executable(test_affichage test_affichage.cpp)
target_link_libraries(test_affichage Vector3d)

add_executable(test_constructeur1 test_constructeur1.cpp)
target_link_libraries(test_constructeur1 Vector3d)

add_executable(test_constructeur2 test_constructeur2.cpp)
target_link_libraries(test_constructeur2 Vector3d)

add_executable(test_constructeur3 test_constructeur3.cpp)
target_link_libraries(test_constructeur3 Vector3d)

add_executable(test_operateur_affectation test_operateur_affectation.cpp)
target_link_libraries(test_operateur_affectation Vector3d)

add_executable(test_operateur_divDouble test_operateur_divDouble.cpp)
target_link_libraries(test_operateur_divDouble Vector3d)

add_executable(test_operateur_divEq test_operateur_divEq.cpp)
target_link_libraries(test_operateur_divEq Vector3d)

add_executable(test_operateur_minus test_operateur_minus.cpp)
target_link_libraries(test_operateur_minus Vector3d)

add_executable(test_operateur_minusEq test_operateur_minusEq.cpp)
target_link_libraries(test_operateur_minusEq Vector3d)

add_executable(test_operateur_minusUnaire test_operateur_minusUnaire.cpp)
target_link_libraries(test_operateur_minusUnaire Vector3d)

add_executable(test_operateur_multDouble test_operateur_multDouble.cpp)
target_link_libraries(test_operateur_multDouble Vector3d)

add_executable(test_operateur_multEq test_operateur_multEq.cpp)
target_link_libraries(test_operateur_multEq Vector3d)

add_executable(test_operateur_oflux test_operateur_oflux.cpp)
target_link_libraries(test_operateur_oflux Vector3d)

add_executable(test_operateur_plus test_operateur_plus.cpp)
target_link_libraries(test_operateur_plus Vector3d)

add_executable(test_operateur_plusEq test_operateur_plusEq.cpp)
target_link_libraries(test_operateur_plusEq Vector3d)
