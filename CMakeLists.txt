cmake_minimum_required(VERSION 2.8)

project(ProjetModelSurf)
enable_testing()

set(CMAKE_BUILD_TYPE Release)
add_subdirectory(${PROJECT_SOURCE_DIR}/src)
add_subdirectory(${PROJECT_SOURCE_DIR}/tests)
add_subdirectory(PCL VERSION 1.9.1.99)

#find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

##
# Programme de test
##

include_directories(${PROJECT_SOURCE_DIR}/tests)
link_directories(${PROJECT_SOURCE_DIR}/tests)

add_test(Test_accesseur tests/Vector3d_tests/test_accesseur)
add_test(valgrindTest_accesseur valgrind tests/Vector3d_tests/test_accesseur)

add_test(Test_affichage tests/Vector3d_tests/test_affichage)
add_test(valgrindTest_affichage valgrind tests/Vector3d_tests/test_affichage)

add_test(Test_constructeur1 tests/Vector3d_tests/test_constructeur1)
add_test(valgrindTest_constructeur1 valgrind tests/Vector3d_tests/test_constructeur1)

add_test(Test_constructeur2 tests/Vector3d_tests/test_constructeur2)
add_test(valgrindTest_constructeur2 valgrind tests/Vector3d_tests/test_constructeur2)

add_test(Test_constructeur3 tests/Vector3d_tests/test_constructeur3)
add_test(valgrindTest_constructeur3 valgrind tests/Vector3d_tests/test_constructeur3)

add_test(Test_operateur_affectation tests/Vector3d_tests/test_operateur_affectation)
add_test(valgrindTest_operateur_affectation valgrind tests/Vector3d_tests/test_operateur_affectation)

add_test(Test_operateur_divDouble tests/Vector3d_tests/test_operateur_divDouble)
add_test(valgrindTest_operateur_divDouble valgrind tests/Vector3d_tests/test_operateur_divDouble)

add_test(Test_operateur_divEq tests/Vector3d_tests/test_operateur_divEq)
add_test(valgrindTest_operateur_divEq valgrind tests/Vector3d_tests/test_operateur_divEq)

add_test(Test_operateur_minus tests/Vector3d_tests/test_operateur_minus)
add_test(valgrindTest_operateur_minus valgrind tests/Vector3d_tests/test_operateur_minus)

add_test(Test_operateur_minusEq tests/Vector3d_tests/test_operateur_minusEq)
add_test(valgrindTest_operateur_minusEq valgrind tests/Vector3d_tests/test_operateur_minusEq)

add_test(Test_operateur_minusUnaire tests/Vector3d_tests/test_operateur_minusUnaire)
add_test(valgrindTest_operateur_minusUnaire valgrind tests/Vector3d_tests/test_operateur_minusUnaire)

add_test(Test_operateur_multDouble tests/Vector3d_tests/test_operateur_multDouble)
add_test(valgrindTest_operateur_multDouble valgrind tests/Vector3d_tests/test_operateur_multDouble)

add_test(Test_operateur_multEq tests/Vector3d_tests/test_operateur_multEq)
add_test(valgrindTest_operateur_multEq valgrind tests/Vector3d_tests/test_operateur_multEq)

add_test(Test_operateur_oflux tests/Vector3d_tests/test_operateur_oflux)
add_test(valgrindTest_operateur_oflux valgrind tests/Vector3d_tests/test_operateur_oflux)

add_test(Test_operateur_plus tests/Vector3d_tests/test_operateur_plus)
add_test(valgrindTest_operateur_plus valgrind tests/Vector3d_tests/test_operateur_plus)

add_test(Test_operateur_plusEq tests/Vector3d_tests/test_operateur_plusEq)
add_test(valgrindTest_operateur_plusEq valgrind tests/Vector3d_tests/test_operateur_plusEq)

add_test(Test tests/test)
add_test(valgrindTestvalgrind tests/test)

add_test(Test_load_off_file tests/test_load_off_file)
add_test(valgrindTest_load_off_filevalgrind tests/test_load_off_file)

add_test(Test_planes tests/test_planes)
add_test(valgrindTest_planesvalgrind tests/test_planes)

add_test(Test_basics tests/Cloud_tests/test_basics)
add_test(valgrindTest_basicsvalgrind tests/Cloud_tests/test_basics)

# version verbose de tous les tests
add_custom_target(check ctest -V)

##
# pour créer une archive
##
SET(ARCHIVE_DIR "${CMAKE_PROJECT_NAME}")
ADD_CUSTOM_TARGET(archive
    COMMAND rm -rf ${ARCHIVE_DIR}
	COMMAND mkdir  ${ARCHIVE_DIR}
	COMMAND cp -r ${CMAKE_SOURCE_DIR}/* ${ARCHIVE_DIR} || true
	COMMAND rm -rf ${ARCHIVE_DIR}/build
	COMMAND mkdir ${ARCHIVE_DIR}/build
  	COMMAND tar --exclude="*~" --exclude="._*" -zcvf ${ARCHIVE_DIR}.tar.gz ${ARCHIVE_DIR}
	COMMAND rm -rf  ${ARCHIVE_DIR}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

# POUR INSTALLER GLUT LIBRARIES
# sudo apt-get install freeglut3-dev
# sudo apt-get install libxmu-dev libxi-dev
