cmake_minimum_required(VERSION 2.8)

project(ProjetModelSurf)
enable_testing()

set(CMAKE_BUILD_TYPE Release)
add_subdirectory(${PROJECT_SOURCE_DIR}/src)
add_subdirectory(${PROJECT_SOURCE_DIR}/test)
add_subdirectory(${PROJECT_SOURCE_DIR}/doc)


##
# Programme de test
##
include_directories(${PROJECT_SOURCE_DIR}/test)
link_directories(${PROJECT_SOURCE_DIR}/test)


#----------------

# ajouter ici les tests

#----------------

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
