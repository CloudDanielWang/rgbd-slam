CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )
PROJECT( Kinect )

SET( CMAKE_CXX_COMPILER "g++")
SET( CMAKE_BUILD_TYPE Release  )
SET( CMAKE_CXX_FLAGS "-O3 -std=c++11" )

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include )
LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib)

ADD_SUBDIRECTORY( ${PROJECT_SOURCE_DIR}/src )
	
