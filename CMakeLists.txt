cmake_minimum_required(VERSION 3.1)
project(realsenseplugin)

set(REALSENSEPLUGIN_MAJOR_VERSION 0)
set(REALSENSEPLUGIN_MINOR_VERSION 1)
set(REALSENSEPLUGIN_VERSION ${REALSENSEPLUGIN_MAJOR_VERSION}.${REALSENSEPLUGIN_MINOR_VERSION})

find_package(realsense2 REQUIRED)
find_package(CImgPlugin QUIET)
#find_package(image QUIET)

find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost 1.58 COMPONENTS system filesystem REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenCVPlugin REQUIRED)

#set(ALL_LIBRARIES ${OpenCV_LIBS})

file(GLOB_RECURSE HEADER_FILES
	"src/*.h"
	"src/*.inl"
)

file(GLOB_RECURSE SOURCE_FILES
	"src/*.cpp"
)

file(GLOB_RECURSE SCENES_FILES
	"scenes/*.scn"
	"scenes/*.xml"
)

file(GLOB_RECURSE IGNORED_FILES
	"ignored/*.h"
	"ignored/*.inl"
	"ignored/*.cpp"
)

file(GLOB_RECURSE DEPRECATED_FILES
	"src/sofa/realsenseplugin/streamer/RealSenseFakeCam.*"
	"deprecated/*.h"
	"deprecated/*.inl"
	"deprecated/*.cpp"
)

#set(HEADER_FILES
#	src/sofa/realsenseplugin/streamer/RealSenseCam.h
#	src/sofa/realsenseplugin/streamer/RealSenseStreamer.h
#	src/sofa/realsenseplugin/streamer/RealSenseMultiCam.h
##	src/sofa/realsenseplugin/streamer/RealSenseFakeCam.h

#	src/sofa/realsenseplugin/RealSenseDeprojector.h
#	src/sofa/realsenseplugin/RealSensePointDeprojector.h
#	src/sofa/realsenseplugin/RealSenseGrabCut.h
#	src/sofa/realsenseplugin/RealSenseDistFrame.h
#	src/sofa/realsenseplugin/RealSenseCalibrator.h

#	src/sofa/realsenseplugin/cv-helpers.hpp
#)

#set(SOURCE_FILES
#	src/sofa/realsenseplugin/streamer/RealSenseCam.cpp
#	src/sofa/realsenseplugin/streamer/RealSenseStreamer.cpp
#	src/sofa/realsenseplugin/streamer/RealSenseMultiCam.cpp
##	src/sofa/realsenseplugin/streamer/RealSenseFakeCam.cpp

#	src/sofa/realsenseplugin/RealSenseDeprojector.cpp
#	src/sofa/realsenseplugin/RealSensePointDeprojector.cpp
#	src/sofa/realsenseplugin/RealSenseGrabCut.cpp
#	src/sofa/realsenseplugin/RealSenseDistFrame.cpp
#	src/sofa/realsenseplugin/RealSenseCalibrator.cpp

#	src/sofa/realsenseplugin/initPlugin.cpp
#)

set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

#set_directory_properties( PROPERTIES COMPILE_DEFINITIONS "" )
set_source_files_properties(${IGNORED_FILES} PROPERTIES HEADER_FILE_ONLY TRUE)
set_source_files_properties(${DEPRECATED_FILES} PROPERTIES HEADER_FILE_ONLY TRUE)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(INCLUDE_DIRS ${${PROJECT_NAME}_INCLUDE_EXPORTS} )

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

#target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>")
#target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>")
#target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include>")

set(REALSENSEPLUGIN_COMPILER_FLAGS "-DSOFA_BUILD_REALSENSEPLUGIN")

#add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES})
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES} ${SCENES_FILES} ${IGNORED_FILES} ${DEPRECATED_FILES} ${README_FILES} )

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${REALSENSEPLUGIN_VERSION})
#set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_REALSENSEPLUGIN")
#set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")
#set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${REALSENSEPLUGIN_COMPILER_FLAGS}")
target_link_libraries(
	${PROJECT_NAME}
	${PCL_LIBRARIES}
	${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY}
	${realsense2_LIBRARY}
	CImgPlugin OpenCVPlugin
	SofaGuiQt
	SofaMeshCollision SofaMiscCollision SofaBaseCollision
	SofaGuiCommon SofaBaseVisual
	SofaExporter SofaLoader
	SofaMiscForceField
	SofaGeneralEngine
)

## Install rules for the library; CMake package configurations files
## Install rules for the library and headers; CMake package configurations files
set(${PROJECT_NAME}_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${${PROJECT_NAME}_INCLUDE_DIRS}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
#sofa_create_package(realsenseplugin ${REALSENSEPLUGIN_VERSION} realsenseplugin realsenseplugin)
sofa_generate_package(
	NAME ${PROJECT_NAME}
	VERSION ${REALSENSEPLUGIN_VERSION}
	TARGETS ${PROJECT_NAME}
	INCLUDE_INSTALL_DIR ${${PROJECT_NAME}_INCLUDE_DIRS}
)

#install(DIRECTORY examples/ DESTINATION share/sofa/plugins/${PROJECT_NAME})

