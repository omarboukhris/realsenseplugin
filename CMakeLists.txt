cmake_minimum_required(VERSION 3.1)
project(realsenseplugin)

set(REALSENSEPLUGIN_MAJOR_VERSION 0)
set(REALSENSEPLUGIN_MINOR_VERSION 1)
set(REALSENSEPLUGIN_VERSION ${REALSENSEPLUGIN_MAJOR_VERSION}.${REALSENSEPLUGIN_MINOR_VERSION})

find_package(realsense2 REQUIRED)
#find_package(CImgPlugin QUIET)
#find_package(image QUIET)

#find_package(PCL 1.8 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

#find_package(Boost 1.59 COMPONENTS system filesystem REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenCVPlugin QUIET)
#find_package(PCLPlugin REQUIRED)

#set(ALL_LIBRARIES ${OpenCV_LIBS})

file(GLOB_RECURSE HEADER_FILES_RS
	"src/sofa/realsenseplugin/calib/*.h"
	"src/sofa/realsenseplugin/projector/*.h"
	"src/sofa/realsenseplugin/streamer/*.h"
	"src/sofa/realsenseplugin/utils/*.h"
	"src/sofa/realsenseplugin/RSData.h"
	"src/sofa/realsenseplugin/cv-helpers.hpp"
)

file(GLOB_RECURSE SOURCE_FILES_RS
	"src/sofa/realsenseplugin/calib/*.cpp"
	"src/sofa/realsenseplugin/projector/*.cpp"
	"src/sofa/realsenseplugin/streamer/*.cpp"
	"src/sofa/realsenseplugin/utils/*.cpp"
	"src/sofa/realsenseplugin/RSData.cpp"
	"src/sofa/realsenseplugin/initPlugin.cpp"
)

list(APPEND "" ${HEADER_FILES_OCV})
list(APPEND "" ${SOURCE_FILES_OCV})
if(OpenCVPlugin_FOUND)
	file(GLOB_RECURSE HEADER_FILES_OCV "src/sofa/realsenseplugin/ImageData2RealSenseDataFrame.h")
	file(GLOB_RECURSE SOURCE_FILES_OCV "src/sofa/realsenseplugin/ImageData2RealSenseDataFrame.cpp")
endif()


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
	"deprecated/*.h"
	"deprecated/*.inl"
	"deprecated/*.cpp"
)

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
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES_RS} ${HEADER_FILES_RS} ${SOURCE_FILES_OCV} ${HEADER_FILES_OCV} ${SCENES_FILES} ${IGNORED_FILES} ${DEPRECATED_FILES} ${README_FILES} )

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${REALSENSEPLUGIN_VERSION})
#set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_REALSENSEPLUGIN")
#set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")
#set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${REALSENSEPLUGIN_COMPILER_FLAGS}")
target_link_libraries(
	${PROJECT_NAME}
	${PCL_LIBRARIES}
#	${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY}
	${realsense2_LIBRARY}
	CImgPlugin ${OpenCV_LIBS}
	SofaGuiQt
	SofaMeshCollision SofaMiscCollision SofaBaseCollision
	SofaGuiCommon SofaBaseVisual
	SofaExporter SofaLoader
	SofaMiscForceField
	SofaGeneralEngine
)
if (OpenCVPlugin_FOUND)
	target_link_libraries( ${PROJECT_NAME} OpenCVPlugin )
endif()

## Install rules for the library; CMake package configurations files
## Install rules for the library and headers; CMake package configurations files
set(${PROJECT_NAME}_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${${PROJECT_NAME}_INCLUDE_DIRS}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
#sofa_create_package(realsenseplugin ${REALSENSEPLUGIN_VERSION} realsenseplugin realsenseplugin)
sofa_create_package_with_targets(
	PACKAGE_NAME ${PROJECT_NAME}
	PACKAGE_VERSION ${REALSENSEPLUGIN_VERSION}
	TARGETS ${PROJECT_NAME}
	INCLUDE_INSTALL_DIR ${${PROJECT_NAME}_INCLUDE_DIRS}
)

#install(DIRECTORY examples/ DESTINATION share/sofa/plugins/${PROJECT_NAME})

