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

set(HEADER_FILES
	src/RealSenseCam.h
#	src/RealSenseFakeCam.h
	src/RealSenseDeprojector.h
	src/RealSensePointDeprojector.h
	src/RealSenseGrabCut.h
	src/RealSenseDistFrame.h
	src/RealSenseCalibrator.h
	src/RealSenseStreamer.h
	src/RealSenseMultiCam.h

	src/cv-helpers.hpp
)

set(SOURCE_FILES
	src/RealSenseCam.cpp
#	src/RealSenseFakeCam.cpp
	src/RealSenseDeprojector.cpp
	src/RealSensePointDeprojector.cpp
	src/RealSenseGrabCut.cpp
	src/RealSenseDistFrame.cpp
	src/RealSenseCalibrator.cpp
	src/RealSenseStreamer.cpp
	src/RealSenseMultiCam.cpp

	src/initPlugin.cpp
)

set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

#set_directory_properties( PROPERTIES COMPILE_DEFINITIONS "" )

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(INCLUDE_DIRS ${${PROJECT_NAME}_INCLUDE_EXPORTS} )

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

#target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>")
#target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>")
#target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include>")

set(REALSENSEPLUGIN_COMPILER_FLAGS "-DSOFA_BUILD_REALSENSEPLUGIN")

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${REALSENSEPLUGIN_VERSION})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_REALSENSEPLUGIN")
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${REALSENSEPLUGIN_COMPILER_FLAGS}")
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
#sofa_create_package(realsenseplugin ${REALSENSEPLUGIN_VERSION} realsenseplugin realsenseplugin)
sofa_generate_package(
	NAME ${PROJECT_NAME}
	VERSION ${REALSENSEPLUGIN_VERSION}
	TARGETS ${PROJECT_NAME}
	INCLUDE_INSTALL_DIR ${${PROJECT_NAME}_INCLUDE_DIRS}
)

#install(DIRECTORY examples/ DESTINATION share/sofa/plugins/${PROJECT_NAME})

