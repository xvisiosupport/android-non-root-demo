cmake_minimum_required(VERSION 3.5)

project(slam_3dof_display_calib)

if ( WIN32 )
    set(xvsdk_DIR "../../cmake/xvsdk")
endif()
find_package( xvsdk REQUIRED )
set(xvsdk_INCLUDE ${xvsdk_INCLUDE_DIRS}/xvsdk})
include_directories( ${xvsdk_INCLUDE} )

set(SRCS slam_3dof_display_calib.cpp)

ADD_EXECUTABLE( ${PROJECT_NAME} ${SRCS} )
TARGET_LINK_LIBRARIES( ${PROJECT_NAME} ${xvsdk_LIBRARIES} ${OpenCV_LIBS} -pthread )

