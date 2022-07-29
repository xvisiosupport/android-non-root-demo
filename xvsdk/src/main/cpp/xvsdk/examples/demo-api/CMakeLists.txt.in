cmake_minimum_required(VERSION 3.5)

project(demo-api)

set(SRC demo-api.cpp)

if ( WIN32 )
    set(xvsdk_DIR "../../cmake/xvsdk")
endif()
find_package( xvsdk REQUIRED )
set(xvsdk_INCLUDE ${xvsdk_INCLUDE_DIRS}/xvsdk})
include_directories( ${xvsdk_INCLUDE} )

find_package(OpenCV QUIET)
if( OpenCV_FOUND )
    include_directories( ${OpenCV_INCLUDE_DIRS} )
    add_definitions( -DUSE_OPENCV )
    set(SRCS ${SRCS} raw2opencv.cpp )
    link_directories( ${OpenCV_LIB_PATH} )
else()
    message("OpenCV not found, ${PROJECT_NAME} will not be able to display images")
endif()

ADD_EXECUTABLE( ${PROJECT_NAME} ${SRC} )
TARGET_LINK_LIBRARIES( ${PROJECT_NAME} xvsdk )

if( NOT WIN32 )
    TARGET_LINK_LIBRARIES( ${PROJECT_NAME} -pthread )

project(pipe_srv)
SET(SRC ../pipe_srv/pipe_srv.cpp ../pipe_srv/pipe_srv.h)
ADD_EXECUTABLE( ${PROJECT_NAME} ${SRC} )

endif()
