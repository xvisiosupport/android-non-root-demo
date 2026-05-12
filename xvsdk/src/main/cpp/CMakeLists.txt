cmake_minimum_required(VERSION 3.10.2)

project(XSlamWrapper VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -lc++_shared")

# Configure CMake
#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include_directories( ${CMAKE_SOURCE_DIR}/include )
include_directories( ${CMAKE_SOURCE_DIR}/xvsdk/include )
include_directories( ${CMAKE_SOURCE_DIR}/xvsdk/include2 )

link_directories( ${CMAKE_SOURCE_DIR}/xvsdk/libs/${ANDROID_ABI} )

add_subdirectory( wrapper )


