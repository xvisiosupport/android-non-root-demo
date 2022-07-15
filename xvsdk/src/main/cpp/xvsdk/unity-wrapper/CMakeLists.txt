cmake_minimum_required(VERSION 3.5.1)

project( xslam-unity-wrapper VERSION 1.2.8 )
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set (CMAKE_CXX_STANDARD 11)

add_definitions( "-DDLL_EXPORTS" )

find_package(OpenCV)

if( OpenCV_FOUND )
    include_directories(${OpenCV_INCLUDE_DIRS})
    link_directories(${OpenCV_LINK_DIRECTORIES})
    SET(PUBLIC_LIBS ${PUBLIC_LIBS} ${OpenCV_LIBS})


    find_package(libusb-1.0 REQUIRED)
    include_directories(${LIBUSB_1_INCLUDE_DIRS})
    link_directories(${LIBUSB_1_LIBDIR})
    SET(PUBLIC_LIBS ${PUBLIC_LIBS} ${LIBUSB_1_LIBRARIES})

    include_directories( ../include/ )
    SET(PUBLIC_LIBS ${PUBLIC_LIBS} xvsdk )

    add_library( ${PROJECT_NAME} SHARED unity-wrapper.cpp )

    target_link_libraries( ${PROJECT_NAME} ${PUBLIC_LIBS} )

    if( NOT WIN32 )
        TARGET_LINK_LIBRARIES( ${PROJECT_NAME} -pthread )
    endif()

    INSTALL(TARGETS ${PROJECT_NAME}
            DESTINATION lib
        COMPONENT libraries
        RUNTIME DESTINATION bin
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
        COMPONENT runtime)

    include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
    add_subdirectory( bin )

else()
    message("OpenCV not found - Skip ${PROJECT_NAME}")
endif()
