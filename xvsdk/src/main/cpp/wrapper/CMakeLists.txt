add_library(xslam_wrapper SHARED xslam_android.cpp yuv_to_rgb.cpp)

set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--as-needed")

SET(LIBS ${LIBS} xslam-xv-sdk)

target_link_libraries(xslam_wrapper log ${LIBS})

install(TARGETS xslam_wrapper DESTINATION lib)
