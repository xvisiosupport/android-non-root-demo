include $(CLEAR_VARS)

LOCAL_MODULE := demo-api
LOCAL_SRC_FILES := ../../examples/demo-api/demo-api.cpp
LOCAL_C_INCLUDES :=  ../../include ../../include2
LOCAL_CPPFLAGS := -O3 -fPIC -std=c++11 -DNDEBUG

ifeq ($(TARGET_ARCH_ABI),armeabi-v7a)
LOCAL_CFLAGS := -O3 -fPIC -DNDEBUG
endif

LOCAL_SHARED_LIBRARIES := xslam-usb-sdk xslam-hid-sdk xslam-uvc-sdk xslam-vsc-sdk xslam-edge-sdk xvuvc usb1.0 jpeg xslam-slam_core xslam-slam_lib-ange xslam-slam_surface-reconstruction xslam_algo_sdk octomap xslam-xv-sdk MNN handskeleton sony_iu456 apriltag xslam_hand et_sdk smoothAbout xvsdk_wrapper et_TrackerSDK opencv_imgcodecs 

include $(BUILD_EXECUTABLE)


include $(CLEAR_VARS)

LOCAL_MODULE := pipe_srv
LOCAL_SRC_FILES := ../../examples/demo-api/pipe_srv.cpp
LOCAL_C_INCLUDES := ../../examples/demo-api/
LOCAL_CPPFLAGS := -O3 -fPIC -std=c++11 -DNDEBUG

ifeq ($(TARGET_ARCH_ABI),armeabi-v7a)
LOCAL_CFLAGS := -O3 -fPIC -DNDEBUG
endif

include $(BUILD_EXECUTABLE)
