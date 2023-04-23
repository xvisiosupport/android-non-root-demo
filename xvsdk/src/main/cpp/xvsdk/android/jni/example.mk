include $(CLEAR_VARS)
LOCAL_SRC_FILES := ../../examples/$(lastword $(subst /, , $(EXAMPLE))).cpp
LOCAL_C_INCLUDES :=	$(LOCAL_PATH)/../../include $(LOCAL_PATH)/../../include2 $(LOCAL_PATH)/../../examples/
LOCAL_MODULE := $(lastword $(subst /, , $(EXAMPLE)))
LOCAL_CPPFLAGS := -O3 -fPIC -std=c++11 -DNDEBUG
LOCAL_CFLAGS := -O3 -fPIC -std=c++11 -DNDEBUG
LOCAL_SHARED_LIBRARIES := xslam-usb-sdk xslam-hid-sdk xslam-uvc-sdk xslam-vsc-sdk xslam-edge-sdk xvuvc usb1.0 jpeg xslam-slam_core xslam-slam_lib-ange xslam_algo_sdk octomap xslam-xv-sdk opencv_core_so opencv_imgproc_so sony_iu456 apriltag xslam_hand et_sdk smoothAbout xvsdk_wrapper et_TrackerSDK opencv_imgcodecs xvsdk_handtracking lib_sensorDriver handskeleton_all handskeleton_all_model TNN SNPE CERES shared_xvhand
include $(BUILD_EXECUTABLE)
