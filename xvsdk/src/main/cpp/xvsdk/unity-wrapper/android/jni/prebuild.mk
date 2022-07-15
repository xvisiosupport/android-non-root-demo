LOCAL_PATH := $(call my-dir)

$(info "Load prebuild")

include $(CLEAR_VARS)
LOCAL_MODULE := xslam_algo_sdk
LOCAL_SRC_FILES := $(xv_dir)/libs/$(TARGET_ARCH_ABI)/libxslam_algo_sdk.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := xslam_core
LOCAL_SRC_FILES := $(xv_dir)/libs/$(TARGET_ARCH_ABI)/libxslam_core.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := xslam_libange
LOCAL_SRC_FILES := $(xv_dir)/libs/$(TARGET_ARCH_ABI)/libxslam_libange.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := xslam_surfacereconstruction
LOCAL_SRC_FILES := $(xv_dir)/libs/$(TARGET_ARCH_ABI)/libxslam_surfacereconstruction.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := octomap
LOCAL_SRC_FILES := $(xv_dir)/libs/$(TARGET_ARCH_ABI)/liboctomap.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_core
LOCAL_SRC_FILES := $(xv_dir)/libs/$(TARGET_ARCH_ABI)/libopencv_core.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_imgcodecs
LOCAL_SRC_FILES := $(xv_dir)/libs/$(TARGET_ARCH_ABI)/libopencv_imgcodecs.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv_imgproc
LOCAL_SRC_FILES := $(xv_dir)/libs/$(TARGET_ARCH_ABI)/libopencv_imgproc.so
include $(PREBUILT_SHARED_LIBRARY)

