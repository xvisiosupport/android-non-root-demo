LOCAL_PATH := $(call my-dir)

$(info "Load xvsdk")

include $(CLEAR_VARS)
LOCAL_MODULE := usb1.0
LOCAL_SRC_FILES := $(xv_dir)/libs/$(TARGET_ARCH_ABI)/libusb1.0.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := xslam-hid-sdk
LOCAL_SRC_FILES := $(xv_dir)/libs/$(TARGET_ARCH_ABI)/libxslam-hid-sdk.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := xslam-uvc-sdk
LOCAL_SRC_FILES := $(xv_dir)/libs/$(TARGET_ARCH_ABI)/libxslam-uvc-sdk.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := xslam-vsc-sdk
LOCAL_SRC_FILES := $(xv_dir)/libs/$(TARGET_ARCH_ABI)/libxslam-vsc-sdk.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := xslam-edge-sdk
LOCAL_SRC_FILES := $(xv_dir)/libs/$(TARGET_ARCH_ABI)/libxslam-edge-sdk.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := xslam-xv-sdk
LOCAL_SRC_FILES := $(xv_dir)/libs/$(TARGET_ARCH_ABI)/libxslam-xv-sdk.so
include $(PREBUILT_SHARED_LIBRARY)

