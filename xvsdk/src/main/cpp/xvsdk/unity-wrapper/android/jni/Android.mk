LOCAL_PATH := $(call my-dir)

src_dir := ../..
xv_dir := $(src_dir)/..

include prebuild.mk
include xvsdk.mk

include $(CLEAR_VARS)
$(warning $(LOCAL_PATH))

include $(CLEAR_VARS)
$(info LOCAL_PATH:=$(LOCAL_PATH))

LOCAL_SRC_FILES := \
	$(src_dir)/unity-wrapper.cpp

LOCAL_C_INCLUDES := \
    bionic \
    framework/libs/base/include \

LOCAL_C_INCLUDES += $(xv_dir)/include


LOCAL_MODULE := xslam-unity-wrapper
LOCAL_SHARED_LIBRARIES := xslam-hid-sdk xslam-uvc-sdk xslam-vsc-sdk xslam-edge-sdk usb1.0 \
						  xslam-xv-sdk \
						  octomap \
						  xslam_algo_sdk \
						  xslam_core \
						  xslam_libange \
						  xslam_surfacereconstruction


LOCAL_CPPFLAGS := -O3 -fPIC -std=c++11 -DNDEBUG
LOCAL_CFLAGS := -O3 -fPIC -DNDEBUG
LOCAL_CFLAGS += -std=gnu99
LOCAL_LDLIBS := -lz
#LOCAL_LDFLAGS := -lm # -lpthread

# OpenCV
LOCAL_SHARED_LIBRARIES += opencv_core opencv_imgcodecs opencv_imgproc

# OpenGL ES
# LOCAL_LDLIBS += -lGLESv2
# LOCAL_CPPFLAGS += -DSUPPORT_OPENGL_ES=1

# FFMPEG
#LOCAL_C_INCLUDES += $(LOCAL_PATH)/../../third-party/ffmpeg/android/include/$(TARGET_ARCH_ABI)
#LOCAL_SHARED_LIBRARIES += avcodec avdevice avfilter avformat avutil swresample swscale
#LOCAL_CPPFLAGS += -DUSE_FFMPEG


$(info LOCAL_CPPFLAGS:=$(LOCAL_CPPFLAGS))

include $(BUILD_SHARED_LIBRARY)

