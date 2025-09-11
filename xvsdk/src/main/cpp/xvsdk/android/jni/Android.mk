LOCAL_PATH := $(call my-dir)

# --------------------------------------------------------------------

include $(CLEAR_VARS)
LOCAL_MODULE := usb1.0
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libusb1.0.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := xvuvc
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libxvuvc.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := jpeg
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libjpeg-turbo1500.so
include $(PREBUILT_SHARED_LIBRARY)


include $(CLEAR_VARS)
LOCAL_MODULE := octomap
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/liboctomap.so
include $(PREBUILT_SHARED_LIBRARY)


include $(CLEAR_VARS)
LOCAL_MODULE := xslam-usb-sdk
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libxslam-usb-sdk.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := xslam-hid-sdk
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libxslam-hid-sdk.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := xslam-uvc-sdk
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libxslam-uvc-sdk.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := xslam-vsc-sdk
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libxslam-vsc-sdk.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := xslam-edge-sdk
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libxslam-edge-sdk.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := xvslam
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libxvslam.so
include $(PREBUILT_SHARED_LIBRARY)


# include $(CLEAR_VARS)
# LOCAL_MODULE := xslam-slam_surface-reconstruction
# LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libxslam_surfacereconstruction.so
# include $(PREBUILT_SHARED_LIBRARY)

# include $(CLEAR_VARS)
# LOCAL_MODULE := opencv_core_so
# LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libopencv_core.so
# include $(PREBUILT_SHARED_LIBRARY)

# include $(CLEAR_VARS)
# LOCAL_MODULE := opencv_imgproc_so
# LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libopencv_imgproc.so
# include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := sony_iu456
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libsony_iu456.so
include $(PREBUILT_SHARED_LIBRARY)


include $(CLEAR_VARS)
LOCAL_MODULE := xslam-xv-sdk
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libxslam-xv-sdk.so
include $(PREBUILT_SHARED_LIBRARY)
include $(CLEAR_VARS)

LOCAL_MODULE := et_sdk
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/lib_et_SDK.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := algInterface
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libAlgInterface.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := GVR
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libGVRAPI.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := smoothAbout
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libSmoothAbout.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := CERT
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libCertPlatform.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := SPLITX
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libsqliteX.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := xvsdk_wrapper
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libxvsdk_wrapper.so
include $(PREBUILT_SHARED_LIBRARY)


include $(CLEAR_VARS)

LOCAL_MODULE := et_TrackerSDK
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/lib_et_TrackerSDK.so
include $(PREBUILT_SHARED_LIBRARY)
include $(CLEAR_VARS)

include $(CLEAR_VARS)
LOCAL_MODULE := externalStream
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libsensordriver.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := XVISP
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libxvisp.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := gaze_eyetracker
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libeyetracking.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := gaze_utility
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libeyetracking_utility.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := IrisCore
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libeye_iris_core.so
include $(PREBUILT_SHARED_LIBRARY)

# include $(CLEAR_VARS)
# LOCAL_MODULE := IrisMNN
# LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libMNN.so
# include $(PREBUILT_SHARED_LIBRARY)

# include $(CLEAR_VARS)
# LOCAL_MODULE := IrisMNNExpress
# LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libMNN_Express.so
# include $(PREBUILT_SHARED_LIBRARY)

# include $(CLEAR_VARS)
# LOCAL_MODULE := IrisMNNVulkan
# LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libMNN_Vulkan.so
# include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := IrisBSP
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libpirisbsp.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := IrisCPP
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libc++_shared.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := zxing
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libZXing.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := wirelessController
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libwirelessController.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := Vueta
LOCAL_SRC_FILES := ../../libs/$(TARGET_ARCH_ABI)/libvueta-and.so
include $(PREBUILT_SHARED_LIBRARY)

# --------------------------------------------------------------------

include $(CLEAR_VARS)

examples = \
	slam_edge_example \
	slam_example \
	tof_example \

$(foreach EXAMPLE, $(examples),\
  $(eval include $(LOCAL_PATH)/example.mk) \
)

exampledirs = \
	slam_3dof_display_calib \
	all_stream \

$(foreach EXAMPLE, $(exampledirs),\
  $(eval include $(LOCAL_PATH)/exampledir.mk) \
)

include $(LOCAL_PATH)/demo.mk

