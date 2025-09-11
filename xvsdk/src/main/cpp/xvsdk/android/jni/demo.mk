include $(CLEAR_VARS)

LOCAL_MODULE := demo-api
LOCAL_SRC_FILES := ../../examples/demo-api/demo-api.cpp
LOCAL_C_INCLUDES :=  $(LOCAL_PATH)/../../include $(LOCAL_PATH)/../../include2
LOCAL_SHARED_LIBRARIES := $(ROOT_REL)/prebuild/prebuild/gaze/android/an/$(TARGET_ARCH_ABI)/libvueta-and.so
LOCAL_CPPFLAGS := -O3 -fPIC -std=c++11 -DNDEBUG

ifeq ($(TARGET_ARCH_ABI),armeabi-v7a)
LOCAL_CFLAGS := -O3 -fPIC -DNDEBUG
endif

LOCAL_SHARED_LIBRARIES := xslam-xv-sdk

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
