include $(CLEAR_VARS)
LOCAL_SRC_FILES := ../../examples/$(lastword $(subst /, , $(EXAMPLE))).cpp
LOCAL_C_INCLUDES :=	$(LOCAL_PATH)/../../include $(LOCAL_PATH)/../../include2 $(LOCAL_PATH)/../../examples/
LOCAL_MODULE := $(lastword $(subst /, , $(EXAMPLE)))
LOCAL_CPPFLAGS := -O3 -fPIC -std=c++11 -DNDEBUG
LOCAL_CFLAGS := -O3 -fPIC -std=c++11 -DNDEBUG
LOCAL_SHARED_LIBRARIES := xslam-xv-sdk
include $(BUILD_EXECUTABLE)
