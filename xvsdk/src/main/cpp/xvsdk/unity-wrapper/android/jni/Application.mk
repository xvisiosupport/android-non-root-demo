APP_PLATFORM := android-25

#APP_ABI := all
APP_ABI := armeabi-v7a arm64-v8a
#APP_ABI := arm64-v8a
#APP_ABI := armeabi-v7a

#APP_STL in [ system stlport_static stlport_shared gnustl_static gnustl_shared c++_static c++_shared none ]
APP_STL := c++_shared
APP_CPPFLAGS := -fexceptions -frtti -std=c++11
APP_LDFLAGS := -llog
