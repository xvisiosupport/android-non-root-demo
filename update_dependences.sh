#!/bin/sh

if [ $# != 1 ] ; then
    echo "usage:\n$0 <xvsdk_dir>"
    exit 1
else
    xvsdk=$1
fi

# Boost no longer needed with latest xvsdk
#if [ ! -d libxslam/src/main/cpp/third-party/boost/ ]; then
#    mkdir libxslam/src/main/cpp/third-party/boost/
#fi
#cp -ruv ${xvsdk}/third-lib/boost/boost libxslam/src/main/cpp/third-party/boost/


mkdir -p xvsdk/src/main/cpp/include
mkdir -p xvsdk/src/main/cpp/include/xslam-hid-sdk/
mkdir -p xvsdk/src/main/cpp/include/xslam-uvc-sdk/
mkdir -p xvsdk/src/main/cpp/include/xslam-vsc-sdk/
mkdir -p xvsdk/src/main/cpp/include/xslam-edge-sdk/
mkdir -p xvsdk/src/main/cpp/include/xslam-slam-sdk/
mkdir -p xvsdk/src/main/cpp/include/xslam-edge-plus-sdk/

cp -ruv ${xvsdk}/third-lib/libusb/libusb/libusb.h xvsdk/src/main/cpp/include/
cp -ruv ${xvsdk}/hid_sdk/include/*  xvsdk/src/main/cpp/include/xslam-hid-sdk/
cp -ruv ${xvsdk}/uvc_sdk/include/*  xvsdk/src/main/cpp/include/xslam-uvc-sdk/
cp -ruv ${xvsdk}/vsc_sdk/include/*  xvsdk/src/main/cpp/include/xslam-vsc-sdk/
cp -ruv ${xvsdk}/edge_sdk/include/*  xvsdk/src/main/cpp/include/xslam-edge-sdk/
cp -ruv ${xvsdk}/xslam_sdk/include/*  xvsdk/src/main/cpp/include/xslam-slam-sdk/
cp -ruv ${xvsdk}/edge_plus_sdk/include/*  xvsdk/src/main/cpp/include/xslam-edge-plus-sdk/

mkdir -p xvsdk/src/main/jniLibs/arm64-v8a
mkdir -p xvsdk/src/main/jniLibs/armeabi-v7a

find ${xvsdk}/android/libs/arm64-v8a/   -name "lib*" -not -name "libopencv*" -exec cp -uv {} xvsdk/src/main/jniLibs/arm64-v8a/ \;
find ${xvsdk}/android/libs/armeabi-v7a/ -name "lib*" -not -name "libopencv*" -exec cp -uv {} xvsdk/src/main/jniLibs/armeabi-v7a/ \;
