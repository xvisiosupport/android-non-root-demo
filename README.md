# XSlam Android non root Demo


android non root demo based on xvsdk_3.2.0-20220727_android_apk


## XSlam android project

XSlam android project look like:

```
android-non-root-demo/
├── app                                      -- android demo module
└── xvsdk/src/main                           -- xvsdk wrapper module
    ├── cpp
    │   ├── wrapper                          -- xvsdk JNI wrapper   
    │   └── xvsdk                            -- xvsdk download from https://www.xvisiotech.com
    └── java                                 -- xvsdk java wrapper 

```

## xvsdk jni wrapper module

xvsdk modules look like:
```
android-non-root-demo/xvsdk/src/main/
├── cpp
│   ├── wrapper                     -- xslam_wrapper.so
│   │    └── xslam_android.cpp      -- xvsdk JNI wrapper
│   │
│   └── xvsdk                       -- xvsdk
│       ├── include                 -- headers
│       ├── include2                -- headers
│       └── libs                    -- libs
└── java
    ├── DeviceWatcher.java          -- USB watcher
    ├── XVisioClass.java            -- load xslam_wrapper.so
    ├── XCamera.java                -- xvsdk java wrapper                  
    ├── ImuListener                 -- imu callback
    ├── PoseListener                -- slam callback
    ├── RgbListener                 -- color camera callback
    ├── SgbmListener                -- sgbm camera callback
    ├── StereoListener              -- fisheye camera callback 
    ├── TofListener                 -- tof camera callback
    └── TofIrListener               -- tof camera IR depth image callback

```

## Headers

Add headers from xvsdk project in xvsdk/src/main/cpp/xvsdk/

xvsdk header folder should look like:
```
xvsdk/src/main/cpp/xvsdk/
├── include
│   ├── opencv2
│   ├── libusb.h
│   ├── unity-wrapper.h
│   ├── xv-sdk.h
│   └── xv-types.h
│     
└── include2
    ├── xv-sdk-ex.h
    └── xv-sdk-private.h

```


## Libs

Add libs from xvsdk project into xvsdk/src/main/cpp/xvsdk/libs/

jniLibs folder should look like:
```
xvsdk/src/main/cpp/xvsdk/libs
├── arm64-v8a
│   ├── libapriltag.so
│   ├── libjpeg-turbo1500.so
│   ├── liboctomap.so
│   ├── libusb1.0.so
│   ├── libxslam_algo_sdk.so
│   ├── libxslam_core.so
│   ├── libxslam-edge-sdk.so
│   ├── libxslam_hand.so
│   ├── libxslam-hid-sdk.so
│   ├── libxslam_libange.so
│   ├── libxslam-usb-sdk.so
│   ├── libxslam-vsc-sdk.so
│   ├── libxslam-xv-sdk.so
│   └──libxvuvc.so
|
├── arm64-v7a
│   ├── libapriltag.so
│   ├── libjpeg-turbo1500.so
│   ├── liboctomap.so
│   ├── libusb1.0.so
│   ├── libxslam_algo_sdk.so
│   ├── libxslam_core.so
│   ├── libxslam-edge-sdk.so
│   ├── libxslam_hand.so
│   ├── libxslam-hid-sdk.so
│   ├── libxslam_libange.so
│   ├── libxslam-usb-sdk.so
│   ├── libxslam-vsc-sdk.so
│   ├── libxslam-xv-sdk.so
│   └──libxvuvc.so
|
└── x86_64
    ├── libjpeg-turbo1500.so
    ├── liboctomap.so
    ├── libusb1.0.so
    ├── libxslam_algo_sdk.so
    ├── libxslam_core.so
    ├── libxslam-edge-sdk.so
    ├── libxslam_hand.so
    ├── libxslam-hid-sdk.so
    ├── libxslam_libange.so
    ├── libxslam-usb-sdk.so
    ├── libxslam-vsc-sdk.so
    ├── libxslam-xv-sdk.so
    └── libxvuvc.so

```
