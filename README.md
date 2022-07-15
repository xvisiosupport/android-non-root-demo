# XSlam Android non root Demo

## Libs

Add libs from xvsdk project into xvsdk/src/main/jniLibs

jniLibs folder should look like:
```
xvsdk/src/main/jniLibs/
├── arm64-v8a
│   ├── dpfp
│   ├── dpfp_threaded
│   ├── fxload
│   ├── hotplugtest
│   ├── libc++_shared.so
│   ├── libusb1.0.so
│   ├── libxslam-vsc-sdk.so
│   ├── listdevs
│   ├── sam3u_benchmark
│   ├── stress
│   └── xusb
└── armeabi-v7a
    ├── dpfp
    ├── dpfp_threaded
    ├── fxload
    ├── hotplugtest
    ├── libc++_shared.so
    ├── libusb1.0.so
    ├── libxslam-vsc-sdk.so
    ├── listdevs
    ├── sam3u_benchmark
    ├── stress
    └── xusb
```

## Headers

Add headers from xvsdk project in xvsdk/src/main/cpp/third-party/*/include/ folders

third-party folder should look like:
```
xvsdk/src/main/cpp/third-party/
├── hid_sdk
│   └── include
│       ├── xslam-hid.hpp
│       ├── xslam-hid-types.hpp
│       └── xslam-version.h
├── libusb
│   └── include
│       └── libusb.h
├── uvc_sdk
│   └── include
│       ├── xslam-uvc.hpp
│       └── xslam-uvc-types.hpp
└── vsc_sdk
    └── include
        ├── xslam-vsc.hpp
        └── xslam-vsc-types.hpp
```
