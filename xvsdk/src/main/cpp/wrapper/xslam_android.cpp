#include <jni.h>
#include <memory>
#include <vector>
#include <time.h>
#include <string>
#include <regex>
#include <sstream>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <mutex>
#include <cmath>

#include <android/log.h>
//#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>
#include <math.h>
#include <xv-sdk.h>
//#include "callVpu.h"
//#include "callVpuDual.h"
#include "unity-wrapper.h"
//#include "xslam_android.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include "fps_count.hpp"
#include "../xvsdk/include2/xv-sdk-ex.h"

#define LOG_TAG "xslam#wrapper"
#define LOG_DEBUG(...)                                                \
    do                                                                \
    {                                                                 \
        __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__); \
    } while (false)

#define LOG_ERROR(...)                                                \
    do                                                                \
    {                                                                 \
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__); \
    } while (false)

#define PI acos(-1)

enum Resolution {
    RGB_1920x1080 = 0,  ///< RGB 1080p
    RGB_1280x720 = 1,  ///< RGB 720p
    RGB_640x480 = 2,  ///< RGB 480p
    RGB_320x240 = 3,  ///< RGB QVGA (not supported now)
    RGB_2560x1920 = 4,  ///< RGB 5m (not supported now)
    RGB_3840x2160 = 5,
};

struct __attribute__((pack)) RgbaStruct {
    unsigned char R, G, B, A;
};

extern std::vector<std::vector<unsigned char>> rgb_colors;

static std::shared_ptr<xv::Device> device;
static int slamId = -1;
static int rgbId = -1;
static int tofId = -1;
static int stereoId = -1;
static int imuId = -1;
static int sgmbId = -1;

class androidout : public std::streambuf {
public:
    enum {
        bufsize = 128
    }; // ... or some other suitable buffer size
    androidout() { this->setp(buffer, buffer + bufsize - 1); }

private:
    int overflow(int c) {
        if (c == traits_type::eof()) {
            *this->pptr() = traits_type::to_char_type(c);
            this->sbumpc();
        }
        return this->sync() ? traits_type::eof() : traits_type::not_eof(c);
    }

    int sync() {
        int rc = 0;
        if (this->pbase() != this->pptr()) {
            char writebuf[bufsize + 1];
            memcpy(writebuf, this->pbase(), this->pptr() - this->pbase());
            writebuf[this->pptr() - this->pbase()] = '\0';

            rc = __android_log_write(ANDROID_LOG_INFO, "std", writebuf) > 0;
            this->setp(buffer, buffer + bufsize - 1);
        }
        return rc;
    }

    char buffer[bufsize];
};

class androiderr : public std::streambuf {
public:
    enum {
        bufsize = 128
    }; // ... or some other suitable buffer size
    androiderr() { this->setp(buffer, buffer + bufsize - 1); }

private:
    int overflow(int c) {
        if (c == traits_type::eof()) {
            *this->pptr() = traits_type::to_char_type(c);
            this->sbumpc();
        }
        return this->sync() ? traits_type::eof() : traits_type::not_eof(c);
    }

    int sync() {
        int rc = 0;
        if (this->pbase() != this->pptr()) {
            char writebuf[bufsize + 1];
            memcpy(writebuf, this->pbase(), this->pptr() - this->pbase());
            writebuf[this->pptr() - this->pbase()] = '\0';

            rc = __android_log_write(ANDROID_LOG_ERROR, "std", writebuf) > 0;
            this->setp(buffer, buffer + bufsize - 1);
        }
        return rc;
    }

    char buffer[bufsize];
};

void yuv2rgb(unsigned char *yuyv_image, int *rgb_image, int width, int height);

static bool m_ready = false;

static JavaVM *jvm = 0;
static jclass s_XCameraClass = nullptr;

static jmethodID s_imuCallback = nullptr;

static jmethodID s_tofCallback = nullptr;
static jmethodID s_tofIrCallback = nullptr;

static jmethodID s_stereoCallback = nullptr;
static jmethodID s_sgbmCallback = nullptr;

static jmethodID s_rgbCallback = nullptr;
static jmethodID s_rgbFpsCallback = nullptr;

static jmethodID s_poseCallback = nullptr;
static jmethodID s_poseCallbackEx = nullptr;

static const xv::sgbm_config sgbm_config
        {
                1,
                1.f,
                0,
                1,
                0,
//  .11285f,
                0.08f,
                96.f,
                255,
                {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0},
                1,
                2.2,
                0,
                0,//1.standard 2.lrcheck 3.extended 4.subpixel
                8000,
                100,
        };

void onImuCallback(xv::Imu const &imu) {
    JNIEnv *jniEnv;
    jvm->AttachCurrentThread(&jniEnv, NULL);
    if (!s_XCameraClass || !jniEnv) {
        return;
    }

    static FpsCount fc;
    static int cnt = 0;

    fc.tic();
    if (cnt++ % 500 == 0) {
        LOG_DEBUG("onImuStream fps = %d", int(fc.fps()));
    }

    if (s_imuCallback == nullptr) {
        s_imuCallback = jniEnv->GetStaticMethodID(s_XCameraClass, "imuCallback", "(DDD)V");
    }

    if (s_imuCallback) {
        jniEnv->CallStaticVoidMethod(s_XCameraClass, s_imuCallback,
                                     static_cast<double>(imu.accel[0]),
                                     static_cast<double>(imu.accel[1]),
                                     static_cast<double>(imu.accel[2]));
    }
}

void stopImuStream() {
    if (!device || !device->imuSensor()) {
        return;
    }

    if (imuId != -1) {
        device->imuSensor()->unregisterCallback(imuId);
    }
    device->imuSensor()->stop();
}

void startImuStream() {
    if (!device || !device->imuSensor()) {
        return;
    }

    imuId = device->imuSensor()->registerCallback(onImuCallback);
    device->imuSensor()->start();
}

void onSlamCallback(xv::Pose const &pose) {
    JNIEnv *jniEnv;
    jvm->AttachCurrentThread(&jniEnv, NULL);
    if (!s_XCameraClass || !jniEnv) {
        return;
    }

    if (s_poseCallback == nullptr) {
        s_poseCallback = jniEnv->GetStaticMethodID(s_XCameraClass, "poseCallback", "(DDDDDD)V");
    }

    if (s_poseCallback) {
        auto pitchYawRoll = xv::rotationToQuaternion(pose.rotation());
        jniEnv->CallStaticVoidMethod(s_XCameraClass, s_poseCallback,
                                     static_cast<double>(pose.x()),
                                     static_cast<double>(pose.y()),
                                     static_cast<double>(pose.z()),
                                     static_cast<double>(pitchYawRoll[0]),
                                     static_cast<double>(pitchYawRoll[1]),
                                     static_cast<double>(pitchYawRoll[2]));
    }
}

void stopSlamStream() {
    if (!device || !device->slam()) {
        return;
    }

    if (slamId != -1) {
        device->slam()->unregisterCallback(slamId);
    }
    device->slam()->stop();
}

void startSlamStream() {
    if (!device || !device->slam()) {
        return;
    }

    slamId = device->slam()->registerCallback(onSlamCallback);
    device->slam()->start(xv::Slam::Mode::Mixed);
}

void onRgbCallback(xv::ColorImage const &rgb) {
    JNIEnv *jniEnv;
    jvm->AttachCurrentThread(&jniEnv, NULL);
    if (!s_XCameraClass || !jniEnv) {
        return;
    }

    if (s_rgbCallback == nullptr) {
        s_rgbCallback = jniEnv->GetStaticMethodID(s_XCameraClass, "rgbCallback", "(II[I)V");
    }

    if (s_rgbCallback) {

        int w = rgb.width;
        int h = rgb.height;
        int s = w * h;

        auto d = rgb.data.get();

        jintArray data = jniEnv->NewIntArray(s);
        jint *body = jniEnv->GetIntArrayElements(data, 0);

        yuv2rgb((unsigned char *) d, body, w, h);

        jniEnv->CallStaticVoidMethod(s_XCameraClass, s_rgbCallback, w, h, data);
        jniEnv->DeleteLocalRef(data);
    }

    if (s_rgbFpsCallback == nullptr) {
        s_rgbFpsCallback = jniEnv->GetStaticMethodID(s_XCameraClass, "rgbFpsCallback", "(I)V");
    }

    if (s_rgbFpsCallback) {
        static FpsCount fc;
        static int cnt = 0;
        fc.tic();

        if (cnt++ % 100 == 0) {
            int fps = std::round(fc.fps());
            jniEnv->CallStaticVoidMethod(s_XCameraClass, s_rgbFpsCallback, fps);
        }
    }
}

void stopRgbStream() {
    if (!device || !device->colorCamera()) {
        return;
    }

    if (rgbId != -1) {
        device->colorCamera()->unregisterCallback(rgbId);
    }
    device->colorCamera()->stop();
}

void startRgbStream() {
    LOG_DEBUG("startRgbStream");
    if (!device || !device->colorCamera()) {
        return;
    }

    stopRgbStream();
    rgbId = device->colorCamera()->registerCallback(onRgbCallback);
    device->colorCamera()->start();
}

void onTofCallback(xv::DepthImage const &im) {
    JNIEnv *jniEnv;
    jvm->AttachCurrentThread(&jniEnv, NULL);
    if (!s_XCameraClass || !jniEnv) {
        return;
    }

    int w = im.width;
    int h = im.height;
    LOG_DEBUG("onTofStream type: %d, width = %d,height = %d", im.type, w, h);
    std::shared_ptr<const xv::DepthImage> tmp = std::make_shared<xv::DepthImage>(im);;
    unsigned srcWidth = tmp->width;
    unsigned srcHeight = tmp->height;
    double distance = 4.5;

    if (s_tofCallback == nullptr) {
        s_tofCallback = jniEnv->GetStaticMethodID(s_XCameraClass, "tofCallback", "(II[I)V");
    }

    if (s_tofIrCallback == nullptr) {
        s_tofIrCallback = jniEnv->GetStaticMethodID(s_XCameraClass, "tofIrCallback", "(II[I)V");
    }

    int s = w * h;
    auto d = const_cast<unsigned char *>(tmp->data.get());

    std::vector<RgbaStruct> rgbVectors;
    rgbVectors.resize(s);

    if (tmp->type == xv::DepthImage::Type::Depth_16) {
        float dmax = 2494.0; // maybe 7494,2494,1498,1249 see mode_manage.h in sony toflib
        const auto tmp_d = reinterpret_cast<int16_t const *>(tmp->data.get());
        for (unsigned int i = 0; i < tmp->height * tmp->width; i++) {
            unsigned short d = tmp_d[i];
            auto max = std::min(255.0f, d * 255.0f / dmax);
            unsigned int u = static_cast<unsigned int>( std::max(0.0f, max));
            const auto &cc = rgb_colors.at(u);
            rgbVectors[i] = RgbaStruct{cc.at(0), cc.at(1), cc.at(2), 255};
            // unsigned char avg = (cc.at(0) + cc.at(1) + cc.at(2)) / 3;
            // rgbVectors[i] = RgbaStruct{avg, avg, avg, 255};
        }
    } else if (tmp->type == xv::DepthImage::Type::IR) {
        // out = cv::Mat::zeros(tof->height, tof->width, CV_8UC3);
        float dmax = 2494.0; // maybe 7494,2494,1498,1249 see mode_manage.h in sony toflib
        auto tmp_d = reinterpret_cast<unsigned short const *>(tmp->data.get());
        for (unsigned int i = 0; i < tmp->height * tmp->width; i++) {
            unsigned short d = tmp_d[i];
            auto max = std::min(255.0f, d * 255.0f / dmax);
            unsigned int u = static_cast<unsigned int>( std::max(0.0f, max));
            //if( u < 15 )
            //    u = 0;
            const auto &cc = rgb_colors.at(u);
            unsigned char avg = (cc.at(0) + cc.at(1) + cc.at(2)) / 3;
            // avg = avg > 100 ? 255 : 0;
            rgbVectors[i] = RgbaStruct{cc.at(0), cc.at(1), cc.at(2), 255};
            // rgbVectors[i] = RgbaStruct{avg, avg, avg, 255};
            // out.at<cv::Vec3b>( i/tof->width, i%tof->width ) = cv::Vec3b(cc.at(2), cc.at(1),cc.at(0) );
        }
    } else if (tmp->type == xv::DepthImage::Type::Depth_32) {
        float dmax = 7.5;
        const auto tmp_d = reinterpret_cast<float const *>(tmp->data.get());
        for (unsigned int i = 0; i < s; i++) {
            const auto &d = tmp_d[i];
            if (d < 0.01 || d > 9.9) {
                rgbVectors[i] = RgbaStruct{0, 0, 0, 255};
            } else {
                auto max = std::min(255.0f, d * 255.0f / dmax);
                unsigned int u = static_cast<unsigned int>(std::max(0.0f, max));
                const auto &cc = rgb_colors.at(u);
                rgbVectors[i] = RgbaStruct{cc.at(0), cc.at(1), cc.at(2), 255};
            }
        }
    }

    jintArray data = jniEnv->NewIntArray(s);
    jint *body = jniEnv->GetIntArrayElements(data, 0);
    memcpy(body, rgbVectors.data(), s * sizeof(RgbaStruct));

// cv::Mat mrgb(srcHeight, srcWidth, CV_8UC4, body);
// cv::cvtColor(adjMap, mrgb, cv::COLOR_BGR2RGBA);

    if (tmp->type == xv::DepthImage::Type::IR && s_tofIrCallback) {
        jniEnv->CallStaticVoidMethod(s_XCameraClass, s_tofIrCallback, w, h, data);
    } else if (tmp->type == xv::DepthImage::Type::Depth_16 && s_tofCallback) {
        jniEnv->CallStaticVoidMethod(s_XCameraClass, s_tofCallback, w, h, data);
    } else if (tmp->type == xv::DepthImage::Type::Depth_32 && s_tofCallback) {
        jniEnv->CallStaticVoidMethod(s_XCameraClass, s_tofCallback, w, h, data);
    }

    jniEnv->DeleteLocalRef(data);
}

void stopTofStream() {
    if (!device || !device->tofCamera()) {
        return;
    }

    if (tofId != -1) {
        device->tofCamera()->unregisterCallback(tofId);
    }
    device->tofCamera()->stop();
}

void startTofStream() {
    LOG_DEBUG("startTofStream");
    if (!device || !device->tofCamera()) {
        return;
    }

    tofId = device->tofCamera()->registerCallback(onTofCallback);
    device->tofCamera()->setSonyTofSetting(xv::TofCamera::SonyTofLibMode::LABELIZE_SF,
                                           xv::TofCamera::Resolution::VGA,
                                           xv::TofCamera::Framerate::FPS_30);
    device->tofCamera()->start();
}

void onStrereoCallback(xv::FisheyeImages const &stereo) {
    JNIEnv *jniEnv;
    jvm->AttachCurrentThread(&jniEnv, NULL);
    if (!s_XCameraClass || !jniEnv) {
        return;
    }

    jvm->AttachCurrentThread(&jniEnv, NULL);

    if (s_stereoCallback == nullptr) {
        s_stereoCallback = jniEnv->GetStaticMethodID(s_XCameraClass, "stereoCallback",
                                                     "(II[I)V");
    }

    if (s_stereoCallback) {
        int w = stereo.images[0].width;
        int h = stereo.images[0].height;
        int s = w * h;

        auto d = stereo.images[0].data.get();

        if (stereo.images.empty() || !stereo.images[0].data || d == nullptr) {
            LOG_DEBUG("onStrereoCallback no fisheyes avaiable");
            return;
        }

        LOG_DEBUG("onStrereoCallback w = %d, h=%d", w, h);
        jintArray data = jniEnv->NewIntArray(s);
        jint *body = jniEnv->GetIntArrayElements(data, 0);
        for (int i = 0; i < w; i++) {
            for (int j = 0; j < h; j++) {
                auto v = d[(w - i - 1) + (h - j - 1) * w];
                body[i + j * w] =
                        0xFF000000 + (v << 16 & 0xFF0000) + (v << 8 & 0xFF00) + (v & 0xFF);
            }
        }
        jniEnv->CallStaticVoidMethod(s_XCameraClass, s_stereoCallback, w, h, data);
        jniEnv->DeleteLocalRef(data);
    }
}

void stopStereoStream() {
    if (!device || !device->fisheyeCameras()) {
        return;
    }

    if (stereoId != -1) {
        device->fisheyeCameras()->unregisterCallback(stereoId);
    }
    device->fisheyeCameras()->stop();
}

void startStereoStream() {
    LOG_DEBUG("startStereoStream");
    if (!device || !device->fisheyeCameras()) {
        return;
    }

    stopStereoStream();
    stereoId = device->fisheyeCameras()->registerCallback(onStrereoCallback);
    device->fisheyeCameras()->start();
}

void onSgbmCallback(xv::SgbmImage const &sgbm_image) {
    JNIEnv *jniEnv;
    jvm->AttachCurrentThread(&jniEnv, NULL);
    if (!s_XCameraClass || !jniEnv) {
        return;
    }

    jvm->AttachCurrentThread(&jniEnv, NULL);

    if (s_sgbmCallback == nullptr) {
        s_sgbmCallback = jniEnv->GetStaticMethodID(s_XCameraClass, "sgbmCallback",
                                                   "(II[I)V");
    }

    if (s_sgbmCallback) {
        if (sgbm_image.type == xv::SgbmImage::Type::Depth) {
            int w = sgbm_image.width;
            int h = sgbm_image.height;
            int s = w * h;

            LOG_DEBUG("onSgbmCallback w = %d, h=%d", w, h);
            std::vector<RgbaStruct> rgbVectors;
            rgbVectors.resize(s);

            double focal_length =
                    sgbm_image.width / (2.f * tan(sgbm_config.fov / 2 / 180.f * M_PI));
            double max_distance_m = (focal_length * sgbm_config.baseline / 1);
            double min_distance_m = sgbm_config.min_distance / 1000.0;
            max_distance_m = std::min(max_distance_m, sgbm_config.max_distance / 1000.0);

            float dmax = max_distance_m;
            const auto tmp_d = reinterpret_cast<int16_t const *>(sgbm_image.data.get());
            for (unsigned int i = 0; i < h * w; i++) {
                unsigned short d = tmp_d[i];
                auto max = std::min(255.0f, d * 255.0f / dmax);
                unsigned int u = static_cast<unsigned int>( std::max(0.0f, max));
                const auto &cc = rgb_colors.at(u);
                rgbVectors[i] = RgbaStruct{cc.at(0), cc.at(1), cc.at(2), 255};

/*                double distance_mm = tmp_d[i];
                double distance_m = distance_mm / 1000.;
                double d = std::max(min_distance_m, std::min(distance_mm, max_distance_m));
                d = (d - min_distance_m) / (min_distance_m - min_distance_m);
                if (distance_mm <= min_distance_m || distance_m > max_distance_m)
                {
                    rgbVectors[i] = RgbaStruct{0,0,0,255};
                }
                else
                {
                    char b = static_cast<char>(255.0 * std::min(std::max(0.0, 1.5 - std::abs(1.0 - 4.0 * (d - 0.5))), 1.0));
                    char g = static_cast<char>(255.0 * std::min(std::max(0.0, 1.5 - std::abs(1.0 - 4.0 * (d - 0.25))), 1.0));
                    char r = static_cast<char>(255.0 * std::min(std::max(0.0, 1.5 - std::abs(1.0 - 4.0 * d)), 1.0));
                    rgbVectors[i] = RgbaStruct{r,g,b,255};
                }*/
            }

            jintArray data = jniEnv->NewIntArray(s);
            jint *body = jniEnv->GetIntArrayElements(data, 0);
            memcpy(body, rgbVectors.data(), s * sizeof(RgbaStruct));

            jniEnv->CallStaticVoidMethod(s_XCameraClass, s_sgbmCallback, w, h, data);
            jniEnv->DeleteLocalRef(data);
        } else if (sgbm_image.type == xv::SgbmImage::Type::PointCloud) {

        }
    }
}

void stopSgbmStream() {
    if (!device || !device->sgbmCamera()) {
        return;
    }

    if (sgmbId != -1) {
        device->sgbmCamera()->unregisterCallback(sgmbId);
    }
    device->sgbmCamera()->stop();
}

void startSgbmStream() {
    LOG_DEBUG("startSgbmStream");
    if (!device || !device->sgbmCamera()) {
        return;
    }

    stopStereoStream();
    device->sgbmCamera()->setSgbmResolution(xv::SgbmCamera::Resolution::SGBM_640x480);
    sgmbId = device->sgbmCamera()->registerCallback(onSgbmCallback);
    device->sgbmCamera()->start(sgbm_config);
}


extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_DeviceWatcher_nAddUsbDevice(JNIEnv
                                                  *env,
                                                  jclass type, jstring
                                                  deviceName_,
                                                  jint fileDescriptor
) {
// const char *deviceName = env->GetStringUTFChars(deviceName_, 0);
// LOG_DEBUG("AddUsbDevice, adding device: %s, descriptor: %d", std::string(deviceName).c_str(), fileDescriptor );

    static bool firstCall = true;
    if (firstCall) {
        firstCall = false;
        std::cout.rdbuf(new androidout);
        std::cerr.rdbuf(new androiderr);
        env->GetJavaVM(&jvm);
        std::cout << "Initialized" << std::endl;
    }

    int fd = fileDescriptor;
    LOG_DEBUG("nAddUsbDevice fd: %d", fd);

    device = xv::getDevice(fd);
    xv::setLogLevel(xv::LogLevel(0));
    if (!device) {
        LOG_DEBUG("nAddUsbDevice getDevice FAIL");
        return;
    }

    m_ready = true;
    usleep(2000 * 1000);
    LOG_DEBUG("nAddUsbDevice inited");
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_nSetSlamMode(JNIEnv *env, jclass type,
                                           jint mode) {
    if (!device || !device->slam()) {
        return;
    }

    LOG_DEBUG("switch to mode %d", mode);
    device->slam()->stop();
    if (mode == 0) {
        device->slam()->start(xv::Slam::Mode::Mixed);
    } else if (mode == 1) {
        device->slam()->start(xv::Slam::Mode::Edge);
    } else if (mode == 2) {
        device->slam()->start(xv::Slam::Mode::EdgeFusionOnHost);
    }
}


extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_nSetRgbSolution(JNIEnv *env, jclass type, jint mode) {
    if (!device || !device->colorCamera()) {
        return;
    }

    LOG_DEBUG("nSetRgbSolution %d", mode);
    switch (mode) {
        case RGB_1920x1080:
            device->colorCamera()->setResolution(xv::ColorCamera::Resolution::RGB_1920x1080);
            break;
        case RGB_1280x720:
            device->colorCamera()->setResolution(xv::ColorCamera::Resolution::RGB_1280x720);
            break;
        case RGB_640x480:
            device->colorCamera()->setResolution(xv::ColorCamera::Resolution::RGB_640x480);
            break;
        case RGB_320x240:
            device->colorCamera()->setResolution(xv::ColorCamera::Resolution::RGB_320x240);
            break;
        case RGB_2560x1920:
            device->colorCamera()->setResolution(xv::ColorCamera::Resolution::RGB_2560x1920);
            break;
        default:
            break;
    }
    LOG_DEBUG("nSetRgbSolution end");
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_DeviceWatcher_nRemoveUsbDevice(JNIEnv *env, jclass type,
                                                     jint fileDescriptor) {

}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_initCallbacks(JNIEnv *env, jclass type) {
    s_XCameraClass = reinterpret_cast<jclass>(env->NewGlobalRef(type));
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_startSlamStream(JNIEnv *env, jclass type) {
    startSlamStream();
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_stopSlamStream(JNIEnv *env, jclass type) {
    stopSlamStream();
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_startImuStream(JNIEnv *env, jclass type) {
    startImuStream();
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_stopImuStream(JNIEnv *env, jclass type) {
    stopImuStream();
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_startRgbStream(JNIEnv *env, jclass type) {
    startRgbStream();
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_stopRgbStream(JNIEnv *env, jclass type) {
    stopRgbStream();
}


extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_startTofStream(JNIEnv *env, jclass type) {
    startTofStream();
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_stopTofStream(JNIEnv *env, jclass type) {
    stopTofStream();
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_startStereoStream(JNIEnv *env, jclass type) {
    startStereoStream();
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_stopStereoStream(JNIEnv *env, jclass type) {
    stopStereoStream();
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_startSgbmStream(JNIEnv *env, jclass type) {
    startSgbmStream();
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_stopSgbmStream(JNIEnv *env, jclass type) {
    stopSgbmStream();
}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_stopCallbacks(JNIEnv *env, jclass type) {
    stopSlamStream();
    stopImuStream();
    stopRgbStream();
    stopTofStream();
    stopStereoStream();
    stopSgbmStream();
}

