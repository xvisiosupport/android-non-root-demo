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
#include <thread>
#include <chrono>
#include <math.h>
#include <xv-sdk.h>
#include <xv-sdk-ex.h>
#include <xv-sdk-private.h>
#include "unity-wrapper.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include "fps_count.hpp"
#include <opencv2/opencv.hpp>
#include <fcntl.h>
#include <sys/stat.h>

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


std::string SAVE_HOME = "";
const long long MAX_RECORD_TIME = 60*60; // seconds
static bool g_recording = false;
static long long g_start_time = 0L;
static int g_slam_cb = -1;
static int g_fisheye_cb = -1;
static int g_rgb1_cb = -1;
static int g_rgb2_cb = -1;
static int g_gesture_cb = -1;

static FpsCount g_slam_fc;
static FpsCount g_fisheye_fc;
static FpsCount g_rgb1_fc;
static FpsCount g_rgb2_fc;
static FpsCount g_gesture_fc;

static std::mutex g_pose_mtx;
static std::mutex g_fisheye_mtx;
static std::mutex g_rgb1_mtx;
static std::mutex g_rgb2_mtx;
static std::mutex g_gesture_mtx;

static xv::Pose g_pose;
static xv::FisheyeImages g_fisheye_img;
static xv::ColorImage g_rgb1_img;
static xv::ColorImage g_rgb2_img;
static std::shared_ptr<const xv::HandPose> g_hand_pose = nullptr;

static std::ofstream g_info_out_stream;
static std::ofstream g_slam_out_stream;
static std::ofstream g_gesture_out_stream;
static int g_fisheye_fd = -1;
static int g_rgb1_fd = -1;
static int g_rgb2_fd = -1;

std::vector<char> g_slam_buffer;
std::vector<char> g_gesture_buffer;

void onImuCallback(xv::Imu const &imu) {
    JNIEnv *jniEnv;
    jvm->AttachCurrentThread(&jniEnv, NULL);
    if (!jniEnv || !s_XCameraClass || !s_imuCallback) {
        return;
    }

    static FpsCount fc;
    static int cnt = 0;

    fc.tic();
    if (cnt++ % 500 == 0) {
        LOG_DEBUG("onImuStream fps = %d", int(fc.fps()));
    }

    jniEnv->CallStaticVoidMethod(s_XCameraClass, s_imuCallback, static_cast<double>(imu.accel[0]),
                                 static_cast<double>(imu.accel[1]),
                                 static_cast<double>(imu.accel[2]));
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

    if (s_poseCallback) {
        auto pitchYawRoll = xv::rotationToPitchYawRoll(pose.rotation());
        jniEnv->CallStaticVoidMethod(s_XCameraClass, s_poseCallback,
                                     static_cast<double>(pose.x()),
                                     static_cast<double>(pose.y()),
                                     static_cast<double>(pose.z()),
                                     static_cast<double>(pitchYawRoll[0]*180/M_PI),
                                     static_cast<double>(pitchYawRoll[1]*180/M_PI),
                                     static_cast<double>(pitchYawRoll[2]*180/M_PI));
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
    if (!jniEnv || !s_XCameraClass) {
        return;
    }

    std::shared_ptr<const xv::DepthImage> tmp = std::make_shared<xv::DepthImage>(im);;
    unsigned w = tmp->width;
    unsigned h = tmp->height;
    double distance = 4.5;
    LOG_DEBUG("onTofStream type: %d, width = %d,height = %d", im.type, w, h);

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
        }
    } else if (tmp->type == xv::DepthImage::Type::IR) {
        // float dmax = 2494.0; // maybe 7494,2494,1498,1249 see mode_manage.h in sony toflib
        auto tmp_d = reinterpret_cast<unsigned short const *>(tmp->data.get());
        unsigned short dmin = tmp_d[0];
        unsigned short dmax = tmp_d[0];
        for (unsigned int i = 0; i < tmp->height * tmp->width; i++) {
            unsigned short d = tmp_d[i];
            if (d > dmax) {
                dmax = d;
            }

            if (d < dmin) {
                dmin = d;
            }
        }

        double dFactor = 255.0 / (double) (dmax - dmin);
        // LOG_DEBUG("onTofCallback IR dmin=%d, dmax=%d, dFactor1=%f", dmin, dmax, dFactor);
        double gamma = 2.8;
        for (unsigned int i = 0; i < tmp->height * tmp->width; i++) {
            unsigned short d = tmp_d[i];
            // int dv = (int) ((d - dmin) * dFactor);
            int dv = (int) (255.0 * pow((1.0 * d) / (dmax - dmin), 1 / gamma));
            unsigned char pixel = (unsigned char) dv;
            rgbVectors[i] = RgbaStruct{pixel, pixel, pixel, 255};
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

bool setPmdTofIRFunction()
{
    bool  ret = false;
    std::vector<unsigned char> result(63);
    bool bOK = device->hidWriteAndRead({0x02,0x10,0xf5,0x02,0x01}, result);
    if(bOK)
    {
        std::cout << "Enable IR successfully" << std::endl;
        ret = true;
    }
    else
    {
        std::cout << "Enable IR failed" << std::endl;
        ret = false;
    }
    return ret;
}

void startTofStream() {
    LOG_DEBUG("startTofStream");
    if (!device || !device->tofCamera()) {
        return;
    }
    setPmdTofIRFunction();
    tofId = device->tofCamera()->registerCallback(onTofCallback);
    device->tofCamera()->setSonyTofSetting(xv::TofCamera::SonyTofLibMode::LABELIZE_SF,
                                           xv::TofCamera::Resolution::VGA,
                                           xv::TofCamera::Framerate::FPS_30);
    device->tofCamera()->start();
}


std::shared_ptr<xv::StereoRectificationMesh> xv_stereo_rectification_mesh1 = nullptr;
std::shared_ptr<xv::StereoRectificationMesh> xv_stereo_rectification_mesh2 = nullptr;

void dump_image(std::vector<xv::GrayScaleImage> images)
{
    for(int i=0; i<images.size(); i++)
    {
        cv::Mat mat;
        mat.create(images[i].height, images[i].width, CV_8UC1);
        std::memcpy(mat.data, images[i].data.get(), images[i].height*images[i].width);
        cv::imwrite("/sdcard/rectify/" + std::to_string(i) + ".png", mat);
    }
}

std::vector<xv::GrayScaleImage> rectiry(const xv::FisheyeImages& stereo_images) {
    const std::size_t image_width = stereo_images.images.at(0).width;
    const std::size_t image_height = stereo_images.images.at(0).height;
    if (xv_stereo_rectification_mesh1 == nullptr) {
        const std::vector<xv::Calibration> calibration = device->fisheyeCameras()->calibration();
        std::vector<xv::Calibration> calib1;
        calib1.push_back(calibration[0]);
        calib1.push_back(calibration[1]);
        xv_stereo_rectification_mesh1 = std::make_shared<xv::StereoRectificationMesh>(calib1, image_width, image_height);
        if (xv_stereo_rectification_mesh2 == nullptr && calibration.size() == 4) {
            std::vector<xv::Calibration> calib2;
            calib2.push_back(calibration[2]);
            calib2.push_back(calibration[3]);
            xv_stereo_rectification_mesh2 = std::make_shared<xv::StereoRectificationMesh>(calib2, image_width, image_height);
        }
    }

    std::vector<xv::GrayScaleImage> rectified_images;
    const std::pair<xv::GrayScaleImage, xv::GrayScaleImage> result1 = xv_stereo_rectification_mesh1->rectify(stereo_images.images.at(0), stereo_images.images.at(1));
    rectified_images.push_back(result1.first);
    rectified_images.push_back(result1.second);

    if (stereo_images.images.size() == 4 && xv_stereo_rectification_mesh2) {
        const std::pair<xv::GrayScaleImage, xv::GrayScaleImage> result2 = xv_stereo_rectification_mesh2->rectify(stereo_images.images.at(2), stereo_images.images.at(3));
        rectified_images.push_back(result2.first);
        rectified_images.push_back(result2.second);
    }

    dump_image(rectified_images);
    return rectified_images;
}

void onStrereoCallback(xv::FisheyeImages const &stereo) {
    JNIEnv *jniEnv;
    jvm->AttachCurrentThread(&jniEnv, NULL);
    if (!s_XCameraClass || !jniEnv) {
        return;
    }

    jvm->AttachCurrentThread(&jniEnv, NULL);
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
                auto v = d[i + j * w];
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

    stopSgbmStream();
    device->sgbmCamera()->setSgbmResolution(xv::SgbmCamera::Resolution::SGBM_640x480);
    sgmbId = device->sgbmCamera()->registerCallback(onSgbmCallback);
    device->sgbmCamera()->start(sgbm_config);
}


extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_nAddUsbDevice(JNIEnv
                                            *env,
                                            jclass type, jstring
                                            deviceName_,
                                            jint fileDescriptor
) {
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
    xv::setLogLevel(xv::LogLevel::info);
    if (!device) {
        LOG_DEBUG("nAddUsbDevice getDevice FAIL");
        return;
    }
    LOG_DEBUG("nAddUsbDevice inited opencv version:%s", cv::getVersionString().c_str());

    m_ready = true;
    device->imuSensor()->registerCallback([](xv::Imu const & imu) {

    });

    device->slam()->reset();
    device->slam()->registerCallback([](xv::Pose const & pose){
        std::lock_guard<std::mutex> lock(g_pose_mtx);
        g_pose = pose;
        g_slam_fc.tic();
    });

    device->fisheyeCameras()->start();
    device->fisheyeCameras()->registerCallback([](xv::FisheyeImages const & stereo){
        std::lock_guard<std::mutex> lock(g_fisheye_mtx);
        g_fisheye_img = stereo;
        g_fisheye_fc.tic();
    });

    device->colorCamera()->start();
    device->colorCamera()->registerCallback([](xv::ColorImage const & rgb){
        std::lock_guard<std::mutex> lock(g_rgb1_mtx);
        g_rgb1_img = rgb;
        g_rgb1_fc.tic();
    });

    device->colorCamera()->startCameras();
    device->colorCamera()->registerCam2Callback([](xv::ColorImage const & rgb){
        std::lock_guard<std::mutex> lock(g_rgb2_mtx);
        g_rgb2_img = rgb;
        g_rgb2_fc.tic();
    });

    device->gesture()->setPlatform(3, true);
    device->gesture()->setParams(9, true);
    device->gesture()->setfisheyeParams(true);
    device->gesture()->setfisheyeIndex(0,1);
    device->gesture()->start();
    device->gesture()->registerSlamKeypointsCallback([](std::shared_ptr<const xv::HandPose> keypoints) {
        std::lock_guard<std::mutex> lock(g_gesture_mtx);
        g_hand_pose = keypoints;
        g_gesture_fc.tic();
    });
}

extern "C" JNIEXPORT jint JNICALL
Java_org_xvisio_xvsdk_XCamera_getFps(JNIEnv *env, jclass type, jint stream) {
    double fps = 0;
    switch(stream) {
        case 1:
            fps = std::round(g_slam_fc.fps());
            break;

        case 2:
            fps = std::round(g_fisheye_fc.fps());
            break;

        case 3:
            fps = std::round(g_rgb1_fc.fps());
            break;

        case 4:
            fps = std::round(g_rgb2_fc.fps());
            break;

        case 5:
            fps = std::round(g_gesture_fc.fps());
            break;

        default:
            break;
    }
    return (jint)fps;
}

int getRecordTime()
{
    long long now = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    int time = now - g_start_time;
    return time < MAX_RECORD_TIME ? time : MAX_RECORD_TIME;
}

bool isRecording()
{
    return g_recording && getRecordTime() < MAX_RECORD_TIME;
}

extern "C" JNIEXPORT jint JNICALL
Java_org_xvisio_xvsdk_XCamera_getRecordTime(JNIEnv *env, jclass type) {
    if(!g_recording)
    {
        return 0;
    }

    return (jint)getRecordTime();
}

static void stopSaveData()
{
    if(g_slam_cb > -1)
    {
        device->slam()->unregisterCallback(g_slam_cb);
        g_slam_cb = -1;
    }
    g_slam_out_stream.flush();
    g_slam_out_stream.close();

    if(g_fisheye_cb > -1)
    {
        device->fisheyeCameras()->unregisterCallback(g_fisheye_cb);
        g_fisheye_cb = -1;
    }

    if(g_fisheye_fd > 0)
    {
        close(g_fisheye_fd);
        g_fisheye_fd = -1;
    }

    if(g_rgb1_cb > -1)
    {
        device->colorCamera()->unregisterCallback(g_rgb1_cb);
        g_rgb1_cb = -1;
    }

    if(g_rgb1_fd > 0)
    {
        close(g_rgb1_fd);
        g_rgb1_fd = -1;
    }

    if(g_rgb2_cb > -1)
    {
        device->colorCamera()->unregisterCam2Callback(g_rgb2_cb);
        g_rgb2_cb = -1;
    }

    if(g_rgb2_fd > 0)
    {
        close(g_rgb2_fd);
        g_rgb2_fd = -1;
    }

    if(g_gesture_cb > -1)
    {
        device->gesture()->unregisterSlamKeypointsCallback(g_gesture_cb);
        g_gesture_cb = -1;
    }
    g_gesture_out_stream.flush();
    g_gesture_out_stream.close();

    char buff[256] = {0};
    int seconds = getRecordTime();
    sprintf(buff, "Record time:%02d:%02d", seconds/60, seconds%60);
    g_info_out_stream << buff << std::endl;
    g_info_out_stream.flush();
    g_info_out_stream.close();

    g_start_time = 0;
}

static void print(std::ofstream &out, xv::CalibrationEx &c)
{
    out << "T:\n";
    out << c.pose.translation()[0] << "\t"
                      << c.pose.translation()[1] << "\t"
                      << c.pose.translation()[2] << "\t\n";

    out << "R:\n";
    for(int i=0;i <3; i++)
    {
        out << c.pose.rotation()[3*i+0] << "\t"
            << c.pose.rotation()[3*i+1] << "\t"
            << c.pose.rotation()[3*i+2] << "\t\n";
    }

    if(c.seucm.size() > 0)
    {
        out << "SEUCM:\n";
        out << c.seucm[0].w << "\t"
            << c.seucm[0].h << "\t"
            << c.seucm[0].fx << "\t"
            << c.seucm[0].fy << "\t"
            << c.seucm[0].u0 << "\t"
            << c.seucm[0].v0 << "\t"
            << c.seucm[0].eu << "\t"
            << c.seucm[0].ev << "\t"
            << c.seucm[0].alpha << "\t"
            << c.seucm[0].beta << "\n\n";
    }
}

static void print(std::ofstream &out, xv::Calibration &c)
{
    out << "T:\n";
    out << c.pose.translation()[0] << "\t"
        << c.pose.translation()[1] << "\t"
        << c.pose.translation()[2] << "\t\n";

    out << "R:\n";
    for(int i=0;i <3; i++)
    {
        out << c.pose.rotation()[3*i+0] << "\t"
            << c.pose.rotation()[3*i+1] << "\t"
            << c.pose.rotation()[3*i+2] << "\t\n";
    }

    if(c.pdcm.size() > 0)
    {
        out << "PDCM:\n";
        out << c.pdcm[0].w << "\t"
            << c.pdcm[0].h << "\t"
            << c.pdcm[0].fx << "\t"
            << c.pdcm[0].fy << "\t"
            << c.pdcm[0].u0 << "\t"
            << c.pdcm[0].v0 << "\t"
            << c.pdcm[0].distor[0] << "\t"
            << c.pdcm[0].distor[1] << "\t"
            << c.pdcm[0].distor[2] << "\t"
            << c.pdcm[0].distor[3] << "\t"
            << c.pdcm[0].distor[4] << "\n\n";
    }
}

static void startSaveData()
{
    std::string cmd = "rm -rf " + SAVE_HOME + "/*";
    system(cmd.c_str());

    g_start_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    std::time_t now = std::time(nullptr);
    std::tm utc_tm;
    gmtime_r(&now, &utc_tm);
    char date[128] = {0};
    sprintf(date, "%04d-%02d-%02d-%02d-%02d-%02d",
            utc_tm.tm_year+1900, utc_tm.tm_mon+1, utc_tm.tm_mday, utc_tm.tm_hour, utc_tm.tm_min, utc_tm.tm_sec);
    std::string SAVE_DIR = SAVE_HOME + "/" + (std::string)date;
    std::string SAVE_RGB1_DIR = SAVE_DIR + "/RGB_Left";
    std::string SAVE_RGB2_DIR = SAVE_DIR + "/RGB_Right";
    std::string SAVE_FISHEYE_DIR = SAVE_DIR + "/Fisheye";
    std::string SAVE_SLAM_DIR = SAVE_DIR + "/Slam";
    std::string SAVE_GESTURE_DIR = SAVE_DIR + "/Gesture";

    mkdir(SAVE_HOME.c_str(), 0777);
    mkdir(SAVE_DIR.c_str(), 0777);
    mkdir(SAVE_RGB1_DIR.c_str(), 0777);
    mkdir(SAVE_RGB2_DIR.c_str(), 0777);
    mkdir(SAVE_FISHEYE_DIR.c_str(), 0777);
    mkdir(SAVE_SLAM_DIR.c_str(), 0777);
    mkdir(SAVE_GESTURE_DIR.c_str(), 0777);

    g_info_out_stream = std::ofstream(SAVE_DIR + "/info.txt", std::ios::app);

    std::string fisheye_file = SAVE_FISHEYE_DIR + "/fisheye_640_480_30_4_gray.raw";
    g_fisheye_fd = open(fisheye_file.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_LARGEFILE, S_IRUSR | S_IWUSR);

    std::string rgb1_file = SAVE_RGB1_DIR + "/Rgb_left_1600_1200_30.mjpg";
    g_rgb1_fd = open(rgb1_file.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_LARGEFILE, S_IRUSR | S_IWUSR);

    std::string rgb2_file = SAVE_RGB2_DIR + "/Rgb_right_1600_1200_30.mjpg";
    g_rgb2_fd = open(rgb2_file.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_LARGEFILE, S_IRUSR | S_IWUSR);

    g_slam_buffer.resize(128*1024);
    g_slam_out_stream.rdbuf()->pubsetbuf(g_slam_buffer.data(), g_slam_buffer.size());
    g_slam_out_stream.open(SAVE_SLAM_DIR + "/slam.txt", std::ios::app);

    g_gesture_buffer.resize(128*1024);
    g_gesture_out_stream.rdbuf()->pubsetbuf(g_gesture_buffer.data(), g_gesture_buffer.size());
    g_gesture_out_stream.open(SAVE_GESTURE_DIR + "/gesture.txt",std::ios::app);

    if(g_info_out_stream)
    {
        std::string uuid = "";
        if(device->info().find("uuid") != device->info().end())
        {
            uuid = device->info()["uuid"];
        }
        g_info_out_stream << "sn:" << uuid << "\n\n";
        g_info_out_stream << "fisheye:\n";
        auto feCalibs = std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->calibrationEx();
        for(int i=0; i<feCalibs.size(); i++)
        {
            print(g_info_out_stream, feCalibs[i]);
        }

        g_info_out_stream << "rgb1:\n";
        auto rgb1Calib = device->colorCamera()->calibration();
        for(int i=0; i<rgb1Calib.size(); i++)
        {
            print(g_info_out_stream, rgb1Calib[i]);
        }

        g_info_out_stream << "rgb2:\n";
        auto rgb2Calib = device->colorCamera()->calibration2();
        for(int i=0; i<rgb1Calib.size(); i++)
        {
            print(g_info_out_stream, rgb1Calib[i]);
        }
        g_info_out_stream.flush();
    }

    if(g_slam_out_stream)
    {
        std::string header = "confidence,timestamp,x,y,z,q0,q1,q2,q3\n";
        g_slam_out_stream << header;
    }

    if(g_gesture_out_stream)
    {
        std::string header = "timestamp";
        for(int i=0; i<52; i++)
        {
            if(i==26)
            {
                header += ",timestamp";
            }
            header += ",x,y,z,q0,q1,q2,q3";
        }
        g_gesture_out_stream << header << "\n";
    }

    g_slam_cb = device->slam()->registerCallback([](xv::Pose const & pose){
        static int count = 0;
        if(isRecording() && g_slam_out_stream)
        {
            count++;
            auto q = xv::rotationToQuaternion(pose.rotation());
            char buf[256] = {0};
            sprintf(buf, "%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                    pose.confidence(), pose.hostTimestamp(), pose.x(), pose.y(), pose.z(),
                    q[0], q[1], q[2], q[3]);
            g_slam_out_stream << buf;

            if(count > 2000)
            {
                g_slam_out_stream.flush();
                count = 0;
            }
        }
    });

    g_fisheye_cb = device->fisheyeCameras()->registerCallback([](xv::FisheyeImages const & stereo){
        if(isRecording() && g_fisheye_fd > 0) {
            for (int i = 0; i < stereo.images.size(); i++) {
                write(g_fisheye_fd, reinterpret_cast<const char *>(stereo.images[i].data.get()), stereo.images[0].width * stereo.images[0].height);
            }
        }
    });

    g_rgb1_cb = device->colorCamera()->registerCallback([](xv::ColorImage const & rgb){
        if(isRecording() && g_rgb1_fd > 0) {
            write(g_rgb1_fd, reinterpret_cast<const char *>(rgb.data.get()), rgb.dataSize);
        }
    });

    g_rgb2_cb = device->colorCamera()->registerCam2Callback([](xv::ColorImage const & rgb){
        if(isRecording() && g_rgb2_fd) {
            write(g_rgb2_fd, reinterpret_cast<const char *>(rgb.data.get()), rgb.dataSize);
        }
    });

    g_gesture_cb = device->gesture()->registerSlamKeypointsCallback([](std::shared_ptr<const xv::HandPose> keypoints) {
        static int count = 0;
        if(isRecording() && g_gesture_out_stream)
        {
            count++;
            std::string line;
            char buf[256] = {0};
            sprintf(buf, "%.3f", keypoints->timestamp[0]);
            line += buf;
            for(int i=0; i<keypoints->pose.size(); i++)
            {
                if(i==26)
                {
                    memset(buf, 0x0, sizeof(buf));
                    sprintf(buf, ",%.3f", keypoints->timestamp[1]);
                    line += buf;
                }

                char tmp[256] = {0};
                auto q = xv::rotationToQuaternion(keypoints->pose[i].rotation());
                sprintf(tmp, ",%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
                        keypoints->pose[i].x(), keypoints->pose[i].y(), keypoints->pose[i].z(),
                        q[0], q[1], q[2], q[3]);
                line += tmp;
            }
            g_gesture_out_stream << line << "\n";

            if(count > 50)
            {
                count = 0;
                g_gesture_out_stream.flush();
            }
        }
    });
}

extern "C" JNIEXPORT jboolean JNICALL
Java_org_xvisio_xvsdk_XCamera_isReady(JNIEnv *env, jclass type) {
    return m_ready && device != nullptr;
}

extern "C" JNIEXPORT jstring JNICALL
Java_org_xvisio_xvsdk_XCamera_getPose(JNIEnv *env, jclass type) {
    xv::Pose pose;
    {
        std::lock_guard<std::mutex> lock(g_pose_mtx);
        pose = g_pose;
    }

    auto q = xv::rotationToQuaternion(pose.rotation());
    char buf[256] = {0};
    sprintf(buf, "confidence:%.1f, T(%.3f, %.3f, %.3f), Q(%.3f, %.3f, %.3f, %.3f)",
            pose.confidence(), pose.x(), pose.y(), pose.z(), q[0], q[1], q[2], q[3]);

    return env->NewStringUTF(buf);
}

extern "C" JNIEXPORT jstring JNICALL
Java_org_xvisio_xvsdk_XCamera_getGesture(JNIEnv *env, jclass type) {
    std::shared_ptr<const xv::HandPose> handPose;
    {
        std::lock_guard<std::mutex> lock(g_gesture_mtx);
        handPose = g_hand_pose;
    }

    if(handPose == nullptr)
    {
        return env->NewStringUTF("");
    }

    char buf[256] = {0};
    sprintf(buf, "left(%.3f, %.3f, %.3f), right(%.3f, %.3f, %.3f)",
            handPose->pose[0].x(), handPose->pose[0].y(), handPose->pose[0].z(),
            handPose->pose[26].x(), handPose->pose[26].y(), handPose->pose[26].z());

    return env->NewStringUTF(buf);
}

extern "C" JNIEXPORT jint JNICALL
Java_org_xvisio_xvsdk_XCamera_getFisheyeImage(JNIEnv *env, jclass type, jobject buffer) {

    std::uint8_t *out_ptr = static_cast<unsigned char *>(env->GetDirectBufferAddress(buffer));
    xv::FisheyeImages stereo;
    {
        std::lock_guard<std::mutex> lock(g_fisheye_mtx);
        stereo = g_fisheye_img;
    }

    if(out_ptr == nullptr || stereo.images.empty())
    {
        return 0;
    }

    std::vector<cv::Mat> mats(stereo.images.size());
    for(size_t k = 0 ; k < stereo.images.size() ; ++k)
    {
        cv::Mat mat;
        mat.create(stereo.images[k].height,stereo.images[k].width,CV_8UC1);
        std::memcpy(mat.data, stereo.images[k].data.get(), stereo.images[k].height*stereo.images[k].width);
        mats[k] = mat;
    }

    cv::Mat gray;
    cv::Mat mat_h1;
    cv::Mat mat_h2;
    cv::hconcat(mats[0], mats[1], mat_h1);
    cv::hconcat(mats[2], mats[3], mat_h2);
    cv::vconcat(mat_h1, mat_h2, gray);

    cv::Size scale_size(640, 480);
    cv::Mat scale_img;
    cv::resize(gray, scale_img, scale_size, 0.5, 0.5, cv::INTER_AREA);
    memcpy(out_ptr, scale_img.data, 640*480);
    return 640*480;
}

int getRgbBuffer(std::uint8_t *buffer, xv::ColorImage &rgb)
{
    std::vector<uchar> buf(const_cast<unsigned char*>(rgb.data.get()), const_cast<unsigned char*>(rgb.data.get()) + rgb.dataSize);
    cv::Mat image = cv::imdecode(buf, cv::IMREAD_COLOR);

    cv::Size scale_size(640, 480);
    cv::Mat scale_img;
    cv::resize(image, scale_img, scale_size, 0.4, 0.4, cv::INTER_AREA);
//    std::vector<uint8_t> output;
//    std::vector<int> params;
//    params.push_back(cv::IMWRITE_JPEG_QUALITY);
//    params.push_back(90);
//    cv::imencode(".jpg", scale_img, output, params);
//    memcpy(buffer, output.data(), output.size());
//    return (int)output.size();
    cv::Mat result;
    cv::cvtColor(scale_img, result, cv::COLOR_BGR2RGBA);
    memcpy(buffer, result.data, 640*480*4);
    return 640*480*4;
}

extern "C" JNIEXPORT jint JNICALL
Java_org_xvisio_xvsdk_XCamera_getRgb1Image(JNIEnv *env, jclass type, jobject buffer) {

    std::uint8_t *out_ptr = static_cast<unsigned char *>(env->GetDirectBufferAddress(buffer));
    xv::ColorImage rgb;
    {
        std::lock_guard<std::mutex> lock(g_rgb1_mtx);
        rgb = g_rgb1_img;
    }

    if(out_ptr == nullptr || rgb.data == nullptr || rgb.dataSize == 0)
    {
        return 0;
    }

    return getRgbBuffer(out_ptr, rgb);
}

extern "C" JNIEXPORT jint JNICALL
Java_org_xvisio_xvsdk_XCamera_getRgb2Image(JNIEnv *env, jclass type, jobject buffer) {

    std::uint8_t *out_ptr = static_cast<unsigned char *>(env->GetDirectBufferAddress(buffer));
    xv::ColorImage rgb;
    {
        std::lock_guard<std::mutex> lock(g_rgb2_mtx);
        rgb = g_rgb2_img;
    }

    if(out_ptr == nullptr || rgb.data == nullptr || rgb.dataSize == 0)
    {
        return 0;
    }

    return getRgbBuffer(out_ptr, rgb);
}

extern "C" JNIEXPORT jboolean JNICALL
Java_org_xvisio_xvsdk_XCamera_nSaveData(JNIEnv *env, jclass type,
                                           jstring path, jboolean status) {
    if (!device) {
        return false;
    }

    if (path != nullptr)
    {
        const char* chars = env->GetStringUTFChars(path, nullptr);
        if (chars != nullptr)
        {
            std::string result(chars);
            env->ReleaseStringUTFChars(path, chars);
            if (!result.empty())
            {
                SAVE_HOME = result + "/xv_save";
            }
        }
    }

    if(SAVE_HOME.empty())
    {
        LOG_DEBUG("get sdcard FAIL!");
        return false;
    }

    LOG_DEBUG("switch to nSaveData %d", status);
    if(g_recording == status)
    {
        return g_recording;
    }

    g_recording = status;
    if(g_recording)
    {
        startSaveData();
    }
    else
    {
        stopSaveData();
    }
    return g_recording;
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
Java_org_xvisio_xvsdk_XCamera_nRemoveUsbDevice(JNIEnv *env, jclass type,
                                               jint fileDescriptor) {

}

extern "C" JNIEXPORT void JNICALL
Java_org_xvisio_xvsdk_XCamera_initCallbacks(JNIEnv *env, jclass type) {
    s_XCameraClass = reinterpret_cast<jclass>(env->NewGlobalRef(type));
    s_imuCallback = env->GetStaticMethodID(s_XCameraClass, "imuCallback", "(DDD)V");
    s_tofCallback = env->GetStaticMethodID(s_XCameraClass, "tofCallback", "(II[I)V");
    s_tofIrCallback = env->GetStaticMethodID(s_XCameraClass, "tofIrCallback", "(II[I)V");
    s_stereoCallback = env->GetStaticMethodID(s_XCameraClass, "stereoCallback", "(II[I)V");
    s_sgbmCallback = env->GetStaticMethodID(s_XCameraClass, "sgbmCallback", "(II[I)V");
    s_rgbCallback = env->GetStaticMethodID(s_XCameraClass, "rgbCallback", "(II[I)V");
    s_rgbFpsCallback = env->GetStaticMethodID(s_XCameraClass, "rgbFpsCallback", "(I)V");
    s_poseCallback = env->GetStaticMethodID(s_XCameraClass, "poseCallback", "(DDDDDD)V");
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

