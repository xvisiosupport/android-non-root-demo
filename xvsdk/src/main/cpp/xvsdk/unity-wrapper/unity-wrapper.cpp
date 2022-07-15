#include "unity-wrapper.h"

#ifdef WIN32
#define NOMINMAX
#include <Windows.h>
#endif

#ifdef __linux__
#include <sys/types.h>
#include <unistd.h>
#endif

#include <iostream>
#include <future>
#include <cstring>
#include <map>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <thread>
#include <queue>

#include <opencv2/opencv.hpp>

#include <libusb.h>
#define XSLAM_VENDOR 0x040e
#define XSLAM_PRODUCT 0xf408

#include <xv-sdk.h>
static std::shared_ptr<xv::Device> device;

static std::mutex s_poseMutex;
static std::shared_ptr<xv::Pose> s_slamPose = nullptr;

static std::mutex s_tofPlaneMutex;
static int s_tofPlaneId = -1;
static std::shared_ptr<const std::vector<xv::Plane>> s_tofPlane = nullptr;

static std::mutex s_stereoPlaneMutex;
static int s_stereoPlaneId = -1;
static std::shared_ptr<const std::vector<xv::Plane>> s_stereoPlane = nullptr;

static std::mutex s_initMutex;
static int s_components = UnityWrapper::COM_ALL;
static bool s_ready = false;
static UnityWrapper::SlamType s_type = UnityWrapper::SlamType::Edge;

#include <memory>
#include <string>
#include <stdexcept>
template<typename ... Args>
std::string sformat( const std::string& format, Args ... args )
{
    size_t size = snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    std::unique_ptr<char[]> buf( new char[ size ] );
    snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

#ifdef __ANDROID__
#include <android/log.h>

class androidout : public std::streambuf {
public:
    enum { bufsize = 128 }; // ... or some other suitable buffer size
    androidout() { this->setp(buffer, buffer + bufsize - 1); }

private:
    int overflow(int c)
    {
        if (c == traits_type::eof()) {
            *this->pptr() = traits_type::to_char_type(c);
            this->sbumpc();
        }
        return this->sync()? traits_type::eof(): traits_type::not_eof(c);
    }

    int sync()
    {
        int rc = 0;
        if (this->pbase() != this->pptr()) {
            char writebuf[bufsize+1];
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
    enum { bufsize = 128 }; // ... or some other suitable buffer size
    androiderr() { this->setp(buffer, buffer + bufsize - 1); }

private:
    int overflow(int c)
    {
        if (c == traits_type::eof()) {
            *this->pptr() = traits_type::to_char_type(c);
            this->sbumpc();
        }
        return this->sync()? traits_type::eof(): traits_type::not_eof(c);
    }

    int sync()
    {
        int rc = 0;
        if (this->pbase() != this->pptr()) {
            char writebuf[bufsize+1];
            memcpy(writebuf, this->pbase(), this->pptr() - this->pbase());
            writebuf[this->pptr() - this->pbase()] = '\0';

            rc = __android_log_write(ANDROID_LOG_ERROR, "std", writebuf) > 0;
            this->setp(buffer, buffer + bufsize - 1);
        }
        return rc;
    }

    char buffer[bufsize];
};
#endif

std::string printable( const unsigned char * data, unsigned int size, bool join = true )
{
    std::stringstream ss;
    ss << std::hex;
    if( join ){
        ss << "0x";
        for(unsigned int i=0;i<size;i++){
            ss << std::setfill('0') << std::setw(2) << int(data[i]);
        }
    }else{
        for(unsigned int i=0;i<size;i++){
            ss << "0x" << std::setfill('0') << std::setw(2) << int(data[i]);
            if( i < size-1 ){
                ss << ",";
            }
        }
    }
    ss << std::dec;
    return ss.str();
}

/** \cond */
struct Position {
    double x;
    double y;
    double z;

    Vector3 toVector() const {
        return {static_cast<float>(x),static_cast<float>(y),static_cast<float>(z)};
    }
};

//struct Orientation {
//    double pitch;
//    double yaw;
//    double roll;
//
//    Vector3 toVector() const {
//        return {static_cast<float>(pitch),static_cast<float>(yaw),static_cast<float>(roll)};
//    }
//};
/** \endcond */

namespace UnityWrapper {

/**
 * \enum SlamType
 * SLAM source
 */

/**
 * \var SlamType::Edge
 * On device SLAM
 *
 * \var SlamType::Mixed
 * On host SLAM
 */

/** \cond */
    void finalInit(int fd, int components)
    {
#ifdef __linux__
        std::cout << "PID: " << getpid() << std::endl;
#endif

    if (fd == -1) {
        auto devices = xv::getDevices(3.);
        if (devices.empty()) {
            return;
        }
        device = devices.begin()->second;
    } else {
        device = xv::getDevice(fd);
    }

    if (device->slam()) {
        device->slam()->registerTofPlanesCallback([](std::shared_ptr<const std::vector<xv::Plane>> plane){
            s_tofPlaneMutex.lock();
            s_tofPlane = plane;
            s_tofPlaneMutex.unlock();
        });

        device->slam()->registerStereoPlanesCallback([](std::shared_ptr<const std::vector<xv::Plane>> plane){
            s_stereoPlaneMutex.lock();
            s_stereoPlane = plane;
            s_stereoPlaneMutex.unlock();
        });

        device->slam()->registerCallback([](const xv::Pose& pose){
            s_poseMutex.lock();
            s_slamPose = std::make_shared<xv::Pose>(pose);
            s_poseMutex.unlock();
        });

         xv::Slam::Mode slamMode = xv::Slam::Mode::Mixed;
         if (s_type == UnityWrapper::SlamType::Edge)
             slamMode = xv::Slam::Mode::Edge;
        device->slam()->start(slamMode);
    }

    s_ready = true;

    std::cout << "xslam_init done" << std::endl;
}

/** \endcond */

/**
 * Init Slam, this will open the device using VIP and PID
 * @param components Select components to enable
 */
bool xslam_init_components(int components)
{

    std::lock_guard<std::mutex> lock(s_initMutex);

    if (s_ready)
        return true;

    //freopen("output.txt", "w", stdout);
    //freopen("error.txt", "w", stderr);

#ifdef ANDROID
    static bool once = false;
    if( !once ){
        once = true;
        std::cout.rdbuf(new androidout);
        std::cerr.rdbuf(new androiderr);
    }
#endif

    std::cout << std::hex << "components: 0x" << components << std::dec << std::endl;

    try {
        finalInit( -1, components );

    } catch ( std::exception &e ) {
        std::cerr << e.what() << std::endl;
#ifdef ANDROID
        __android_log_print(ANDROID_LOG_WARN,    "XVisio", "Failed to init (%s)", e.what());
#endif
        return false;
    }

    return true;
}

/**
 * Init Slam, this will open the device using given file descriptor
 * @param fd
 * @param components Select components to enable
 */
bool xslam_init_components_with_fd(int fd, int components)
{
    std::lock_guard<std::mutex> lock(s_initMutex);

    if (s_ready)
        return true;

#ifdef ANDROID
    static bool once = false;
    if( !once ){
        once = true;
        std::cout.rdbuf(new androidout);
        std::cerr.rdbuf(new androiderr);
    }
#endif

    std::cout << std::hex << "fd: " << fd << std::dec << std::endl;
    std::cout << std::hex << "components: 0x" << components << std::dec << std::endl;

    try{
        finalInit( fd, components );

    } catch ( std::exception &e ) {
        std::cerr << e.what() << std::endl;
#ifdef ANDROID
        __android_log_print(ANDROID_LOG_WARN,    "XVisio", "Failed to init with fd=%d (%s)", fd, e.what());
#endif
        return false;
    }

    return true;
}

/**
 * Init Slam, this will open the device using VIP and PID. This will open all channel and streams
 * @param components Select components to enable
 */
bool xslam_init()
{
    return xslam_init_components( COM_ALL );
}

/**
 * Init Slam, this will open the device using given file descriptor. This will open all channel and streams
 * @param fd
 */
bool xslam_init_with_fd( int fd )
{
   return xslam_init_components_with_fd( fd, COM_ALL );
}

/**
 * Uninit Slam, this will close the device
 */
bool xslam_uninit( )
{
    s_ready = false;

    try{
        if (device) {
            if (device->slam())
                device->slam()->stop();
            device.reset();
        }
    } catch ( std::exception &e ) {
        std::cerr << e.what() << std::endl;
#ifdef ANDROID
        __android_log_print(ANDROID_LOG_WARN,    "XVisio", "Failed to uninit: %s", e.what());
#endif
        return false;
    }
    return true;
}

/**
 * Set SLAM type
 * \see SlamType
 * @param type SLAM source
 */
void xslam_slam_type(SlamType type)
{
    if (s_type != type) {
        s_type = type;
    }
    if (device && device->slam()) {
        xv::Slam::Mode slamMode = xv::Slam::Mode::Mixed;
        if (s_type == UnityWrapper::SlamType::Edge)
            slamMode = xv::Slam::Mode::Edge;
        device->slam()->start(slamMode);
    }
}

/**
 * Get 6 DOF from slam source, right-hand coordinate
 * @param position in meter
 * @param orientation Euler angle {pitch, yaw, roll}
 * @param timestamp in µs
 * @return
 */
bool xslam_get_6dof(Vector3 *position, Vector3 *orientation, long long *timestamp)
{
    std::shared_ptr<xv::Pose> pose;
    s_poseMutex.lock();
    pose = s_slamPose;
    s_poseMutex.unlock();
    if( pose == nullptr ){
        return false;
    }

    if( timestamp ){
        *timestamp = pose->edgeTimestampUs();
    }

    if( position ){
        position->x = pose->x();
        position->y = pose->y();
        position->z = pose->z();
    }

    if( orientation ){
        auto pitchYawRoll = xv::rotationToPitchYawRoll(pose->rotation());
        orientation->x = pitchYawRoll[0];
        orientation->y = pitchYawRoll[1];
        orientation->z = pitchYawRoll[2];
    }

    return true;
}


bool xslam_get_transform_matrix(float *matrix, long long *timestamp, int *status)
{
    if( matrix == nullptr ){
        std::cerr << "Matrix reference is null" << std::endl;
        return false;
    }

    std::shared_ptr<xv::Pose> pose;
    s_poseMutex.lock();
    pose = s_slamPose;
    s_poseMutex.unlock();
    if( !pose ){
        std::cout << "No pose for type " << s_type << std::endl;
#ifdef ANDROID
        __android_log_print(ANDROID_LOG_INFO,    "XVisio", "No pose for type %d", s_type);
#endif
        return false;
    }

    if( timestamp ){
        *timestamp = pose->edgeTimestampUs();
    }

    if( status ){
        *status = 0;
    }

    // Set to identity
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            matrix[i*4+j] = (i==j)?1:0;
        }
    }

    // [  0  4  8 12
    //    1  5  9 13
    //    2  6 10 14
    //    3  7 11 15 ]

    // Change coordinate from right hand to left hand

    // Position
    matrix[12] = pose->x();
    matrix[13] = -pose->y();
    matrix[14] = pose->z();

    // Rotation
#if USE_TR
    matrix[0]  = pose->rotation()[0];
    matrix[1]  = pose->rotation()[1];
    matrix[2]  = pose->rotation()[2];

    matrix[4]  = pose->rotation()[3];
    matrix[5]  = pose->rotation()[4];
    matrix[6]  = pose->rotation()[5];

    matrix[8]  = pose->rotation()[6];
    matrix[9]  = pose->rotation()[7];
    matrix[10] = pose->rotation()[8];
#else
    matrix[0]  = pose->rotation()[0];
    matrix[1]  = pose->rotation()[3];
    matrix[2]  = pose->rotation()[6];

    matrix[4]  = pose->rotation()[1];
    matrix[5]  = pose->rotation()[4];
    matrix[6]  = pose->rotation()[7];

    matrix[8]  = pose->rotation()[2];
    matrix[9]  = pose->rotation()[5];
    matrix[10] = pose->rotation()[8];
#endif

    return true;
}

bool xslam_get_transform(Matrix4x4 *matrix, long long *timestamp, int *status)
{
    if( matrix == nullptr ){
        std::cerr << "Matrix reference is null" << std::endl;
#ifdef ANDROID
        __android_log_print(ANDROID_LOG_INFO,    "XVisio", "Matrix reference is null");
#endif
        return false;
    }

    return xslam_get_transform_matrix(matrix->m, timestamp, status);
}

bool xslam_ready()
{
    return s_ready;
}

/**
 * @brief Get the last image data in RGB format
 * @brief xslam_get_rgb_image_RGB Get the last image data in RGB format
 * @param data
 * @param width
 * @param height
 * @param timestamp pointer to timestamp, update to new image's timestamp. if new image's timestamp <= timestamp, function will return false
 * @return
 */
bool xslam_get_rgb_image_RGB(unsigned char* data, int width, int height, long long* timestamp)
{
    return true;
}

/**
 * @brief xslam_get_rgb_image_RGBA Get the last image data in RGBA format
 * @param data
 * @param width
 * @param height
 * @param timestamp pointer to timestamp, update to new image's timestamp. if new image's timestamp <= timestamp, function will return false
 * @return
 */
bool xslam_get_rgb_image_RGBA(unsigned char* data, int width, int height, long long* timestamp)
{
    return true;
}

/**
 * @brief Get the last image data in YUV format
 * @param data Should be 1.5 * height * width allocated unsigned char array
 * @param width pointer to destination width. if <=0 will set to origin width
 * @param height pointer to destination height. if <=0 will set to origin height
 * @param timestamp pointer to timestamp, update to new image's timestamp. if new image's timestamp <= timestamp, function will return false
 * @return
 */
bool xslam_get_rgb_image_YUV(unsigned char *data, int *width, int *height, long long* timestamp)
{
    return true;
}

int xslam_get_rgb_width()
{
        return 0;
}

int xslam_get_rgb_height()
{
        return 0;
}

/**
 * @brief xslam_get_left_image Get the last left image data in RGBA format
 * @param data
 * @param width
 * @param height
 * @return
 */
bool xslam_get_left_image(unsigned char* data, int width, int height)
{
    return true;
}

/**
 * @brief xslam_get_right_image Get the last right image data in RGBA format
 * @param data
 * @param width
 * @param height
 * @return
 */
bool xslam_get_right_image(unsigned char* data, int width, int height)
{
    return true;
}

int xslam_get_stereo_width()
{
        return 0;
}

int xslam_get_stereo_height()
{
        return 0;
}

int xslam_get_stereo_max_points()
{
    return 1024;
}

bool xslam_get_left_points(Vector2 *points, int *size)
{
    return true;
}

bool xslam_get_right_points(Vector2 *points, int *size)
{
    return true;
}

int xslam_get_tof_width()
{
        return 0;
}

int xslam_get_tof_height()
{
        return 0;
}

bool xslam_get_tof_image(unsigned char* data, int width, int height)
{
    return true;
}

bool xslam_get_depth_data(float *data)
{
    return true;
}

bool xslam_get_cloud_data(Vector3 *cloud)
{
    return true;
}

bool xslam_get_imu( Vector3 *accel, Vector3 *gyro, Vector3 *magn, long long *timestamp )
{
    return false;
}

bool xslam_get_imu_array(Vector3 *imu, long long *timestamp)
{
    return false;
}

bool xslam_get_event( int *type, int *state, long long *timestamp )
{
    return false;
}

bool xslam_get_3dof( Orientation *o )
{
    return false;
}

bool xslam_reset_slam()
{
    if (device && device->slam()) {
        return device->slam()->reset();
    }
    return false;
}

bool xslam_set_aec(int p1, int p2, int p3, int p4, int p5, int p6)
{
    return false;
}

bool xslam_set_imu_configuration(int mode, int stereoOffsetUs, int edgePredUs, bool edgeImuFusion, bool imuSyncedWithinEdgePacket)
{
    return false;
}

bool xslam_set_flip(bool flip)
{
    return false;
}

bool xslam_set_post_filter(bool enabled, float rotationParam, float translationParam)
{
    return false;
}

bool xslam_set_imu_fusion(int imuFusionMode, bool synced, float delay, float prediction)
{
    return false;
}

bool xslam_set_rgb_source(RgbSource source)
{
    return true;
}

bool xslam_set_rgb_resolution(RgbResolution res)
{
    return false;
}

void xslam_start_rgb_stream()
{
}

void xslam_stop_rgb_stream()
{
}

void xslam_start_tof_stream()
{
    if (device->tofCamera()) {
        device->tofCamera()->start();
    }
}

void xslam_stop_tof_stream()
{
    if (device->tofCamera()) {
        device->tofCamera()->stop();
    }
}

void xslam_start_stereo_stream()
{
}

void xslam_stop_stereo_stream()
{
}

void xslam_start_speaker_stream()
{
}

void xslam_stop_speaker_stream()
{
}

bool xslam_write_hid(unsigned char *wdata, int wlen)
{
    return false;
}

bool xslam_write_and_read_hid(unsigned char *wdata, int wlen, unsigned char *rdata, int rlen)
{
    return false;
}

bool readIMUBias(imu_bias *bias)
{
    return false;
}

bool readStereoFisheyesCalibration(stereo_fisheyes *calib, int *imu_fisheye_shift_us)
{
    return false;
}

bool readDisplayCalibration(pdm_calibration *calib)
{
    return false;
}

bool readToFCalibration(pdm_calibration *calib)
{
    return false;
}

bool readRGBCalibration(rgb_calibration *calib)
{
    return false;
}

bool readStereoFisheyesPDMCalibration(stereo_pdm_calibration *calib)
{
    return false;
}

bool readStereoDisplayCalibration(stereo_pdm_calibration *calib)
{
    return false;
}

bool xslam_set_cnn_model_s(const std::string &path)
{
   return false;
}

bool xslam_set_cnn_descriptor_s(const std::string &path)
{
   return false;
}

bool xslam_set_cnn_source(int source)
{
   return false;
}

bool xslam_set_cnn_model(const char *path)
{
    return xslam_set_cnn_model_s( path );
}

bool xslam_set_cnn_descriptor(const char *path)
{
    return xslam_set_cnn_descriptor_s( path );
}


int xslam_transfer_speaker_buffer(const unsigned char *data, int len)
{
    return -1;
}

bool xslam_play_sound( const unsigned char *data, int len )
{
    return false;
}

bool xslam_play_sound_file( const char *path )
{
    return false;
}

bool xslam_is_playing( )
{
    return false;
}

void xslam_stop_play( )
{
}

//std::queue<std::shared_ptr<XSlam::audio>> qmic;
std::thread mic_thread;
std::mutex mq;
static cb_data mic_cb;
static int mic_cb_id = -1;

bool xslam_set_mic_callback( cb_data cb )
{
    return false;
}

void xslam_unset_mic_callback()
{
}

void serializePlane(const std::vector<xv::Plane> &planes, unsigned char *data, int *len)
{
    if (data == nullptr || *len < 64) {
        std::cerr << "bad plane out memory" << std::endl;
        return;
    }

    int maxlen = *len;

    *((int *)data) = planes.size();
    data += 4;
    *len = 4;

    for (int i = 0; i < planes.size(); i++) {
        //Vector3d: std::array<double,3>
        //struct Plane {
        //    std::vector<Vector3d> points;
        //    Vector3d normal;
        //    double d;
        //    std::string id;
        //};
        auto& plane = planes[i];
        int alen = 4 + plane.points.size() * 3 * sizeof(double) + 3 * sizeof(double) + sizeof(double) + 4 + plane.id.size();
        if (*len + alen > maxlen) {
            std::cerr << sformat("Plane too big. curr plane id:%d, plane count:%zu, (max len,used len,curr_plane_data_len):(%d,%d,%d) curr points count:%zu", i,  planes.size(), maxlen, *len, alen, planes[i].points.size()) << std::endl;
            return;
        }
        *((int *)data) = plane.points.size();
        data += 4;
        if (plane.points.size() > 0)
        {
            std::memcpy(data, &plane.points[0], plane.points.size() * 3 * sizeof(double));
            data += plane.points.size() * 3 * sizeof(double);
        }
        std::memcpy(data, &plane.normal, 3 * sizeof(double));
        data += 3 * sizeof(double);
        std::memcpy(data, &plane.d, sizeof(double));
        data += sizeof(double);
        *((int *)data) = plane.id.size();
        data += 4;
        std::memcpy(data, plane.id.c_str(), plane.id.size());
        data += plane.id.size();
        *len += alen;
    }
}

bool xslam_start_detect_plane_from_tof()
{
    if (device->slam() && device->tofCamera()) {
        s_tofPlaneId = device->slam()->registerTofPlanesCallback([](std::shared_ptr<const std::vector<xv::Plane>> plane){
             if (plane->size() > 0) {
                std::cerr << "tof got plane, id: " << plane->at(0).id << ", [" << plane->at(0).normal[0] << "," << plane->at(0).normal[1] << "," << plane->at(0).normal[2] << "]" << std::endl;
                s_tofPlaneMutex.lock();
                s_tofPlane = plane;
                s_tofPlaneMutex.unlock();
            }
        });
        device->tofCamera()->start();
        return true;
    }
    return false;
}

bool xslam_stop_detect_plane_from_tof()
{
    if (device->slam() && device->tofCamera()) {
        device->slam()->unregisterTofPlanesCallback(s_tofPlaneId);
        device->tofCamera()->stop();
        return true;
    }
    return false;
}

bool xslam_get_plane_from_tof(unsigned char *data, int *len)
{
    if (s_tofPlane)
    {
        std::shared_ptr<const std::vector<xv::Plane>> planes;
        s_tofPlaneMutex.lock();
        planes = s_tofPlane;
        s_tofPlaneMutex.unlock();
        serializePlane(*planes.get(), data, len);
        return true;
    }
	return false;
}

bool xslam_start_detect_plane_from_stereo()
{
    if (device->slam()) {
        s_stereoPlaneId = device->slam()->registerStereoPlanesCallback([](std::shared_ptr<const std::vector<xv::Plane>> plane){
             if (plane->size() > 0) {
                std::cerr << "stereo got plane, id: " << plane->at(0).id << ", [" << plane->at(0).normal[0] << "," << plane->at(0).normal[1] << "," << plane->at(0).normal[2] << "]" << std::endl;
                s_stereoPlaneMutex.lock();
                s_stereoPlane = plane;
                s_stereoPlaneMutex.unlock();
            }
        });
        return true;
    }
    return false;
}

bool xslam_stop_detect_plane_from_stereo()
{
    if (device->slam()) {
        device->slam()->unregisterStereoPlanesCallback(s_stereoPlaneId);
        return true;
    }
    return false;
}

bool xslam_get_plane_from_stereo(unsigned char *data, int *len)
{
    if (s_stereoPlane)
    {
        std::shared_ptr<const std::vector<xv::Plane>> planes;
        s_stereoPlaneMutex.lock();
        planes = s_stereoPlane;
        s_stereoPlaneMutex.unlock();
        serializePlane(*planes.get(), data, len);
        return true;
    }
    return false;
}



}
