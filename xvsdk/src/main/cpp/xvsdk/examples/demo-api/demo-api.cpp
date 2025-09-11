#include <xv-sdk.h>

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <thread>
#include <chrono>
#include <atomic>
#include <iterator>
#include <mutex>
#include "../../include2/xv-sdk-ex.h"
#include "fps_count.hpp"
#include "pipe_srv.h"
#ifdef _WIN32
#include <corecrt_math_defines.h>
#endif

std::shared_ptr<xv::Device> device = nullptr;

xv::XV_IRIS_DATA irisData;

static struct xv::sgbm_config global_config = {
    1 ,//enable_dewarp
    1.0, //dewarp_zoom_factor
    0, //enable_disparity
    1, //enable_depth
    0, //enable_point_cloud
    0.08, //baseline
    96, //fov
    255, //disparity_confidence_threshold
    {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, //homography
    1, //enable_gamma
    2.2, //gamma_value
    0, //enable_gaussian
    0, //mode
    8000, //max_distance
    100, //min_distance
};


void enrollCallback(const xv::XV_IRIS_DATA &enrollData)
{
    printf("enter into enroll callback\n");
    irisData = enrollData;
}

void irisCallback(const xv::XV_IRIS_DATA &irisData)
{
    printf("enter into iris callback\n");
}

int requestCmdAllPlatform(const char* tip_info, int size)
{
#ifdef _WIN32
    std::cout << tip_info << std::endl;
    std::string cCmd = "";
    std::cin >> cCmd;
    return atoi(cCmd.c_str());
#else
    return vsc_client_pipe_request_cmd(tip_info, size);
#endif
}

std::string map_filename = "map.bin";
std::string map_shared_filename = "map_shared.bin";
std::atomic_int localized_on_reference_percent(0);
std::filebuf mapStream;
bool enable_output_log = true;
int slamStartMode = 0;

void imuCallback(std::shared_ptr<const xv::Imu> imu)
{
    static FpsCount fc;
    fc.tic();
    static int k = 0;
    if (k++ % 100 == 0) {
        if(enable_output_log){
            std::cout << "imu" << "@" << std::round(fc.fps()) << "fps"
                      << " Time=(" << imu->edgeTimestampUs << " " << imu->hostTimestamp << "),"
                      << " Gyro=(" << imu->gyro[0] << " " << imu->gyro[1] << " " << imu->gyro[2] << "),"
                      << " Accel=(" << imu->accel[0] << " " << imu->accel[1] << " " << imu->accel[2] << "),"
                      << " Temperature=" << imu->temperature
                      << std::endl;
        }
    }
}
//add event and cnn code
//add event callback
void eventCallback(xv::Event const& event)
{
    std::string strType = "Unknown";
    std::string strEvent = "Unknown";
    std::stringstream str;
    switch (event.type)
    {
    case 0x01:
        strType = "key1";
        if (event.state == 0x02) strEvent = "trigger";
        break;
    case 0x02:
        strType = "P-sensor";
        if(event.state == 0x00) strEvent = "away from P-sensor";
        else if(event.state == 0x01) strEvent = "close to P-sensor";
        break;
    case 0x06:
        strType = "als";
        strEvent = std::to_string(event.state);
        break;
    case 0x07:
        strType = "tp";
        if (event.state == 0x00) strEvent = "none";
        else if (event.state == 0x03) strEvent = "single-click";
        else if (event.state == 0x04) strEvent = "double-click";
        else if (event.state == 0x05) strEvent = "right-slip";
        else if (event.state == 0x06) strEvent = "left-slip";
        else if (event.state == 0x07) strEvent = "down-slip";
        else if (event.state == 0x08) strEvent = "up-slip";
        break;
    case 0x09:
        strType = "key2";
        if (event.state == 0x02) strEvent = "trigger";
        break;
    case 0xF0:
        strType = "v-sync";
        str << event.state;
        str >> strEvent;
        break;
    default:
        break;
    }
    if(enable_output_log){
        std::cout << "****Event@" << "edgeTimestampUs:" << event.edgeTimestampUs
                  << ";  Type:" << strType << ";  State:" << strEvent << std::endl;
    }
}

//add CNN callback
void cnnCallback(std::vector<xv::Object> objs)
{
    static int k = 0;
    if (k++ % 5 == 0) {
        for (int i = 0; i < objs.size(); i++)
        {
            xv::Object obj = objs.at(i);
            if(enable_output_log){
                std::cout << obj.width << "x" << obj.height << " # " << obj.type << " # confidence:" << obj.confidence << "; " << std::endl;
            }
        }
    }
}

void  fisheyeLCallback(xv::FisheyeImages const& fisheye) {
    static FpsCount fc;
    fc.tic();
    static int k = 0;
    if (k++ % 50 == 0 && fisheye.images.size() >= 1) {
        if(enable_output_log){
            std::cout << fisheye.images.at(0).width << "x" << fisheye.images.at(0).height << "@" << std::round(fc.fps()) << "fps" << std::endl;
        }
    }
}

void  fisheyeRCallback(xv::FisheyeImages const& fisheye) {
    static FpsCount fc;
    fc.tic();
    static int k = 0;
    if (k++ % 50 == 0 && fisheye.images.size() >= 2) {
        if(enable_output_log){
            std::cout << "Right image" << fisheye.images.at(1).width << "x" << fisheye.images.at(1).height << "@" << std::round(fc.fps()) << "fps" << std::endl;
        }
    }
}

void orientationCallback(xv::Orientation const& o)
{
    static FpsCount fc;
    fc.tic();
    static int k = 0;
    if (k++ % 100 == 0) {
        auto& q = o.quaternion();
        if(enable_output_log){
            std::cout << "orientation" << "@" << std::round(fc.fps()) << "fps"
                      << " 3dof=(" << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << "),"
                      << std::endl;
        }
    }
}

void GestureCallbackEX(xv::GestureData const& gesture)
{
    for (int i = 0; i < 2; i++)
    {
        if (gesture.index[i] != -1)
        {
            if(enable_output_log){
                std::cout << "gesture host timestamp = " << gesture.hostTimestamp << std::endl;
                std::cout << "gesture edge timestamp = " << gesture.edgeTimestampUs << std::endl;
                std::cout << "gesture index = " << gesture.index[i] << std::endl;
            }
        }
    }
}

void GesturePosCallbackEX(std::shared_ptr<const std::vector<xv::Pose>> poses)
{
    if(enable_output_log){
        std::cout << "keypoints 21Dof is : " << std::endl;
        for (auto pose : *poses.get())
        {
            std::cout << "x = " << pose.x() << " " << "y = " << pose.y() << " " << "z = " << pose.z() << std::endl;
        }
    }
}

void eyetrackingCallback(xv::EyetrackingImage const& o)
{
    static FpsCount fc;
    fc.tic();
    static int k = 0;
    if (k++ % 30 == 0) {
        if(enable_output_log){
            std::cout << "[eyetracking]" << o.images[0].width << "x" << o.images[0].height << "@" << std::round(fc.fps()) << "fps"
                      << std::endl;
        }
    }
}

void stereoCallback(std::shared_ptr<const xv::FisheyeImages> stereo)
{
    static FpsCount fc;
    fc.tic();
    static int k = 0;
    if (k++ % 50 == 0) {
        if(enable_output_log){
            std::cout << stereo->images[0].width << "x" << stereo->images[0].height + stereo->images[1].height << "@" << std::round(fc.fps()) << "fps" << std::endl;
        }
    }
}

void poseCallback(xv::Pose const& pose) {
    static FpsCount fc;
    fc.tic();
    static int k = 0;
    if (k++ % 100 == 0) {
        auto t = pose.translation();
        auto r = xv::rotationToPitchYawRoll(pose.rotation());
        if(enable_output_log){
            std::cout << "slam-callback-pose" << "@" << std::round(fc.fps()) << "fps"
                      << " p=(" << t[0] << " " << t[1] << " " << t[2]
                      << " ), r=(" << r[0] << " " << r[1] << " " << r[2] << " )"
                      << ", Confidence= " << pose.confidence()
                      << std::endl;
        }
    }
}

std::thread tpos;
bool stop = false;
bool start3DofGet = false;
bool start3DofGetAt = false;
double t = -1;

void stopGetPose()
{
    stop = true;
    if (tpos.joinable())
        tpos.join();
}
void startGetPose(std::shared_ptr<xv::Slam> slam)
{
    stopGetPose();
    stop = false;
    tpos = std::thread([slam] {
        double prediction = 0.005;

        long n = 0;
        long nb_ok = 0;
        xv::Pose pose;
        while (!stop) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));

            bool ok = slam->getPose(pose, prediction);

            if (ok) {
                nb_ok++;
                static int k = 0;
                if (k++ % 100 == 0) {
                    auto t = pose.translation();
                    auto r = xv::rotationToPitchYawRoll(pose.rotation());
                    if(enable_output_log){
                        //std::cout << "slam-get-pose [" << p->x << "," << p->y << "," << p->z << "]" << std::endl;
                        std::cout << std::setprecision(5) << "slam-get-pose"
                                  << " p=(" << pose.x() << "," << pose.y() << "," << pose.z() << "," << r[0]*180/M_PI << "," << r[1]*180/M_PI << "," << r[2]*180/M_PI << " )"
                                  << ", Confidence= " << pose.confidence()
                                  << std::endl;
                    }
                }
            }
            n++;
        }
        if(enable_output_log){
            std::cout << "Nb get pose ok: " << 100.0 * double(nb_ok) / n << "% (" << nb_ok << "/" << n << ")" << std::endl;
        }
        });

}

void startGetPoseAt(std::shared_ptr<xv::Slam> slam)
{
    if(enable_output_log){
        std::cout << "Wait 5s ...\n";
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));
    stop = false;
    tpos = std::thread ([slam]{
        while(!stop)
        {
            auto now = std::chrono::steady_clock::now();

            t = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
            t += 0.0123;
            xv::Pose poseAt;
            if (slam->getPoseAt(poseAt, t)) {
                static int k = 0;
                if (k++ % 100 == 0) {
                    auto pitchYawRoll = xv::rotationToPitchYawRoll(poseAt.rotation());
                    if(enable_output_log){
                        std::cout << "slam-poseAt" << " (" << poseAt.x() << "," << poseAt.y() << "," << poseAt.z() << "," << pitchYawRoll[0] * 180 / M_PI << "," << pitchYawRoll[1] * 180 / M_PI << "," << pitchYawRoll[2] * 180 / M_PI << ")" << std::endl;
                    }
                }
            }

            // to simulate the 60Hz loop
            std::this_thread::sleep_until(now + std::chrono::microseconds(long(1. / 60. * 1e6)));
        }

        });
}

xv::Orientation orientation30ms;
xv::Orientation orientationAt;

void Start3DofGet(std::shared_ptr<xv::OrientationStream> orientation)
{
    stop = false;
    start3DofGet = true;
    tpos = std::thread ([orientation]{
        while(!stop)
        {
            auto now = std::chrono::steady_clock::now();
            if (orientation->get(orientation30ms, 0.030)) {
                static int k = 0;
                if (k++ % 100 == 0) {
                    auto& q = orientation30ms.quaternion();
                    if(enable_output_log){
                        std::cout << "orientation30ms"
                                  << " 3dof=(" << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << "),"
                                  << std::endl;
                    }
                }
            }
            // to simulate the 60Hz loop
            std::this_thread::sleep_until(now + std::chrono::microseconds(long(1. / 60. * 1e6)));
        }

        });
}

void Stop3DofGet()
{
    stop = true;
    if (tpos.joinable())
    {
        tpos.join();
    }
}

void Start3DofGetAt(std::shared_ptr<xv::OrientationStream> orientation)
{
    stop = false;
    start3DofGetAt = true;
    tpos = std::thread ([orientation]{
        while(!stop)
        {
            auto now = std::chrono::steady_clock::now();
            if(slamStartMode == 0)
                t = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() * 1e-6;
            else
                t = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
            t += 0.0123;
            if (orientation->getAt(orientationAt, t)) {
                static int k = 0;
                if (k++ % 100 == 0) {
                    auto& q = orientationAt.quaternion();
                    if(enable_output_log){
                        std::cout << "orientationAt"
                                  << " 3dof=(" << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << "),"
                                  << std::endl;
                    }
                }
            }
            // to simulate the 60Hz loop
            std::this_thread::sleep_until(now + std::chrono::microseconds(long(1. / 60. * 1e6)));
        }

        });
}

void Stop3DofGetAt()
{
    stop = true;
    if (tpos.joinable())
    {
        tpos.join();
    }
}

void cslamSavedCallback(int status_of_saved_map, int map_quality)
{
    if(enable_output_log){
        std::cout << " Save map (quality is " << map_quality << "/100) and switch to CSlam:";
        switch (status_of_saved_map)
        {
        case  2: std::cout << " Map well saved. " << std::endl; break;
        case -1: std::cout << " Map cannot be saved, an error occured when trying to save it." << std::endl; break;
        default: std::cout << " Unrecognized status of saved map " << std::endl; break;
        }
    }
    mapStream.close();
}

void cslamSwitchedCallback(int map_quality)
{
    if(enable_output_log){
        std::cout << " map (quality is " << map_quality << "/100) and switch to CSlam:";
    }
    mapStream.close();
}





void rgbCallback(xv::ColorImage const& rgb) {
    static FpsCount fc;
    fc.tic();
    static int k = 0;
    if (k++ % 10 == 0) {
        if(enable_output_log){
            std::cout << "rgb: " << rgb.width << "x" << rgb.height << "@" << std::round(fc.fps()) << "fps" << std::endl;
        }
    }
}

void rgb2Callback(xv::ColorImage const& rgb) {
    static FpsCount fc;
    fc.tic();
    static int k = 0;
    if (k++ % 10 == 0) {
        if(enable_output_log){
            std::cout << "rgb2: " << rgb.width << "x" << rgb.height << "@" << std::round(fc.fps()) << "fps" << std::endl;
        }
    }
}


void planeCallback(std::shared_ptr<const std::vector<xv::Plane>> planes)
{
    if (planes)
    {
        for (auto const& plane : *planes.get()) {
            static int k;
            if (k++ % 20 == 0) {
                if(enable_output_log){
                    std::cout << "got plane, id: " << plane.id << ", [" << plane.normal[0] << "," << plane.normal[1] << "," << plane.normal[2] << "]" << std::endl;
                    for (auto const& x : plane.points)
                        std::cout << "(" << x[0] << "," << x[1] << " " << x[2] << ")";
                    std::cout << std::endl;
                }
            }
        }
    }
}

void gestureCallback(xv::GestureData const& gesture)
{
    for (int i = 0; i < 2; i++)
    {
        if (gesture.index[i] != -1)
        {
            if(enable_output_log){
                std::cout << "gesture host timestamp = " << gesture.hostTimestamp << std::endl;
                std::cout << "gesture edge timestamp = " << gesture.edgeTimestampUs << std::endl;
                std::cout << "gesture pos x = " << gesture.position[i].x << std::endl;
                std::cout << "gesture pos y = " << gesture.position[i].y << std::endl;
                std::cout << "gesture index = " << gesture.index[i] << std::endl;
            }
        }
    }
}

void dynamicgestureCallback(xv::GestureData const& gesture)
{
    for (int i = 0; i < 2; i++)
    {
        if (gesture.index[i] != -1)
        {
            if(enable_output_log){
                std::cout << "dynamic gesture host timestamp = " << gesture.hostTimestamp << std::endl;
                std::cout << "dynamic gesture edge timestamp = " << gesture.edgeTimestampUs << std::endl;
                std::cout << "dynamic gesture index = " << gesture.index[i] << std::endl;
            }
        }
    }
}

void keypointsCallback(std::shared_ptr<const std::vector<xv::keypoint>> keypoints)
{
    if (keypoints->size() == 21 || keypoints->size() == 42)
    {
        if(enable_output_log){
            std::cout << "keypoints 21Dof is : " << std::endl;
            for (auto keypoint : *keypoints.get())
            {
                std::cout << "x = " << keypoint.x << " " << "y = " << keypoint.y << " " << "z = " << keypoint.z << std::endl;
            }
        }
    }
}

void slamkeypointsCallback(std::shared_ptr<const xv::HandPose> keypoints)
{
    if(enable_output_log){
        int count = 0;
        auto results = *keypoints;
        std::cout << "scale: " << results.scale[0] << " " << results.scale[1] << std::endl;
        for (auto keypoint : results.pose)
        {
            if((keypoint.x() + keypoint.y() + keypoint.z()) > 1e-3)
            {
                std::cout << "x = " << keypoint.x() << " " << "y = " << keypoint.y() << " " << "z = " << keypoint.z() << std::endl;
                count++;
            }
        }
        double left_timestamp = results.timestamp[0];
        double right_timestamp = results.timestamp[1];
        double fisheye_timestamp = results.fisheye_timestamp;
        std::cout << "left_result interval: " << fisheye_timestamp-left_timestamp << std::endl;
        std::cout << "right_result interval: " << fisheye_timestamp-right_timestamp << std::endl;
        // std::cout << "keypoints 26Dof base on slam is : " << count << std::endl;
    }
}

void objDetRKNN3588Callback(const std::vector<xv::Det2dObject>& res)
{
    if(enable_output_log){
        std::cout << "******************" << std::endl;
        for (int i=0; i<res.size(); ++i){
            printf("obj: %d id: %i name: %s score: %f box: [%f, %f, %f, %f], keypoints_size: %li \n", 
                    i, res[i].idx, res[i].name.c_str(), res[i].score, res[i].top, res[i].left, res[i].width, res[i].height, res[i].keypoints.size());
            for (int j=0; j<res[i].keypoints.size(); ++j){
                printf("x:%f y:%f z:%f \n", res[i].keypoints[j].x, res[i].keypoints[j].y, res[i].keypoints[j].z);
            }
        }
    }
}

void gazeCallback(xv::XV_ET_EYE_DATA_EX const& gazeData)
{
    if(enable_output_log){
        std::cout << "enter gazeCallback" << std::endl;
        printf("xvsdk_gaze timestamp = %lld\n", gazeData.timestamp);
        printf("xvsdk_gaze ipd = %f\n", gazeData.ipd);
        // printf("xvsdk_gaze recommend = %d\n", gazeData.recommend);
        printf("xvsdk_gaze leftGaze gazePoint x = %f, y = %f, z = %f\n", gazeData.leftGaze.gazePoint.x, gazeData.leftGaze.gazePoint.y, gazeData.leftGaze.gazePoint.z);
        // printf("xvsdk_gaze leftGaze rawPoint x = %f, y = %f, z = %f\n", gazeData.leftGaze.rawPoint.x, gazeData.leftGaze.rawPoint.y, gazeData.leftGaze.rawPoint.z);
        // printf("xvsdk_gaze leftGaze smoothPoint x = %f, y = %f, z = %f\n", gazeData.leftGaze.smoothPoint.x, gazeData.leftGaze.smoothPoint.y, gazeData.leftGaze.smoothPoint.z);
        printf("xvsdk_gaze rightGaze gazePoint x = %f, y = %f, z = %f\n", gazeData.rightGaze.gazePoint.x, gazeData.rightGaze.gazePoint.y, gazeData.rightGaze.gazePoint.z);
        // printf("xvsdk_gaze rightGaze rawPoint x = %f, y = %f, z = %f\n", gazeData.rightGaze.rawPoint.x, gazeData.rightGaze.rawPoint.y, gazeData.rightGaze.rawPoint.z);
        // printf("xvsdk_gaze rightGaze smoothPoint x = %f, y = %f, z = %f\n", gazeData.rightGaze.smoothPoint.x, gazeData.rightGaze.smoothPoint.y, gazeData.rightGaze.smoothPoint.z);
        printf("xvsdk_gaze left pupil x = %f, y = %f\n", gazeData.leftPupil.pupilCenter.x, gazeData.leftPupil.pupilCenter.y);
        printf("xvsdk_gaze right pupil x = %f, y = %f\n", gazeData.rightPupil.pupilCenter.x, gazeData.rightPupil.pupilCenter.y);
    }
}

template<class F, std::size_t N>
std::ostream& operator<<(std::ostream& o, const std::array<F, N>& v)
{
    o << "[";
    for (int i = 0;i < N;i++) {
        o << v.at(i);
        if (i < N - 1) {
            o << ", ";
        }
    }
    o << "]";
    return o;
}
std::ostream& operator<<(std::ostream& o, const xv::UnifiedCameraModel& m)
{
    o << "{";
    o << "w=" << m.w << ", ";
    o << "h=" << m.h << ", ";
    o << "fx=" << m.fx << ", ";
    o << "fy=" << m.fy << ", ";
    o << "u0=" << m.u0 << ", ";
    o << "v0=" << m.v0 << ", ";
    o << "xi=" << m.xi;
    o << "}";
    return o;
}

std::ostream& operator<<(std::ostream& o, const xv::PolynomialDistortionCameraModel& m)
{
    o << "{";
    o << "w=" << m.w << ", ";
    o << "h=" << m.h << ", ";
    o << "fx=" << m.fx << ", ";
    o << "fy=" << m.fy << ", ";
    o << "u0=" << m.u0 << ", ";
    o << "v0=" << m.v0 << ", ";
    o << "distor=" << m.distor;
    o << "}";
    return o;
}

std::ostream& operator<<(std::ostream& o, const xv::SpecialUnifiedCameraModel &m)
{
    o << "{";
    o << "w=" << m.w << ", ";
    o << "h=" << m.h << ", ";
    o << "fx=" << m.fx << ", ";
    o << "fy=" << m.fy << ", ";
    o << "u0=" << m.u0 << ", ";
    o << "v0=" << m.v0 << ", ";
    o << "eu=" << m.eu;
    o << "ev=" << m.ev;
    o << "alpha=" << m.alpha;
    o << "beta=" << m.beta;
    o << "}";
    return o;
}

std::ostream& operator<<(std::ostream& o, xv::CalibrationEx const& c)
{
    o << "Calibration:" << std::endl;
    o << " R:" << c.pose.rotation() << std::endl;
    o << " T: " << c.pose.translation() << std::endl;
    for(int i=0;i<c.ucm.size();i++){
        o << "UCM" << i << ": " << c.ucm.at(i) << std::endl;
    }
    for(int i=0;i<c.pdcm.size();i++){
        o << "PDCM" << i << ": " << c.pdcm.at(i) << std::endl;
    }
    for(int i=0;i<c.seucm.size();i++){
        o << "SEUCM" << i << ": " << c.seucm.at(i) << std::endl;
    }
    return o;
}

std::ostream& operator<<(std::ostream& o, xv::Calibration const& c)
{
    o << "Calibration:" << std::endl;
    o << " R:" << c.pose.rotation() << std::endl;
    o << " T: " << c.pose.translation() << std::endl;
    for (int i = 0;i < c.ucm.size();i++) {
        o << "UCM" << i << ": " << c.ucm.at(i) << std::endl;
    }
    for (int i = 0;i < c.pdcm.size();i++) {
        o << "PDCM" << i << ": " << c.pdcm.at(i) << std::endl;
    }
    return o;
}

std::ostream& operator<<(std::ostream& o, const std::vector<xv::Calibration>& calibs)
{
    for (auto c : calibs) {
        if(enable_output_log){
            std::cout << c << std::endl;
        }
    }
    return o;
}

std::ostream& operator<<(std::ostream& o, const std::vector<xv::CalibrationEx> &calibs)
{
    for (auto c : calibs) {
        if(enable_output_log){
            std::cout << c << std::endl;
        }
    }
    return o;
}

void SetRgbResolution(int cmd, std::shared_ptr<xv::ColorCamera> camera)
{
    switch (cmd)
    {
    case 0:
        camera->setResolution(xv::ColorCamera::Resolution::RGB_1920x1080);
        break;
    case 1:
        camera->setResolution(xv::ColorCamera::Resolution::RGB_1280x720);
        break;
    case 2:
        camera->setResolution(xv::ColorCamera::Resolution::RGB_640x480);
        break;
    case 3:
        camera->setResolution(xv::ColorCamera::Resolution::RGB_320x240);
        break;
    case 4:
        camera->setResolution(xv::ColorCamera::Resolution::RGB_2560x1920);
        break;
    case 5:
        camera->setResolution(xv::ColorCamera::Resolution::RGB_3840x2160);
        break;
    default:
        if(enable_output_log){
            std::cout << "set rgb resolution format wrong" << std::endl;
        }
        return;
        break;
    }
    if(enable_output_log){
        std::cout << "set rgb resolution successfully" << std::endl;
    }
}

bool SetTofDistanceMode(int cmd, std::shared_ptr<xv::TofCamera> camera)
{
    bool bOk = false;
    switch (cmd)
    {
    case 0:
        bOk = camera->setDistanceMode(xv::TofCamera::DistanceMode::Short);
        break;
    case 1:
        bOk = camera->setDistanceMode(xv::TofCamera::DistanceMode::Middle);
        break;
    case 2:
        bOk = camera->setDistanceMode(xv::TofCamera::DistanceMode::Long);
        break;
    default:
        break;
    }
    return bOk;
}

bool SetTofStreamMode(int cmd, std::shared_ptr<xv::TofCamera> camera)
{
    bool bOk = false;
    switch (cmd)
    {
    case 0:
        bOk = camera->setStreamMode(xv::TofCamera::StreamMode::DepthOnly);
        break;
    case 1:
        bOk = camera->setStreamMode(xv::TofCamera::StreamMode::CloudOnly);
        break;
    case 2:
        bOk = camera->setStreamMode(xv::TofCamera::StreamMode::DepthAndCloud);
        break;
    case 3:
        bOk = camera->setStreamMode(xv::TofCamera::StreamMode::None);
        break;
    case 4:
        bOk = camera->setStreamMode(xv::TofCamera::StreamMode::CloudOnLeftHandSlam);
        break;
    default:
        break;
    }
    return bOk;
}

void cslamLocalizedCallback(float percent)
{
    static int k = 0;
    if (k++ % 100 == 0) {
        localized_on_reference_percent = static_cast<int>(percent * 100);
        if(enable_output_log){
            std::cout << "localized: " << localized_on_reference_percent << "%" << std::endl;
        }
    }
}

void cameraOnOffSwitch(int switchMode, std::shared_ptr<xv::Device> device)
{
    bool bOk = false;
    switch (switchMode)
    {
    case 1:
        bOk = device->fisheyeCameras()->start();
        if (bOk)
        {
            if(enable_output_log){
                std::cout << "fisheye camera start successfully" << std::endl;
            }
        }
        break;
    case 2:
        bOk = device->fisheyeCameras()->stop();
        if (bOk)
        {
            if(enable_output_log){
                std::cout << "fisheye camera stop successfully" << std::endl;
            }
        }
        break;
    case 3:
        bOk = device->colorCamera()->start();
        if (bOk)
        {
            if(enable_output_log){
                std::cout << "rgb camera start successfully" << std::endl;
            }
        }
        break;
    case 4:
        bOk = device->colorCamera()->start();
        if (bOk)
        {
            if(enable_output_log){
                std::cout << "rgb camera stop successfully" << std::endl;
            }
        }
        break;
    case 5:
        bOk = device->tofCamera()->start();
        if (bOk)
        {
            if(enable_output_log){
                std::cout << "tof camera start successfully" << std::endl;
            }
        }
        break;
    case 6:
        bOk = device->tofCamera()->stop();
        if (bOk)
        {
            if(enable_output_log){
                std::cout << "tof camera stop successfully" << std::endl;
            }
        }
        break;
    default:
        break;
    }
}

void GetTagDetection(std::shared_ptr<xv::FisheyeCameras> fisheye, std::string tagDetectorId)
{
    stop = false;
    tpos = std::thread ([fisheye, tagDetectorId]{
        while (!stop) {
            auto t0 = std::chrono::steady_clock::now();
            std::this_thread::sleep_until(t0 + std::chrono::milliseconds(25));
            auto detections = std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(fisheye)->getTagDetections(tagDetectorId);
            if (!detections.empty())
            {
                if(enable_output_log){
                    std::cout << "Tag detections: ";
                    for (auto const& d : detections) {
                        auto const& pose = d.second;
                        auto pitchYawRoll = xv::rotationToPitchYawRoll(pose.rotation());
                        std::cout << "id=" << d.first
                                  << " (" << pose.x() << "," << pose.y() << "," << pose.z() << ","
                                  << pitchYawRoll[0] * 180 / M_PI << "," << pitchYawRoll[1] * 180 / M_PI << "," << pitchYawRoll[2] * 180 / M_PI << ") " << pose.confidence() << std::endl;
                    }
                }
            }
            else {
                if(enable_output_log){
                    std::cout << "Tag empty " << std::endl;;
                }
            }
        }
        });
}

void GetTagDetectionrgb(std::shared_ptr<xv::ColorCamera> fisheye, std::string tagDetectorId)
{
    stop = false;
    tpos = std::thread ([fisheye, tagDetectorId]{
        while (!stop) {
            auto t0 = std::chrono::steady_clock::now();
            std::this_thread::sleep_until(t0 + std::chrono::milliseconds(25));
            auto detections = std::dynamic_pointer_cast<xv::ColorCameraEx>(fisheye)->getTagDetections(tagDetectorId);
            if (!detections.empty())
            {
                if(enable_output_log){
                    std::cout << "Tag detections: ";
                    for (auto const& d : detections) {
                        auto const& pose = d.second;
                        auto pitchYawRoll = xv::rotationToPitchYawRoll(pose.rotation());
                        std::string codeStr = std::dynamic_pointer_cast<xv::ColorCameraEx>(device->colorCamera())->getCode(tagDetectorId,0);
                        printf("qr code: %s\n", codeStr.c_str());
                        std::cout << "id=" << d.first
                                  << " (" << pose.x() << "," << pose.y() << "," << pose.z() << ","
                                  << pitchYawRoll[0] * 180 / M_PI << "," << pitchYawRoll[1] * 180 / M_PI << "," << pitchYawRoll[2] * 180 / M_PI << ") " << pose.confidence() << std::endl;
                    }
                }
            }
            else {
                if(enable_output_log){
                    std::cout << "Tag empty " << std::endl;;
                }
            }
        }
        });
}

void StopGetTag()
{
    stop = true;
    if (tpos.joinable())
    {
        tpos.join();
    }
}

class TOFCallBackFun;
std::string timeShowStr(std::int64_t edgeTimestampUs, double hostTimestamp);
std::string timeShowStr(double hostTimestamp);


std::string  tagDetectorId;
static struct xv::sgbm_config default_config = {
    1,
    3.5,
    1,
    1,
    0,
    0.11285,
    69,
    255,
    {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0},
    0,
    2.2,
    0,
    0,
    5000,
    100,

};

class TofCameraParas
{
public:
    TofCameraParas() :
        m_distanceMode(xv::TofCamera::DistanceMode::Short),
        m_sonyFramerate(xv::TofCamera::Framerate::FPS_5),
        m_sonyTofLibMode(xv::TofCamera::SonyTofLibMode::IQMIX_DF),
        m_sonyResolution(xv::TofCamera::Resolution::VGA),
        m_streamMode(xv::TofCamera::StreamMode::DepthOnly),
        m_normalTofFramerate(10.0)
    {

    }

    void defaultTofCameraParas()
    {
        m_distanceMode = xv::TofCamera::DistanceMode::Short;
        m_sonyFramerate = xv::TofCamera::Framerate::FPS_5;
        m_sonyTofLibMode = xv::TofCamera::SonyTofLibMode::IQMIX_DF;
        m_sonyResolution = xv::TofCamera::Resolution::VGA;
        m_streamMode = xv::TofCamera::StreamMode::DepthOnly;
        m_normalTofFramerate = 10.0;
    }

    const xv::TofCamera::DistanceMode& getDistanceMode() const
    {
        return m_distanceMode;
    }

    void setDistanceMode(const xv::TofCamera::DistanceMode&& distanceMode)
    {
        m_distanceMode = distanceMode;
    }

    void setDistanceMode(const xv::TofCamera::DistanceMode& distanceMode)
    {
        m_distanceMode = distanceMode;
    }

    const xv::TofCamera::Framerate& getSonyFramerate() const
    {
        return m_sonyFramerate;
    }

    void setSonyFramerate(const xv::TofCamera::Framerate&& framerate)
    {
        m_sonyFramerate = framerate;
    }

    void setSonyFramerate(const xv::TofCamera::Framerate& framerate)
    {
        m_sonyFramerate = framerate;
    }

    const xv::TofCamera::SonyTofLibMode& getSonyTofLibMode() const
    {
        return m_sonyTofLibMode;
    }

    void setSonyTofLibMode(const xv::TofCamera::SonyTofLibMode&& sonyTofLibMode)
    {
        m_sonyTofLibMode = sonyTofLibMode;
    }

    void setSonyTofLibMode(const xv::TofCamera::SonyTofLibMode& sonyTofLibMode)
    {
        m_sonyTofLibMode = sonyTofLibMode;
    }

    const xv::TofCamera::Resolution& getSonyResolution() const
    {
        return m_sonyResolution;
    }

    void setSonyResolution(const xv::TofCamera::Resolution& resolution)
    {
        m_sonyResolution = resolution;
    }

    void setSonyResolution(const xv::TofCamera::Resolution&& resolution)
    {
        m_sonyResolution = resolution;
    }

    float getNormalTofFramerate()const
    {
        return m_normalTofFramerate;
    }

    void setNormalTofFramerate(float framerate)
    {
        m_normalTofFramerate = framerate;
    }

    void setStreamMode(const xv::TofCamera::StreamMode& streamMode)
    {
        this->m_streamMode = streamMode;
    }

    void setStreamMode(const xv::TofCamera::StreamMode&& streamMode)
    {
        this->m_streamMode = streamMode;
    }

    const xv::TofCamera::StreamMode& getStreamMode()
    {
        return this->m_streamMode;
    }

private:
    xv::TofCamera::DistanceMode m_distanceMode;
    xv::TofCamera::Framerate m_sonyFramerate;
    xv::TofCamera::SonyTofLibMode m_sonyTofLibMode;
    xv::TofCamera::Resolution m_sonyResolution;
    xv::TofCamera::StreamMode m_streamMode;
    float m_normalTofFramerate;
};

class TOFCallBackFun
{
public:
    static void tofCallback(xv::DepthImage const& tof)
    {
        std::string types[] = { "Depth_16", "Depth_32", "IR", "Cloud", "Raw", "Eeprom" };
        static FpsCount fc;
        int type = static_cast<int>(tof.type);
        if (tof.type != xv::DepthImage::Type::Depth_16 &&
            tof.type != xv::DepthImage::Type::Depth_32 &&
            tof.type != xv::DepthImage::Type::Cloud)
        {
            return;
        }
        fc.tic();
        static int k = 1;
        if (k++ % 20 == 0)
        {
            xv::TofCamera::Manufacturer manufacturer = m_tofCamera->getManufacturer();
            if (m_tofCamera && m_tofCameraParas.getStreamMode() == xv::TofCamera::StreamMode::CloudOnly &&
                manufacturer == xv::TofCamera::Manufacturer::Sony)
            {
                auto points = m_tofCamera->depthImageToPointCloud(tof)->points;
                auto firstPoint = points.begin();
                if(enable_output_log){
                    std::cout << "type:sony tof cloud point@" << tof.width << "x" << tof.height << "@" << std::round(fc.fps()) << "fps" << std::endl;
                }
            }
            else if (m_tofCamera)
            {
                if(enable_output_log){
                    std::cout << "type:" << types[type] << "@" << tof.width << "x" << tof.height << "@" << std::round(fc.fps()) << "fps" << std::endl;
                }
            }
            k = 1;
        }
    }

    static void colorDepthImageCallback(const xv::DepthColorImage& depthColor)
    {
        static FpsCount fc;
        fc.tic();
        static int k = 0;
        if (k++ % 15 == 0)
        {
            if(enable_output_log){
                std::cout << "[ RGBD ]" << timeShowStr(depthColor.hostTimestamp)
                          << depthColor.width << "x" << depthColor.height << "@"
                          << std::round(fc.fps()) << "fps" << std::endl;
            }
        }
    }

public:
    static std::shared_ptr<xv::TofCamera> m_tofCamera;
    static TofCameraParas m_tofCameraParas;
};

std::shared_ptr<xv::TofCamera> TOFCallBackFun::m_tofCamera = nullptr;
TofCameraParas TOFCallBackFun::m_tofCameraParas;



#ifdef _WIN32
#else
void cam_sig_handler(int sig)
{
    switch (sig) {
    case SIGINT:
        printf("catch SIGINT\n");
        break;
    case SIGHUP:
        printf("catch SIGHUB\n");
        break;
    case SIGTERM:
        printf("catch SIGTERM\n");
        break;
    }
    vsc_client_pipe_terminal_srv();
    exit(1);
}
#endif






class SGBMCameraParas
{
public:
    SGBMCameraParas() :
        m_type(xv::SgbmImage::Type::Depth)
    {
    }

    void setStreamMode(const xv::SgbmImage::Type& type)
    {
        m_type = type;
    }

    void setStreamMode(const xv::SgbmImage::Type&& type)
    {
        m_type = type;
    }

    const xv::SgbmImage::Type& getStreamMode()
    {
        return m_type;
    }

private:
    xv::SgbmImage::Type m_type;
};
class SGBMCallback
{
public:
    static void sgbmCallback(const xv::SgbmImage& sgbm_image)
    {
        static FpsCount fc;
        fc.tic();
        static int k = 0;
        if (k++ % 10 == 0) {
            xv::SgbmImage::Type streamMode = paras.getStreamMode();
            if (streamMode == xv::SgbmImage::Type::Depth)
            {
                if(enable_output_log){
                    std::cout << "ImageType:Depth@" << "sgbm_image " << sgbm_image.width << "x" << sgbm_image.height << "@" << std::round(fc.fps()) << "fps" << std::endl;
                }
            }
            else if (streamMode == xv::SgbmImage::Type::PointCloud)
            {
                auto pointcloud = sgbmCamera->depthImageToPointCloud(sgbm_image);
                if(enable_output_log){
                    std::cout << "ImageType:PointCloud@" << "sgbm_image " << sgbm_image.width << "x" << sgbm_image.height << "@" << std::round(fc.fps()) << "fps" << std::endl;
                }
            }
        }
    }

public:
    static SGBMCameraParas paras;
    static std::shared_ptr<xv::SgbmCamera> sgbmCamera;
};
SGBMCameraParas SGBMCallback::paras;
std::shared_ptr<xv::SgbmCamera> SGBMCallback::sgbmCamera;

class BaseMenu
{
public:
    static void errorMesgMenu(void)
    {
        const char menu_errorMsg[] =
        {
            "The value you entered is invalid! \n"
            "Please enter 1 key to continue.\n"
        };
        requestCmdAllPlatform(menu_errorMsg, sizeof(menu_errorMsg));
    }
};

class SGBMCameraSettingMenu : public BaseMenu
{
public:
    static int mainMenu()
    {
        const char meun_sgbm[] =
        {
            "1 : Set stream mode.                         \n"
            "2 : Set sgbm resolution.                         \n"
            "0 : Paras set done,run SGBM camera.          \n"
            "enter select:"
        };
        int cmd = requestCmdAllPlatform(meun_sgbm, sizeof(meun_sgbm));
        if (cmd < 0 || cmd > 2)
        {
            cmd = -1;
        }
        return cmd;
    }

    static int streamModeMenu()
    {
        const char meun_sgbmStream[] =
        {
            "1 : Depth.                       \n"
            "2 : PointCloud.                  \n"
            "enter select:"
        };
        int cmd = requestCmdAllPlatform(meun_sgbmStream, sizeof(meun_sgbmStream));
        if (cmd < 1 || cmd > 2)
        {
            cmd = -1;
            errorMesgMenu();
        }
        return cmd;
    }

    static int resolutionMenu()
    {
        const char meun_sgbmResolution[] = {
            "0: SGBM_640x480     ///< SGBM 480p\n"
            "1: SGBM_1280x720    ///< SGBM 720p\n"
            "------------------------------\n"
            "enter select:"
        };
        int cmd = requestCmdAllPlatform(meun_sgbmResolution, sizeof(meun_sgbmResolution));
        if (cmd < 0 || cmd > 1)
        {
            cmd = -1;
            errorMesgMenu();
        }
        return cmd;
    }
};

void setSGBMCameraPara(std::shared_ptr<xv::Device> device)
{
    bool isContinued = true;
    static SGBMCameraParas paras;
    while (isContinued)
    {
        int cmd = SGBMCameraSettingMenu::mainMenu();
        switch (cmd)
        {
        case 1:
        {
            int cmdStream = SGBMCameraSettingMenu::streamModeMenu();
            if (cmdStream != -1)
            {
                paras.setStreamMode(static_cast<xv::SgbmImage::Type>(cmdStream));
            }
            break;
        }

        case 2:
        {
            //set sgbm solution
            int nSgbmControlCmd = SGBMCameraSettingMenu::resolutionMenu();
            device->sgbmCamera()->setSgbmResolution((xv::SgbmCamera::Resolution)nSgbmControlCmd);
            break;
        }

        case 0:
            isContinued = false;
        default:
            break;
        }
    }
    SGBMCallback::paras = paras;
}





class TofSettingMenu : public BaseMenu
{
public:
    static int mainMenu(void)
    {
        const char meun_tofSetting[] =
        {
            "\n"
            "TOF camera setting menu                      \n"
            "1 : Use TOF default paras and run TOF camera.\n"
            "2 : Set normal TOF framerate(just for Pmd).  \n"
            "3 : Set distanceMode(just for Pmd).          \n"
            "4 : Set sony TOF lib mode.                   \n"
            "5 : Set sony TOF resolution.                 \n"
            "6 : Set sony TOF framerate.                  \n"
            "7 : Set TOF stream mode.                     \n"
            "0 : Paras set done,run TOF camera.           \n"
            "enter select:"
        };
        int cmd = requestCmdAllPlatform(meun_tofSetting, sizeof(meun_tofSetting));
        if (cmd < 0 || cmd > 7)
        {
            cmd = -1;
        }
        return cmd;
    }

    static int sonyFramerateMenu(void)
    {
        const char menu_framerate[] =
        {
            "This menu just support Sony TOF  \n"
            "1 : 5 fps                        \n"
            "2 : 10 fps                       \n"
            "3 : 15 fps                       \n"
            "4 : 20 fps                       \n"
            "5 : 25 fps                       \n"
            "6 : 30 fps                       \n"
            "enter select:"
        };
        int cmd_framerate = requestCmdAllPlatform(menu_framerate, sizeof(menu_framerate));
        if (cmd_framerate >= 1 && cmd_framerate <= 6)
        {
            cmd_framerate -= 1;
        }
        else
        {
            cmd_framerate = -1;
            errorMesgMenu();
        }
        return cmd_framerate;
    }

    static int distanceModeMenu(void)
    {
        const char menu_distanceMode[] =
        {
            "1 : Short                        \n"
            "2 : Middle                       \n"
            "3 : Long                         \n"
            "enter select:"
        };

        int cmd_distanceMode = requestCmdAllPlatform(menu_distanceMode, sizeof(menu_distanceMode));
        if (cmd_distanceMode >= 1 && cmd_distanceMode <= 3)
        {
            cmd_distanceMode -= 1;
        }
        else
        {
            cmd_distanceMode = -1;
            errorMesgMenu();
        }
        return cmd_distanceMode;
    }
    static int sonyLibModeMenu(void)
    {
        const char menu_sonyLibMode[] =
        {
            "1 : IQMIX_DF                    \n"
            "2 : IQMIX_SF                    \n"
            "3 : LABELIZE_DF                 \n"
            "4 : LABELIZE_SF                 \n"
            "5 : M2MIX_DF                    \n"
            "6 : M2MIX_SF                    \n"
            "enter select:"
        };

        int cmd_sonyLibMode = requestCmdAllPlatform(menu_sonyLibMode, sizeof(menu_sonyLibMode));
        if (cmd_sonyLibMode >= 1 && cmd_sonyLibMode <= 6)
        {
            cmd_sonyLibMode -= 1;
        }
        else
        {
            cmd_sonyLibMode = -1;
            errorMesgMenu();
        }
        return cmd_sonyLibMode;
    }

    static int sonyResolutionMenu()
    {
        const char menu_resolution[] =
        {
            "This menu just support Sony TOF\n"
            "1 : VGA                        \n"
            "2 : QVGA                       \n"
            "enter select:"
        };

        int cmd_resolution = requestCmdAllPlatform(menu_resolution, sizeof(menu_resolution));
        if (cmd_resolution >= 1 && cmd_resolution <= 2)
        {
            cmd_resolution -= 1;
        }
        else
        {
            cmd_resolution = -1;
            errorMesgMenu();
        }
        return cmd_resolution;
    }

    static int normalTofFramerateMenu()
    {
        const char menu_normalTofFramerate[] =
        {
            "1 : 5Hz                     \n"
            "2 : 10Hz                    \n"
            "3 : 15Hz                    \n"
            "4 : 20Hz                    \n"
            "5 : 25Hz                    \n"
            "6 : 30Hz                    \n"
            "enter select:"
        };

        int cmd_framerate = requestCmdAllPlatform(menu_normalTofFramerate, sizeof(menu_normalTofFramerate));
        if (cmd_framerate < 1 || cmd_framerate > 6)
        {
            cmd_framerate = -1;
            errorMesgMenu();
        }
        else
        {
            cmd_framerate *= 5;
        }
        return cmd_framerate;
    }

    static int streamModeMenu(void)
    {
        const char menu_streamMode[] =
        {
            "1 : DepthOnly.       \n"
            "2 : CloudOnly.       \n"
            "5 : CloudSlam.       \n"
            "enter select:"
        };
        int cmd_streamMode = requestCmdAllPlatform(menu_streamMode, sizeof(menu_streamMode));
        if (cmd_streamMode < 1 || cmd_streamMode > 5)
        {
            cmd_streamMode = -1;
            errorMesgMenu();
        }
        else
        {
            cmd_streamMode -= 1;
        }
        return cmd_streamMode;
    }
};

void setTofParas(const std::shared_ptr<xv::Device> device)
{
    bool isContinued = true;
    static TofCameraParas tofCameraParas;

    while (isContinued)
    {
        int cmd = TofSettingMenu::mainMenu();
        switch (cmd)
        {
        case 1: // Use tof default paras and run tof camera
            tofCameraParas.defaultTofCameraParas();
            isContinued = false;
            break;
        case 2: // Set normal TOF framerate.
        {
            int cmd_normalFramerate = TofSettingMenu::normalTofFramerateMenu();
            if (cmd_normalFramerate != -1)
            {
                tofCameraParas.setNormalTofFramerate(static_cast<float>(cmd_normalFramerate));
            }
        }
        break;
        case 3: // Set distanceMode.
        {
            int cmd_distanceMode = TofSettingMenu::distanceModeMenu();
            if (cmd_distanceMode != -1)
            {
                tofCameraParas.setDistanceMode(static_cast<xv::TofCamera::DistanceMode>(cmd_distanceMode));
            }
        }
        break;
        case 4: // Set sony TOF lib mode.
        {
            int cmd_sonyLibMode = TofSettingMenu::sonyLibModeMenu();
            if (cmd_sonyLibMode != -1)
            {
                tofCameraParas.setSonyTofLibMode(static_cast<xv::TofCamera::SonyTofLibMode>(cmd_sonyLibMode));
            }
        }
        break;
        case 5: // Set sony TOF resolution.
        {
            int cmd_resolution = TofSettingMenu::sonyResolutionMenu();
            if (cmd_resolution != -1)
            {
                tofCameraParas.setSonyResolution(static_cast<xv::TofCamera::Resolution>(cmd_resolution));
            }
        }
        break;
        case 6: //Set sony TOF framerate.
        {
            int cmd_frmerate = TofSettingMenu::sonyFramerateMenu();
            if (cmd_frmerate != -1)
            {
                tofCameraParas.setSonyFramerate(static_cast<xv::TofCamera::Framerate>(cmd_frmerate));
            }
        }
        break;
        case 7: //Set TOF stream mode.
        {
            int cmd_streamMode = TofSettingMenu::streamModeMenu();
            if (cmd_streamMode != -1)
            {
                tofCameraParas.setStreamMode(static_cast<xv::TofCamera::StreamMode>(cmd_streamMode));
            }
        }
        break;
        case 0: //Paras set done,run TOF camera.
            isContinued = false;
            break;
        default:
            break;
        }
    }

    xv::TofCamera::Manufacturer tofManu = device->tofCamera()->getManufacturer();
    if (tofManu == xv::TofCamera::Manufacturer::Sony)
    {
        device->tofCamera()->setSonyTofSetting(tofCameraParas.getSonyTofLibMode(),
            tofCameraParas.getSonyResolution(),
            tofCameraParas.getSonyFramerate());
        device->tofCamera()->setStreamMode(tofCameraParas.getStreamMode());
    }
    else if (tofManu == xv::TofCamera::Manufacturer::Pmd)
    {
        device->tofCamera()->setFramerate(tofCameraParas.getNormalTofFramerate());
        device->tofCamera()->setStreamMode(tofCameraParas.getStreamMode());
        // invoke this function with setSonyTofSetting function will cause the device to restart.
        device->tofCamera()->setDistanceMode(tofCameraParas.getDistanceMode());
    }

    TOFCallBackFun::m_tofCameraParas = tofCameraParas;
    if(enable_output_log){
        std::cout << "Tof fr:" << tofCameraParas.getNormalTofFramerate() << std::endl;
        std::cout << "Distance mode:" << static_cast<int>(tofCameraParas.getDistanceMode()) << std::endl;
        std::cout << "Sony Tof Lib Mode:" << static_cast<int>(tofCameraParas.getSonyTofLibMode()) << std::endl;
        std::cout << "Sony Resolution:" << static_cast<int>(tofCameraParas.getSonyResolution()) << std::endl;
        std::cout << "Sony Framerate:" << static_cast<int>(tofCameraParas.getSonyFramerate()) << std::endl;
        std::cout << "Stream mode:" << static_cast<int>(tofCameraParas.getStreamMode()) << std::endl;
    }
}

std::string timeShowStr(std::int64_t edgeTimestampUs, double hostTimestamp)
{
    char s[1024];
    double now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() * 1e-6;
    std::sprintf(s, " (device=%lld host=%.4f now=%.4f delay=%.4f) ", (long long)edgeTimestampUs, hostTimestamp, now, now - hostTimestamp);
    return std::string(s);
}

std::string timeShowStr(double hostTimestamp)
{
    char s[1024];
    double now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() * 1e-6;
    std::sprintf(s, " (host=%.4f now=%.4f delay=%.4f) ", hostTimestamp, now, now - hostTimestamp);
    return std::string(s);
}

void colorCameraCallback(xv::ColorImage const& rgb)
{
    static FpsCount fc;
    fc.tic();
    static int k = 0;
    if (k++ % 25 == 0)
    {
        if(enable_output_log){
            std::cout << "rgb@" << timeShowStr(rgb.edgeTimestampUs, rgb.hostTimestamp)
                      << rgb.width << "x" << rgb.height << "@"
                      << std::round(fc.fps()) << "fps" << std::endl;
        }
    }
}

class ColorCameraParas
{
public:
    ColorCameraParas() :
        m_resolution(xv::ColorCamera::Resolution::RGB_320x240)
    {

    }

    void setResolution(const xv::ColorCamera::Resolution resolution)
    {
        m_resolution = resolution;
    }

    const xv::ColorCamera::Resolution& getResolution(void)
    {
        return m_resolution;
    }

private:
    xv::ColorCamera::Resolution m_resolution;
};

class ColorCameraParasSettingMenu : public BaseMenu
{
public:
    static int mainMenu(void)
    {
        const char meun_main[] =
        {
            "\n"
            "    Color camera setting menu     \n"
            "1 : Set color camera resolution.  \n"
            "0 : Set color camera paras and run color camera.\n"
            "enter select:"
        };
        int cmd = requestCmdAllPlatform(meun_main, sizeof(meun_main));
        if (cmd < 0 || cmd > 1)
        {
            cmd = -1;
            errorMesgMenu();
        }
        return cmd;
    }

    static int resolutionMenu(void)
    {
        const char meun_resolution[] =
        {
            "1: RGB_1920x1080   ///< RGB 1080p\n"
            "2: RGB_1280x720    ///< RGB 720p\n"
            "3: RGB_640x480     ///< RGB 480p\n"
            "------------------------------\n"
            "enter select:"
        };
        int cmd = requestCmdAllPlatform(meun_resolution, sizeof(meun_resolution));
        if (cmd < 1 || cmd > 6)
        {
            cmd = -1;
        }
        else
        {
            cmd -= 1;
        }
        return cmd;
    }

    static void setColorCameraParas(const std::shared_ptr<xv::Device> device)
    {
        static ColorCameraParas paras;
        bool isContinued = true;
        while (isContinued)
        {
            int cmd = mainMenu();
            switch (cmd)
            {
            case 1:
            {
                int resolutionCmd = resolutionMenu();
                if (resolutionCmd != -1)
                {
                    paras.setResolution(static_cast<xv::ColorCamera::Resolution>(resolutionCmd));
                }
                break;
            }
            case 0:
                isContinued = false;
            default:
                break;
            }
        }
        if(enable_output_log){
            std::cout << "Color camera resolution:" << static_cast<int>(paras.getResolution()) << std::endl;
        }
        bool result = device->colorCamera()->setResolution(paras.getResolution());
    }
};

void startRGBDFunction(const std::shared_ptr<xv::Device> device, int& imuId, int& rgbId,
    int& tofId, int& tofRgbdId)
{
    device->orientationStream()->unregisterCallback(imuId) ? imuId = -1 : imuId;
    const std::shared_ptr<xv::TofCamera> tofCamera = device->tofCamera();
    const std::shared_ptr<xv::ColorCamera> colorCamera = device->colorCamera();
    if (tofCamera && colorCamera)
    {
        ColorCameraParasSettingMenu::setColorCameraParas(device);

        if (rgbId != -1) {
            device->colorCamera()->stop();
            device->colorCamera()->unregisterCallback(rgbId);
            rgbId = -1;
        }
        if (tofId != -1) {
            device->tofCamera()->stop();
            device->tofCamera()->unregisterCallback(tofId);
            tofId = -1;
        }
        if(tofRgbdId != -1){
            device->tofCamera()->stop();
            device->tofCamera()->unregisterColorDepthImageCallback(tofRgbdId);
            tofRgbdId = -1;
        }
        if (rgbId == -1)
        {
            // this step must be
            rgbId = colorCamera->registerCallback(colorCameraCallback);
        }
        colorCamera->start();

        setTofParas(device);
        if (tofId == -1)
        {
            tofId = tofCamera->registerCallback(TOFCallBackFun::tofCallback);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        tofCamera->start();
        if (tofRgbdId == -1)
        {
            tofRgbdId = tofCamera->registerColorDepthImageCallback(TOFCallBackFun::colorDepthImageCallback);
        }
    }
}

void deviceStatusCallback(const std::vector<unsigned char>& deviceStatus)
{
    if(deviceStatus.size() < 42){
        std::cout << "device status size error!" << std::endl;
        return;
    }

    if(!enable_output_log) {
        return;
    }

    std::cout << "temperature: ";
    for(int i = 11; i < 17; i ++){
        std::cout << (int)deviceStatus[i] << " ";
    }

    std::cout << " cpu_temp: " << (int)deviceStatus[17] << "  ";

    std::cout << "fan: ";
    for(int i = 18; i < 29; i ++){
        std::cout << (int)deviceStatus[i] << " ";
    }

    std::cout << " soft_reset: " << (int)deviceStatus[29] << "  ";
    std::cout << "freq: ";
    for(int i = 30; i < 34; i ++){
        std::cout << (int)deviceStatus[i] << " ";
    }
    std::cout << "rgb: " << (int)deviceStatus[34] << "  ";
    std::cout << "fe: " << (int)deviceStatus[35] << "  ";
    std::cout << "tof: " << (int)deviceStatus[36] << "  ";
    std::cout << "uac_speak: " << (int)deviceStatus[37] << "  ";
    std::cout << "uac_mic: " << (int)deviceStatus[38] << "  ";
    std::cout << "audio_speak: " << (int)deviceStatus[39] << "  ";
    std::cout << "audio_mic: " << (int)deviceStatus[40] << "  ";
    std::cout << "dp: " << (int)deviceStatus[41] << "  ";
    std::cout << "panel: " << (int)deviceStatus[42] << "  ";
    std::cout << "\n";
}

void gpsDataCallback(const std::vector<unsigned char>& gpsData)
{
    printf("gps data size: %d\n", gpsData.size());
    for (int i = 0; i < gpsData.size(); i++)
    {
        printf(" %x", gpsData[i]);
    }
    printf("\n");
}

void gpsDistanceDataCallback(const xv::GPSDistanceData &gpsDistanceData)
{
    printf("gps distance: %d, signal: %d\n", gpsDistanceData.distance, gpsDistanceData.signal);
}

void feDewarpCallback(xv::FisheyeImages const & stereo)
{
    static FpsCount fc;
    fc.tic();
    static int k = 0;
    if(k++%50==0){
        std::cout << "stereo dewarp "  << timeShowStr(stereo.edgeTimestampUs, stereo.hostTimestamp) << stereo.images[0].width << "x" << stereo.images[0].height << "@" << std::round(fc.fps()) << "fps" << std::endl;
    }
}

void STMDataCallback(const xv::TerrestrialMagnetismData &stmData)
{
    std::cout << "STM offset data: " << stmData.offset[0] << ", " << stmData.offset[1] << ", " << stmData.offset[2] << std::endl;
    std::cout << "STM angles data: " << stmData.angles[0] << ", " << stmData.angles[1] << ", " << stmData.angles[2] << std::endl;
    std::cout << "STM magnetic data: " << stmData.magnetic[0] << ", " << stmData.magnetic[1] << ", " << stmData.magnetic[2] << std::endl;
    std::cout << "STM level: " << stmData.level << std::endl;
}

bool xvhandOpenCLEnvCheck()
{
#if defined __ANDROID__
    std::string envstr = getenv("LD_LIBRARY_PATH");
    if(enable_output_log)
    {
        std::cout << "LD_LIBRARY_PATH = " << envstr << std::endl;
    }
    auto idx0 = envstr.find("/vendor/lib64");
    auto idx1 = envstr.find("/system/vendor/lib64");

    if(idx0 == std::string::npos && idx1 == std::string::npos)
        return false;
#endif
    return true;
}

xv::FisheyeImages s_images;
std::mutex s_fe_mutex;
xv::Pose s_pose;
void Get4EyeTagDetection(xv::AprilTagDetector& detector)
{
    stop = false;
    tpos = std::thread ([&detector]{
        while (!stop) {
            auto t0 = std::chrono::steady_clock::now();
            std::this_thread::sleep_until(t0 + std::chrono::milliseconds(25));
            std::vector<xv::TagPose> tags = detector.detect(s_images, 0.16);
            std::cout << tags.size() << std::endl;
            for(auto p : tags)
            {
                auto tagPose = s_pose * p.transform;
                auto pitchYawRoll = xv::rotationToPitchYawRoll(tagPose.rotation());
                std::cout << "tag pose: " << tagPose.x() << "," << tagPose.y() << "," << tagPose.z() << "," << pitchYawRoll[0]*180/M_PI << "," << pitchYawRoll[1]*180/M_PI << "," << pitchYawRoll[2]*180/M_PI << std::endl;
            }
        }
        });
}

int main( int argc, char* argv[] ) try
{
    // may change the log level this way :
    //std::shared_ptr<xv::Device> device = nullptr;

    std::string json = "";
    if (argc == 2) {
        std::ifstream ifs(argv[1]);
        if (!ifs.is_open()) {
            if(enable_output_log){
                std::cerr << "Failed to open: " << argv[1] << std::endl;
            }
            return -1;
        }

        std::stringstream fbuf;
        fbuf << ifs.rdbuf();
        json = fbuf.str();
    }

    static const char meun_main[] = {
        "\n\nDemo\n"
        "------------------------------\n"
        "1 : Init SDK and get IMU\n"
        "2 : Stop get IMU, start slam, get 6dof\n"
        "3 : Stop slam, stop get 6dof, get IMU\n"
        "4 : Get eyetracking data, stop get IMU\n"
        "5 : Stop get eyetracking data, get IMU\n"
        "9 : Stop get IMU, get rgb data\n"
        "10: Stop rgb, get IMU\n"
        "11: Stop get IMU, get tof data\n"
        "12: Stop tof, get IMU\n"
        "13: Start tof plane detection, start slam, stop get IMU, get 6dof\n"
        "14: Stop tof plane detection, stop slam, get IMU\n"
        "15: Stop get IMU, switch to edge mode, get edge 6dof\n"
        "16: Stop get edge 6dof, switch to mixed mode, get IMU\n"
        "17: Test display: open display,set brightness to 2,set brightness to 9\n"
        "18: Test display: close display\n"
        "19: Stop get IMU, start slam, get 6dof\n"
        "20: Save shared map\n"
        "21: Stop slam, start CSLAM using shared map, start slam\n"
        "22: Stop slam, get IMU\n"
        "23: Start stereo plane detection, start slam, stop get IMU, get 6dof\n"
        "24: Stop stereo plane detection, stop slam, get IMU\n"
        "25: Stop get IMU, switch to EdgeFusionOnHost mode, get 6dof with callback\n"
        "26: Stop get 6dof with callback, stop EdgeFusionOnHost slam, get IMU\n"
        "27: Stop get IMU, switch to EdgeFusionOnHost mode, get 6dof with get-pose\n"
        "28: Stop get 6dof with get-pose, stop EdgeFusionOnHost slam, get IMU\n"
        "29: Call 3Dof get()\n"
        "30: Stop call 3Dof get(), get IMU\n"
        "31: Call 3Dof getAt()\n"
        "32: Stop call 3Dof getAt(), get IMU\n"
        "33: Stop get IMU, switch to mixed mode, get 6dof with callback\n"
        "34: Stop get 6dof with callback, stop mix slam, get IMU\n"
        "35: Stop get IMU, switch to mixed mode, get 6dof with get-pos-at\n"
        "36: Stop get 6dof with get-pos-at, stop mix slam, get IMU\n"
        "37: Stop get IMU, read rgb calibration\n"
        "38: Stop get IMU, set rgb resolution\n"
        "39: Stop get rgb data, get IMU\n"
        "40: Stop get IMU, set rbg format\n"
        "41: Stop get IMU, set rbg exposure mode, white balance control, ISO level, EXP level\n"
        "42: Stop get IMU, set eyetracking exposure, gain\n"
        "43: Stop get IMu, set eyetracking led control\n"
        "44: Stop get IMU, read fisheye calibration\n"
        "45: Stop get IMU, read TOF calibration\n"
        "46: Stop get IMU, set TOF stream mode\n"
        "47: Stop get IMU, set TOF distance mode\n"
        "48: Stop get IMU, set event data\n"
        "49: Stop get IMU, set CNN data\n"
        "50: Stop get IMU, get gesture stream data.Warning: must first specify the path of opencl,export LD_LIBRARY_PATH=/xxxx/:/vendor/lib64/\n"
        "51: Stop get gesture stream data, get IMU\n"
        "52: Stop get IMU, get dynamic gesture stream data.Warning: must first specify the path of opencl,export LD_LIBRARY_PATH=/xxxx/:/vendor/lib64/\n"
        "53: Stop get dynamic gesture stream data, get IMU\n"
        "54: Stop get IMU, get gesture keypoints.Warning: must first specify the path of opencl,export LD_LIBRARY_PATH=/xxxx/:/vendor/lib64/\n"
        "55: Stop get gesture keypoints, get IMU\n"
        "56: Stop get IMU, get gesture keypoints based on slam.Warning: must first specify the path of opencl,export LD_LIBRARY_PATH=/xxxx/:/vendor/lib64/\n"
        "57: Stop get gesture keypoints based on slam, get IMU\n"
        "59: Get display calibration\n"
        "60: Camera on/off switch\n"
        "61: Start april tag detection\n"
        "62: Stop april tag detection, get IMU\n"
        "63: Start surface callback\n"
        "64: Stop surface callback, get IMU\n"
        "65: Start event callback\n"
        "66: Stop event callback, get IMU\n"
        "67: Stop get IMU, start SGBM\n"
        "68: Stop SGBM, get IMU\n"
        "69: Stop get IMU, start get gaze data\n"
        "77: Start get device status data\n"
        "78: Stop get device status data, get IMU\n"
        "79: Stop get IMU, switch the fisheye resolution to HIGH\n"
        "80: Stop get IMU, switch the fisheye resolution to MEDIUM\n"
        "81: Stop get fisheye data, get IMU\n"
        "82: Start RGBD\n"
        "83: Stop RGBD, get IMU\n"
        "84: Change FE framerate into 50Hz\n"
        "85: Change FE framerate into 60Hz\n"
        "86: Change RGB to AF/MF mode\n"
        "87: Change RGB local distance\n"
        "88: Start get device status data with SLAM\n"
        "89: Stop get device status data and SLAM\n"
        "90: Start get device status data without IMU\n"
        "99: Enable/Disable output log\n"
        "100: Start gps callback\n"
        "101: Stop gps callback\n"
        "102: Start gps distance callback\n"
        "103: Stop gps distance callback\n"
        "104: Start fe dewarp callback\n"
        "105: Stop fe dewarp callback\n"
        "109: Start STM callback\n"
        "110: Stop STM callback\n"
        "111: Get four eye apriltag\n"
        "112: Get QRcode apriltag\n"
        "0 : exit program\n"
        "------------------------------\n"
        "enter select:"
    };


    int cond = 1;
    int cmd = -1;
    int lastcmd = 0;
    int recvcnt = 0;
    int cnnId = -1;
    int eventId = -1;
    int iFisheyeId = -1;
    int iFisheyeLId = -1;
    int iFisheyeRId = -1;
    int imuId = -1;
    int poseId = -1;
    int rgbId = -1;
    int rgb2Id = -1;
    int tofId = -1;
    int planeId = -1;
    int stereoId = -1;
    int eyetrackingId = -1;
    int tofRgbdId = -1;
    int gestureId = -1;
    int dynamicgestureId = -1;
    int keypointsId = -1;
    int slamkeypointsId = -1;
    int sgbmId = -1;
    int gazeCallbackId = -1;
    int GesturePosEXId = -1;
    int deviceStatusId = -1;
    int gpsDataId = -1;
    int gpsDistanceDataId = -1;
    int feDewarpId = -1;
    int enrollDataId = -1;
    int identifyDataId = -1;
    int STMDataId = -1;
    int objDetRKNN3588Id = -1;
    int rgbLThermalFusionCallbackID = -1;
    int rgbRThermalFusionCallbackID = -1;
    int irCallbackID1 = -1;
    int irCallbackID2 = -1;


#ifdef _WIN32
    std::string cCmd = "";

    std::cout << meun_main << std::endl;
#else
    int retval = vsc_client_pipe_init();
    if (retval != 0) {
        printf("client pipe init fail %d\n", retval);
        return retval;
    }
    vsc_client_pipe_get_srv_pid();
    signal(SIGINT, cam_sig_handler);
#endif
    while (cond) {
#ifdef _WIN32
        std::cin >> cCmd;
        if (cCmd == "m")
        {
            if (device)
            {
                //stop all running items
                if (device->display())
                {
                    device->display()->close();
                }

                if (device->orientationStream())
                {
                    //device->orientationStream()->stop();
                    device->orientationStream()->unregisterCallback(imuId);
                }

                if (device->eyetracking())
                {
                    //device->eyetracking()->stop();
                    device->eyetracking()->unregisterCallback(eyetrackingId);
                }

                if (device->colorCamera())
                {
                    //device->colorCamera()->stop();
                    device->colorCamera()->unregisterCallback(rgbId);
                }

                if (device->tofCamera())
                {
                    //device->tofCamera()->stop();
                    device->tofCamera()->unregisterCallback(tofId);
                }

                if (device->slam())
                {
                    stopGetPose();
                    //device->slam()->stop();
                    device->slam()->unregisterCallback(poseId);
                    device->slam()->unregisterTofPlanesCallback(planeId);
                    device->slam()->unregisterStereoPlanesCallback(stereoId);
                }
            }
            printf("show menu\n");
            std::cout << meun_main << std::endl;
            continue;
        }

        cmd = atoi(cCmd.c_str());;
#else
        cmd = requestCmdAllPlatform(meun_main, sizeof(meun_main));
#endif
        if (cmd == 0) {
            printf("program will exit\n");
            cond = 0;
            break;
        }
        // else if (lastcmd == cmd) {
        //     recvcnt++;
        //     if (recvcnt > 5) {
        //         printf("client lose disconnect \n");
        //         cond = 0;
        //     }
        // }
        lastcmd = cmd;

        printf("cmd: %d\n", cmd);
        switch (cmd) {
        case 1:
        {
            if(enable_output_log){
                std::cout << "xvsdk version: " << xv::version() << std::endl;
            }
            const char slamMode[] = {
                "0: Normal\n"
                "1: vision only\n"
                "2: vision with gyro\n"
                "------------------------------\n"
                "enter select:"
            };
            slamStartMode = requestCmdAllPlatform(slamMode, sizeof(slamMode));

            // return a map of devices with serial number as key, wait at most x seconds if no device detected
            auto devices = xv::getDevices(10., json, nullptr, xv::SlamStartMode(slamStartMode));

            // if no device: quit
            if (devices.empty()) {
                if(enable_output_log){
                    std::cerr << "Timeout for device detection." << std::endl;
                }
                return EXIT_FAILURE;
            }

            // take the first device in the map
            device = devices.begin()->second;

            xv::setLogLevel(xv::LogLevel(2));
//            xv::registerPlugEventCallback([&](std::shared_ptr<xv::Device> d, xv::PlugEventType type){
//                if(d->id() != device->id()){
//                    return;
//                }
//                if(type == xv::PlugEventType::Plugin){
//                    if(start3DofGet && stop){
//                        Start3DofGet(device->orientationStream());
//                    }else if(start3DofGetAt && stop){
//                        Start3DofGetAt(device->orientationStream());
//                    }
//                }else {
//                    if(start3DofGet){
//                        Stop3DofGet();
//                    }else if(start3DofGetAt){
//                        Stop3DofGetAt();
//                    }
//                }
//            });
            if (!device->slam()) {
                if(enable_output_log){
                    std::cerr << "Host SLAM algorithm not supported." << std::endl;
                }
                return EXIT_FAILURE;
            }

            // get IMU
            device->orientationStream()->start();
            imuId = device->orientationStream()->registerCallback(orientationCallback);
            TOFCallBackFun::m_tofCamera = device->tofCamera();
            SGBMCallback::sgbmCamera = device->sgbmCamera();
        }
        break;
        case 2:
        case 19:
        {
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            if (device->fisheyeCameras()) {
                iFisheyeId = device->fisheyeCameras()->registerCallback([](xv::FisheyeImages const& images) {});
            }

            // start mix slam
            device->slam()->start(xv::Slam::Mode::Mixed);

            // get mixed 6dof
            startGetPose(device->slam());
            break;
        }
        case 3:
        case 22:
        case 28:
        case 36:
            // Stop get mixed 6dof
            stopGetPose();

            // stop mix slam
            device->slam()->stop();

            // get IMU
            imuId = device->orientationStream()->registerCallback(orientationCallback);

            break;
        case 4:
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            // Get eyetracking data
            device->eyetracking()->start();
            eyetrackingId = device->eyetracking()->registerCallback(eyetrackingCallback);

            break;
        case 5:
            // Stop get eyetracking data
            device->eyetracking()->stop();
            device->eyetracking()->unregisterCallback(eyetrackingId);

            // get IMU
            imuId = device->orientationStream()->registerCallback(orientationCallback);

            break;
        case 9:
        {
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
                imuId = -1;
            }
            const char meun_rgb[] = {
                "0: Start RGB stream\n"
                "1: Start RGB1 stream\n"
                "------------------------------\n"
                "enter select:"
            };

            int nRgbCmd = -1;

            nRgbCmd = requestCmdAllPlatform(meun_rgb, sizeof(meun_rgb));

            if(nRgbCmd == 0){
                // Get rgb data
                device->colorCamera()->start();
                rgbId = device->colorCamera()->registerCallback(rgbCallback);
            }else if(nRgbCmd == 1){
                device->colorCamera()->startCameras();
                rgb2Id = device->colorCamera()->registerCam2Callback(rgb2Callback);
            }else {
                if(enable_output_log){
                    std::cout << "bad command\n" << std::endl;
                }
            }

            break;
        }
        case 10:
        case 39:
        {
            const char meun_rgb[] = {
                "0: Stop RGB stream\n"
                "1: Stop RGB1 stream\n"
                "------------------------------\n"
                "enter select:"
            };

            int nRgbCmd = -1;

            nRgbCmd = requestCmdAllPlatform(meun_rgb, sizeof(meun_rgb));

            if(nRgbCmd == 0){
                // Stop get rgb data
                if (device->colorCamera()) {
                    device->colorCamera()->stop();
                    device->colorCamera()->unregisterCallback(rgbId);
                }
            }else if(nRgbCmd == 1){
                if (device->colorCamera()) {
                    device->colorCamera()->stopCameras();
                    device->colorCamera()->unregisterCam2Callback(rgb2Id);
                }
            }else {
                if(enable_output_log){
                    std::cout << "bad command\n" << std::endl;
                }
            }

            // get IMU
            if(imuId == -1){
                imuId = device->orientationStream()->registerCallback(orientationCallback);
            }

            break;
        }
        case 11:
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }
            // set tof paras
            if (tofId != -1)
            {
                device->tofCamera()->unregisterCallback(tofId);
                tofId = -1;
            }
            device->tofCamera()->start();
            setTofParas(device);
            tofId = device->tofCamera()->registerCallback(TOFCallBackFun::tofCallback);
            break;
        case 12:
            // Stop get tof data
            if (device->tofCamera()) {
                device->tofCamera()->stop();
                device->tofCamera()->unregisterCallback(tofId);
            }


            // get IMU
            imuId = device->orientationStream()->registerCallback(orientationCallback);

            break;
        case 13:
        {
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            if (device->fisheyeCameras()) {
                iFisheyeId = device->fisheyeCameras()->registerCallback([](xv::FisheyeImages const& images) {});
            }

            auto slam = device->slam();
            auto slamEx = dynamic_cast<xv::SlamEx*>(slam.get());
            //slamEx->setEnableSurface(true);
            //slamEx->setEnableSurfacePlanes(true);            // Get plane data
            // Get plane data
            device->tofCamera()->start();
            // !!! Must call registerTofPlanesCallback before slam()->start
            planeId = device->slam()->registerTofPlanesCallback(planeCallback);

            // start mix slam
            device->slam()->start(xv::Slam::Mode::Mixed);

            // get mixed 6dof
            startGetPose(device->slam());

            break;
        }
        case 14:
            // stop mix slam
            device->slam()->stop();

            // Stop get mixed 6dof
            stopGetPose();

            // Stop get plane data
            if (device->tofCamera()) {
                device->tofCamera()->stop();
            }
            device->slam()->unregisterTofPlanesCallback(planeId);

            // get IMU
            imuId = device->orientationStream()->registerCallback(orientationCallback);

            break;
        case 15:
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            // switch to edge mode
            device->slam()->start(xv::Slam::Mode::Edge);

            // get edge 6dof
            poseId = device->slam()->registerCallback(poseCallback);

            break;
        case 16:
        case 26:
        case 34:
            // stop get edge 6dof
            device->slam()->unregisterCallback(poseId);

            // stop slam
            device->slam()->stop();

            // get IMU
            imuId = device->orientationStream()->registerCallback(orientationCallback);

            break;
        case 17:
        {
            device->display()->open();
            std::this_thread::sleep_for(std::chrono::seconds(3));
            if(enable_output_log){
                std::cout << "set brightness level to 2" << std::endl;
            }
            bool bok = device->display()->setBrightnessLevel(2);
            if (bok)
            {
                if(enable_output_log){
                    std::cout << "set brightness level to 2 succeed" << std::endl;
                }
            }
            else {
                if(enable_output_log){
                    std::cout << "set brightness level to 2 failed" << std::endl;
                }
            }
            std::this_thread::sleep_for(std::chrono::seconds(3));
            if(enable_output_log){
                std::cout << "set brightness level to 9" << std::endl;
            }
            bok = device->display()->setBrightnessLevel(9);
            if (bok)
            {
                if(enable_output_log){
                    std::cout << "set brightness level to 9 succeed" << std::endl;
                }
            }
            else {
                if(enable_output_log){
                    std::cout << "set brightness level to 9 failed" << std::endl;
                }
            }
            break;
        }
        case 18:
            device->display()->close();
            break;
        case 20:
            // save shared map
            //slam->stopSlamAndSaveMap(map_shared_filename);
            if (mapStream.open(map_filename, std::ios::binary | std::ios::out | std::ios::trunc) == nullptr) {
                if(enable_output_log){
                    std::cout << "open " << map_filename << " failed." << std::endl;
                }
                break;
            }
            device->slam()->saveMapAndSwitchToCslam(mapStream, cslamSavedCallback, cslamLocalizedCallback);

            break;
        case 21:
            // Stop get IMU
            if (device->orientationStream()) {
                device->orientationStream()->unregisterCallback(imuId);
            }
            // stop mix slam
            if(enable_output_log){
                std::cout << "stop slam" << std::endl;
            }
            device->slam()->stop();

            if (device->fisheyeCameras()) {
                iFisheyeId = device->fisheyeCameras()->registerCallback([](xv::FisheyeImages const& images) {});
            }

            //std::this_thread::sleep_for( std::chrono::seconds(1) );

            // start mix slam
            if(enable_output_log){
                std::cout << "start slam" << std::endl;
            }
            device->slam()->start(xv::Slam::Mode::Mixed);

            // start cslam using shared map
            if(enable_output_log){
                std::cout << "load cslam map and switch to cslam" << std::endl;
            }
            if (mapStream.open(map_filename, std::ios::binary | std::ios::in) == nullptr) {
                if(enable_output_log){
                    std::cout << "open " << map_filename << " failed." << std::endl;
                }
                break;
            }
            device->slam()->loadMapAndSwitchToCslam(
                mapStream,
                cslamSwitchedCallback,
                cslamLocalizedCallback
            );

            break;
        case 23:
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            if (device->fisheyeCameras()) {
                iFisheyeId = device->fisheyeCameras()->registerCallback([](xv::FisheyeImages const& images) {});
            }

            // Get plane data
            // !!! Must call registerStereoPlanesCallback before slam()->start
            planeId = device->slam()->registerStereoPlanesCallback(planeCallback);

            // start mix slam
            device->slam()->start(xv::Slam::Mode::Mixed);

            // get mixed 6dof
            startGetPose(device->slam());

            break;
        case 24:
            // stop mix slam
            device->slam()->stop();

            // Stop get mixed 6dof
            stopGetPose();

            // Stop get plane data
            device->slam()->unregisterStereoPlanesCallback(planeId);

            // get IMU
            imuId = device->orientationStream()->registerCallback(orientationCallback);

            break;
        case 25:
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            if (device->fisheyeCameras()) {
                iFisheyeId = device->fisheyeCameras()->registerCallback([](xv::FisheyeImages const& images) {});
            }

            // switch to EdgeFusionOnHost mode
            device->slam()->start(xv::Slam::Mode::EdgeFusionOnHost);

            // get EdgeFusionOnHost 6dof with callback
            poseId = device->slam()->registerCallback(poseCallback);

            break;
        case 27:
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            if (device->fisheyeCameras()) {
                iFisheyeId = device->fisheyeCameras()->registerCallback([](xv::FisheyeImages const& images) {});
            }

            // start EdgeFusionOnHost slam
            device->slam()->start(xv::Slam::Mode::EdgeFusionOnHost);

            // get EdgeFusionOnHost 6dof with get-pose
            startGetPose(device->slam());

            break;
        case 29:
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            //stop getAt 3dof
            Stop3DofGetAt();

            //call get 3dof
            Start3DofGet(device->orientationStream());
            break;
        case 30:
            //stop get 3dof
            Stop3DofGet();
            start3DofGet = false;
            // get IMU
            imuId = device->orientationStream()->registerCallback(orientationCallback);
            break;
        case 31:
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            //stop get 3dof
            Stop3DofGet();

            //call getAt 3dof
            Start3DofGetAt(device->orientationStream());
            break;
        case 32:
            //stop getAt 3dof
            Stop3DofGetAt();
            start3DofGetAt = false;
            // get IMU
            imuId = device->orientationStream()->registerCallback(orientationCallback);
            break;
        case 33:
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            if (device->fisheyeCameras()) {
                iFisheyeId = device->fisheyeCameras()->registerCallback([](xv::FisheyeImages const& images) {});
            }

            // switch to Mixed mode
            device->slam()->start(xv::Slam::Mode::Mixed);

            // get Mixed 6dof with callback
            poseId = device->slam()->registerCallback(poseCallback);
            break;
        case 35:
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            if (device->fisheyeCameras()) {
                iFisheyeId = device->fisheyeCameras()->registerCallback([](xv::FisheyeImages const& images) {});
            }

            // switch to Mixed mode
            device->slam()->start(xv::Slam::Mode::Mixed);

            // get Mixed 6dof with get-pos-at
            startGetPoseAt(device->slam());
            break;
        case 37:
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            // Stop get rgb data
            if (device->colorCamera()) {
                device->colorCamera()->unregisterCallback(rgbId);
            }

            // Get rgb calibration
            if(enable_output_log){
                std::cout << "RGB calibration:" << std::endl;
                std::cout << device->colorCamera()->calibration() << std::endl;
            }

            break;
        case 38:
        {
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
                imuId = -1;
            }

            device->colorCamera()->start();

            // Stop get rgb data
            if (device->colorCamera()) {
                device->colorCamera()->unregisterCallback(rgbId);
            }

            const char meun_rgbResolution[] = {
                "0: RGB_1920x1080   ///< RGB 1080p\n"
                "1: RGB_1280x720    ///< RGB 720p\n"
                "2: RGB_640x480     ///< RGB 480p\n"
                "3: RGB_320x240     ///< RGB QVGA(not supported now)\n"
                "4: RGB_2560x1920   ///< RGB 5m(not supported now)\n"
                "5: RGB_3840x2160   ///< RGB 8M\n"
                "------------------------------\n"
                "enter select:"
            };

            int nRgbControlCmd = -1;

            nRgbControlCmd = requestCmdAllPlatform(meun_rgbResolution, sizeof(meun_rgbResolution));
            SetRgbResolution(nRgbControlCmd, device->colorCamera());

            rgbId = device->colorCamera()->registerCallback(rgbCallback);
        }
        break;
        case 40:
        {
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            // Stop get rgb data
            if (device->colorCamera()) {
                device->colorCamera()->unregisterCallback(rgbId);
            }

            int nRgbFormat = -1;

            xv::DeviceSetting setting = { 0x03030046 };

            const char meun_rgbFormat[] = {
                "0: YUYV\n"
                "1: YUV420p\n"
                "2: JPEG\n"
                "3: NV12\n"
                "4: H265\n"
                "------------------------------\n"
                "enter select:"
            };

            nRgbFormat = requestCmdAllPlatform(meun_rgbFormat, sizeof(meun_rgbFormat));
            if (nRgbFormat == 2)
            {
                break;
            }else if (nRgbFormat == 0 || nRgbFormat == 1)
            {
                nRgbFormat = nRgbFormat + 1;
            }
            setting.args.val[0] = nRgbFormat;

            bool bOk = (nRgbFormat <= 4) && device->control(setting);
            if(enable_output_log){
                if (bOk)
                {
                    std::cout << "RGB format setting successfully" << std::endl;
                }
                else {
                    std::cout << "RGB format setting failed" << std::endl;
                }
            }
        }
        break;
        case 41:
        {
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            // Stop get rgb data
            if (device->colorCamera()) {
                device->colorCamera()->unregisterCallback(rgbId);
            }

            xv::DeviceSetting setting = { 0x00000000 };
            int nRgbControl = -1, nExpMode = -1, nAWB = -1, nISO = -1, nEXP = -1;

            const char meun_rgbControl[] = {
                "0: exposure mode\n"
                "1: white balance\n"
                "2: ISO level\n"
                "3: EXP level\n"
                "------------------------------\n"
                "enter select:"
            };

            nRgbControl = requestCmdAllPlatform(meun_rgbControl, sizeof(meun_rgbControl));

            switch (nRgbControl)
            {
            case 0:
            {
                const char meun_rgbExpMode[] = {
                    "0: auto exposure\n"
                    "1: manual exposure\n"
                    "------------------------------\n"
                    "enter select:"
                };
                nExpMode = requestCmdAllPlatform(meun_rgbExpMode, sizeof(meun_rgbExpMode));
                setting = { 0x03030005 };
                setting.args.exp.exp_mode = nExpMode;
            }
            break;
            case 1:
            {
                const char meun_rgbAWB[] = {
                "0: AWB_OFF\n"
                "1: AWB_AUTO\n"
                "2: AWB_INCAN\n"
                "3: AWB_FLOUR\n"
                "4: AWB_WARM_FLOUR\n"
                "5: AWB_DAYLIGHT\n"
                "6: AWB_CLOUDY\n"
                "7: AWB_TWILIGHT\n"
                "8: AWB_SHADOW\n"
                "------------------------------\n"
                "enter select:"
                };
                nAWB = requestCmdAllPlatform(meun_rgbAWB, sizeof(meun_rgbAWB));
                setting = { 0x03030007 };
                setting.args.awb.awb_mode = nAWB;
            }
            break;
            case 2:
            {
                const char meun_rgbISO[] = {
                "iso level: 1-16 (100-1600)\n"
                "------------------------------\n"
                "enter select:"
                };
                nISO = requestCmdAllPlatform(meun_rgbISO, sizeof(meun_rgbISO));
                setting = { 0x03030022 };
                setting.args.exp.iso_mode = nISO;
            }
            break;
            case 3:
            {
                const char meun_rgbEXP[] = {
                "exp level: -9 ~ 9"
                "------------------------------\n"
                "enter select:"
                };
                nEXP = requestCmdAllPlatform(meun_rgbEXP, sizeof(meun_rgbEXP));
                setting = { 0x03030006 };
                setting.args.exp.exp_level = nEXP;
            }
            break;
            default:
                break;
            }
            bool bOk = device->control(setting);
            if(enable_output_log){
                if (bOk)
                {
                    std::cout << "RGB setting successfully" << std::endl;
                }
                else {
                    std::cout << "RGB setting failed" << std::endl;
                }
            }

        }
        break;
        case 42:
        {
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            int nLeftGain = -1, nRightGain = -1, nLeftTime = -1, nRightTime = -1;
            const char meun_EyeTrackingLeftGain[] = {
                    "Left eye exposure gain, [0, 255]\n"
                    "------------------------------\n"
                    "enter select:"
            };
            nLeftGain = requestCmdAllPlatform(meun_EyeTrackingLeftGain, sizeof(meun_EyeTrackingLeftGain));

            const char meun_EyeTrackingLeftTime[] = {
                    "Left eye exposure time, in milliseconds\n"
                    "------------------------------\n"
                    "enter select:"
            };
            nLeftTime = requestCmdAllPlatform(meun_EyeTrackingLeftTime, sizeof(meun_EyeTrackingLeftTime));

            const char meun_EyeTrackingRightGain[] = {
                    "Right eye exposure gain, [0, 255]\n"
                    "------------------------------\n"
                    "enter select:"
            };
            nRightGain = requestCmdAllPlatform(meun_EyeTrackingRightGain, sizeof(meun_EyeTrackingRightGain));

            const char meun_EyeTrackingRightTime[] = {
                    "Right eye exposure time, in milliseconds\n"
                    "------------------------------\n"
                    "enter select:"
            };
            nRightTime = requestCmdAllPlatform(meun_EyeTrackingRightTime, sizeof(meun_EyeTrackingRightTime));

            bool bOk = device->eyetracking()->setExposure(nLeftGain, nLeftTime, nRightGain, nRightTime);
            if(enable_output_log){
                if (bOk)
                {
                    std::cout << "Eyetracking exposure setting successfully" << std::endl;
                }
                else {
                    std::cout << "Eyetracking exposure setting failed" << std::endl;
                }
            }
        }
        break;
        case 43:
        {
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            int nEye = -1, nLed = -1, nBright = -1;
            const char meun_EyeTrackingLed[] = {
                    "0: left\n"
                    "1: right\n"
                    "2: both\n"
                    "------------------------------\n"
                    "enter select:"
            };
            nEye = requestCmdAllPlatform(meun_EyeTrackingLed, sizeof(meun_EyeTrackingLed));

            const char meun_EyeTrackingLedCtrl[] = {
                    "[0,7]:led index, 8:all\n"
                    "------------------------------\n"
                    "enter select:"
            };
            nLed = requestCmdAllPlatform(meun_EyeTrackingLedCtrl, sizeof(meun_EyeTrackingLedCtrl));

            const char meun_EyeTrackingBright[] = {
                    "[0,255], 0 is off\n"
                    "------------------------------\n"
                    "enter select:"
            };
            nBright = requestCmdAllPlatform(meun_EyeTrackingBright, sizeof(meun_EyeTrackingBright));


            bool bOk = device->eyetracking()->setLedBrighness(nEye, nLed, nBright);
            if(enable_output_log){
                if (bOk)
                {
                    std::cout << "Eyetracking setting successfully" << std::endl;
                }
                else {
                    std::cout << "Eyetracking setting failed" << std::endl;
                }
            }
        }
        break;
        case 44:
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            // Stop get Fisheye data
            device->fisheyeCameras()->unregisterCallback(rgbId);

            // Get Fisheye calibration
            if(enable_output_log){
                std::cout << "Fisheye calibration:" << std::endl;
                std::cout << std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->calibrationEx() << std::endl;
            }
            break;
        case 45:
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            // Get TOF calibration
            if(enable_output_log){
                std::cout << "TOF calibration:" << std::endl;
                std::cout << device->tofCamera()->calibration() << std::endl;
            }
            break;
        case 46:
        {
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            int nStreamMode = -1;
            const char meun_TOFStreamMode[] = {
                    "0: DepthOnly\n"
                    "1: CloudOnly\n"
                    "2: DepthAndCloud\n"
                    "3: none\n"
                    "4: CloudOnLeftHandSlam\n"
                    "------------------------------\n"
                    "enter select:"
            };
            nStreamMode = requestCmdAllPlatform(meun_TOFStreamMode, sizeof(meun_TOFStreamMode));

            bool bOk = SetTofStreamMode(nStreamMode, device->tofCamera());
            if(enable_output_log){
                if (bOk)
                {
                    std::cout << "Tof distance setting successfully" << std::endl;
                }
                else {
                    std::cout << "Tof distance setting failed" << std::endl;
                }
            }
        }
        break;
        case 47:
        {
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            int nDisMode = -1;
            const char meun_TOFDistance[] = {
                    "0: short\n"
                    "1: middle(Midlle=Short for 010/009 TOF)\n"
                    "2: long\n"
                    "------------------------------\n"
                    "enter select:"
            };
            nDisMode = requestCmdAllPlatform(meun_TOFDistance, sizeof(meun_TOFDistance));

            bool bOk = SetTofDistanceMode(nDisMode, device->tofCamera());
            if(enable_output_log){
                if (bOk)
                {
                    std::cout << "Tof distance setting successfully" << std::endl;
                }
                else {
                    std::cout << "Tof distance setting failed" << std::endl;
                }
            }
        }
        break;
        case 48:
        {
            if (device == nullptr)
            {
                if(enable_output_log){
                    std::cerr << "Device is nullptr! Please init first..." << std::endl;
                }
                break;
            }
            device->eventStream()->unregisterCallback(imuId);

            if (device->orientationStream()) {
                device->orientationStream()->unregisterCallback(imuId);
                device->orientationStream()->stop();
            }
            // Stop get rgb data if colorCamera is
            if (device->colorCamera())
            {
                device->colorCamera()->unregisterCallback(rgbId);
            }
            if (device->imuSensor())
            {
                device->imuSensor()->unregisterCallback(imuId);
            }
            // get event data
            const char alsMode[] = {
                "0: common event\n"
                "1: als\n"
                "2: exit\n"
                "------------------------------\n"
                "enter select:"
            };
            bool isExit = false;
            // int iAlsMode = requestCmdAllPlatform(alsMode, sizeof(alsMode));
            while (!isExit)
            {
                int iAlsMode = requestCmdAllPlatform(alsMode, sizeof(alsMode));
                std::vector<unsigned char> vecWrite;
                std::vector<unsigned char> vecRead;
                bool ret;
                //open als
                if (iAlsMode == 0)
                {
                    vecWrite.push_back(0x02);
                    vecWrite.push_back(0xFD);
                    vecWrite.push_back(0x67);
                    vecWrite.push_back(0x00);
                    ret = device->hidWriteAndRead(vecWrite, vecRead);
                    if(enable_output_log){
                        if (!ret)
                            std::cerr << "close als error!" << std::endl;
                    }
                    //recover event catch
                    device->eventStream()->stop();
                    device->eventStream()->start();

                    if (eventId == -1)
                    {
                        eventId = device->eventStream()->registerCallback(eventCallback);
                    }
                }
                else if (iAlsMode == 1)
                {
                    vecWrite.push_back(0x02);
                    vecWrite.push_back(0xFD);
                    vecWrite.push_back(0x67);
                    vecWrite.push_back(0x01);
                    ret = device->hidWriteAndRead(vecWrite, vecRead);
                    if(enable_output_log){
                        if (!ret)
                            std::cerr << "open als error!" << std::endl;
                        else {
                            std::cout << "open als success!" << std::endl;
                            std::cout << "read data: " << std::endl;
                            for (int i = 0; i < vecRead.size(); i++)
                            {
                                std::cout << std::hex << (unsigned int)(unsigned char)vecRead.at(i) << " ";
                            }
                            std::cout << std::endl;
                        }
                    }
                    //recover event catch
                    device->eventStream()->stop();
                    device->eventStream()->start();

                    if (eventId == -1)
                    {
                        eventId = device->eventStream()->registerCallback(eventCallback);
                    }
                }
                else if (iAlsMode == 2)
                {
                    isExit = true;
                    vecWrite.push_back(0x02);
                    vecWrite.push_back(0xFD);
                    vecWrite.push_back(0x67);
                    vecWrite.push_back(0x00);
                    ret = device->hidWriteAndRead(vecWrite, vecRead);
                    if(enable_output_log){
                        if (!ret)
                            std::cerr << "close als error!" << std::endl;
                        else {
                            std::cout << "close als success!" << std::endl;
                        }
                    }
                    //recover event catch
                    device->eventStream()->stop();
                    device->eventStream()->start();
                    device->eventStream()->unregisterCallback(eventId);
                    eventId = -1;
                    device->orientationStream()->start();
                    imuId = device->orientationStream()->registerCallback(orientationCallback);
                }
            }
            break;
        }
        case 49:
        {
            if (device == nullptr)
            {
                if(enable_output_log){
                    std::cerr << "Device is nullptr! Please init first..." << std::endl;
                }
                break;
            }
            if (device->orientationStream()) {
                device->orientationStream()->unregisterCallback(imuId);
                device->orientationStream()->stop();
            }
            if (!device->objectDetector())
            {
                if(enable_output_log){
                    std::cerr << "No object detector" << std::endl;
                }
                break;
            }
            cnnId = device->objectDetector()->registerCallback(cnnCallback);
            device->objectDetector()->start();

            if (!device->objectDetector()->setModel("CNN_2x8x_r14_5.blob"))
                if(enable_output_log){
                    std::cerr << "***setModel error!" << std::endl;
                }
            if (!device->objectDetector()->setDescriptor("config_tensorflow.json"))
                if(enable_output_log){
                    std::cerr << "***setDescriptor error!" << std::endl;
                }

            const char meunCamera[] = {
                "1: camera source recognition\n"
                "2: switch to Left fisheye\n"
                "3: switch to Right fisheye\n"
                "4: switch to RGB\n"
                "5: switch to TOF\n"
                "0: Exit\n"
                "------------------------------\n"
                "enter select:"
            };
            bool isExit = false;
            while (!isExit)
            {
                int cameraCtrl = requestCmdAllPlatform(meunCamera, sizeof(meunCamera));
                switch (cameraCtrl) {
                case 1:
                {
                    xv::ObjectDetector::Source source = device->objectDetector()->getSource();
                    if (source == xv::ObjectDetector::Source::LEFT){
                        if(enable_output_log){
                            std::cout << "-----------------camera source: LEFT-----------------" << std::endl;
                        }
                    }
                    else if (source == xv::ObjectDetector::Source::RIGHT){
                        if(enable_output_log){
                            std::cout << "-----------------camera source: RIGHT-----------------" << std::endl;
                        }
                    }
                    else if (source == xv::ObjectDetector::Source::RGB){
                        if(enable_output_log){
                            std::cout << "-----------------camera source: RGB-----------------" << std::endl;
                        }
                    }
                    else if (source == xv::ObjectDetector::Source::TOF){
                        if(enable_output_log){
                            std::cout << "-----------------camera source: TOF-----------------" << std::endl;
                        }
                    }
                    break;
                }
                case 2:
                    if (device->colorCamera()) {
                        device->colorCamera()->stop();
                    }
                    if (device->tofCamera()) {
                        device->tofCamera()->stop();
                    }
                    if (device->objectDetector()) {
                        device->objectDetector()->setSource(xv::ObjectDetector::Source::LEFT);
                    }
                    if (device->fisheyeCameras()) {
                        device->fisheyeCameras()->unregisterCallback(iFisheyeLId);
                        device->fisheyeCameras()->unregisterCallback(iFisheyeRId);
                        iFisheyeLId = device->fisheyeCameras()->registerCallback(fisheyeLCallback);
                        device->fisheyeCameras()->start();
                    }
                    break;
                case 3:
                    if (device->colorCamera()) {
                        device->colorCamera()->stop();
                    }
                    if (device->tofCamera()) {
                        device->tofCamera()->stop();
                    }
                    if (device->objectDetector()) {
                        device->objectDetector()->setSource(xv::ObjectDetector::Source::RIGHT);
                    }
                    if (device->fisheyeCameras()) {
                        device->fisheyeCameras()->unregisterCallback(iFisheyeLId);
                        device->fisheyeCameras()->unregisterCallback(iFisheyeRId);
                        iFisheyeRId = device->fisheyeCameras()->registerCallback(fisheyeRCallback);
                        device->fisheyeCameras()->start();
                    }
                    break;
                case 4:
                    if (device->tofCamera()) {
                        device->tofCamera()->stop();
                    }
                    if (device->objectDetector()) {
                        device->objectDetector()->setSource(xv::ObjectDetector::Source::RGB);
                    }
                    if (device->colorCamera()) {
                        device->colorCamera()->unregisterCallback(rgbId);
                        rgbId = device->colorCamera()->registerCallback(rgbCallback);
                        device->colorCamera()->start();
                    }
                    break;
                case 5:
                    if (device->colorCamera()) {
                        device->colorCamera()->stop();
                    }
                    if (device->objectDetector()) {
                        device->objectDetector()->setSource(xv::ObjectDetector::Source::TOF);
                    }
                    if (device->tofCamera()) {
                        device->tofCamera()->unregisterCallback(tofId);
                        tofId = device->tofCamera()->registerCallback(TOFCallBackFun::tofCallback);
                        device->tofCamera()->start();
                    }
                    break;
                case 0:
                {
                    if (device->colorCamera()) {
                        device->colorCamera()->stop();
                        device->colorCamera()->unregisterCallback(rgbId);
                    }
                    if (device->tofCamera()) {
                        device->tofCamera()->stop();
                        device->tofCamera()->unregisterCallback(tofId);
                    }
                    if (device->fisheyeCameras()) {
                        device->fisheyeCameras()->stop();
                        device->fisheyeCameras()->unregisterCallback(iFisheyeLId);
                        device->fisheyeCameras()->unregisterCallback(iFisheyeRId);
                    }
                    if (device->objectDetector()) {
                        device->objectDetector()->stop();
                        device->objectDetector()->unregisterCallback(cnnId);
                    }
                    isExit = true;
                    break;
                }
                default:
                    break;
                }
            }
            break;
        }
        case 50:
        {
            if(!xvhandOpenCLEnvCheck())
            {
                if(enable_output_log){
                    std::cout << "gesture start failed! The path of opencl is not included in LD_LIBRARY_PATH" << std::endl;
                    std::cout << "you should call export LD_LIBRARY_PATH=/xxxx:/vendor/lib64/" << std::endl;
                }
                break;
            }

            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }
            if(enable_output_log){
                std::cout << "gesture start" << std::endl;
            }

            device->gesture()->start();
            if(enable_output_log){
                std::cout << "gesture register" << std::endl;
            }
            gestureId = device->gesture()->registerCallback(gestureCallback);

            break;
        }
        case 51:
        {
            bool ok = device->gesture()->unregisterCallback(gestureId);
            if (ok)
            {
                if(enable_output_log){
                    std::cout << "unregister callback successfully" << std::endl;
                }
            }
            ok = device->gesture()->stop();
            if (ok)
            {
                if(enable_output_log){
                    std::cout << "gesture stop successfully" << std::endl;
                }
            }
            imuId = device->orientationStream()->registerCallback(orientationCallback);
            break;
        }
        case 52:
        {
            if(!xvhandOpenCLEnvCheck())
            {
                if(enable_output_log){
                    std::cout << "gesture start failed! The path of opencl is not included in LD_LIBRARY_PATH" << std::endl;
                    std::cout << "you should call export LD_LIBRARY_PATH=/xxxx:/vendor/lib64/" << std::endl;
                }
                break;
            }

            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }
            if(enable_output_log){
                std::cout << "gesture start" << std::endl;
            }

            device->gesture()->start();
            if(enable_output_log){
                std::cout << "gesture dynamic register" << std::endl;
            }
            dynamicgestureId = device->gesture()->registerDynamicGestureCallback(dynamicgestureCallback);

            break;
        }
        case 53:
        {
            bool ok = device->gesture()->UnregisterDynamicGestureCallback(dynamicgestureId);
            if (ok)
            {
                if(enable_output_log){
                    std::cout << "unregister callback successfully" << std::endl;
                }
            }
            ok = device->gesture()->stop();
            if (ok)
            {
                if(enable_output_log){
                    std::cout << "gesture stop successfully" << std::endl;
                }
            }
            imuId = device->orientationStream()->registerCallback(orientationCallback);
            break;
        }
        case 54:
        {
            if(!xvhandOpenCLEnvCheck())
            {
                if(enable_output_log){
                    std::cout << "gesture start failed! The path of opencl is not included in LD_LIBRARY_PATH" << std::endl;
                    std::cout << "you should call export LD_LIBRARY_PATH=/xxxx:/vendor/lib64/" << std::endl;
                }
                break;
            }

            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }
            if(enable_output_log){
                std::cout << "gesture start" << std::endl;
            }

            device->gesture()->start();
            if(enable_output_log){
                std::cout << "gesture keypoints register " << std::endl;
            }
            keypointsId = device->gesture()->registerKeypointsCallback(keypointsCallback);

            break;
        }
        case 55:
        {
            bool ok = device->gesture()->unregisterKeypointsCallback(keypointsId);
            if (ok)
            {
                if(enable_output_log){
                    std::cout << "unregister callback successfully" << std::endl;
                }
            }
            ok = device->gesture()->stop();
            if (ok)
            {
                if(enable_output_log){
                    std::cout << "gesture stop successfully" << std::endl;
                }
            }
            imuId = device->orientationStream()->registerCallback(orientationCallback);
            break;
        }
        case 56:
        {
            if(!xvhandOpenCLEnvCheck())
            {
                if(enable_output_log){
                    std::cout << "gesture start failed! The path of opencl is not included in LD_LIBRARY_PATH" << std::endl;
                    std::cout << "first call export LD_LIBRARY_PATH=/xxxx:/vendor/lib64/" << std::endl;
                }
                break;
            }

            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }
            if(enable_output_log){
                std::cout << "slam start" << std::endl;
            }
            if (device->fisheyeCameras()) {
                iFisheyeId = device->fisheyeCameras()->registerCallback([](xv::FisheyeImages const& images) {});
            }
            // start mix slam
            device->slam()->start(xv::Slam::Mode::Mixed);

            if(enable_output_log){
                std::cout << "gesture start" << std::endl;
            }
            device->gesture()->setPlatform(3, true);
            device->gesture()->setParams(9, true);

            bool bOk = device->gesture()->start();
            if(enable_output_log){
                std::cout << "gesture slam keypoints register " << std::endl;
            }
            slamkeypointsId = device->gesture()->registerSlamKeypointsCallback(slamkeypointsCallback);

            break;
        }
        case 57:
        {
            bool ok = device->gesture()->unregisterSlamKeypointsCallback(slamkeypointsId);
            if (ok)
            {
                if(enable_output_log){
                    std::cout << "unregister callback successfully" << std::endl;
                }
            }

            ok = device->gesture()->stop();
            if (ok)
            {
                if(enable_output_log){
                    std::cout << "gesture stop successfully" << std::endl;
                }
            }

            // stop mix slam
            device->slam()->stop();
            if(enable_output_log){
                std::cout << "slam stop" << std::endl;
            }

            imuId = device->orientationStream()->registerCallback(orientationCallback);
            break;
        }
        case 59:
        {
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            if (device->display())
            {
                device->display()->open();
                if(enable_output_log){
                    std::cout << "Display calibration:" << std::endl;
                    std::cout << device->display()->calibration() << std::endl;
                }
            }

            break;
        }
        case 60:
        {
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            const char cameraSwitch[] = {
                "------------------------------\n"
                "1: fisheye camera on\n"
                "2: fisheye camera off\n"
                "3: rgb camera on\n"
                "4: rgb camera off\n"
                "5: tof camera on\n"
                "6: tof camera off\n"
                "0: Exit\n"
                "------------------------------\n"
                "enter select the combination:"
            };

            int switchMode = requestCmdAllPlatform(cameraSwitch, sizeof(cameraSwitch));

            if (switchMode == 0)
            {
                break;
            }

            int hundred = switchMode / 100;
            cameraOnOffSwitch(hundred, device);
            int ten = (switchMode / 10) % 10;
            cameraOnOffSwitch(ten, device);
            int bit = switchMode % 10;
            cameraOnOffSwitch(bit, device);

            break;
        }
        case 61:
        {
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            if (device->fisheyeCameras()) {
                iFisheyeId = device->fisheyeCameras()->registerCallback([](xv::FisheyeImages const& images) {});
            }

            device->fisheyeCameras()->start();

            device->slam()->start(xv::Slam::Mode::Mixed);
            //std::dynamic_pointer_cast<xv::DeviceEx>(device)->slam2()->start(xv::Slam::Mode::Edge);
            if(enable_output_log){
                std::cout << "start  startTagDetector" << std::endl;
            }
            tagDetectorId = std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->startTagDetector(device->slam(), "36h11", 0.16, 50.);

            if (!tagDetectorId.empty())
            {
                if(enable_output_log){
                    std::cout << "start  GetTagDetection" << std::endl;
                }
                GetTagDetection(device->fisheyeCameras(), tagDetectorId);
            }
            else {
                if(enable_output_log){
                    std::cout << "tagDetectorId is empty" << std::endl;
                }
            }
            break;
        }
        case 62:
        {
            StopGetTag();
            bool result = std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->stopTagDetector(tagDetectorId);
            if(enable_output_log){
                if (result)
                {
                    std::cout << "stopTagDetector successfully" << std::endl;
                }
                else {
                    std::cout << "stopTagDetector failed" << std::endl;
                }
            }
            imuId = device->orientationStream()->registerCallback(orientationCallback);
            break;
        }
        case 63:
        {
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }
            if (device->fisheyeCameras()) {
                iFisheyeId = device->fisheyeCameras()->registerCallback([](xv::FisheyeImages const& images) {});
            }
            if(enable_output_log){
                std::cout << "call tofcamera start" << std::endl;
            }
            bool useTof = device->tofCamera()->start();
            if(enable_output_log){
                std::cout << "setFramerate(5.)" << std::endl;
            }
            device->tofCamera()->setFramerate(5.);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if(enable_output_log){
                std::cout << "setFramerate(15.)" << std::endl;
            }
            device->tofCamera()->setFramerate(15.);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if(enable_output_log){
                std::cout << "setFramerate(5.)" << std::endl;
            }
            device->tofCamera()->setFramerate(5.);
            if(enable_output_log){
                std::cout << "call slam start" << std::endl;
            }
            auto slam = device->slam();
            if (useTof && slam)
            {
                auto slamEx = dynamic_cast<xv::SlamEx*>(slam.get());
                if(enable_output_log){
                    std::cout << "call setEnableSurfaceReconstruction" << std::endl;
                }
                slamEx->setEnableSurfaceReconstruction(true);
                if(enable_output_log){
                    std::cout << "call setEnableSurfaceTexturing" << std::endl;
                }
                slamEx->setEnableSurfaceTexturing(false);
                if(enable_output_log){
                    std::cout << "call setSurfacePointCloudDecimationFactor" << std::endl;
                }
                slamEx->setSurfacePointCloudDecimationFactor(3);
                if(enable_output_log){
                    std::cout << "call registerSurfaceCallback" << std::endl;
                }
                slamEx->registerSurfaceCallback([](std::shared_ptr<const xv::ex::Surfaces> ptr)
                    {
                    if(enable_output_log){
                        std::cout << "get surface callback" << std::endl;
                    }
                    });
                device->slam()->start();
            }
            break;
        }
        case 64:
        {
            if (device->tofCamera()) {
                device->tofCamera()->stop();
            }
            device->slam()->stop();
            imuId = device->orientationStream()->registerCallback(orientationCallback);
            break;
        }
        case 65:
        {
            if (device->orientationStream()) {
                device->orientationStream()->unregisterCallback(imuId);
            }

            // std::vector<unsigned char> vecWrite;
            // std::vector<unsigned char> vecRead;
            // bool ret;

            // vecWrite.resize(63);
            // vecWrite[0] = 0x02;
            // vecWrite[1] = 0xfe;
            // vecWrite[2] = 0x20;
            // vecWrite[3] = 0x21;
            // ret = device->hidWriteAndRead(vecWrite, vecRead);

            // if (!ret)
            // {
            //     if(enable_output_log){
            //         std::cout << "hid command 21 send failed" << std::endl;
            //     }
            //     break;
            // }
            // else {
            //     if(enable_output_log){
            //         std::cout << "send commad 02 fe 20 21 succeed" << std::endl;
            //     }
            //     std::this_thread::sleep_for(std::chrono::milliseconds(32));
            //     vecWrite[0] = 0x02;
            //     vecWrite[1] = 0xfe;
            //     vecWrite[2] = 0x20;
            //     vecWrite[3] = 0x22;
            //     ret = device->hidWriteAndRead(vecWrite, vecRead);
            //     if (!ret)
            //     {
            //         if(enable_output_log){
            //             std::cout << "hid command 22 send failed" << std::endl;
            //         }
            //         break;
            //     }
            //     else {
            //         if(enable_output_log){
            //             std::cout << "send commad 02 fe 20 22 succeed" << std::endl;
            //         }
            //     }
            // }

            // get event data
            if(enable_output_log){
                std::cout << "start eventstream" << std::endl;
            }
            device->eventStream()->start();
            if(enable_output_log){
                std::cout << "register event callback" << std::endl;
            }
            eventId = device->eventStream()->registerCallback(eventCallback);
            if(enable_output_log){
                std::cout << "event callback id = " << eventId << std::endl;
            }

            break;
        }
        case 66:
        {
            if(device->eventStream()){
                device->eventStream()->stop();
                device->eventStream()->start();
                device->eventStream()->unregisterCallback(eventId);
            }
            imuId = device->orientationStream()->registerCallback(orientationCallback);
            break;
        }
        case 67:
        {
            if (device->orientationStream()) {
                device->orientationStream()->unregisterCallback(imuId);
            }
            // start SGBM
            sgbmId = device->sgbmCamera()->registerCallback(SGBMCallback::sgbmCallback);
            setSGBMCameraPara(device);
            bool bOk = device->sgbmCamera()->start(default_config);
            if(enable_output_log){
                if (bOk)
                {
                    std::cout << "start SGBM successfully" << std::endl;
                }
                else {
                    std::cout << "start SGBM failed" << std::endl;
                }
            }
            break;
        }
        case 68:
        {
            // Stop SGBM
            device->sgbmCamera()->unregisterCallback(sgbmId);
            bool bOk = device->sgbmCamera()->stop();
            if(enable_output_log){
                if (bOk)
                {
                    std::cout << "stop SGBM successfully" << std::endl;
                }
                else {
                    std::cout << "stop SGBM failed" << std::endl;
                }
            }
            imuId = device->orientationStream()->registerCallback(orientationCallback);
            break;
        }
        case 69:
        {
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            //only support android platform with gaze.
            if(enable_output_log){
                std::cout << "gaze start" << std::endl;
            }
            if (device->gaze())
            {
                std::string path = "/data/local/tmp";
                device->gaze()->setConfigPath(path);
                bool result = device->gaze()->start();
                if(result)
                {
                    if(enable_output_log){
                        std::cout << "start register callback" << std::endl;
                    }
                    gazeCallbackId = device->gaze()->registerCallback(gazeCallback);
                    if(enable_output_log){
                        std::cout << "gaze call back id = " << gazeCallbackId << std::endl;
                    }
                }
            }

            break;
        }
        case 70:
        {
            //only support android platform with gaze.
            if(device->gaze()){
                device->gaze()->unregisterCallback(gazeCallbackId);
                device->gaze()->stop();
            }
            imuId = device->orientationStream()->registerCallback(orientationCallback);
            break;
        }
        case 76:
        {
            if (device->orientationStream()) {
                device->orientationStream()->unregisterCallback(imuId);
            }
#if !defined __ANDROID__ || !defined __x86_64__
            //only support android platform with gaze.
            if (device->gaze())
            {
                if(enable_output_log){
                    std::cout << "start StartCalibration" << std::endl;
                }
                xv::GazeCalibration calibration;
                int result = calibration.StartCalibration(0);
                if(enable_output_log){
                    std::cout << "StartCalibration result : " << result << std::endl;
                }
            }
#endif
            break;
        }
        case 77:
        {
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }
            std::vector<unsigned char> result;

            bool bOK = device->hidWriteAndRead({0x02, 0xfe, 0x36, 01}, result);
            deviceStatusId = device->deviceStatus()->registerCallback(deviceStatusCallback);
            break;
        }
        case 78:
        {
            if(device->deviceStatus()){
                device->deviceStatus()->unregisterCallback(deviceStatusId);
            }
            imuId = device->orientationStream()->registerCallback(orientationCallback);
            break;
        }
        case 79:
        {
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            if (device->fisheyeCameras()) {
                if (iFisheyeLId != -1)
                {
                    device->fisheyeCameras()->unregisterCallback(iFisheyeLId);
                    iFisheyeLId = -1;
                }
                if (iFisheyeRId != -1)
                {
                    device->fisheyeCameras()->unregisterCallback(iFisheyeRId);
                    iFisheyeRId = -1;
                }

                iFisheyeLId = device->fisheyeCameras()->registerCallback(fisheyeLCallback);
                iFisheyeRId = device->fisheyeCameras()->registerCallback(fisheyeRCallback);
                // device->fisheyeCameras()->start();
                std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->setResolutionMode(xv::FisheyeCamerasEx::ResolutionMode::HIGH);
            }

            break;
        }
        case 80:
        {
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            if (device->fisheyeCameras()) {
                if (iFisheyeLId != -1)
                {
                    device->fisheyeCameras()->unregisterCallback(iFisheyeLId);
                    iFisheyeLId = -1;
                }
                if (iFisheyeRId != -1)
                {
                    device->fisheyeCameras()->unregisterCallback(iFisheyeRId);
                    iFisheyeRId = -1;
                }

                iFisheyeLId = device->fisheyeCameras()->registerCallback(fisheyeLCallback);
                iFisheyeRId = device->fisheyeCameras()->registerCallback(fisheyeRCallback);
                std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->setResolutionMode(xv::FisheyeCamerasEx::ResolutionMode::MEDIUM);
            }
            break;
        }
        case 81:
        {
            if (device->fisheyeCameras()) {
                if (iFisheyeLId != -1)
                {
                    device->fisheyeCameras()->unregisterCallback(iFisheyeLId);
                    iFisheyeLId = -1;
                }
                if (iFisheyeRId != -1)
                {
                    device->fisheyeCameras()->unregisterCallback(iFisheyeRId);
                    iFisheyeRId = -1;
                }
            }
            imuId = device->orientationStream()->registerCallback(orientationCallback);
            break;
        }
        case 82:
        {
            startRGBDFunction(device, imuId, rgbId, tofId, tofRgbdId);
            break;
        }
        case 83:
        {
            if (device->colorCamera()) {
                device->colorCamera()->stop();
                device->colorCamera()->unregisterCallback(rgbId);
            }
            if (device->tofCamera()) {
                device->tofCamera()->stop();
                device->tofCamera()->unregisterCallback(tofId);
                device->tofCamera()->unregisterColorDepthImageCallback(tofRgbdId);
            }
            imuId = device->orientationStream()->registerCallback(orientationCallback);
            break;
        }
        case 84:
        {
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            std::vector<unsigned char> result;

            bool bOK = device->hidWriteAndRead({0x02, 0xab, 0xce, 50}, result);
            if(enable_output_log){
                if(bOK)
                    std::cout << "set FE framerate to 50Hz successfully" << std::endl;
                else
                    std::cout << "set FE framerate to 50Hz failed" << std::endl;
            }

            break;
        }
        case 85:
        {
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            std::vector<unsigned char> result;

            bool bOK = device->hidWriteAndRead({0x02, 0xab, 0xce, 60}, result);
            if(enable_output_log){
                if(bOK)
                    std::cout << "set FE framerate to 60Hz successfully" << std::endl;
                else
                    std::cout << "set FE framerate to 60Hz failed" << std::endl;
            }
            break;
        }
        case 86:
        {
            const char meun_RGBmode[] = {
                    "0: AF\n"
                    "1: MF\n"
                    "------------------------------\n"
                    "enter select:"
            };
            int rgbMode = requestCmdAllPlatform(meun_RGBmode, sizeof(meun_RGBmode));

            if(rgbMode == 0) device->colorCamera()->setRGBMode(xv::ColorCamera::Mode::AF);
            else if(rgbMode == 1) device->colorCamera()->setRGBMode(xv::ColorCamera::Mode::MF);
            break;
        }
        case 87:
        {
            const char meun_RGBDistance[] = {
                    "input RGB local distance(1-255)\n"
                    "------------------------------\n"
                    "enter select:"
            };
            int rgbDistance = requestCmdAllPlatform(meun_RGBDistance, sizeof(meun_RGBDistance));
            device->colorCamera()->setRGBFocalDistance(rgbDistance);
            break;
        }
        case 88:
        {
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }
            std::vector<unsigned char> result;
            if (device->fisheyeCameras()) {
                iFisheyeId = device->fisheyeCameras()->registerCallback([](xv::FisheyeImages const& images) {});
            }

            // start mix slam
            device->slam()->start(xv::Slam::Mode::Mixed);

            // get mixed 6dof
            bool bOK = device->hidWriteAndRead({0x02, 0xfe, 0x36, 01}, result);
            deviceStatusId = device->deviceStatus()->registerCallback(deviceStatusCallback);
            break;
        }
        case 89:
        {
            if(device->deviceStatus()){
                device->deviceStatus()->unregisterCallback(deviceStatusId);
            }
            // stop mix slam
            device->slam()->stop();

            // get IMU
            imuId = device->orientationStream()->registerCallback(orientationCallback);
            break;
        }
        case 90:
        {
            if(enable_output_log){
                std::cout << "xvsdk version: " << xv::version() << std::endl;
            }
            xv::setLogLevel(xv::LogLevel(1));
            auto devices = xv::getDevices(10., json, nullptr, xv::SlamStartMode::Normal);

            // if no device: quit
            if (devices.empty()) {
                if(enable_output_log){
                    std::cerr << "Timeout for device detection." << std::endl;
                }
                return EXIT_FAILURE;
            }

            // take the first device in the map
            device = devices.begin()->second;
            std::vector<unsigned char> result;

            bool bOK = device->hidWriteAndRead({0x02, 0xfe, 0x36, 01}, result);
            deviceStatusId = device->deviceStatus()->registerCallback(deviceStatusCallback);
            break;
        }
        case 99:
        {
            const char menu_enableLog[] =
            {
                "1 : Enable                    \n"
                "2 : Disable                    \n"
                "enter select:"
            };

            int cmd_enable = requestCmdAllPlatform(menu_enableLog, sizeof(menu_enableLog));
            if(cmd_enable == 1){
                enable_output_log = true;
            }else if(cmd_enable == 2){
                enable_output_log = false;
            }
            break;
        }
        case 100:
        {
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            if(device->gpsModule())
            {
                device->gpsModule()->start();
                std::cout << "register gps callback" << std::endl;
                gpsDataId = device->gpsModule()->registerCallback(gpsDataCallback);
            }

            break;
        }
        case 101:
        {
            if(device->gpsModule())
            {
                std::cout << "unregister gps callback" << std::endl;
                device->gpsModule()->unregisterCallback(gpsDataId);
                device->gpsModule()->stop();
            }
            break;
        }
        case 102:
        {
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }
            
            if(device->gpsDistanceModule())
            {
                device->gpsDistanceModule()->start();
                std::cout << "register gps distance callback" << std::endl;
                gpsDistanceDataId = device->gpsDistanceModule()->registerCallback(gpsDistanceDataCallback);
            }

            
            break;
        }
        case 103:
        {
            if(device->gpsDistanceModule())
            {
                std::cout << "unregister gps callback" << std::endl;
                device->gpsDistanceModule()->unregisterCallback(gpsDistanceDataId);
                device->gpsDistanceModule()->stop();
            }
            break;
        }
        case 104:
        {
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }
            
            if(device->fisheyeCameras() && device->fisheyeCameras()->checkAntiDistortionSupport())
            {
                device->sgbmCamera()->start(global_config);
                device->fisheyeCameras()->start();
                feDewarpId = device->fisheyeCameras()->registerAntiDistortionCallback(feDewarpCallback);
            }
            
            break;
        }
        case 105:
        {
            if(device->fisheyeCameras())
            {
                device->fisheyeCameras()->stop();
                device->sgbmCamera()->stop();
            }
            break;
        }
        // case 106:
        // {
        //     if(device->orientationStream()){
        //         device->orientationStream()->unregisterCallback( imuId );
        //     }

        //     std::cout << "start iris enroll" << std::endl;
        //     device->iris()->start();

        //     device->iris()->setUserName("Xvisio");
        //     enrollDataId = device->iris()->registerEnrollCallback(enrollCallback);
        //     break;
        // }
        // case 107:
        // {
        //     device->iris()->UnregisterEnrollCallback(enrollDataId);
        //     device->iris()->stop();
        //     break;
        // }
        // case 108:
        // {
        //     if(device->orientationStream()){
        //         device->orientationStream()->unregisterCallback( imuId );
        //     }

        //     device->iris()->start();
        //     std::cout << "start load iris info" << std::endl;
        //     // std::cout << "irisData.size: " << irisData.size << std::endl;
        //     // FILE* file = fopen("./feature.txt", "r");
        //     // unsigned char* in = (unsigned char*)malloc(6532);
        //     // int bytes_read = fread(in, sizeof(unsigned char), 6532, file);
        //     // std::cout << "byte read " << bytes_read << std::endl;
        //     // std::cout <<  "load iris info" << std::endl;
        //     // bool bOK = device->iris()->loadIrisInfo(in, 1);
        //     bool bOK = device->iris()->loadIrisInfo(&irisData.feature[0], 1);
        //     std::cout << "start iris identify" << std::endl;
        //     device->iris()->registerIdentifyCallback(irisCallback);

        //     break;
        // }
        case 109:
        {
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }
            
            if(device->terrestrialMagnetismModule())
            {
                device->terrestrialMagnetismModule()->start();
                std::cout << "register STM callback" << std::endl;
                STMDataId = device->terrestrialMagnetismModule()->registerCallback(STMDataCallback);
            }

            break;
        }
        case 110:
        {
            if(device->terrestrialMagnetismModule())
            {
                device->terrestrialMagnetismModule()->unregisterCallback(STMDataId);
                device->terrestrialMagnetismModule()->stop();
            }
            break;
        }
        case 111:
        {
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }
            
            auto c = std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->calibrationEx();
            c.pop_back();
            c.pop_back();
            xv::AprilTagDetector detector(c);

            device->fisheyeCameras()->registerCallback([](xv::FisheyeImages const & stereo){
                s_fe_mutex.lock();
                s_images.edgeTimestampUs = stereo.edgeTimestampUs;
                s_images.hostTimestamp = stereo.hostTimestamp;
                s_images.id = stereo.id;
                s_images.images.resize(0);
                s_images.images.push_back(stereo.images[0]);
                s_images.images.push_back(stereo.images[1]);
                s_fe_mutex.unlock();
            });

            device->fisheyeCameras()->start();

            device->slam()->start();
            device->slam()->getPoseAt(s_pose, s_images.hostTimestamp);

            Get4EyeTagDetection(detector);

            break;
        }
        case 112:
        {
            // Stop get IMU
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

             device->colorCamera()->start();
                rgbId = device->colorCamera()->registerCallback(rgbCallback);
            // if (device->fisheyeCameras()) {
            //     iFisheyeId = device->fisheyeCameras()->registerCallback([](xv::FisheyeImages const& images) {});
            // }

            // device->fisheyeCameras()->start();

            device->slam()->start(xv::Slam::Mode::Mixed);
            //std::dynamic_pointer_cast<xv::DeviceEx>(device)->slam2()->start(xv::Slam::Mode::Edge);
            if(enable_output_log){
                std::cout << "start  startTagDetector" << std::endl;
            }
            tagDetectorId = std::dynamic_pointer_cast<xv::ColorCameraEx>(device->colorCamera())->startTagDetector(device->slam(), "36h11", 0.16, 50.);

            if (!tagDetectorId.empty())
            {
                if(enable_output_log){
                    std::cout << "start  GetTagDetection" << std::endl;
                }
                GetTagDetectionrgb(device->colorCamera(), tagDetectorId);
            }
            else {
                if(enable_output_log){
                    std::cout << "tagDetectorId is empty" << std::endl;
                }
            }
            break;
        }
        case 113:
        {
            device->slam()->poseReset();
            break;
        }
        case 114:
        {
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            if(enable_output_log){
                std::cout << "obj detector(RKNN3588) start " << std::endl;
            }
            device->objectDetectorRKNN3588()->setModel("objdet_config.json");
            bool bOk = device->objectDetectorRKNN3588()->start();
            if(enable_output_log){
                std::cout << "obj detector(RKNN3588) register " << std::endl;
            }
            objDetRKNN3588Id = device->objectDetectorRKNN3588()->registerCallback(objDetRKNN3588Callback);
            break;
        }
        case 115:
        {
            bool ok = device->objectDetectorRKNN3588()->unregisterCallback(objDetRKNN3588Id);
            if (ok){
                if(enable_output_log){
                    std::cout << "unregister callback successfully" << std::endl;
                }
            }
            ok = device->objectDetectorRKNN3588()->stop();
            if (ok){
                if(enable_output_log){
                    std::cout << "obj detector(RKNN3588) stop successfully" << std::endl;
                }
            }

            imuId = device->orientationStream()->registerCallback(orientationCallback);
            std::cout << "register imu successfully" << std::endl;
            break;
        }
        case 116:
        {
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            auto deviceEX = std::dynamic_pointer_cast<xv::DeviceEx>(device);
            deviceEX->RGB_L_ThermalFusionCamera()->start();
            rgbLThermalFusionCallbackID = deviceEX->RGB_L_ThermalFusionCamera()->registerCallback(rgbCallback);
            break;
        }
        case 117:
        {
            auto deviceEX = std::dynamic_pointer_cast<xv::DeviceEx>(device);
            deviceEX->RGB_L_ThermalFusionCamera()->unregisterCallback(rgbLThermalFusionCallbackID);
            deviceEX->RGB_L_ThermalFusionCamera()->stop();
            break;
        }
        case 118:
        {
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            auto deviceEX = std::dynamic_pointer_cast<xv::DeviceEx>(device);
            deviceEX->RGB_R_ThermalFusionCamera()->start();
            rgbRThermalFusionCallbackID = deviceEX->RGB_R_ThermalFusionCamera()->registerCallback(rgb2Callback);
            break;
        }
        case 119:
        {
            auto deviceEX = std::dynamic_pointer_cast<xv::DeviceEx>(device);
            deviceEX->RGB_R_ThermalFusionCamera()->unregisterCallback(rgbRThermalFusionCallbackID);
            deviceEX->RGB_R_ThermalFusionCamera()->stop();
            break;
        }
        case 120:
        {
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            std::array<float, 3> buffer = {0.0, 0.0, 1.0};
            auto deviceEX = std::dynamic_pointer_cast<xv::DeviceEx>(device);
            deviceEX->RGB_L_ThermalFusionCamera()->writeRGBThermalFusionParams(buffer);
            deviceEX->RGB_R_ThermalFusionCamera()->writeRGBThermalFusionParams(buffer);
            break;
        }
        case 121:
        {
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }

            volatile static double lastTimestamp1 = 0;
            volatile static int64_t lastEdge1 = 0;
            device->irTrackingCamera()->start();
            irCallbackID1 = device->irTrackingCamera()->registerCallback([](xv::IrTrackingImage const& irtracking)
            {
                static FpsCount fc;
                fc.tic();
                static int k1 = 0;
                if (k1++ % 15 == 0) {
                    std::cout << "ir1 hosttimestamp: " << irtracking.hostTimestamp << ", last timestamp: " << lastTimestamp1 << ", cost: " << irtracking.hostTimestamp-lastTimestamp1 << " seconds" << std::endl;
                    std::cout << "ir1 edgetimestamp: " << irtracking.deviceTimestamp << ", last timestamp: " << lastEdge1 << ", cost: " << irtracking.deviceTimestamp-lastEdge1 << " seconds" << std::endl;
                    std::cout << "ir1 fps " << fc.fps() << std::endl; 
                }
                lastTimestamp1 = irtracking.hostTimestamp;
                lastEdge1 = irtracking.deviceTimestamp;
            });
            break;
        }
        case 122:
        {
            device->irTrackingCamera()->unregisterCallback(irCallbackID1);
            device->irTrackingCamera()->stop();

            break;
        }
        case 123:
        {
            if(device->orientationStream()){
                device->orientationStream()->unregisterCallback( imuId );
            }
            
            volatile static double lastTimestamp2 = 0;
            volatile static int64_t lastEdge2 = 0;
            device->irTrackingCamera()->startCamera2();
            irCallbackID2 = device->irTrackingCamera()->registerCamera2Callback([](xv::IrTrackingImage const& irtracking)
            {
                static FpsCount fc;
                fc.tic();
                static int k2 = 0;
                if (k2++ % 15 == 0) {
                    std::cout << "ir2 hosttimestamp: " << irtracking.hostTimestamp << ", last timestamp: " << lastTimestamp2 << ", cost: " << irtracking.hostTimestamp-lastTimestamp2 << " seconds" << std::endl;
                    std::cout << "ir2 edgetimestamp: " << irtracking.deviceTimestamp << ", last timestamp: " << lastEdge2 << ", cost: " << irtracking.deviceTimestamp-lastEdge2 << " seconds" << std::endl;
                    std::cout << "ir2 fps " << std::round(fc.fps()) << std::endl; 
                }
                lastTimestamp2 = irtracking.hostTimestamp;
                lastEdge2 = irtracking.deviceTimestamp;
            });
            break;
        }
        case 124:
        {
            device->irTrackingCamera()->unregisterCamera2Callback(irCallbackID2);
            device->irTrackingCamera()->stopCamera2();
            break;
        }
        default:
            printf("bad command\n");
        }
    }
    stop = true;
    if (tpos.joinable()) {
        tpos.join();
    }
#ifdef _WIN32
#else
    vsc_client_pipe_terminal_srv();
#endif
    return EXIT_SUCCESS;
}
catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
