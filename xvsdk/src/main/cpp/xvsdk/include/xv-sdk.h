#pragma once

#include <map>
#include <functional>

#include "xv-types.h"

namespace xv {

class Device;


/**
 * @brief Stream interface.
 */
template <typename T>
class Stream {
public:

    using Data = T;

    /**
     * @brief start streaming.
     */
    virtual bool start() = 0;
    /**
     * @brief stop streaming.
     */
    virtual bool stop() = 0;

    /**
     * @brief Register callback to receive data.
     */
    virtual int registerCallback(std::function<void (T)>) = 0;
    /**
     * @brief Unregister callback.
     */
    virtual bool unregisterCallback(int callbackId) = 0;

};

/**
 * @brief Camera interface
 */
class Camera {
public:

    /**
     * @brief Get the camera calibration.
     *
     * The frames coordinates are defined according to the IMU frame coordinates. If 2 fisheyes cameras the first is left and second is right camera.
     */
    virtual const std::vector<Calibration>& calibration();

    // TODO add more settings like AWB in later release
    virtual bool setResolution( int resolution );
    virtual bool setFramerate( float framerate );

    // aecMode 0:auto 1:manual
    /**
     * @brief Exposure setting.
     * @param[in] aecMode 0:auto exposure 1:manual exposure
     * @param[in] exposureGain Only valid in manual exposure mode, [0,255]
     * @param[in] exposureTimeMs Only valid in manual exposure mode, in milliseconds
     */
    virtual bool setExposure( int aecMode = 0,  int exposureGain = 0,  float exposureTimeMs = 0.0 );

    /**
     * @brief Set output image brightness. Only valid in auto exposure mode
     * @param[in] brightness brightness of image, [0,255]
     */
    virtual bool setBrightness( int brightness );

    virtual ~Camera(){}
};

/**
 * @brief The class to give access to data provided by the IMU sensor.
 */
class ImuSensor : virtual public Stream<Imu const &> {
public:

    virtual ~ImuSensor(){}
};

/**
 * @brief A class to handle callbacks of events.
 *
 * DataStructure Data structure of PSensor status data:
 * 
 * - Header:
 * 
 *      Bytes index: 0
 * 
 *      Bytes length: 2
 * 
 *      Const value: 0x01, 0xbd
 * 
 * - Sensor type:
 * 
 *      Bytes index: 2
 * 
 *      Bytes length: 1
 * 
 *      Const value: 0xdf
 * 
 *  - PSensor Data:
 *      
 *      Bytes index: 60
 * 
 *      Bytes length: 2
 * 
 *      value: 0x02, 0x00-far away from psensor, 0x02, 0x01-near psensor.
 * @code 
 example:
  
    device->eventStream()->registerCallback( [](xv::Event const & event){

        if(event.type == 0x02)
        {
        }

    });
 @endcode
 * 
 */
class EventStream : virtual public Stream<Event const &> {
public:

    virtual ~EventStream(){}
};

/**
 * @brief The class to give access to 3dof data which converted from raw IMU data.
 */
class OrientationStream : virtual public Stream<Orientation const &> {
public:

    /**
     * @brief Get the current orientation of the device.
     *
     * The orientation (3dof) is the rotation of the IMU frame coordinates based on the world frame coordinates. The world frame coordinates coorespond the IMU coordinates when #startOrientation().
     *
     * @param[out] result orientation corresponding to the timestamp "now" + "prediction"
     * @param[in] prediction (in s) amount of prediction to use to get the orientation corresponding to the future
     * @return true if ok, false else.
     */
    virtual bool get(Orientation& pose, double prediction = 0.) = 0;

    /**
     * @brief Get the orientation of the device at a given timestamp.
     *
     * The orientation (3dof) is the rotation of the IMU frame coordinates based on the world frame coordinates. The world frame coordinates coorespond the IMU coordinates when #startOrientation().
     *
     * @param[out] result orientation corresponding to the timestamp, need to be not too in the pass or too in the future
     * @param[in] timestamp of the requested orientation, in s based on the host clock `std::chrono::steady_clock()`
     * @return true if the orientation can be returned, false else. If timestamp is too in the past or too in the future, return false.
     */
    virtual bool getAt(Orientation& pose, double timestamp) = 0;
    virtual ~OrientationStream(){}
};

/**
 * @brief The class to handle callbacks of the multi cameras for the visual SLAM.
 *
 * FisheyeCameras will get 2 or 4 #Calibration parameters. If 2 fisheyes cameras the first is left and second is right camera. For fisheye, per #Calibration only have one #UnifiedCameraModel and one #PolynomialDistortionCameraModel.
 */
class FisheyeCameras : virtual public Stream<FisheyeImages const &>, virtual public Camera {
public:
    virtual ~FisheyeCameras(){}
};

/**
 * @brief The class to handle informations about the display (if device can display like a HMD)
 */
class Display {
public:

    /**
     * @brief Get calibrations.
     *
     * Calibration parameters of the multiple parts of the dispaly (for HMD, first display is usually for left eye and second is for right eye).
     */
    virtual const std::vector<Calibration>& calibration() = 0;

    /**
     * @brief Turn on the display
     */
    virtual bool open() = 0;

    /**
     * @brief Turn off the display
     */
    virtual bool close() = 0;

    /**
     * @brief Set brightness level.
     *
     * @param[in] level display brightness level
     */
    virtual bool setBrightnessLevel( int level ) = 0;
};

/**
 * @brief A class to handle callbacks of the color image.
 */
class ColorCamera : virtual public Stream<ColorImage const &>, virtual public Camera {
public:
    enum class Resolution {
        RGB_1920x1080 = 0,  ///< RGB 1080p
        RGB_1280x720  = 1,  ///< RGB 720p
        RGB_640x480   = 2,  ///< RGB 480p
        RGB_320x240   = 3,  ///< RGB QVGA (not supported now)
        RGB_2560x1920 = 4,  ///< RGB 5m (not supported now)
        RGB_3840x2160 = 5,
    };

    virtual bool setResolution( const Resolution &resolution ) = 0;

    virtual ~ColorCamera(){}
};

/**
 * @brief A class to handle callbacks of the ToF camera
 */
class TofCamera : virtual public Stream<DepthImage const &>, virtual public Camera {
public:
    enum class StreamMode { DepthOnly = 0, CloudOnly, DepthAndCloud, None, CloudOnLeftHandSlam};
    enum class DistanceMode { Short = 0, Middle, Long };
    enum class SonyTofLibMode { IQMIX_DF ,IQMIX_SF, LABELIZE_DF ,LABELIZE_SF ,M2MIX_DF ,M2MIX_SF };
    enum class Framerate{ FPS_5 ,FPS_10 ,FPS_15 ,FPS_20 ,FPS_25 ,FPS_30 };
    enum class Resolution{ Unknown = -1,VGA = 0 ,QVGA ,HQVGA};
    enum class Manufacturer {Unknown = -1, Pmd = 0, Sony};

    /**
     * @brief Gives access to composed image with RBG color on depth images.
     */
    virtual int registerColorDepthImageCallback(std::function<void(const DepthColorImage&)>) = 0;
    virtual bool unregisterColorDepthImageCallback(int callbackId) = 0;

    /**
     * @brief Convert a depth image to point cloud for sony TOF.
     * @return  The point cloud of valid depth image pixels in ToF frame coordinates, nullptr if something went wrong.
     * @note The coordinate system of the point cloud is the camera coordinate system, and the data unit is millimeters.
     */
    virtual std::shared_ptr<PointCloud> depthImageToPointCloud(DepthImage const &) const = 0;

    /**
     * @brief format a depth image to point cloud for pmd TOF(Cloud Only).
     * @return  The point cloud of valid depth image pixels in ToF frame coordinates, nullptr if something went wrong.
     */
    virtual std::shared_ptr<PointCloud> formatPmdCloudToPointCloudStruct(DepthImage const &) const = 0;

    /**
     * @brief Set which stream will be reported. Not work with sony TOF.
     */
    virtual bool setStreamMode(StreamMode mode) = 0;

    /**
     * @brief Set distance mode.
     *
     * Midlle=Short for 010/009 TOF.
     * 
     * SonyTof:
     *  Short   LABELIZE_SF_VGA_30FPS
     *  Middle  M2_DF_VGA_30FPS
     *  Long    IQ_DF_VGA_30FPS
     */
    virtual bool setDistanceMode(DistanceMode mode) = 0;

    /**
     * @brief Get current resolution
     * @return Resolution{ Unknown = -1,VGA = 0 ,QVGA ,HQVGA};
     */
    virtual Resolution getResolution() = 0;

    /**
     * @brief Get tof Manufacturer
     * @return Manufacturer {Unknown = -1, Pmd = 0, Sony};
     */
    virtual Manufacturer getManufacturer() = 0;

    /**
     * @brief Set lib mode.
     *
     * 1. IQMIX_DF
     * 2. IQMIX_SF
     * 3. LABELIZE_DF
     * 4. LABELIZE_SF
     * 5. M2MIX_DF
     * 6. M2MIX_SF
     */
    virtual bool setLibWorkMode(SonyTofLibMode mode) = 0;

    /**
     * @brief Set work mode.
     */
    virtual bool setMode(int mode) = 0;

    /**
     * @brief SonyTof Settings
    */
    virtual bool setSonyTofSetting(SonyTofLibMode mode, Resolution resolution, Framerate frameRate) = 0;

    /**
     * @brief set SonyTof filter file
    */
    virtual void setFilterFile(std::string filePath) = 0;

    virtual ~TofCamera() {}
};

/**
 * @brief The class to represent the component doing the 6dof tracking with SLAM algorithm on host.
 *
 * For Mixed mode, callback will get the last computed SLAM pose. Callback is call on each IMU recieved because SLAM pose also use IMU for update.
 */
class Slam : virtual public Stream<Pose const&> {

public:
    enum class Mode { Edge = 0, Mixed, EdgeFusionOnHost };

    /**
     * @brief Get #Mode
     * @return return slam mode of this object, mode is const.
     */
    virtual Mode mode() const = 0;

    /**
     * @brief Start slam of current mode.
     */
    virtual bool start() override = 0;

    /**
     * @brief Start slam of specific mode.
     *
     * Stop old mode slam(if running), switch to new mode, and then start slam of new mode.
     */
    virtual bool start(Mode mode) = 0;

    /**
     * @brief Reset the 6dof tracker (SLAM)
     * @return return true if well reset, else if something went wrong.
     */
    virtual bool reset() = 0;

    /**
     * @brief Get the current 6dof pose of the device.
     *
     * The 6dof pose is the rotation and translation of the IMU frame coordinates based on the world frame coordinates. The world frame coordinates coorespond to the VLSAM map and the device
     * coordinates is based on the IMU coordinates.
     *
     * @param[out] pose corresponding to the timestamp "now" + "prediction"
     * @param[in] prediction (in s) amount of prediction to use to get a pose corresponding to the future
     * @return true if ok, false else.
     */
    virtual bool getPose(Pose& pose, double prediction = 0.) = 0;

    /**
     * @brief Get the 6dof pose of the device at a given timestamp.
     *
     * The 6dof pose is the rotation and translation of the IMU frame coordinates based on the world frame coordinates. The world frame coordinates coorespond to the VLSAM map and the device
     * coordinates is based on the IMU coordinates.
     *
     * @param[out] pose result pose corresponding to the timestamp, need to be not too in the pass or too in the future
     * @param[in] timestamp of the wanted pose, in s based on the host clock `std::chrono::steady_clock()`
     * @return true if the pose can be returned, false else. If timestamp is too in the past or too in the future, return false.
     */
    virtual bool getPoseAt(Pose& pose, double timestamp) = 0;

    /**
     * @brief Register a callback called when visual SLAM compute a new unfiltered pose.
     * @return Id of the callback (used to unregister the callback).
     */
    virtual int registerVisualPoseCallback(std::function<void (const Pose&)> lostCallback) = 0;
    virtual bool unregisterVisualPoseCallback(int callbackId) = 0;
    #define XVSDK_HAS_SLAM_VISUAL_POSE_CALLBACK

    /**
     * @brief Register a callback called when SLAM is lost.
     * @return Id of the callback (used to unregister the callback).
     */
    virtual int registerLostCallback(std::function<void ()> lostCallback) = 0;
    virtual bool unregisterLostCallback(int callbackId) = 0;

    /**
     * @brief Callback to get the detected planes using stereo cameras and SLAM
     *
     * The vector contains planes with current planes and each plane has an ID. Between mutiple calls new planes can be added, previous planes updated or merged. If a plane disappear from the vector, it means it was merged with other.
     *
     * @return Id of the callback (used to unregister the callback).
     */
    virtual int registerStereoPlanesCallback(std::function<void (std::shared_ptr<const std::vector<Plane>>)> planeCallback) = 0;
    virtual bool unregisterStereoPlanesCallback(int callbackId) = 0;
    virtual bool clearStereoPlanes() = 0;

    /**
     * @brief Callback to get the detected planes using ToF camera and SLAM
     *
     * The vector contains planes with current planes and with plane ID as key. Between mutiple calls new planes can be added, previous planes updated or merged. If a plane disappear from the vector, it means it was merged with other.
     *
     * @return Id of the callback (used to unregister the callback).
     */
    virtual int registerTofPlanesCallback(std::function<void (std::shared_ptr<const std::vector<Plane>>)> planeCallback) = 0;
    virtual bool unregisterTofPlanesCallback(int callbackId) = 0;
    virtual bool clearTofPlanes() = 0;


    /**
     * @brief Callback to get the SLAM map updates.
     * @return Id of the callback (used to unregister the callback).
     */
    virtual int registerMapCallback(std::function<void (std::shared_ptr<const xv::SlamMap>)> mapCallback) = 0;
    virtual bool unregisterMapCallback(int callbackId) = 0;

    /**
     * @brief Load a SLAM map and use it as an immutable reference map.
     *
     * @param mapStream the input map stream for loading the map
     * @param done_callback When the switch is done the callback will be called. The input of the callback is the quality result (0-100) of the reference map.
     * @param localized_on_reference_map Call the callback if the SLAM uses a reference map and is localized on the reference map. The input parameter is the threshold for the percentage [0.f,1.f]
     * of reference map usage according to whole map (reference map and dynamic map).
     * If SLAM is not currently using enough the reference map and the usage is below this threshold, then the callback is called with the current percentage [0.f,1.f] of
     * 3D points from the reference map.
     */
    virtual bool loadMapAndSwitchToCslam(std::streambuf& mapStream, std::function<void(int /* status of load map */)> done_callback, std::function<void(float)> localized_on_reference_map={}) = 0;
    /**
     * @brief Save a SLAM map and use it as an immutable reference map.
     *
     * @param mapStream the output map stream to for writing the map
     * @param done_callback When the switch is done the callback will be called. The input of the callback is the quality result (0-100) of the reference map.
     * @param localized_on_reference_map Call the callback if the SLAM uses a reference map and is localized on the reference map. The input parameter is the threshold for the percentage [0.f,1.f]
     * of reference map usage according to whole map (reference map and dynamic map).
     * If SLAM is not currently using enough the reference map and the usage is below this threshold, then the callback is called with the current percentage [0.f,1.f] of
     * 3D points from the reference map.
     */
    virtual bool saveMapAndSwitchToCslam(std::streambuf& mapStream, std::function<void(int /* status of save map */, int /* map quality */)> done_callback, std::function<void(float)> localized_on_reference_map={}) = 0;

    /**
     * @brief slam pose scale calibration.
     *
     * @param raw data
     * @return pose recalibrated data
     */
    virtual Pose poseScaleCalibration(const Pose& pose) = 0;

    virtual ~Slam() {}
};

/**
 * @brief A class to handle callbacks of the object detector (CNN)
 */
class ObjectDetector : virtual public Stream<std::vector<Object> const&> {
public:
    enum class Source { LEFT = 0, RIGHT, RGB, TOF };

    virtual bool setDescriptor( const std::string &filepath ) = 0;
    virtual bool setModel( const std::string &filepath ) = 0;
    virtual bool setSource( const Source &source ) = 0;
    virtual xv::ObjectDetector::Source getSource() const = 0;
    virtual xv::ObjectDescriptor getDescriptor() const = 0;

    virtual int registerCnnRawCallback(std::function<void (std::shared_ptr<CnnRawWrapper> const&)> poseCallback) = 0;
    virtual bool unregisterCnnRawCallback(int callbackId) = 0;

    virtual ~ObjectDetector() {}
};

/**
 * @brief A class to handle callbacks of the SGBM.
 */
class SgbmCamera : virtual public Stream<SgbmImage const &>, virtual public Camera {
public:
    enum class Resolution {
        SGBM_640x480   = 0,  ///< SGBM 480p
        SGBM_1280x720  = 1,  ///< SGBM 720p
    };
    enum class Mode { Hardware = 0, Software };
    /**
     * @brief Must be called befor start.
     */
    virtual Mode mode() const = 0;

    virtual bool start(const std::string &sgbmConfig) = 0;
    virtual bool start(const sgbm_config &sgbmConfig) = 0;
    virtual bool setConfig(const std::string &sgbmConfig) = 0;
    virtual bool setSgbmResolution(const xv::SgbmCamera::Resolution & resolution) = 0;
    virtual xv::SgbmCamera::Resolution getSgbmResolution() = 0;
    /**
     * @brief convert DepthImage to pointCloud .
     * 
     * @param sgbmImage 
     * @return std::shared_ptr<PointCloud> 
     * @note The coordinate system of the point cloud is the camera coordinate system, with the left camera as the origin.
     * @note The x, y, depth of the point cloud are in millimeters.
     */
    virtual std::shared_ptr<PointCloud> depthImageToPointCloud(SgbmImage const &sgbmImage) const = 0;
    virtual ~SgbmCamera() {}
};

/**
 * @brief A class to handle callbacks of the thermal camera.
 */
class ThermalCamera : virtual public Stream<ThermalImage const &>, virtual public Camera {
public:

    enum class Mode { TEMPERATURE = 0, TEMPERTURE = 0, GREY };
    virtual bool setMode( Mode mode ) = 0;
    virtual ~ThermalCamera() {}
};

/**
 * @brief A class to handle callbacks of the eyetracking camera.
 */
class EyetrackingCamera : virtual public Stream<EyetrackingImage const &>, virtual public Camera {
public:

    /**
     * @brief Set eyetracking exposure
     *
     * @param[in] leftGain Left eye exposure gain, [0, 255]
     * @param[in] leftTimeMs Left eye exposure time, in milliseconds
     * @param[in] rightGain Right eye exposure gain, [0, 255]
     * @param[in] rightTimeMs Right eye exposure time, in milliseconds
     */
    virtual bool setExposure( int leftGain, float leftTimeMs, int rightGain, float rightTimeMs ) = 0;

    /**
     * @brief Set eyetracking led brightness (in s)
     *
     * @param[in] eye 0:left, 1:right, 2:both
     * @param[in] led [0,7]:led index, 8:all
     * @param[in] brightness [0,255], 0 is off
     */
    virtual bool setLedBrighness( int eye, int led, int brightness ) = 0;

    virtual ~EyetrackingCamera() {}
};

/**
 * @brief A class to handle callbacks of the gaze data.
 */
class GazeStream : virtual public Stream<XV_ET_EYE_DATA_EX const &>{
public:

    virtual ~GazeStream() {}
};

/**
 * @brief A class to handle gusture.
 */
class GestureStream : virtual public Stream<GestureData const &> {
public:

    /**
     * @brief Callback to get the dynamic gesture information.
     *
     * @return Id of the callback (used to unregister the callback).
     */
    virtual int registerDynamicGestureCallback(std::function<void (GestureData const &)>) = 0;
    virtual bool UnregisterDynamicGestureCallback(int callbackID) = 0;

    /**
     * @brief Callback to get the keypoints 21Dof information.
     *
     * The vector contains gesture keypoints 21Dof, size 21 means one hand, vector size 42 means two hands,  2D points, z isn't used by default.
     *
     * @return Id of the callback (used to unregister the callback).
     */
    virtual int registerKeypointsCallback(std::function<void (std::shared_ptr<const std::vector<keypoint>>)> callback) = 0;
    virtual bool unregisterKeypointsCallback(int callbackId) = 0;

    /**
     * @brief Callback to get the keypoints 21Dof information based on slam position.
     *
     * The vector contains gesture keypoints 21Dof based on slam position,  size 21 means one hand, vector size 42 means two hands,  3D points with depth value.
     *
     * @return Id of the callback (used to unregister the callback).
     */
    virtual int registerSlamKeypointsCallback(std::function<void (std::shared_ptr<const std::vector<keypoint>>)> callback) = 0;
    virtual bool unregisterSlamKeypointsCallback(int callbackId) = 0;

    /**
     * @brief Set MNN configuration file path.
     *
     * @param[in] config string value, end with \
     */
    virtual void setConfigPath(std::string config) = 0;

    virtual ~GestureStream() {}
};

/**
 * @brief A class to handle extern gusture data. Only support on Android now.
 */
class GestureStreamEX : virtual public Stream<GestureData const &>{
public:

    /**
     * @brief Start extend gesture stream.
     *
     * @param[in] jvm void* value
     *
     * @return Result of start method, true:succeed, false:failed.
     */
    virtual bool start(void* jvm) = 0;

    /**
     * @brief Callback to get the gesture keypoints pose information based on slam position.
     *
     * The vector contains gesture keypoints pose based on slam position,  size 25 means one hand, vector size 50 means two hands,  3D points with depth value.
     *
     * @return Id of the callback (used to unregister the callback).
     */
    virtual int registerPosCallback(std::function<void (std::shared_ptr<const std::vector<xv::Pose>>)> callback) = 0;
    virtual bool unregisterPosCallback(int callbackId) = 0;

    /**
     * @brief Manually get the gesture informatio at specific timestamp.
     *
     * @param[in] pose xv::Pose value, the pose when you call this method, use getPoseAt to get.
     *
     * @param[in] timestamp double value, the timestamp value when you call this method.
     *
     * @return GestureData at specific timestamp.
     */
    virtual GestureData getGesture(xv::Pose pose, double timestamp) = 0;

    /**
     * @brief Manually get the gesture keypoints pose informatio at specific timestamp.
     *
     * @param[in] pose xv::Pose value, the pose when you call this method, use getPoseAt to get.
     *
     * @param[in] timestamp double value, the timestamp value when you call this method.
     *
     * @return Gesture keypoints pose at specific timestamp.
     */
    virtual std::vector<xv::Pose> GetGesturePose(xv::Pose pose, double timestamp) = 0;

    virtual ~GestureStreamEX() {}
};

/**
 * @brief A class to handle MIC. Adjust volumn through source.
 */
class MicStream : virtual public Stream<MicData const &> {
public:

    virtual ~MicStream() {}
};

/**
 * @brief A class to handle speaker. Adjust the sound source(PCM) volume to adjust the volume.
 */
class Speaker {
public:

    virtual bool enable() = 0;
    virtual bool disable() = 0;
    /**
     * @brief Send a small time slice of sound data.
     */
    virtual int  send(const std::uint8_t *data, int len) = 0;

    /**
     * @brief Async play sound file in new thread.
     */
    virtual bool play(const std::string &path) = 0;
    /**
     * @brief Async play buffer in new thread.
     */
    virtual bool play(const std::uint8_t *data, int len) = 0;
    /**
     * @brief If async playing.
     */
    virtual bool isPlaying() = 0;
    /**
     * @brief Rigster a callback for async play end.
     */
    virtual int registerPlayEndCallback( std::function<void ()> ) = 0;
    virtual bool unregisterPlayEndCallback( int callbackId ) = 0;

    virtual ~Speaker() {}
};

/**
 * @brief A class to handle device status event stream.
 * 
 * DataStructure Data structure of device status data:
 * 
 * - Header:
 * 
 *      Bytes index: 0
 * 
 *      Bytes length: 2
 * 
 *      Const value: 0x01, 0xaf
 * 
 * - Sensor type:
 * 
 *      Bytes index: 2
 * 
 *      Bytes length: 1
 * 
 *      Const value: 0x62
 * 
 * - Timestamp:
 * 
 *      Bytes index: 3
 * 
 *      Bytes length: 8
 * 
 * - Sensor tempreture:
 * 
 *      Bytes index: 11
 * 
 *      Bytes length: 6
 * 
 *      Comments: Two bytes for each sensor, factor is 0.0625, unit is centigrade. For example: if the value is 0x02c0, the tempreture value will be 704*0.0625 = 44.0 centigrade
 * 
 * - CPU tempreture:
 * 
 *      Bytes index: 17
 * 
 *      Bytes length: 1
 * 
 *      Comments: factor is 1, unit is centigrade.
 * 
 * - Current fan speed:
 * 
 *      Bytes index: 18
 * 
 *      Bytes length: 2
 * 
 * - Average fan speed:
 * 
 *      Bytes index: 20
 * 
 *      Bytes length: 2
 * 
 * - Fan speed change:
 * 
 *      Bytes index: 22
 * 
 *      Bytes length: 2
 * 
 * - Previous fan speed:
 * 
 *      Bytes index: 24
 * 
 *      Bytes length: 2
 * 
 * - Curent fan duty cycle:
 * 
 *      Bytes index: 26
 * 
 *      Bytes length: 1
 * 
 * - Fan duty cycle change:
 * 
 *      Bytes index: 27
 * 
 *      Bytes length: 1
 * 
 * - Previous fan duty cycle:
 * 
 *      Bytes index: 28
 * 
 *      Bytes length: 1
 * 
 * - Soft reset:
 * 
 *      Bytes index: 29
 * 
 *      Bytes length: 1
 * 
 *      Comment: 0x80-soft reset unsupported, 0x81-soft reset supported
 * 
 * - Frequency:
 * 
 *      Bytes index: 30
 * 
 *      Bytes length: 4
 * 
 * - RGB switch:
 * 
 *      Bytes index: 34
 * 
 *      Bytes length: 1
 * 
 *      Comment: 0x0-offline, 0x01-online
 * 
 * - Fisheye switch:
 * 
 *      Bytes index: 35
 * 
 *      Bytes length: 1
 * 
 *      Comment: 0x00-offline, 0x01-online. The upper four bits are status of four cameras, bit7-sen_right2, bit6-sen_left2, bit5-sen_right, bit4-sen_left
 * 
 * - TOF switch:
 * 
 *      Bytes index: 36
 * 
 *      Bytes length: 1
 * 
 *      Comment: 0x00-offline, 0x01-online
 * 
 * - UAC speaker switch:
 * 
 *      Bytes index: 37
 * 
 *      Bytes length: 1
 * 
 *      Comment: 0x00-offline, 0x01-online with normal streaming, 0x02-online with interrupted streaming
 * 
 * - UAC mic switch:
 * 
 *      Bytes index: 38
 * 
 *      Bytes length: 1
 * 
 *      Comment: 0x00-offline, 0x01-online with normal streaming, 0x02-online with interrupted streaming
 * 
 * - Audio speaker switch:
 * 
 *      Bytes index: 39
 * 
 *      Bytes length: 1
 * 
 *      Comment: 0x00-offline, 0x01-online
 * 
 * - Audio mic switch:
 * 
 *      Bytes index: 40
 * 
 *      Bytes length: 1
 * 
 *      Comment: 0x00-offline, 0x01-online
 * 
 * - DP switch:
 * 
 *      Bytes index: 41
 * 
 *      Bytes length: 1
 * 
 *      Comment: 0x00-offline, 0x01-online
 * 
 * - Panel switch:
 * 
 *      Bytes index: 42
 * 
 *      Bytes length: 1
 * 
 *      Comment: 0x00-offline, 0x01-online
 * 
 * - USB status:
 * 
 *      Bytes index: 43
 * 
 *      Bytes length: 1
 * 
 *      Comment: 0x00-interface up, 0x01-interface down, 0x02-suspend, 0x03-resume, 0x04-reset
 * 
 * - USB reset flag:
 * 
 *      Bytes index: 44
 * 
 *      Bytes length: 1
 * 
 *      Comment: 0x00-normal, 0x01-reset
 * 
 * DataStructure of handle controller data:
 * 
 * - Header:
 * 
 *      Bytes index: 0
 * 
 *      Bytes length: 2
 * 
 *      Const value: 0x01, 0xaf
 * 
 * - Sensor type:
 * 
 *      Bytes index: 2
 * 
 *      Bytes length: 1
 * 
 *      Const value: 0x71
 * 
 * - Handle controller type:
 * 
 *      Bytes index: 3
 * 
 *      Bytes length: 1
 * 
 *      Const value: 0x22-left handle controller, 0x23-right handle controller
 * 
 * - Timestamp:
 * 
 *      Bytes index: 4
 * 
 *      Bytes length: 2
 * 
 *      Const value: 0x22-left handle controller, 0x23-right handle controller
 * 
 * - Acc x:
 * 
 *      Bytes index: 4
 * 
 *      Bytes length: 2
 * 
 * - Acc y:
 * 
 *      Bytes index: 4
 * 
 *      Bytes length: 2
 * 
 * - Acc z:
 * 
 *      Bytes index: 4
 * 
 *      Bytes length: 2
 * 
 * - Gyro x:
 * 
 *      Bytes index: 4
 * 
 *      Bytes length: 2
 * 
 * - Gyro y:
 * 
 *      Bytes index: 4
 * 
 *      Bytes length: 2
 * 
 * - Gyro z:
 * 
 *      Bytes index: 4
 * 
 *      Bytes length: 2
 * 
 * DataStructure of handle controller key eventdata:
 * 
 * - Header:
 * 
 *      Bytes index: 0
 * 
 *      Bytes length: 2
 * 
 *      Const value: 0x01, 0xaf
 * 
 * - Sensor type:
 * 
 *      Bytes index: 2
 * 
 *      Bytes length: 1
 * 
 *      Const value: 0x71
 * 
 * - Handle controller type:
 * 
 *      Bytes index: 3
 * 
 *      Bytes length: 1
 * 
 *      Const value: 0x32-left handle controller, 0x33-right handle controller
 * 
 * - Trigger:
 * 
 *      Bytes index: 4
 * 
 *      Bytes length: 1
 * 
 * - Side trigger:
 * 
 *      Bytes index: 5
 * 
 *      Bytes length: 1
 * 
 * - Rocker x:
 * 
 *      Bytes index: 6
 * 
 *      Bytes length: 2
 * 
 * - Rocker y:
 * 
 *      Bytes index: 8
 * 
 *      Bytes length: 2
 * 
 * - Key:
 * 
 *      Bytes index: 10
 * 
 *      Bytes length: 1
 * 
 * - Battery:
 * 
 *      Bytes index: 11
 * 
 *      Bytes length: 2
 * @code 
 example:
  
    device->deviceStatus()->registerCallback( [](const std::vector<unsigned char>& deviceStatus){
       
    });
 @endcode
 */
class DeviceStatusStream : virtual public Stream<std::vector<unsigned char> const &> {
public:

    virtual ~DeviceStatusStream(){}
};


/**
 * @brief Class to get tracking results and raw outputs with a connected device.
 *
 * This class is the main entry point of the API, it gives access to the device and algorithms. See xv::getDevices() to have an instance corresponding to a device.
 *
 * A device can have multiple components, accessible with member functions :
 * - #xv::Device::slam() : 6dof tracker doing the SLAM algorithm on host based on informations coming from device (stereo camera, IMU sensor, ..)
 * - #xv::Device::imuSensor() : sensor with at least 3-axis accelerometer and 3-axis gyrometer
 * - #xv::Device::fisheyeCameras(): typically 2 fisheye cameras used for Visual SLAM
 * - #xv::Device::tofCamera(): a depth camera sensor
 * - #xv::Device::edge(): used to run some algorithm directly on embedded device (typically Visual SLAM) when it is possible to choose between host and edge processing
 * - #xv::Device::display(): used to handle informations about the display (if device can display like a HMD)
 * - #xv::Device::objectDetector(): used to run and get results of the CNN object detector
 *
 * If a device does not support a component or doesn't have the component (for example ToF), the accessor function will return `null_ptr`.
 * The data streams and processings under a component are activated only if at least one callback is registerd to the component. If all the callbacks are unregistered then steams can be deactivated.
 */
class Device {

public:


    /**
     * @brief Get informations (Serial Number, version ...) about the device.
     * @return A map with key and values of the informations.
     */
    virtual std::map<std::string, std::string> info() = 0;

    /**
     * @brief Get the SLAM component.
     */
    virtual std::shared_ptr<Slam> slam() = 0;

    /**
     * @brief Get the IMU sensor of the device.
     */
    virtual std::shared_ptr<ImuSensor> imuSensor() = 0;

    /**
     * @brief Get the event component.
     */
    virtual std::shared_ptr<EventStream> eventStream() = 0;

    /**
     * @brief Get the 3dof component.
     */
    virtual std::shared_ptr<OrientationStream> orientationStream() = 0;

    /**
     * @brief Get the stereo cameras component of the device.
     */
    virtual std::shared_ptr<FisheyeCameras> fisheyeCameras() = 0;

    /**
     * @brief Get the color camera component of the device.
     */
    virtual std::shared_ptr<ColorCamera> colorCamera() = 0;

    /**
     * @brief Get the ToF component of the device.
     */
    virtual std::shared_ptr<TofCamera> tofCamera() = 0;

    /**
     * @brief Get the SGBM component of the device.
     */
    virtual std::shared_ptr<SgbmCamera> sgbmCamera() = 0;

    /**
     * @brief Get the thermal component of the device.
     */
    virtual std::shared_ptr<ThermalCamera> thermalCamera() = 0;

    /**
     * @brief Get the eyetracking component of the device.
     */
    virtual std::shared_ptr<EyetrackingCamera> eyetracking() = 0;

    /**
     * @brief Get the gaze data of the device.
     */
    virtual std::shared_ptr<GazeStream> gaze() = 0;

   /**
     * @brief Get the gesture component.
     */
    virtual std::shared_ptr<GestureStream> gesture() = 0;

    virtual std::shared_ptr<GestureStreamEX> gestureEX() = 0;

    /**
     * @brief Get the MIC component of the device.
     */
    virtual std::shared_ptr<MicStream> mic() = 0;

    /**
     * @brief Get the speaker component of the device.
     */
    virtual std::shared_ptr<Speaker> speaker() = 0;

    /**
     * @brief Get the display component.
     */
    virtual std::shared_ptr<Display> display() = 0;

    /**
     * @brief Get the object detection component.
     */
    virtual std::shared_ptr<ObjectDetector> objectDetector() = 0;

    /**
     * @brief Get the device status component.
     */
    virtual std::shared_ptr<DeviceStatusStream> deviceStatus() = 0;

    /**
     * @brief Let device sleep.
     */
    virtual bool sleep(int level = 0) = 0;
    /**
     * @brief Wake up device.
     */
    virtual bool wakeup() = 0;

    /**
     * @brief Control device.
     */
    virtual bool control(const DeviceSetting &setting) = 0;

    /**
     * @brief Write HID control command and read result. HID command list:
     * 
     * - Refresh rate Setting command: 
     * 
     *      Header: 0x02, 0xfe, 0x20, 0x03, 0x01, 0x09
     * 
     *      Data: 0x02-72hz, 0x03-90hz
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xfe, 0x20, 0x03, 0x01, 0x09, 0x03}, result);
     @endcode
     *
     * - Breathing lamp chip standby setting command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x01
     * 
     *      Data: 0x01
     * 
     *      Comment: If you want to wake up the sensor, send mode set command to the needed mode.
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x01, 0x01}, result);
     @endcode
     * 
     * - Monochrome breathing lamp cycle setting command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x02,0x01
     * 
     *      Data: 0x01-FF0000 color cycle, 0x02-FFF300 color cycle, 0x03-36FF48 color cycle, 0x04-62F1FF color cycle, 0x05-000CFF color cycle, 0x06-8000FF color cycle.
     * 
     *      Comment: The default rise time is 1.04s, storage time is 0.004s, fall time is 1.04s, closing time is 0.04s, and other times are 0. To change time, refer to breathing speed setting command.
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x02, 0x01, 0x01}, result);
     @endcode
     * 
     * - 4-color breathing lamp cycle setting command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x02, 0x02
     * 
     *      Data: 0x04
     * 
     *      Comment: The default rise time is 1.04s, storage time is 0.004s, fall time is 1.04s, closing time is 0.04s, and other times are 0. To change time, refer to breathing speed setting command. 4-color breathing cycle: 00F5A9->00CBF5->0C61F5->D700EF.
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x02, 0x02, 0x04}, result);
     @endcode
     * 
     * - Breathing lamp speed setting command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x05, 0x30-rise time and storage time setting, 0x05, 0x31-fall time and closing time setting.
     * 
     *      Data: The upper four bits are rise time or fall time, the lower four bits are storage time or closing time.
     * 
     *      Comment: Rise time or fall time: 0000-0s, 0001-0.13s, 0010-0.26s, 0011-0.38s, 0100-0.51s, 0101-0.77s, 0110-1.04s, 0111-1.6s, 1000-2.1s, 1001-2.6s, 1010-3.1s, 1011-4.2s, 1100-5.2s, 1101-6.2s, 1110-7.3s, 1111-8.3s. Storage time or closing time: 0000-0.04s, 0001-0.13s, 0010-0.26s, 0011-0.38s, 0100-0.51s, 0101-0.77s, 0110-1.04s, 0111-1.6s, 1000-2.1s, 1001-2.6s, 1010-3.1s, 1011-4.2s, 1100-5.2s, 1101-6.2s, 1110-7.3s, 1111-8.3s.
     * @code 
     example:

        std::vector<unsigned char> result;

        //set rise time to 1.04s
        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x05, 0x30, 0x60}, result);
     @endcode
     * 
     * - Constant breathing lamp light cycle switch setting command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x02, 0x03
     * 
     *      Data: 0x08-8 color constant light cycle, 0x14-20 color constant light cycle, 0x60-96 color constant light cycle
     * 
     *      Comment: The default change time is 0.1s. To change time, refer to constant breathing lamp light cycle switch speed setting command. 8 colors cycle: FF18FF->FF1010->FF8000->EFFF00 ->00FF00->00FFFF ->1858FF ->8A00FF. 20 colors cycle: E000FF->E80093->FF000D->E82400->FF5300->E87500->FFA400->E8B200->FFE100->E8E800->97FF00->2DE800->00FF2B->00EB7C->00FFE5->00B0EB->0069FF->0012EB->4000FF->8500EB. 96 colors cycle: FF00FE->F000FF->E000FF->CF00FE->C001FF->B000FF->A000FF->8F00FF->7F00FF->700FF->6000FF->5000FF->3F00FF->2F00FE->2001FF->1000FF->0000FE->0110FF->0020FF->0030FF->0140FF->0050FF->0060FF->0071FE->0080FF->0090FF->00A0FE->00AFFE->00C0FF->00D0FF->01E0FF->00F0FF->01FFFF->00FFF1->00FFE1->00FFD0->01FFC1->00FFB1->00FFA1->01FE91->00FE81->00FF71->00FF61->01FF51->00FF41->00FF31->00FE20->01FF11->00FF01->10FF01->1FFF00->30FF00->40FF01->50FF00->5FFF00->6FFF00->80FF00->90FF00->A0FF01->AFFF00->C0FF00->D0FF00->E0FF01->EFFF00->FFFF01->FFF001->FFE001->FED000->FFC000->FFB001->FF9F00->FF9000->FF7F00->FF7000->FF6100->FF5001->FF4001->FE3000->FF2000->FF1001->FE0000->FF0010->FF0020->FF0030->FF0140->FF0050->FF0060->FE0070->FF0080->FF0090->FF01A1->FE00B0->FF00C0->FF00D0->FF00E0->FF00F0
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x02, 0x03, 0x08}, result);
     @endcode
     *
     * - Constant breathing lamp light cycle switch speed setting command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x07, 0x03
     * 
     *      Data: 0~255
     * 
     *      Comment: The unit is 100ms, set single color light time, only effect in constant light cycle switch.
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x07, 0x03, 0x00}, result);
     @endcode
     * 
     * - Breathing lamp real color mode setting command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x02
     * 
     *      Data: 0x04
     * 
     *      Comment: Composed of red, green and blue lights. The real color can be formed if the on-off time of the three lights is inconsistent.The default red color rise time is 1.04s, red color storage time is 2.1s, red color fall time is 1.04s, red color closing time is 2.6s, red color delay time is 0s. The default green color rise time is 1.04s, green color storage time is 2.1s, green color fall time is 1.04s, green color closing time is 1.6s, green color delay time is 1.04s. The default blue color rise time is 1.04s, blue color storage time is 2.1s, blue color fall time is 1.04s, blue color closing time is 0.004s, blue color delay time is 3.1s. To change time, refer to breathing lamp real color mode speed setting command.
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x02, 0x04}, result);
     @endcode
     * 
     * - Breathing lamp real color mode speed setting command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x05, 0x30-red color rise time and storage time setting, 0x05, 0x31-red color fall time and closing time setting, 0x05, 0x32-red color delay time. 0x05, 0x35-blue color rise time and storage time setting, 0x05, 0x36-blue color fall time and closing time setting, 0x05, 0x37-blue color delay time. 0x05, 0x3a-green color rise time and storage time setting, 0x05, 0x3b-green color fall time and closing time setting, 0x05, 0x3c-green color delay time.
     * 
     *      Data: The upper four bits are rise time or fall time, the lower four bits are storage time, closing time or delay time.
     * 
     *      Comment: Rise time or fall time: 0000-0s, 0001-0.13s, 0010-0.26s, 0011-0.38s, 0100-0.51s, 0101-0.77s, 0110-1.04s, 0111-1.6s, 1000-2.1s, 1001-2.6s, 1010-3.1s, 1011-4.2s, 1100-5.2s, 1101-6.2s, 1110-7.3s, 1111-8.3s. Storage time or closing time: 0000-0.04s, 0001-0.13s, 0010-0.26s, 0011-0.38s, 0100-0.51s, 0101-0.77s, 0110-1.04s, 0111-1.6s, 1000-2.1s, 1001-2.6s, 1010-3.1s, 1011-4.2s, 1100-5.2s, 1101-6.2s, 1110-7.3s, 1111-8.3s. Delay time: 0000-0s, 0001-0.13s, 0010-0.26s, 0011-0.38s, 0100-0.51s, 0101-0.77s, 0110-1.04s, 0111-1.6s, 1000-2.1s, 1001-2.6s, 1010-3.1s, 1011-4.2s, 1100-5.2s, 1101-6.2s, 1110-7.3s, 1111-8.3s.
     * @code 
     example:

        std::vector<unsigned char> result;

        //set red color delay time to 1.04s
        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x05, 0x30, 0x06}, result);
     @endcode
     * 
     * - Breathing lamp status display setting command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x03-green, 0x04-red
     * 
     *      Data: 0x01
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x03, 0x01}, result);
     @endcode
     * 
     * - Breathing lamp brightness setting command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x05, 0x03
     * 
     *      Data: 0x00-3.1875 Ma, 0x01-6.375 Ma, 0x10-12.75Ma, 0x11-25.5 Ma
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x05, 0x03, 0x00}, result);
     @endcode
     * 
     * - Breathing lamp register write command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x05
     * 
     *      Data: value0-register address, value1-register value.
     *      
     *      Comment: For detailed information, please refer to AW2026 chip register manual.
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x05, 0x00, 0x00}, result);
     @endcode
     * 
     * - Breathing lamp register read command: 
     * 
     *      Header: 0x02, 0xbe, 0x9a
     * 
     *      Type: 0x06
     * 
     *      Data: value0-register address, value1-register value.
     * 
     *      Comment: The data will be saved in result data. For detailed information, please refer to AW2026 chip register manual.
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xbe, 0x9a, 0x06, 0x00, 0x00}, result);
     @endcode
     * 
     * - Set auto display pane brightness command: 
     * 
     *      Header: 0x02, 0xfe, 0x20
     * 
     *      Type: 0x04
     * 
     *      Data: 0x00-close, 0x01-open.
     * 
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xfe, 0x20, 0x04, 0x00}, result);
     @endcode
     * 
     * - Set display pane brightness level command: 
     * 
     *      Header: 0x02, 0xfe, 0x20
     * 
     *      Type: 0x02 or 0x07
     * 
     *      Data: Range: 0x01-0x07, 0x01-darkest, 0x07 brightest.
     * 
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xfe, 0x20, 0x02, 0x01}, result);
     @endcode
     * 
     * - Get display pane brightness setting command: 
     * 
     *      Header: 0x02, 0xfe, 0x20
     * 
     *      Type: 0x02 or 0x07
     * 
     *      Comment: The result except the header shows auto mode and brightness level. value1-auto mode, value2 brightness level.
     * 
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xfe, 0x20, 0x02}, result);
     @endcode
     * 
     * - Save display pane brightness mode command: 
     * 
     *      Header: 0x02, 0xfe, 0x20
     * 
     *      Type: 0x0f
     * 
     *      Data: 0x00-close, 0x01-open.
     * 
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xfe, 0x20, 0x0f}, result);
     @endcode
     * 
     * - Save display pane brightness level command: 
     * 
     *      Header: 0x02, 0xfe, 0x20
     * 
     *      Type: 0x10
     * 
     *      Data: Range: 0x01-0x07, 0x01-darkest, 0x07 brightest.
     * 
     * @code 
     example:

        std::vector<unsigned char> result;

        bool bOK = hidWriteAndRead({0x02, 0xfe, 0x20, 0x10}, result);
     @endcode
     * 
     */
    virtual bool hidWriteAndRead(const std::vector<unsigned char> &command, std::vector<unsigned char> &result) = 0;
    /**
     * @brief Write UVC control command and read result.
     */
    virtual bool uvcWriteAndRead(const std::vector<unsigned char> &command, std::vector<unsigned char> &result) = 0;
    /**
     * @brief Write VSC control command and read result.
     */
    virtual bool vscWriteAndRead(const std::vector<unsigned char> &command, std::vector<unsigned char> &result) = 0;

    /**
     * @brief set enable camera synchronize.
    */
    virtual bool enableSync(bool isEnable) = 0;

    /**
     * @brief Return the serial number of the device.
     */
    virtual std::string id() const = 0;

    virtual ~Device(){}

};


/**
 * \defgroup xv_functions Global functions
 * @{
 */

/**
 * @brief Get xvsdk version.
 */
Version version();

/**
 * @brief Retrieve all the detected XVisio devices.
 * If no device is found after the timeout is reached, the result will be empty.
 * @param timeOut : wait until the timeout is reached or find at least one device.
 * @param stopWaiting : stop scanning when become true.
 * @param desc : Load device according to feature in desc(json string). SDK can choose device feature from desc accoring SN or hardware version. The desc also contains default values of slam algorithm(old SDK is INI file).
 * @return A map with key corresponding to device ID and the value is a #Device.
 */
std::map<std::string,std::shared_ptr<Device>> getDevices(double timeOut = 0., const std::string& desc = "", bool* stopWaiting = nullptr);

/**
 * @brief Change the log level.
 */
void setLogLevel(LogLevel l);

/**
 * @brief Register the callback for hotplug.
 * @return Id of the callback (used to unregister the callback).
 */
int registerPlugEventCallback(const std::function<void (std::shared_ptr<Device> device, PlugEventType type)> &Callback, const std::string& desc = "");
/**
 * @brief Unregister a plug callback.
 */
bool unregisterHotplugCallback( int callbackID );

/**
 * @}
 */


/**
 * \defgroup xv_android_functions Functions for Android
 * @{
 */
/**
 * @brief Retrieve #Device by given descriptor. Only for Android.
 * @param fd : file descriptor opened by android USBManager.
 * @return A #Device.
 */
std::shared_ptr<Device> getDevice(int fd);
/**
 * @brief Retrieve #Device by given descriptor. Only for Android.
 * @param fd : file descriptor opened by android USBManager.
 * @param desc : load device according to feature in desc. SDK can choose device feature from desc accoring SN or version.
 * @return A #Device.
 */
std::shared_ptr<Device> getDevice(int fd, std::string const& desc);

/**
 * @brief Tell sdk device has disconnected. Only for Android.
 */
bool detachDevice(int fd );

/**
 * @brief Retrieve default device description.
 */
std::string getDefaultDescription();

/**
 * @brief Get UTC time.
 */
void getUTCTIme(DateTime* utc);

/**
 * @}
 */

/**
 * @brief save pcd image file, now only support Windows and Unix
 * @note The parameter path specifies the path to save the file,
 *       parameter point is the PointCloud data,
 *       parameter precision saves the precision of the data (the default is 8),
 *       parameter removeInvalidPoints specifies whether to remove invalid points (the default is true).
*/
bool savePointCloudToPcd(const std::string &path, std::shared_ptr<PointCloud> const & point, int precision = 8, bool removeInvalidPoints = true);


}
