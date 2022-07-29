#pragma once

#include "xv-sdk-ex.h"
#include <stdexcept>

#ifdef WIN32
#ifdef DLL_EXPORTS
#define DLL_API __declspec(dllexport)
#else
#define DLL_API __declspec(dllimport)
#endif
#else
#define DLL_API
#endif

namespace xv {

/**
 * \defgroup xv_rotation_utils functions to help manipulation of rotations
 */
/**
 * @brief Linear extrapolation of a rotation matrix.
 * @param input The rotation to extrapolate in the future
 * @param angularVelocity The angular velocity (rad/s) for the linear prediction.
 * @param prediction Amount (in s) of prediction.
 * @return The extrapolated (predicted) rotation.
 */
Matrix3f rotPrediction(Matrix3f const& input, Vector3f const& angularVelocity, double prediction);

/**
 * @brief Linear extrapolation of a rotation matrix.
 * @param input The rotation to extrapolate in the future
 * @param angularVelocity The angular velocity (rad/s) for the linear prediction.
 * @param prediction Amount (in s) of prediction.
 * @return The extrapolated (predicted) rotation.
 */
Matrix3d rotPrediction(Matrix3d const& input, Vector3d const& angularVelocity, double prediction);

/**
 * @brief 2nd order extrapolation of a rotation matrix.
 * @param input The rotation to extrapolate in the future
 * @param angularVelocity The angular velocity (rad/s) for the extrapolation.
 * @param angularAcceleration The angular acceleration (rad/s/s) for the extrapolation.
 * @param prediction Amount (in s) of prediction.
 * @return The extrapolated (predicted) rotation.
 */
Matrix3d rotPrediction(Matrix3f const& input, Vector3f const& angularVelocity, Vector3f const& angularAcceleration, double prediction);

/**
 * @brief 2nd order extrapolation of a rotation matrix.
 * @param input The rotation to extrapolate in the future
 * @param angularVelocity The angular velocity (rad/s) for the extrapolation.
 * @param angularAcceleration The angular acceleration (rad/s/s) for the extrapolation.
 * @param prediction Amount (in s) of prediction.
 * @return The extrapolated (predicted) rotation.
 */
Matrix3d rotPrediction(Matrix3d const& input, Vector3d const& angularVelocity, Vector3d const& angularAcceleration, double prediction);

/**
 * @brief Return the linear interpolation of two rotations matrices.
 * @param a The first rotation matrix (t=0)
 * @param b The second rotation matrix (t=1)
 * @param t Ratio to select where is the interpolation between a (t=0) and b (t=1). t=0.5 means the average between a and b.
 * @return The interpolated rotation matrix.
 */
Matrix3d rotLinearInterpolation(const Matrix3d &a, const Matrix3d &b, double t);

/**
 * @brief Return the linear interpolation of two rotations matrices.
 * @param a The first rotation matrix (t=0)
 * @param b The second rotation matrix (t=1)
 * @param t Ratio to select where is the interpolation between a (t=0) and b (t=1). t=0.5 means the average between a and b.
 * @return The interpolated rotation matrix.
 */
Matrix3f rotLinearInterpolation(const Matrix3f &a, const Matrix3f &b, double t);

/** @}
 *
*/

class DeviceImpl;
class DeviceDriver;
class TimeServer;
struct CallbackMaps;



/**
 * @brief Base class of slam implemnetations (edge, mixed, edge fusion on host ...)
 * Not in public API because public API use modes to switch between implementations. Implements almost all of xv::Slam except features related to modes.
 */
class SlamBase {

    std::unique_ptr<CallbackMaps> m_callbackMaps;

protected:
    const std::shared_ptr<DeviceDriver> m_deviceDriver;
    std::shared_ptr<TimeServer> m_timeServer;

public:

    explicit SlamBase(std::shared_ptr<DeviceDriver> d);

    CallbackMaps& callbackMaps();


    virtual bool start() {throw std::runtime_error("Invalid call, not implemented.");}
    virtual bool stop() {throw std::runtime_error("Invalid call, not implemented.");}
    virtual bool reset() {throw std::runtime_error("Invalid call, not implemented.");}
    virtual bool getPose(Pose &, double );
    virtual bool getPoseAt(Pose &, double );


    virtual int registerCallback(std::function<void (const Pose &)> cb);
    virtual bool unregisterCallback(int cb);
    virtual int registerVisualPoseCallback(std::function<void (const Pose &)> cb);
    virtual bool unregisterVisualPoseCallback(int cb);
    virtual int registerLostCallback(std::function<void ()>);
    virtual bool unregisterLostCallback(int );

    virtual int registerStereoPlanesCallback(std::function<void (std::shared_ptr<const std::vector<Plane> >)> );
    virtual bool unregisterStereoPlanesCallback(int );
    virtual int registerTofPlanesCallback(std::function<void (std::shared_ptr<const std::vector<Plane> >)> );
    virtual bool unregisterTofPlanesCallback(int );
    virtual bool clearStereoPlanes();
    virtual bool clearTofPlanes();

    virtual int registerMapCallback(std::function<void (std::shared_ptr<const SlamMap>)> );
    virtual bool unregisterMapCallback(int );

    virtual int registerLocal3dPointsCallback(std::function<void (std::shared_ptr<const std::vector<std::array<double,3>>>)>);
    virtual bool unregister3dPointsCallback(int);

    virtual bool startCslam(std::streambuf &, std::function<void (float)> ) {throw std::runtime_error("Invalid call, not implemented.");}
    virtual bool loadMapAndSwitchToCslam(std::streambuf &, std::function<void (int)> , std::function<void (float)> ) {throw std::runtime_error("Invalid call, not implemented.");}
    virtual bool saveMapAndSwitchToCslam(std::streambuf &, std::function<void (int, int)> , std::function<void (float)> ) {throw std::runtime_error("Invalid call, not implemented.");}
    virtual bool switchToCSlam(std::function<void (int)> , std::function<void (float)> ) {throw std::runtime_error("Invalid call, not implemented.");}
    virtual bool saveMap(std::streambuf &, bool ) {throw std::runtime_error("Invalid call, not implemented.");}

    virtual bool getLastVSlamPose(Pose &) {throw std::runtime_error("Invalid call, not implemented.");}

    virtual bool getPointCloud(std::shared_ptr<const ex::PointClouds>&) {throw std::runtime_error("Invalid call, not implemented.");}
    virtual int registerPointCloudCallback(std::function<void (std::shared_ptr<const ex::PointClouds>)> );
    virtual bool unregisterPointCloudCallback(int );

    virtual bool getSurface(std::shared_ptr<const ex::Surfaces>&) {throw std::runtime_error("Invalid call, not implemented.");}
    virtual int registerSurfaceCallback(std::function<void (std::shared_ptr<const ex::Surfaces>)> );
    virtual bool unregisterSurfaceCallback(int );

    virtual bool startSurfaceReconstruction();
    virtual bool stopSurfaceReconstruction();
    virtual bool startPlaneDetection();
    virtual bool stopPlaneDetection();

    virtual bool addTags(std::vector<TagInfo> const& v);
    virtual bool onTagUpdate(std::function<void(std::string const& tagId, xv::Transform const& pose, double const& tagSize)>);

    virtual ~SlamBase();

};

class ImuSensorCalibration {

    // gyro corrected = m_gyroScaleInv * (gyro - gyroBias)
    // accel corrected = m_accelScaleInv * (gyro - accelBias)

    double m_temperature; // Temperature of the calibration parameters (in K)
    xv::Vector3d m_gyroBias; // gyrometer bias (in bits or rad/s)
    xv::Vector3d m_accelBias; // accelerometer bias (in bits or g)
    xv::Matrix3d m_gyroScaleInv;
    xv::Matrix3d m_accelScaleInv;

private:

    void apply(xv::Vector3d &x, xv::Vector3d bias, xv::Matrix3d scaleInv) {
        Vector3d xx = x;
        for (std::size_t i=0; i<3; ++i)
            xx[i] -= bias[i];
        for (std::size_t i=0; i<3; ++i)
            x[i] = scaleInv[0+i*3]*xx[0] + scaleInv[1+i*3]*xx[1] + scaleInv[2+i*3]*xx[2];
    }

public:

    ImuSensorCalibration() {
        setGyroScale({1.,1.,1.});
        setAccelScale({1.,1.,1.});
    }
    ImuSensorCalibration(double temperature, xv::Vector3d const& gyroBias, xv::Vector3d const& accelBias, xv::Vector3d const& gyroScale={1.,1,1}, xv::Vector3d const& accelScale={1.,1,1})
        : m_temperature(temperature), m_gyroBias(gyroBias), m_accelBias(accelBias) {
        setGyroScale(gyroScale);
        setAccelScale(accelScale);
    }

    double temperature() const {return m_temperature;}
    void setTemperature(double t) { m_temperature = t; }

    xv::Vector3d const& gyroBias() const { return m_gyroBias; }
    void setGyroBias(xv::Vector3d const& b) { m_gyroBias = b; }
    xv::Matrix3d const& gyroScaleInv() const { return m_gyroScaleInv; }
    void setGyroScale(double scale) {
        setGyroScale({scale,scale,scale});
    }
    void setGyroScale(xv::Vector3d scale) {
        m_gyroScaleInv = {
            1./scale[0], 0, 0,
            0, 1./scale[1], 0,
            0, 0, 1./scale[2]
        };
    }
    void setGyroScaleInv(xv::Matrix3d scaleInv) {
        m_gyroScaleInv = scaleInv;
    }

    xv::Vector3d const& accelBias() const { return m_accelBias; }
    void setAccelBias(xv::Vector3d const& b) { m_accelBias = b; }
    xv::Matrix3d const& accelScaleInv() const { return m_accelScaleInv; }
    void setAccelScale(double scale) {
        setAccelScale({scale,scale,scale});
    }
    void setAccelScale(xv::Vector3d scale) {
        m_accelScaleInv = {
            1./scale[0], 0, 0,
            0, 1./scale[1], 0,
            0, 0, 1./scale[2]
        };
    }
    void setAccelScaleInv(xv::Matrix3d scaleInv) {
        m_accelScaleInv = scaleInv;
    }

    void apply(Imu& imu) {
        apply(imu.gyro, m_gyroBias, m_gyroScaleInv);
        apply(imu.accel, m_accelBias, m_accelScaleInv);
    }

};

class DevicePrivate : virtual public DeviceEx {

private :

   std::shared_ptr<SlamBase> m_slamVo;
   std::shared_ptr<SlamBase> m_slamHostOnly;
   std::shared_ptr<SlamBase> m_slamEdgeLocHostMap;

public:

   static bool s_slamVisionOnlyEnabled;
   static bool s_slamHostOnlyEnabled;
   static bool s_slamEnableEdgeLocHostMapping; // if enabled, the EdgeFusionOnHost will do the loc on edge and mapping on host,

   struct SlamVisionOnlyParams {
       bool m_useSmoothFilter;
       double m_cutSpeedSmoothFilter;
       double m_cutAngularSpeedSmoothFilter;
       SlamVisionOnlyParams() : m_useSmoothFilter(true), m_cutSpeedSmoothFilter(0.5),m_cutAngularSpeedSmoothFilter(0.5)  {}
   };

   bool initSlamVisionOnly(std::shared_ptr<DeviceImpl> d, bool enableOnlineLoopClosure=false);
   void setSlamVisionOnlyParams(SlamVisionOnlyParams const&p);

   // setXXXCalibration : only set the current calibration
   // writeXXXCalibration : set the current calibration and write the calibration on device

   virtual bool getImuCalibration(ImuSensorCalibration&) {return false; }
   virtual bool setImuCalibration(const ImuSensorCalibration&) {return false;}
   virtual bool writeImuCalibration(const ImuSensorCalibration&) {return false;}

   virtual bool writeDisplayCalibration(const std::vector<CalibrationEx>&) { return false; }
   
   virtual bool writeRgbCalibration(const std::vector<CalibrationEx>&) { return false; }
   virtual bool writeTofCalibration(const std::vector<CalibrationEx>&) { return false; }

   virtual bool setTofIrEnabled(bool enabled) {return false; }

   virtual bool writeFisheyeCalibration(const std::vector<CalibrationEx>&, double imuFisheyeTimestampOffset) { return false; }
   virtual double getImuFisheyeTimestampOffset() { return 0.; }

   virtual bool writeEyetrackingCalibration(const std::vector<Calibration>&) { return false; }

   /**
    * @brief Provide a SLAM without IMU data.
    */
   std::shared_ptr<SlamBase> slamVisionOnly();

   bool initSlamHostOnly(std::shared_ptr<DeviceImpl> d, bool enableOnlineLoopClosure=false, bool enableSurface=false, bool enableSurfaceTexturing=false, bool surfaceMultiResolutionMesh=false, bool surfaceMobileObjects=false, bool enableSurfacePlanes=false, bool surfaceUseFisheyes=false, bool surfaceUseFisheyeTexturing=false);

   /**
    * @brief Provide a SLAM only on host (only use IMU and fisheye images from the device).
    */
   std::shared_ptr<SlamBase> slamHostOnly();

   /**
    * @brief Provide a SLAM with localization on edge and mapping on host.
    */
   std::shared_ptr<SlamBase> slamEdgeLocHostMap();


   virtual ~DevicePrivate();
};

/**
 * @brief A mesh to find the new position of pixel of a Warp used for undistortion or rectification for stereo alignement
 */
class ImageWarpMesh {
    std::uint16_t w;
    std::uint16_t h;
    std::vector<xv::Vector2f> map;
    std::vector<std::array<std::int32_t,4>> bilinearInterpolateCache;
    std::vector<std::array<float,4>> bilinearInterpolateFloatCache;
public:
    ImageWarpMesh();
    ImageWarpMesh(std::uint16_t w, std::uint16_t h);
    xv::Vector2f const& pixel(int x, int y) const;
    xv::Vector2f& pixel(int x, int y);
    std::uint16_t width() const;
    std::uint16_t height() const;
    void initBilinearInterpolate(std::uint16_t w, std::uint16_t h);
    /**
     * @brief This unwarp function is optimize for full image unwarp and need to call #initBilinearInterpolate before
     * @param inputImage the image to unwarp
     * @return the unwarped image
     */
    GrayScaleImage unwarp(const GrayScaleImage &inputImage) const;
    /**
     * @brief This unwarp function is optimize for full image unwarp and need to call #initBilinearInterpolate before
     * @param inputImage the image to unwarp
     * @return the unwarped image
     */
    RgbImage unwarp(const RgbImage &inputImage) const;
};

/**
 * @brief Create an undistor image warp from camera calibrations and image size.
 *
 * The new camera model corresponding to the warp is camera perspective with u0,v0,fx,fy from original parameters
 *
 * @param cameraCalibrations : calibration of the camera that gives the input image of the warp
 * @param w : width of the input images
 * @param h : height of the input image
 * @return The image warp that undistor the images
 */
std::pair<ImageWarpMesh,PerspectiveCameraModel> createImageWarpMeshWithPdcm(std::vector<Calibration> cameraCalibrations, uint16_t w, uint16_t h, double zoomFactor=1.);

/**
 * @brief Create an undistor image warp from camera calibrations and image size.
 *
 * The new camera model corresponding to the warp is camera perspective with u0,v0,fx,fy from original parameters
 *
 * @param cameraCalibrations : calibration of the camera that gives the input image of the warp
 * @param w : width of the input images
 * @param h : height of the input image
 * @return The image warp that undistor the images
 */
std::pair<ImageWarpMesh,PerspectiveCameraModel> createImageWarpMeshWithUcm(std::vector<Calibration> cameraCalibrations, uint16_t w, uint16_t h, double zoomFactor=1.);

/**
 * @brief A class to compute the rectification or undistortion mesh to help undistor stereo images.
 */
class StereoRectificationMesh {
    ImageWarpMesh left;
    ImageWarpMesh right;

public:
    StereoRectificationMesh(std::vector<xv::Calibration> const& calib, bool useUcm=true);
    StereoRectificationMesh(std::vector<xv::Calibration> const& calib, double focal, double baseline);
    StereoRectificationMesh(std::vector<xv::Calibration> const& calib, std::vector<xv::Calibration> const& displayCalib);

    std::pair<xv::GrayScaleImage, xv::GrayScaleImage> rectify(const xv::GrayScaleImage &img_l, const xv::GrayScaleImage &img_r) const;

    double disparityToDepth(double disparity) const;

    static std::pair<xv::GrayScaleImage,xv::GrayScaleImage> applyMesh(xv::GrayScaleImage const& img_l, xv::GrayScaleImage const& img_r, ImageWarpMesh const& leftMesh, ImageWarpMesh const& rightMesh);
    static std::pair<ImageWarpMesh, ImageWarpMesh> initInverse(const std::vector<xv::Calibration> &calib);

    double focal() const;
    double baseline() const;

    xv::Transform const& leftVirtualPose() const;
    xv::Transform const& rightVirtualPose() const;

    ImageWarpMesh const& leftWarp() const;
    ImageWarpMesh const& rightWarp() const;

private:
    double m_baseline = -1e9;
    double m_focal = -1e9;
    xv::Transform m_P1;
    xv::Transform m_P2;
    void init(std::vector<xv::Calibration> const& calib);
    void init(const std::vector<xv::Calibration> &calib, const std::vector<xv::Calibration> &displayCalib);
    void initPdcm(std::vector<xv::Calibration> const& calib);
};

}
