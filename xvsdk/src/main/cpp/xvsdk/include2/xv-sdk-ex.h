#pragma once

#include <xv-sdk.h>

#include <mutex>
#include <unordered_map>

namespace x {
    class AprilTagDetector;
}

namespace xv {

/**
 * @brief Perspective Camera Model
 */
struct PerspectiveCameraModel {
    /**
     * @brief Image width (in pixel)
     */
    std::uint16_t w;
    /**
     * @brief Image height (in pixel)
     */
    std::uint16_t h;
    /**
     * @brief Focal length in width direction (in pixel)
     */
    double fx;
    /**
     * @brief Focal length in height direction (in pixel)
     */
    double fy;
    /**
     * @brief Optical axis intersection in width direction (in pixel)
     */
    double u0;
    /**
     * @brief Optical axis intersection in height direction (in pixel)
     */
    double v0;
};

/**
 * @brief Special Extended Unified Camera Model
 */
struct SpecialUnifiedCameraModel
{
  /**
   * @brief Image width (in pixel)
   */
  int w;
  /**
   * @brief Image height (in pixel)
   */
  int h;
  /**
   * @brief Focal length in width direction (in pixel)
   */
  double fx;
  /**
   * @brief Focal length in height direction (in pixel)
   */
  double fy;
  /**
   * @brief Optical axis intersection in width direction (in pixel)
   */
  double u0;
  /**
   * @brief Optical axis intersection in height direction (in pixel)
   */
  double v0;
  /**
   * @brief Optical center of distortion in width direction (in pixel)
   */
  double eu;
  /**
   * @brief Optical center of distortion in width direction (in pixel)
   */
  double ev;
  /**
   * @brief Optical center of distortion in height direction (in pixel)
   */
  double alpha;
  /**
   * @brief alpha
   */
  double beta;
  /**
   * @brief beta
   */
};


struct CalibrationEx : public Calibration {
/*
    std::vector<UnifiedCameraModel> ucm; //! List of Unified Camera Model parameters for differents camera resolutions (see UnifiedCameraModel#w and UnifiedCameraModel#h to find the corresponding resolution of the parameter set).
    std::vector<PolynomialDistortionCameraModel> pdcm; //! List of Polynomial Distortion Camera Model parameters for differents camera resolutions (see UnifiedCameraModel#w and UnifiedCameraModel#h to find the corresponding resolution of the parameter set).
*/
    std::vector<SpecialUnifiedCameraModel> seucm; //! List of Polynomial Distortion Camera Model parameters for differents camera resolutions (see UnifiedCameraModel#w and UnifiedCameraModel#h to find the corresponding resolution of the parameter set).
};


/**
 * Features detections and descriptors
 * @tparam F number of Fisheyes
 * @tparam DESC_SIZE number of bytes for a descriptor
 */
template <std::size_t F, std::size_t DESC_SIZE>
struct FisheyeKeyPoints {
    std::int64_t version; //! Field use to help parsing and future evolutions of this struct
    std::int64_t id; //! Unique id given by the edge to this instance
    double hostTimestamp = std::numeric_limits<double>::infinity(); //!< host timestamp of the physical measurement (in second based on the `std::chrono::steady_clock`), need to activate IMU stream (with SLAM or IMU callback) to have this value.
    std::int64_t edgeTimestampUs = std::numeric_limits<std::int64_t>::min(); //!< timestamp of the physical measurement (in microsecond based on edge clock).
    struct Features {
       std::size_t size; //! number of feature detected
       std::shared_ptr<const float> keypoints; //! list of detected keypoints (xy) in image (size x 2 float elements)
       std::shared_ptr<const unsigned char> descriptors; //! list of (DESC_SIZE x size) keypoints descriptors in image
    };
    std::array<Features,F> descriptors; //! usually first is left and second is right, it is the same order fisheyes calibrations

    /**
     * @brief Size of a feature descriptor
     */
    std::size_t descriptorSize() const { return DESC_SIZE; }
};

#if 0
struct FisheyeKeyPointsDescriptor {
    std::int64_t version; //! Field use to help parsing and future evolutions of this struct
    std::int64_t id; //! Unique id given by the edge to this instance
    double hostTimestamp = std::numeric_limits<double>::infinity(); //!< host timestamp of the physical measurement (in second based on the `std::chrono::steady_clock`), need to activate IMU stream (with SLAM or IMU callback) to have this value.
    std::int64_t edgeTimestampUs = std::numeric_limits<std::int64_t>::min(); //!< timestamp of the physical measurement (in microsecond based on edge clock).
    struct Features {
       std::size_t size; //! number of feature detected
       std::shared_ptr<const float> keypoints; //! list of detected keypoints (xy) in image (size x 2 float elements)
       std::shared_ptr<const unsigned char> descriptors; //! list of (DESC_SIZE x size) keypoints descriptors in image
    };
    std::vector<Features> descriptors; //! usually first is left and second is right, it is the same order fisheyes calibrations

    /**
     * @brief Size of a feature descriptor
     */
    std::size_t descriptorSize() const { return 32; }
};
#endif

enum class StereoMode {
    IMAGES = 0, ///< Images only
    IMAGES_AND_DATA, ///< Images and data
    DATA,  ///< Data only
    NO_STREAMS, ///< No stream
};


/**
 * @brief Data from IMU sensor for the XVisio handle.
 */
struct HandleImu {
    std::array<unsigned char,32> raw;

    //enum class Position {Head = 0, Left, Right};
    //enum class DataType {Init = 0, Work};

    //Position position; //!< 3-axis gyrometer values of left handle (in rad/s)
    //DataType dataType; //!< 3-axis gyrometer values of left handle (in rad/s)
    //int status;
    //Vector3d lgyro; //!< 3-axis gyrometer values of left handle (in rad/s)
    //Vector3d rgyro; //!< 3-axis gyrometer values of right handle (in rad/s)
    //Vector3d laccel; //!< 3-axis accelerometer values of left handle (in m/s²)
    //Vector3d raccel; //!< 3-axis accelerometer values of right handle (in m/s²)
    //double hostTimestamp = std::numeric_limits<double>::infinity(); //!< host timestamp of the physical measurement (in second based on the `std::chrono::steady_clock`).
    //std::uint32_t edgeTimestampUs = std::numeric_limits<std::uint32_t>::min(); //!< timestamp of the physical measurement (in microsecond based on edge clock).
};


/**
 * @brief The class to give access to data provided by the IMU sensor.
 */
class HandleImuSensor : virtual public Stream<HandleImu const &> {
public:

    virtual ~HandleImuSensor(){}
};


/**
 * @brief For senior developer.
 */
class DeviceEx : virtual public Device {

public:

    static std::shared_ptr<DeviceEx> instance; /// FIXME not compatible with multi-device

    /**
     * @brief Get the secondary SLAM component to make edge and mixed slam work at sametime. Default in edge mode.
     */
    virtual std::shared_ptr<Slam> slam2() = 0;

    virtual std::shared_ptr<HandleImuSensor> handleImuSensor() = 0;
    virtual std::shared_ptr<FisheyeCameras> handleFisheyeCameras() = 0;

    /**
     * @brief Start use external IMU.
     */
    virtual void enableImuInput() = 0;
    /**
     * @brief Stop use external IMU.
     */
    virtual void disableImuInput() = 0;
    /**
     * @brief Send external IMU to SDK.
     */
    virtual void pushImu(Imu const& imu, bool with_bias_correction=false, bool with_timestamp_correction=false) = 0;

    enum class StereoInputType {ImageOnly, KeyPointsOnly, Both, None};

    /**
     * @brief Start use external fisheye image and/or key points.
     *
     * only image,only key points,both
     */
    virtual bool enableStereoInput(StereoInputType type) = 0;
    /**
     * @brief Stop use external fisheye image and/or key points.
     */
    virtual bool disableStereoInput() = 0;
    /**
     * @brief Send external fisheye image and/or key points to SDK.
     */
    virtual void pushStereo(FisheyeKeyPoints<2,32> const& stereo) = 0;

    virtual bool setDisplayCalibration(const std::vector<CalibrationEx>&) = 0;
    virtual bool setRgbCalibration(const std::vector<CalibrationEx>&) = 0;
    virtual bool setTofCalibration(const std::vector<CalibrationEx>&) = 0;
    virtual bool getFisheyeCalibration(std::vector<CalibrationEx>&, double& imuFisheyeTimestampOffset) = 0;
    virtual bool setFisheyeCalibration(const std::vector<CalibrationEx>&, double imuFisheyeTimestampOffset) = 0;
    virtual bool setEyetrackingCalibration(const std::vector<CalibrationEx>&) = 0;

    virtual bool setImuOffset( int offset ) = 0;
    virtual bool setImuMode( int mode ) = 0;


    virtual bool setFeMode( StereoMode mode ) = 0;

    virtual bool setEdgePrediction( int prediction ) = 0;

    /**
     * @brief Get the third SLAM component to make edge and mixed slam work at sametime. Default is edge with fusion on host mode.
     */
    virtual std::shared_ptr<Slam> slam3() = 0;

    /**
     * @brief Return the customize flash array.
     */
    virtual std::string getCustomizeFlash48BytesArray1() = 0;

    /**
     * @brief Set the customize flash array.
     */
    virtual bool setCustomizeFlash48BytesArray1(std::string flashArray) = 0;

    virtual void setDeviceOffsetStatus() = 0;


    virtual ~DeviceEx() { }
};

/**
 * @deprecated
 */
struct FisheyeImageKeyPoints {
    double hostTimestamp = std::numeric_limits<double>::infinity(); //!< host timestamp of the physical measurement (in second based on the `std::chrono::steady_clock`), need to activate IMU stream (with SLAM or IMU callback) to have this value.
    std::int64_t edgeTimestampUs = (std::numeric_limits<std::int64_t>::min)(); //!< timestamp of the physical measurement (in microsecond based on edge clock).
    struct Detections {
        std::vector<Vector2<std::uint16_t>> keypoints; //! list of detected keypoints in image
        std::vector<std::array<std::int16_t,16>> descriptors; //! list of keypoints descriptors in image
        GrayScaleImage image; //! Fisheye image
    };
    std::vector<Detections> detections; //! List of image detections (typically 2, first is left and second image is the right image)
    std::int64_t id;//! Unique id given by the edge to this instance
};

struct TagInfo {
  std::string family;
  int id;
  double size;
};

struct TagPoseInfo : public TagInfo {
  xv::Transform transform;
};

struct TagPose {
    int tagId;
    xv::Transform transform;
    double confidence; // in [0,1]
};

struct TagDetection {
    int tagId;
    std::array<std::array<double,2>,4> corners;
    double confidence; // in [0,1]
};

class AprilTagDetector {
public:
    /**
     * @brief Construct an AprilTag detector
     * @param c multi camera calibration
     * @param f name of the AprilTag family to use (support: "36h11" "25h9" "16h5" and "14h12")
     */
    explicit AprilTagDetector(std::vector<xv::CalibrationEx> const& c, std::string const& f="36h11", bool subpixelic=false);

    /**
     * @brief Construct an AprilTag detector on single view
     * @param c camera intrinsics
     * @param camerPose camera pose
     * @param f name of the AprilTag family to use (support: "36h11" "25h9" "16h5" and "14h12")
     */
    explicit AprilTagDetector(xv::PolynomialDistortionCameraModel const& c, xv::Transform const& camerPose, std::string const& f="36h11", bool subpixelic=false);

    /**
     * @brief Construct an AprilTag detector on single view
     * @param c camera calibration
     * @param camerPose camera pose
     * @param f name of the AprilTag family to use (support: "36h11" "25h9" "16h5" and "14h12")
     */
    explicit AprilTagDetector(xv::UnifiedCameraModel const& c, xv::Transform const& camerPose, std::string const& f="36h11", bool subpixelic=false);
    
    /**
     * @brief Construct an AprilTag detector on single view
     * @param c camera calibration
     * @param camerPose camera pose
     * @param f name of the AprilTag family to use (support: "36h11" "25h9" "16h5" and "14h12")
     */
    explicit AprilTagDetector(xv::SpecialUnifiedCameraModel const& c, xv::Transform const& camerPose, std::string const& f="36h11", bool subpixelic=false);

    /**
     * @brief Construct an AprilTag detector without camera calibration (only 2D detections are available)
     * @param c multi camera calibration
     * @param f name of the AprilTag family to use (support: "36h11" "25h9" "16h5" and "14h12")
     */
    explicit AprilTagDetector(std::string const& f="36h11", bool subpixelic=false);

    /**
     * @brief Detect AprilTags in Fisheye Images and return the poses of the tags
     * @param tagSize : of the side of the april tag to detect (in m)
     * @return vector of pairs with tag id and pose of the tag
     */
    std::vector<TagPose> detect(xv::FisheyeImages const& fe, double tagSize) const;

    /**
     * @brief Detect AprilTags in a grayscale image and return the poses of the tags
     * @param tagSize : of the side of the april tag to detect (in m)
     * @return vector of pairs with tag id and pose of the tag
     */
    std::vector<TagPose> detect(xv::GrayScaleImage const& im, double tagSize) const;

    /**
     * @brief Detect AprilTags in a grayscale image and return the 4 corners of each detected tag
     * @return vector of pairs with tag id and 4 corners of the tag (in pxl)
     */
    std::vector<TagDetection> detect(const xv::GrayScaleImage& fe);

    /**
     * @brief Compute the poses of the tags detections
     * @param detections: tag detections
     * @param tagSize: size of the tag (in m)
     * @return the tag poses
     */
    std::vector<TagPose> detectionsToPoses(const std::vector<TagDetection>& detections, double tagSize);

    /**
     * @brief Detect AprilTags in a multiple cameras system and return the 4 corners of each detected tag
     * @return map of detections by tag id, each vector of tag detection has the size of the number of images
     */
    std::map<int, std::vector<TagDetection> > detect(const xv::FisheyeImages &fe);
    /**
     * @brief Compute the poses of the tags detections for multiple cameras case
     * @param detections: tag detections grouped by tagId, each vector of tag detection has the size of the number of images
     * @param tagSize: size of the tag (in m)
     * @return the tag poses
     */
    std::vector<TagPose> detectionsToPoses(const std::map<int, std::vector<TagDetection> > &detectionsByTagId, double tagSize);

    /**
     * @brief Start a tag detectors
     * @param slam : SLAM to use for localisation of the tag. The detected tags will be in world frame coordinates as defined by the SLAM.
     * @param tagFamily : can be "41h12" "36h11" "25h9" or "16h5" (AprilTag)
     * @param size : size in m of the real tag side
     * @param refreshRate : the refresh rate used for the detection (in Hz)
     * @return Id of the started detector.
     */
    static std::string startTagDetector(std::shared_ptr<xv::FisheyeCameras>, std::shared_ptr<Slam> slam, std::string const& tagFamily, double size, double refreshRate);
    /**
     * @brief Start a tag detectors
     * @param slam : SLAM to use for localisation of the tag. The detected tags will be in world frame coordinates as defined by the SLAM.
     * @param tagFamily : can be "41h12" "36h11" "25h9" or "16h5" (AprilTag)
     * @param size : size in m of the real tag side
     * @param refreshRate : the refresh rate used for the detection (in Hz)
     * @return Id of the started detector.
     */
    static std::string startTagDetector(std::shared_ptr<xv::ColorCamera>, std::shared_ptr<Slam> slam, std::string const& tagFamily, double size, double refreshRate);

    /**
     * @brief Stop a tag detector.
     * @param detectorId : detector id (see output of #startTagDetector)
     * @return True if succeeded to stop the detector.
     */
    static bool stopTagDetector(std::string const& detectorId);
    /**
     * @brief Get the current localized tag detections in SLAM world frame coordinates.
     *
     * @param detectorId : detector id
     * @return Poses of all the detected tags, even if the tag is not visible, if it was once detected it remains in this output map.
     */
    static std::map<int,xv::Pose> getTagDetections(std::string const& detectorId);


private:
    std::shared_ptr <x::AprilTagDetector> pimpl;
};


class ColorCameraEx : public ColorCamera, public std::enable_shared_from_this<ColorCameraEx> {

public:
    std::shared_ptr<ColorCameraEx> getThis();

    /**
     * @brief Start a tag detectors
     * @param slam : SLAM to use for localisation of the tag. The detected tags will be in world frame coordinates as defined by the SLAM.
     * @param tagFamily : can be "41h12" "36h11" "25h9" or "16h5" (AprilTag)
     * @param size : size in m of the real tag side
     * @param refreshRate : the refresh rate used for the detection (in Hz)
     * @return Id of the started detector.
     */
    std::string startTagDetector(std::shared_ptr<Slam> slam, std::string const& tagFamily, double size, double refreshRate);
    /**
     * @brief Stop a tag detector.
     * @param detectorId : detector id (see output of #startTagDetector)
     * @return True if succeeded to stop the detector.
     */
    bool stopTagDetector(std::string const& detectorId);
    /**
     * @brief Get the current localized tag detections in SLAM world frame coordinates.
     *
     * @param detectorId : detector id
     * @return Poses of all the detected tags, even if the tag is not visible, if it was once detected it remains in this output map.
     */
    std::map<int,xv::Pose> getTagDetections(std::string const& detectorId);
};

/**
 * @brief For senior developer.
 */
class FisheyeCamerasEx : public FisheyeCameras, public std::enable_shared_from_this<FisheyeCamerasEx>
{

    std::mutex m_aprilTagDetectorsMtx;
    std::unordered_map<std::string, std::shared_ptr<AprilTagDetector>> m_aprilTagDetectors;
    std::shared_ptr<AprilTagDetector> getDetector(std::string const& tagFamily);

    std::mutex m_lastFisheyeImageMtx;
    xv::FisheyeImages m_lastFisheyeImage;
    int m_lastFisheyeImageCbId = -1;

    class TagDetectorThread;

    std::shared_ptr<FisheyeCamerasEx> getThis();

public:

    virtual int registerKeyPointsCallback(std::function<void (const FisheyeKeyPoints<2,32>&)>) = 0;
    virtual int registerKeyPointsCallback(std::function<void (const FisheyeKeyPoints<4,32>&)>) = 0;
    virtual bool unregisterKeyPointsCallback(int callbackId) = 0;
    virtual bool unregisterKeyPoints4Callback(int callbackId) = 0;

    virtual const std::vector<CalibrationEx>& calibrationEx() = 0;

    virtual xv::FisheyeImages lastImages() = 0;

    /**
     * @brief The ResolutionMode enum
     */
    enum class ResolutionMode {
        LOW = 1, ///< Low resolution (typically QVGA)
        MEDIUM = 2, ///< Medium resolution (typically VGA)
        HIGH = 3 ///< High resolution (typically HD 720)
    };
    virtual bool setResolutionMode(ResolutionMode mode) = 0;

    /**
     * @brief Detect tags in a fisheye image
     * @param img : fisheye image to used for detection
     * @param tagFamily : can be "41h12" "36h11" "25h9" or "16h5" (AprilTag)
     * @return Vector of tag detections, each tag detection is a pair containing the tag id (AprilTag id) and four corners coordinates (in pixel) of the detected tags.
     */
    std::vector<std::pair<int,std::array<Vector2d,4>>> detectTags(xv::GrayScaleImage const& img, std::string const& tagFamily);

    /**
     * @brief Detect tags in the last fisheye images
     * @param tagFamily : can be "41h12" "36h11" "25h9" or "16h5" (AprilTag)
     * @param size : size in m of the real tag side
     * @return Vector of tag detections, each tag detection is a pair containing the tag id (AprilTag id) and the 6dof poses (in Fisheyes frame coordinates).
     * The confidence of the pose reflects the confidence of the tag detection.
     */
    std::vector<std::pair<int,xv::Pose>> detectTags(std::string const& tagFamily, double size);
    /**
     * @brief Detect tags in fisheye images and return poses of the tags
     * @param fe : fisheye images to use for detection
     * @param tagFamily : can be "41h12" "36h11" "25h9" or "16h5" (AprilTag)
     * @param size : size in m of the real tag side
     * @return Vector of tag detections, each tag detection is a pair containing the tag id (AprilTag id) and the 6dof poses (in Fisheyes frame coordinates).
     * The confidence of the pose reflects the confidence of the tag detection.
     */
    std::vector<std::pair<int,xv::Pose>> detectTags(xv::FisheyeImages const& fe, std::string const& tagFamily, double size);
    /**
     * @brief Detect tags in the last fisheye images and return poses of the tags in SLAM world frame coordinates
     * @param slam : the slam to use for localization of detection in SLAM world frame coordinates
     * @param tagFamily : can be "41h12" "36h11" "25h9" or "16h5" (AprilTag)
     * @param size : size in m of the real tag side
     * @return Vector of tag detections, each tag detection is a pair containing the tag id (AprilTag id) and the 6dof poses (in Fisheyes frame coordinates).
     * The confidence of the pose reflects the confidence of the tag detection.
     */
    std::vector<std::pair<int,xv::Pose>> detectTags(std::shared_ptr<Slam> slam, std::string const& tagFamily, double size);

    /**
     * @brief Start a tag detectors
     * @param slam : SLAM to use for localisation of the tag. The detected tags will be in world frame coordinates as defined by the SLAM.
     * @param tagFamily : can be "41h12" "36h11" "25h9" or "16h5" (AprilTag)
     * @param size : size in m of the real tag side
     * @param refreshRate : the refresh rate used for the detection (in Hz)
     * @return Id of the started detector.
     */
    std::string startTagDetector(std::shared_ptr<Slam> slam, std::string const& tagFamily, double size, double refreshRate);
    /**
     * @brief Stop a tag detector.
     * @param detectorId : detector id (see output of #startTagDetector)
     * @return True if succeeded to stop the detector.
     */
    bool stopTagDetector(std::string const& detectorId);
    /**
     * @brief Get the current localized tag detections in SLAM world frame coordinates.
     *
     * @param detectorId : detector id
     * @return Poses of all the detected tags, even if the tag is not visible, if it was once detected it remains in this output map.
     */
    std::map<int,xv::Pose> getTagDetections(std::string const& detectorId);

    virtual ~FisheyeCamerasEx() { }

};

/**
 * @brief Compute the pixel shift to go from tracker pose p0 to tracker pose p1
 *
 * If p0 and p1 are at the same translation (rotation onlty) this function can be used for ATW and the parameter d is useless. If p0 and p1 are with different translation the pixel shift correspond
 * to a virtual object at d meter from the display.
 *
 * @param p0 : tracker pose
 * @param p1 : tracker pose (need to be in same frame coordinate as p)1
 * @param displayExtrinsics : display pose in tracker pose
 * @param displayCalib : display intrinsics
 * @param d : distance of the virtual object to use to estimate the pixel shift
 * @return (x,y) pixel shift corresponding to the motion from p0 to p1
 */
xv::Vector2d getPixelShift(xv::Pose const& p0, xv::Pose const& p1, xv::Transform const& displayExtrinsics, xv::CameraModel const& displayCalib, double d=1.);

class CameraEx : public Camera {
public:
    virtual const std::vector<CalibrationEx>& calibrationEx() = 0;
    virtual std::shared_ptr<CameraModel> cameraModel() { return nullptr; }
    virtual std::vector<std::shared_ptr<CameraModel>> camerasModel() {
        if (cameraModel())
            return {cameraModel()};
        else
            return {};
    }
};

class DisplayEx : public Display {
public:
    virtual const std::vector<CalibrationEx>& calibrationEx() = 0;
    virtual std::vector<std::shared_ptr<CameraModel>> camerasModel() {
            return {};
    }
};

namespace ex {

struct PointCloud
{
  std::int32_t version;
  std::uint64_t id;
  std::uint32_t xyznSize;
  std::shared_ptr<const std::array<float,6>> xyzn; //!< oriented point cloud in sensor frame
  Transform sensorPose;                            //!< sensor pose in world
};

struct PointClouds {
    std::map<std::uint64_t, xv::ex::PointCloud> pointClouds;
};

struct Surface
{
  std::int32_t version;
  std::uint64_t id;

  std::uint32_t verticesSize;
  std::shared_ptr<const std::array<float,3>> vertices;
  std::shared_ptr<const std::array<float,3>> vertexNormals;

  std::uint32_t trianglesSize;
  std::shared_ptr<const std::array<std::uint32_t,3>> triangles;

  std::shared_ptr<const std::array<float,2>> textureCoordinates; //!< one per vertex
  std::uint32_t textureWidth;
  std::uint32_t textureHeight;
  std::shared_ptr<const std::uint8_t> textureRgba;    //!< row major
};

struct Surfaces {
    std::map<std::uint64_t, xv::ex::Surface> surfaces;
};

}

class SlamEx : public Slam {

protected:

    bool m_enableOnlineLoopClosure = false;
    bool m_enableSurface = false;
    bool m_enableSurfaceTexturing = false;
    bool m_enableSurfaceMultiResolutionMesh = false;
    bool m_enableSurfaceMobileObjects = false;
    bool m_enableSurfacePlanes = false;
    bool m_surfaceUseFisheyes = false; //!< instead of Tof as depth source
    bool m_surfaceUseFisheyeTexturing = true; //!< fisheye texturing instead of RGB texturing
    double m_surfaceMinVoxelSize = 0.1;

public:

    virtual void setEnableOnlineLoopClosure(bool enable) { m_enableOnlineLoopClosure = enable; }
    virtual void setEnableSurface(bool enable) { m_enableSurface = enable; }
    virtual void setEnableSurfaceTexturing(bool enable) { m_enableSurfaceTexturing = enable; }
    virtual void setEnableSurfaceMultiResolutionMesh(bool enable) { m_enableSurfaceMultiResolutionMesh = enable;}
    virtual void setEnableSurfaceMobileObjects(bool enable) { m_enableSurfaceMobileObjects = enable;}
    virtual void setEnableSurfacePlanes(bool enable) { m_enableSurfacePlanes = enable;}
    virtual void setSurfaceUseFisheyes(bool use) { m_surfaceUseFisheyes = use;}
    virtual void setSurfaceUseFisheyeTexturing(bool use) { m_surfaceUseFisheyeTexturing = use;}
    virtual void setSurfaceMinVoxelSize(double size) { m_surfaceMinVoxelSize = size; }
    virtual int registerLocal3dPointsCallback(std::function<void (std::shared_ptr<const std::vector<std::array<double,3>>>)>) = 0;
    virtual bool unregister3dPointsCallback(int callbackId) = 0;
    virtual bool getLastVSlamPose(Pose &) { return false; }

    virtual bool startSurfaceReconstruction() { return false; }
    virtual bool stopSurfaceReconstruction() { return false; }
    virtual bool startPlaneDetection() { return false; }
    virtual bool stopPlaneDetection() { return false; }

    /**
     * @brief Define a map of tags poses
     * @param tagFamily : can be "41h12" "36h11" "25h9" or "16h5" (AprilTag)
     * @param tagSize : size in m of the real tag side
     * @param tagIds : AprilTag id of each tag
     * @param poses : poses of the AprilTag in a world frame coordinate. This world frame coordinates will then be used
     */
    virtual void setTagsMap(std::string const& tagFamily, double tagSize, std::vector<int> tagIds, std::vector<xv::Transform> poses) = 0;

    /**
     * @brief Get the current 6dof pose of the device in tags map
     *
     * Same as #getPose but return true even if it can localize on the map defined by #setTagsMap. The output pose is relative to this tags map thanks to the tags detection.
     *
     * @param[out] pose corresponding to the timestamp "now" + "prediction"
     * @param[in] prediction (in s) amount of prediction to use to get a pose corresponding to the future
     * @return true if localized on tag map, false else.
     */
    virtual bool getPoseInTagsMap(Pose& pose, double prediction = 0.) = 0;

    /**
    * @brief Callback to get the reconstructed pointcloud (is computed faster than surface reconstruction but is not a mesh).
    * @return Id of the callback (used to unregister the callback).
    */
    virtual int registerPointCloudCallback(std::function<void (std::shared_ptr<const ex::PointClouds>)>) = 0;
    virtual bool unregisterPointCloudCallback(int callbackId) = 0;
    virtual bool getPointCloud(std::shared_ptr<const ex::PointClouds>&) = 0;

    /**
     * @brief Callback to get the reconstructed surface updates.
     * @return Id of the callback (used to unregister the callback).
     */
    virtual int registerSurfaceCallback(std::function<void (std::shared_ptr<const xv::ex::Surfaces>)>) = 0;
    virtual bool unregisterSurfaceCallback(int callbackId) = 0;
    virtual bool getSurface(std::shared_ptr<const xv::ex::Surfaces>&) = 0;

    /**
     * @brief Set the tag tos use in SLAM; must be done before calling slam->start()
     * @param v : list of tags
     * @return success
     */
    virtual bool addTags(std::vector<TagInfo> const& v) = 0;

    /**
     * @brief Get the tags used in the SLAM
     * @param tagId : unique tag identifier
     * @param pose : pose of the tag in the SLAM coordinate frame
     * @param tagSize : size of the tag
     * @return success
     */
    virtual bool onTagUpdate(std::function<void(std::string const& tagId, xv::Transform const& pose, double const& tagSize)>) = 0;
};

/**
 * @brief To compute the 3D position of 2D pixel point of RGB image.
 *
 * It uses raytrace of RGB pixel and ToF image to get the depth. If SLAM is running, then the output 3D position is in World frame coordinate of the SLAM,
 * else it is relative to the IMU frame coordinate.
 */
class RgbPixelPoseWithTof {

public:
    RgbPixelPoseWithTof(std::shared_ptr<xv::Device> d);

    /**
     * @brief Get the position of the pointed area in RGB image, it uses ToF to determine the depth.
     *
     * If SLAM is running, then the output pointerPose is in World frame coordinate of the SLAM, else it is relative to the IMU frame coordinate.
     *
     * @param pointerPose: 3D position in World frame coordinate of the point selected in RGB
     * @param hostTimestamp: timestamp corresponding to the pointing
     * @param rgbPixelPoint: xy position of the point in color image (in pixel)
     * @param radius: size of the area to select for ToF 3D points selection
     * @return true if succes, false else
     */
    bool getRgbPixel3dPoseAt(xv::Vector3d& pointerPose, double hostTimestamp, xv::Vector2d const& rgbPixelPoint, double radius);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

public:
    ~RgbPixelPoseWithTof();
};

/**
* eye data function pointer
*/
typedef void (* xv_ET_point_process_callback)(int index, int percent, void* context);//!<Callback function used to receive calibration progress of calibration point.
typedef void (* xv_ET_point_finish_callback)(int index, int error, void* context);//!<Callback function used to receive the completion status of calibration point.

/**
 * @brief A class to handle interfaces of the gaze calibration operations.
 */
class GazeCalibration{
public:

    /**
     * @brief Start calibration
     *
     * @param[in] points Total number of calibration points.
     * @return int 
	 *  0 success
	 * -1 Failed to start calibration, no permission.
	 * -2 Initialization failed.
	 * -212 or -7001 Parameter error
     */
    int StartCalibration(int points);

    /**
     * @brief Start calibrating a point
     *
     * @param[in] eye Calibrate the left and right eyes, 1-left eye and 2-right eye. Note that the left and right eyes are not calibrated together and must be calibrated separately.
     * @param[in] index Calibration point index.
     * @param[in] point Datum coordinates of calibration points (normalized values are used for X and y).
     * @param[in] cb1 The callback when this point is calibrated.
     * @param[in] context1 The callback corresponding to CB1 is used to pass the context of the caller, can be empty.
     * @param[in] cb2 The callback when eyeball information of each image is recalled.
     * @param[in] context2 The callback corresponding to CB2 is used to pass the context of the caller, can be empty.
     * @return int 
	 *  0 success
	 * -1 Failed to start calibration, no permission.
	 * -2 Initialization failed.
	 * -221 Eye is not set.
     * -221222 Parameter error, start calibration index point error.
     */
    int StartCalibrationPoints(int eye, int index, const xv::XV_ET_POINT_2D* point, 
        xv::xv_ET_point_process_callback cb1, void* context1, 
        xv::xv_ET_point_finish_callback cb2, void* context2);

    /**
     * @brief Cancel calibration
     *
     * @param[in] eye Calibrate the left and right eyes, 1-left eye and 2-right eye. Note that the left and right eyes are not calibrated together and must be calibrated separately.
     * @return int 
	 *  0 success
	 * -1 Failed to start calibration, no permission.
	 * -2 Initialization failed.
	 * -241 Eye is not set.
     */
    int CancelCalibration(int eye);

    /**
     * @brief Compute calibration
     *
     * @param[in] eye Calibrate the left and right eyes, 1-left eye and 2-right eye. Note that the left and right eyes are not calibrated together and must be calibrated separately.
     * @param[out] out_coe Calibration coefficient.
     * @return int 
	 *  0 success
	 * -1 Failed to start calibration, no permission.
	 * -2 Initialization failed.
     * -251 Eye is not set.
     * -253 Calibration coefficient parameter is null.
	 * -251252 Parameter error
     */
    int ComputeCalibration(int eye, xv::XV_ET_COEFFICIENT* out_coe);

    /**
     * @brief Complete calibration
     *
     * @return int 
	 *  0 success
	 * -1 Failed to start calibration, no permission.
	 * -2 Initialization failed.
     */
    int CompleteCalibration();

    /**
     * @brief Set the calibration range and default calibration factor.
     *
     * @param[in] eye Calibrate the left and right eyes, 1-left eye and 2-right eye. Note that the left and right eyes are not calibrated together and must be calibrated separately.
     * @param[in] minX Minimum value of X coordinate system.
     * @param[in] maxX Maximum value of X coordinate system.
     * @param[in] minY Minimum value of Y coordinate system.
     * @param[in] maxY Maximum value of Y coordinate system.
     * @param[in] coe Default coefficient, can be null.
     * @return int 
	 *  0 success
	 * -1 Failed to start calibration, no permission.
	 * -2 Initialization failed.
     * -3 Wrong sdk version.
     * -271 Eye is not set.
	 * -272 Parameter error
     */
    int SetDefaultCalibration(int eye, float minX, float maxX, float minY, float maxY, const xv::XV_ET_COEFFICIENT* coe);

    /**
     * @brief Input camera image
     *
     * @param[in] image Input image.
     * @param[in] size Image size.
     * @param[in] width Image width.
     * @param[in] height Image height.
     * @param[in] timestamp Image timestamp.
     * @return int 
	 *  0 success
     */
    int InputCameraImage(const unsigned char* image, int size, int width, int height, long long timestamp) ;
};

/**
 * @brief Obtain a virtual #Device, and input sensor data to the sdk from caller.
 * @return A #Device.
 */
std::shared_ptr<Device> getVirtualDevice();

}
