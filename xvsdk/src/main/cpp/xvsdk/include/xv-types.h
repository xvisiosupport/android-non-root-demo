#pragma once
#define NOMINMAX

#include <array>
#include <vector>
#include <limits>
#include <string>
#include <memory>

#if defined(__ANDROID__) && !defined(__x86_64__)
#define __XV_DRIVER_ONLY__
#endif

namespace xv {

/**
 * @brief Log levels.
 */
enum class LogLevel {
        debug = 1,
        info,
        warn,
        err,
        critical,
        off
};

enum class SlamStartMode {
    Normal,
    VisionOnly,
    VisionWithGYRO
};

enum DeviceSupport {
    ONLYUSB,
    ONLYDRIVER,
    USBANDDRIVER,
    ONLY_WIRELESS_CONTROLLER
};


/**
 * @brief Plug event types.
 */
enum class PlugEventType { Plugin, Unplug };

/**
 * \defgroup packed_struct Align(1) structs.
 * @{
 */
#pragma pack(1)

/**
 * @brief Exposure settings.
 */
struct ExpSetting{
	std::uint32_t iso_value;
	std::uint32_t exp_abs;
	std::uint8_t  exp_mode;
	std::int8_t   exp_level;
	std::uint8_t  exp_anti;    // ANTIBANDING_MODES
	std::uint8_t  iso_mode;
};

/**
 * @brief AWB settings.
 */
struct AwbSetting
{
    std::uint8_t awb_mode;
    std::uint8_t awb_lock;
};

/**
 * @brief AF settings.
 */
struct AfSetting
{
    std::uint8_t af_mode;
    std::uint8_t af_dist;
};

/**
 * @brief Device setting.
 */
struct DeviceSetting
{
    std::uint32_t cmd;   //sensorId:8 + channel:8 + sensorCmd:16

	union {
		std::uint8_t val[24];
		AwbSetting awb;
		ExpSetting exp;
		AfSetting  af;
        std::int16_t val16;
        std::int32_t val32;
	}args;
};

struct IspAecSetting
{
    int32_t maxIsoGainValue;
    int32_t minIsoGainValue;
    int32_t maxExposureTimeUs;
    int32_t minExposureTimeUs;
    uint8_t targetMeanPixelValue;
    int32_t ratioIso;
    int32_t cpt_aec_no_modify_range;
    float exposureTime_control_porprotion_value;
    float exposureTime_control_integral_value;
    float isoGain_control_porprotion_value;
    float isoGain_control_integral_value;
    uint32_t ISP_AEC_MODE;
};

#pragma pack()
/** @} */


/**
 * \defgroup Version Version helper.
 * @{
 */
/**
 * @brief The Version struct
 */
struct Version{
    Version( int major = 0, int minor = 0, int patch = 0 );
    Version( const std::string &s );

    int major = 0; //!< Major number of the version
    int minor = 0; //!< Minor number of the version
    int patch = 0; //!< Patch number of the version

    int key() const;
    std::string toString() const;

    static int (max)();
};

bool operator<(const Version &a, const Version &b);
bool operator<=(const Version &a, const Version &b);
bool operator==(const Version &a, const Version &b);
bool operator!=(const Version &a, const Version &b);
bool operator>=(const Version &a, const Version &b);
bool operator>(const Version &a, const Version &b);

std::ostream& operator<<(std::ostream& o, Version const& v);
/** @} */


template<class F> using Vector2 = std::array<F,2>; //!< \var 2D vector
template<class F> using Vector3 = std::array<F,3>; //!< \var 3D vector
template<class F> using Vector4 = std::array<F,4>; //!< \typedef 4D vector
template<class F> using Matrix3 = std::array<F,9>; //!< \typedef Row major 3x3 matrix

typedef Vector2<bool> Vector2b; //!< \typedef 2D vector of `bool'
typedef Vector3<bool> Vector3b; //!< \typedef 3D vector of `bool'
typedef Vector4<bool> Vector4b; //!< \typedef 4D vector of `bool'
typedef Vector2<double> Vector2d; //!< \typedef 2D vector of `double'
typedef Vector3<double> Vector3d; //!< \typedef 3D vector of `double'
typedef Vector4<double> Vector4d; //!< \typedef 4D vector of `double'
typedef Matrix3<double> Matrix3d; //!< \typedef Row major 3x3 matrix of `double'
typedef Vector2<float> Vector2f; //!< \typedef 2D vector of `float'
typedef Vector3<float> Vector3f; //!< \typedef 3D vector of `float'
typedef Vector4<float> Vector4f; //!< \typedef 4D vector of `float'
typedef Matrix3<float> Matrix3f; //!< \typedef Row major 3x3 matrix of `float'

/**
 * \defgroup xv_rotation_conversions functions to convert between 3D rotations representations
 * @{
 */
/**
 * @brief Convert a rotation matrix to quaternion.
 * @param rot: 3x3 row major rotation matrix
 * @return quaternion [qx,qy,qz,qw]
 */
Vector4f rotationToQuaternion(Matrix3f const& rot);
/**
 * @brief Convert a rotation matrix to quaternion.
 * @param rot: 3x3 row major rotation matrix
 * @return quaternion [qx,qy,qz,qw]
 */
Vector4d rotationToQuaternion(Matrix3d const& rot);

/**
 * @brief Convert quaternion to rotation matrix.
 * @param q: quaternion [qx,qy,qz,qw]
 * @return 3x3 row major rotation matrix
 */
Matrix3f quaternionToRotation(Vector4f const& q);

/**
 * @brief Convert quaternion to rotation matrix.
 * @param q: quaternion [qx,qy,qz,qw]
 * @return 3x3 row major rotation matrix
 */
Matrix3d quaternionToRotation(Vector4d const& q);

/**
 * @brief Deprecated. Same to #quaternionToRotation.
 */
Matrix3f quaternionsToRotation(Vector4f const& q);

/**
 * @brief Convert rotation Euler angles.
 *
 * Be carefull of gimbal lock when using Euler angles.
 *
 * @param rot: 3x3 row major rotation matrix
 * @return [pitch, yaw, roll] in radians
 */
Vector3d rotationToPitchYawRoll(Matrix3d const& rot);

/**
 * @brief Convert rotation Euler angles.
 *
 * Be carefull of gimbal lock when using Euler angles.
 *
 * @param rot: 3x3 row major rotation matrix
 * @return [pitch, yaw, roll] in radians
 */
Vector3f rotationToPitchYawRoll(Matrix3f const& rot);

/** @}
 *
*/

/**
 * Namespace with details (not needed for API usage).
 */
namespace details {

/**
 * Pose of an object in a parent frame coordinate, it correspond to the transformation from object (current) frame coordinates to parent frame coordinates.
 */
template <class F=double>
class Transform_ {

protected:
    Vector3<F> m_translation;
    Matrix3<F> m_rotation;

public:

    static Transform_ Identity() {
        return Transform_({0.,0,0},{1.,0,0,0,1,0,0,0,1});
    }

    Transform_() {};
    Transform_(Vector3<F> const& t, Matrix3<F> const& r={}) : m_translation(t), m_rotation(r) {};

    /**
     * @brief Composition operator for transformations.
     */
    Transform_& operator*=(Transform_ const& q);

    /**
    * @brief Get the translation part of the transformation
    * @return [x,y,z] vector
    */
    Vector3<F> const& translation() const {return m_translation;}
    /**
    * @brief Set the translation part of the transformation
    */
    void setTranslation(Vector3<F> const& v) {m_translation = v;}
    /**
    * @brief Set the translation part of the transformation using pointer to 3D array.
    */
    void setTranslation(F const* v) {std::copy(v, v+3, m_translation.data());}

    /**
    * @brief Get the rotation matrix part of the transformation.
    * @return row major 3x3 matrix
    */
    Matrix3<F> const& rotation() const {return m_rotation;}
    /**
    * @brief Get the rotation matrix part of the transformation.
    */
    void setRotation(Matrix3<F> const& v) {m_rotation = v;}
    /**
    * @brief Set the rotation matrix (row major 3x3) part of the transformation using pointer to array of size 9.
    */
    void setRotation(F const* v) {std::copy(v, v+9, m_rotation.data());}

    /**
    * @brief X coordinate of the translation.
    */
    F x() const { return m_translation[0]; }
    /**
    * @brief Y coordinate of the translation.
    */
    F y() const { return m_translation[1]; }
    /**
    * @brief Z coordinate of the translation.
    */
    F z() const { return m_translation[2]; }
    /**
     * @brief Compute the inverse transformation
     * @return Return the inverse transformation
     */
    Transform_<F> inverse() const;
};

template <class F=double>
Transform_<F> operator*(Transform_<F> lhs, const Transform_<F>& rhs) {
    lhs *= rhs;
    return lhs;
}

/**
 * @brief Transform a vector (from current object coordinates to parent frame coordinates).
 * @param p Point in current object coordinates
 * @return Point in parent frame coordinates.
 */
Vector3d operator*(const Transform_<double>& a, const Vector3d& p);

/**
 * @brief Transform a vector (from current object coordinates to parent frame coordinates).
 * @param p Point in current object coordinates
 * @return Point in parent frame coordinates.
 */
Vector3f operator*(const Transform_<float>& a, const Vector3f& p);

/**
 * @brief Compute the inverse transformation.
 */
Transform_<float> inverse(const Transform_<float>& t);

/**
 * @brief Compute the inverse transformation.
 */
Transform_<double> inverse(const Transform_<double>& t);


template <class F=double>
class TransformQuat_ {

protected:
    Vector3<F> m_translation;
    Vector4<F> m_quaternions;

public:

    static TransformQuat_ Identity() {
        return TransformQuat_({0.,0,0},{0.,0,0,0,1});
    }

    TransformQuat_() {}
    TransformQuat_(Vector3<F> const& t, Vector4<F> const& q={}) : m_translation(t), m_quaternions(q) {}

    TransformQuat_& operator*=(TransformQuat_ const& q);

    Vector3<F> const& translation() const;
    void setTranslation(Vector3<F> const& v);
    void setTranslation(F const* v);

    /**
    * @brief Get the quaternion [qx,qy,qz,qw] of the rotation.
    */
    Vector4<F> const& quaternion() const;
    /**
    * @brief Set the quaternion [qx,qy,qz,qw] of the rotation.
    */
    void setQuaternion(Vector4<F> const& v);
    /**
    * @brief Set the quaternion [qx,qy,qz,qw] of the rotation using pointer to 4D array.
    */
    void setQuaternion(F const* v);

    /**
    * @brief X coordinate of the translation.
    */
    F x() const { return m_translation[0]; }
    /**
    * @brief Y coordinate of the translation.
    */
    F y() const { return m_translation[1]; }
    /**
    * @brief Z coordinate of the translation.
    */
    F z() const { return m_translation[2]; }

    /**
    * @brief qx quaternion composant
    */
    F qx() const { return m_quaternions[0]; }
    /**
    * @brief qx quaternion composant
    */
    F qy() const { return m_quaternions[1]; }
    /**
    * @brief qx quaternion composant
    */
    F qz() const { return m_quaternions[2]; }
    /**
    * @brief qx quaternion composant
    */
    F qw() const { return m_quaternions[3]; }
};
template <class F=double>
TransformQuat_<F> operator*(TransformQuat_<F> lhs, const TransformQuat_<F>& rhs) {
    lhs *= rhs;
    return lhs;
}

template <class F=double>
class PoseQuat_ : public TransformQuat_<F> {
    double m_hostTimestamp = std::numeric_limits<double>::infinity();
    std::int64_t m_edgeTimestampUs = std::numeric_limits<std::int64_t>::min();

public:

    PoseQuat_() {}
    PoseQuat_(double t, TransformQuat_<F>const& tr={}) : m_hostTimestamp(t), TransformQuat_<F> (tr) {}
    PoseQuat_(std::int64_t t, TransformQuat_<F>const& tr={}) : m_edgeTimestampUs(t), TransformQuat_<F> (tr) {}
    PoseQuat_(double t, std::int64_t te,  TransformQuat_<F>const& tr={}) : m_hostTimestamp(t), m_edgeTimestampUs(te), TransformQuat_<F> (tr) {}

    /**
    * @brief Get the edge timestamp of the pose (in microseconds).
    */
    std::int64_t edgeTimestampUs() const {return m_edgeTimestampUs;};
    /**
    * @brief Set the edge timestamp of the pose (in microseconds).
    */
    void setEdgeTimestampUs(std::int64_t t) { m_edgeTimestampUs = t;};
    /**
    * @brief Get the host timestamp corresponding to the pose (in s).
    */
    double hostTimestamp() const {return m_hostTimestamp;};
    /**
    * @brief Set the host timestamp corresponding to the pose (in s).
    */
    void setHostTimestamp(double t) {m_hostTimestamp = t;};
};

template <class F=double>
class PoseRot_ : public Transform_<F> {
    double m_hostTimestamp = std::numeric_limits<double>::infinity();
    std::int64_t m_edgeTimestampUs = std::numeric_limits<std::int64_t>::min();

public:
    PoseRot_() {}
    PoseRot_(double t, Transform_<F>const& tr={}) : m_hostTimestamp(t), Transform_<F> (tr) {}
    PoseRot_(std::int64_t t, Transform_<F>const& tr={}) : m_edgeTimestampUs(t), Transform_<F> (tr) {}
    PoseRot_(double t, std::int64_t te,  Transform_<F>const& tr={}) : m_hostTimestamp(t), m_edgeTimestampUs(te), Transform_<F> (tr) {}

    /**
    * @brief Get the edge timestamp of the pose (in microseconds).
    */
    std::int64_t edgeTimestampUs() const {return m_edgeTimestampUs;};
    /**
    * @brief Set the edge timestamp of the pose (in microseconds).
    */
    void setEdgeTimestampUs(std::int64_t t) { m_edgeTimestampUs = t;};
    /**
    * @brief Get the host timestamp of the pose (in s).
    */
    double hostTimestamp() const {return m_hostTimestamp;};
    /**
    * @brief Set the host timestamp of the pose (in s).
    */
    void setHostTimestamp(double t) {m_hostTimestamp = t;};
};

template <class F=double>
class Pose_ : public details::PoseRot_<F> {

private:

    Vector4<F> m_quaternions;
    double m_confidence = 0;

public:

    static Pose_ Identity() {
        return Pose_({0.,0,0},{1.,0,0,0,1,0,0,0,1});
    }

    Pose_() {}

    /**
    * @brief Construct a pose with a specified confidence.
    */
    Pose_(double c) : m_confidence(c) {}

    /**
    * @brief Construct a pose with a translation, rotation, timestamps and confidence.
    */
    Pose_(Vector3<F> const& translation, Matrix3<F> const& rotation,
          double hostTimestamp = std::numeric_limits<double>::infinity(), std::int64_t edgeTimestamp = (std::numeric_limits<std::int64_t>::min)(), double c=0.)
        : m_confidence(c), PoseRot_<F>::PoseRot_(hostTimestamp, edgeTimestamp, {translation, rotation}), m_quaternions(rotationToQuaternion(rotation)) {}

    /**
    * @brief Get the confidence of the pose. Value in [0,1], 0 means lost.
    */
    double confidence() const {return m_confidence;}

    /**
    * @brief Set the confidence of the pose. Value in [0,1], 0 means lost.
    */
    void setConfidence(double c) { m_confidence = c;}

    /**
    * @brief Get the quaternion of the rotation.
    */
    Vector4<F> const& quaternion() const {return m_quaternions;}
    /**
    * @brief Set the quaternion of the rotation.
    */
    void setQuaternion(Vector4<F> const& v) {
        m_quaternions = v;
        PoseRot_<F>::m_rotation = quaternionToRotation(Pose_<F>::m_quaternions);
    };
    /**
    * @brief Set the quaternion of the rotation using pointer of 4D array.
    */
    void setQuaternion(F const* v) {
        std::copy(v, v+4, m_quaternions.data());
        PoseRot_<F>::m_rotation = quaternionToRotation(Pose_<F>::m_quaternions);
    };
    /**
    * @brief @copybrief xv::details::PoseRot_<F>::setRotation()
    */
    void setRotation(Matrix3<F> const& v) {
        PoseRot_<F>::m_rotation = v;
        m_quaternions = rotationToQuaternion(PoseRot_<F>::m_rotation);
    };
    /**
    * @brief Set the quaternion of the rotation using pointer of 4D array.
    */
    void setRotation(F const* v) {
        std::copy(v, v+9, PoseRot_<F>::m_rotation.data());
        m_quaternions = rotationToQuaternion(PoseRot_<F>::m_rotation);
    };



};

template <class F>
class PosePred_ : public Pose_<F> {

protected:
    Vector3<F> m_linearVelocity = Vector3<F>{0.,0.,0.};
    Vector3<F> m_angularVelocity = Vector3<F>{0.,0.,0.};
    Vector3<F> m_linearAcceleration = Vector3<F>{0.,0.,0.};
    Vector3<F> m_angularAcceleration = Vector3<F>{0.,0.,0.};

public:

    static PosePred_ Identity() {
        return PosePred_({0.,0,0},{1.,0,0,0,1,0,0,0,1});
    }

    PosePred_() {}

    /**
    * @brief Construct a pose with a specified confidence.
    */
    PosePred_(double c) : Pose_<F>::Pose_(c) {}

    /**
    * @brief Construct a pose with a translation, rotation, timestamps and confidence.
    */
    PosePred_(Vector3<F> const& translation, Matrix3<F> const& rotation,
          double hostTimestamp = std::numeric_limits<double>::infinity(), std::int64_t edgeTimestamp = (std::numeric_limits<std::int64_t>::min)(), double c=0.)
        : Pose_<F>::Pose_(translation, rotation, hostTimestamp, edgeTimestamp, c) {}

    Transform_<F> const& transform() const { return *this; }

    /**
     * @brief Get the linear velocity (in m/s) of the pose.
     */
    Vector3<F> const& linearVelocity() const {return m_linearVelocity; }
    /**
     * @brief Get the angular velocity (x,y,z) in rad/s
     */
    Vector3<F> const& angularVelocity() const {return m_angularVelocity; }
    /**
     * @brief Set the linear velocity (in m/s)
     */
    void setLinearVelocity(Vector3<F> const& v) { m_linearVelocity=v; }
    /**
     * @brief Set the angular velocity (x,y,z) in rad/s
     */
    void setLinearVelocity(F const* v) {
            std::copy(v, v+3, m_linearVelocity.data());
    }
    /**
     * @brief Set an estimation of instantaneous angular velocity of the pose.
     * @param v [wx,wy,wz] angular velocity
     */
    void setAngularVelocity(Vector3<F> const& v) { m_angularVelocity=v; }
    /**
     * @brief Set an estimation of instantaneous angular velocity of the pose.
     * @param v [wx,wy,wz] angular velocity
     */
    void setAngularVelocity(F const* v) {
        std::copy(v, v+3, m_angularVelocity.data());
    }

    /**
     * @brief Get the linear acceleration (in m/s/s) of the pose.
     */
    Vector3<F> const& linearAcceleration() const {return m_linearAcceleration; }
    /**
     * @brief Get the angular acceleration (x,y,z) in rad/s/s
     */
    Vector3<F> const& angularAcceleration() const {return m_angularAcceleration; }
    /**
     * @brief Set the linear acceleration (in m/s)
     */
    void setLinearAcceleration(Vector3<F> const& v) { m_linearAcceleration=v; }
    /**
     * @brief Set the angular acceleration (x,y,z) in rad/s/s
     */
    void setLinearAcceleration(F const* v) {
            std::copy(v, v+3, m_linearAcceleration.data());
    }
    /**
     * @brief Set an estimation of instantaneous angular acceleration of the pose.
     * @param v [ax,ay,az] angular acceleration
     */
    void setAngularAcceleration(Vector3<F> const& v) { m_angularAcceleration=v; }
    /**
     * @brief Set an estimation of instantaneous angular acceleration of the pose.
     * @param v [ax,ay,az] angular acceleration
     */
    void setAngularAcceleration(F const* v) {
        std::copy(v, v+3, m_angularAcceleration.data());
    }
};

}

/**
 * @brief Represents a transformation (or pose) with translation and rotation matrix.
 */
struct Transform : public details::Transform_<double> {
    Transform();
    Transform(details::Transform_<double> && o);
    Transform(Vector3d const& t, Matrix3d const& r);
};
/**
 * @brief Represents atransformation (or pose) with translation and rotation matrix in `float` type.
 */
struct TransformF : public details::Transform_<float> {
    TransformF();
    TransformF(details::Transform_<float> && o);
    TransformF(Vector3f const& t, Matrix3f const& r);
};

/**
 * @brief Transform a vector (from current object coordinates to parent frame coordinates).
 * @param p Point in current object coordinates
 * @return Point in parent frame coordinates.
 */
Vector3f operator*(const TransformF& a, const Vector3f& p);
/**
 * @brief Transform a vector (from current object coordinates to parent frame coordinates).
 * @param p Point in current object coordinates
 * @return Point in parent frame coordinates.
 */
Vector3d operator*(const Transform& lhs, const Vector3d& rhs);
/**
 * @brief Compute the inverse transformation.
 */
TransformF inverse(const TransformF& t);
/**
 * @brief Compute the inverse transformation.
 */
Transform inverse(const Transform& t);


/**
 * @brief Represents a `float` typed transformation (or pose) with translation and quaternion for rotation.
 */
struct TransformQuat : public details::TransformQuat_<double> {
    TransformQuat();
    TransformQuat(Vector3d const& t, Vector4d const& q);
};
/**
 * @brief Represents a `float` typed transformation (or pose) with translation and quaternion for rotation in `float` type.
 */
struct TransformQuatF : public details::TransformQuat_<float> {
    TransformQuatF();
    TransformQuatF(Vector3f const& t, Vector4f const& q);
};

/**
 * @brief Polynomial Distortion Model for camera intrisics
 *
 *  Projection and raytrace formula can be found here:
https://docs.opencv.org/3.4.0/d4/d94/tutorial_camera_calibration.html
    distor[0] : k1
    distor[1] : k2
    distor[2] : p1
    distor[3] : p2
    distor[4]: k3
 */
struct PolynomialDistortionCameraModel {
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
     * @brief Distortion parameters.
     *
     * https://docs.opencv.org/3.4.0/d4/d94/tutorial_camera_calibration.html
        distor[0] : k1
        distor[1] : k2
        distor[2] : p1
        distor[3] : p2
        distor[4]: k3
     */
    std::array<double,5> distor;
    PolynomialDistortionCameraModel();
};

/**
 * @brief Unified Camera Model
 */
struct UnifiedCameraModel {
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
     * @brief xi parameter of Unified Camera Model
     */
    double xi;
};

/**
 * @brief Calibration parameters of a camera using Unified Camera Model for camera intrinsics
 */
struct UcmCameraCalibration {
    Transform pose;
    UnifiedCameraModel intrinsics;
};

/**
 * @brief Calibration parameters of a camera using Polynomial Distortion Model for camera intrinsics
 */
struct PdmCameraCalibration {
    Transform pose;
    PolynomialDistortionCameraModel intrinsics;
};

/**
 * @brief Class representing a 6dof pose at a timestamp with a linear model for prediction.
 *
 * This class has both quaternion and rotation matrix to represent the 3D rotation.
 *
 */
struct Pose : public details::PosePred_<double> {
    static Pose Identity() {
        return Pose({0.,0,0},{1.,0,0,0,1,0,0,0,1});
    }
    Pose();
    /**
    * @brief Construct a pose with a translation, rotation, timestamps and confidence.
    */
    Pose(Vector3d const& translation, Matrix3d const& rotation,
          double hostTimestamp = std::numeric_limits<double>::infinity(), std::int64_t edgeTimestamp = (std::numeric_limits<std::int64_t>::min)(), double c=0.);
    /**
     * @brief Prediction of the pose based on angular and linear velocity and acceleration.
     * @param dt amount of prediction (in s)
     * @return The predicted (2nd order extrapolation) of the orientation.
     */
    Pose prediction(double dt) const;
};

/**
 * @brief Class representing a 6dof pose at a timestamp with a linear model for prediction.
 *
 * This class has both quaternion and rotation matrix to represent the 3D rotation.
 *
 */
struct PoseF : public details::PosePred_<float> {
    static Pose Identity() {
        return Pose({0.,0,0},{1.,0,0,0,1,0,0,0,1});
    }
    PoseF();
    /**
    * @brief Construct a pose with a translation, rotation, timestamps and confidence.
    */
    PoseF(Vector3f const& translation, Matrix3f const& rotation,
          double hostTimestamp = std::numeric_limits<double>::infinity(), std::int64_t edgeTimestamp = (std::numeric_limits<std::int64_t>::min)(), double c=0.);

    /**
     * @brief Prediction of the pose based on angular and linear velocity and acceleration.
     * @param dt amount of prediction (in s)
     * @return The predicted (2nd order extrapolation) of the orientation.
     */
    PoseF prediction(double dt) const;
};

/**
 * @brief Event.
 */
struct Event {
    double hostTimestamp = std::numeric_limits<double>::infinity(); //!< host timestamp of the physical measurement (in second based on the `std::chrono::steady_clock`).
    std::int64_t edgeTimestampUs = (std::numeric_limits<std::int64_t>::min)(); //!< timestamp of the physical measurement (in microsecond based on edge clock).
    int type = 0; //!< Type of event
    int state = 0; //!< State of the event
};

/**
 * @brief Orientation only (3dof) of the pose.
 */
class Orientation {

    Matrix3d m_rotation;
    Vector4d m_quaternions; //!< [qx,qy,qz,qw]
    Vector3d m_angularVelocity = Vector3d{0.,0.,0.};
    Vector3d m_angularAcceleration = Vector3d{0.,0.,0.};

public:
    double hostTimestamp = std::numeric_limits<double>::infinity(); //!< host timestamp of the plane (in second based on the `std::chrono::steady_clock`).
    std::int64_t edgeTimestampUs = (std::numeric_limits<std::int64_t>::min)(); //!< timestamp of the plane (in microsecond based on edge clock).

    Orientation();

    /**
    * @brief Construct an orientation (3dof) rotation and timestamps.
    */
    Orientation(Matrix3d const& rotation,
          double hostTimestamp = std::numeric_limits<double>::infinity(), std::int64_t edgeTimestamp = (std::numeric_limits<std::int64_t>::min)());

    /**
    * @brief Construct an orientation (3dof) rotation and timestamps.
    */
    Orientation(Vector4d const& quaternion,
          double hostTimestamp = std::numeric_limits<double>::infinity(), std::int64_t edgeTimestamp = (std::numeric_limits<std::int64_t>::min)());

    /**
    * @brief Get the quaternion of the rotation [qx,qy,qz,qw].
    */
    Vector4d const& quaternion() const;
    /**
    * @brief Set the quaternion of the rotation [qx,qy,qz,qw].
    */
    void setQuaternion(Vector4d const& v);
    /**
    * @brief Set the quaternion of the rotation using pointer of 4D array [qx,qy,qz,qw].
    */
    void setQuaternion(double const* v);

    /**
    * @brief @copybrief details::PoseRot_<F>::rotation()
    */
    Matrix3d const& rotation() const {return m_rotation;}

    /**
    * @brief @copybrief details::PoseRot_<F>::setRotation()
    */
    void setRotation(Matrix3d const& v);
    /**
    * @brief Set the quaternion of the rotation using pointer of 4D array.
    */
    void setRotation(double const* v);
    /**
     * @brief An estimation of instantaneous angular velocity of the pose.
     * @return [wx,wy,wz] angular velocity
     */
    Vector3d const& angularVelocity() const;

    /**
     * @brief Set an estimation of instantaneous angular velocity of the pose.
     * @param v [wx,wy,wz] angular velocity
     */
    void setAngularVelocity(Vector3d const& v);

    /**
     * @brief Set an estimation of instantaneous angular velocity of the pose.
     * @param v [wx,wy,wz] angular velocity
     */
    void setAngularVelocity(double const* v);

    /**
     * @brief Set an estimation of instantaneous angular acceleration of the pose.
     * @param v [ax,ay,az] angular acceleration (rad/s/s)
     */
    void setAngularAcceleration(Vector3d const& v);

    /**
     * @brief Set an estimation of instantaneous angular acceleration of the pose.
     * @param v [ax,ay,az] angular acceleration (rad/s/s)
     */
    void setAngularAcceleration(double const* v);

    /**
     * @brief Prediction of the orientation based on angular velocity and acceleration.
     * @param dt amount of prediction (in s)
     * @return The predicted (2nd order extrapolation) of the orientation.
     */
    Orientation prediction(double dt) const;

};

/**
 * @brief A 3D plane definition
 *
 * The plane is represented with normal equation x n[0] + y n[1] + z n[2] - d = 0
 */
struct Plane {
    /// @brief Plane unique identifier
    std::string id;

    /// @brief Unit vector normal to the plane
    Vector3d normal;

    /// @brief Signed distance to origin.
    /// Signed distance between the plane and the origin of the world. The distance is
    /// signed according to the direction of the normale.
    double d;

    /// @brief Points lying at the border of the plane.
    /// Array of 3D points lying on the plane that describes the
    /// polygon that borders the actually detected area.
    std::vector<Vector3d> points;

    /// @brief Flat, 3D, triangle mesh describing the detailed plane geometry extents.
    /// More convenient than the border points.
    std::vector<Vector3d> vertices;
    std::vector<std::array<uint32_t,3>> triangles;
};


/**
 * @brief A sparse SLAM map with 3D points.
 */
struct SlamMap
{
  std::vector<Vector3d> vertices;
};

/**
 * @brief Generic camera model
 */
class CameraModel {
public:
    virtual std::int32_t width() const {return 0; }
    virtual std::int32_t height() const {return 0; }
    virtual bool project(double const* /*p3d*/, double* /*p2d*/) const {return false;};
    virtual bool raytrace(double const* /*p2d*/, double* /*p3d*/) const {return false;};
};

/**
 * @brief Calibration (extrinsics and intrinsics).
 */
struct Calibration {
    Transform pose; //! pose of the sensor(camera and display) in the IMU frame coordinates.
    std::vector<UnifiedCameraModel> ucm; //! Deprecated, better to use camerasModel; List of Unified Camera Model parameters for differents camera resolutions (see UnifiedCameraModel#w and UnifiedCameraModel#h to find the corresponding resolution of the parameter set).
    std::vector<PolynomialDistortionCameraModel> pdcm; //! Deprecated, better to use camerasModel; List of Polynomial Distortion Camera Model parameters for differents camera resolutions (see UnifiedCameraModel#w and UnifiedCameraModel#h to find the corresponding resolution of the parameter set).
    std::vector<std::shared_ptr<CameraModel>> camerasModel; //! Can be ucm, pdcm or any other calibration model loaded from the device
};

/**
 * @brief Data from IMU sensor of the XVisio device.
 *
 * Contains temperature, 3-axis gyrometer, 3-axis accelerometer and 3-axis magnetometer measures.
 */
struct Imu {
    Vector3d gyro; //!< 3-axis gyrometer values (in rad/s)
    Vector3d accel; //!< 3-axis accelerometer values (in m/s²)
    Vector3b accelSaturation; //!< 3-axis accel saturation status (true if saturating)
    Vector3d magneto; //!< 3-axis magnetometer values
    double temperature; //!< sensor temperature (in K)
    double hostTimestamp = std::numeric_limits<double>::infinity(); //!< host timestamp of the physical measurement (in second based on the `std::chrono::steady_clock`).
    std::int64_t edgeTimestampUs = (std::numeric_limits<std::int64_t>::min)(); //!< timestamp of the physical measurement (in microsecond based on edge clock).
};

/**
 * @brief A grayscale image that is usually an image from a camera used for visual SLAM
 */
struct GrayScaleImage {
    std::size_t width; //!< width of the image (in pixel)
    std::size_t height; //!< height of the image (in pixel)
    std::shared_ptr<const std::uint8_t> data; //! image data (in row-major). Data size = width*height
};

/**
 * @brief Images coming from #xv::FisheyeCameras sensor system used for visual SLAM
 */
struct FisheyeImages {
    double hostTimestamp = std::numeric_limits<double>::infinity(); //!< host timestamp of the physical measurement (in second based on the `std::chrono::steady_clock`), need to activate IMU stream (with SLAM or IMU callback) to have this value.
    std::int64_t edgeTimestampUs = (std::numeric_limits<std::int64_t>::min)(); //!< timestamp of the physical measurement (in microsecond based on edge clock).
    std::vector<GrayScaleImage> images; //! List of images (typically 2, first is left and second image is the right image)
    std::int64_t id;//! Unique id given by the edge to this instance
};

/**
 * @brief A color image in RGB format
 */
struct RgbImage {
    std::size_t width = 0; //!< width of the image (in pixel)
    std::size_t height = 0; //!< height of the image (in pixel)
    std::shared_ptr<const std::uint8_t> data = nullptr; //! image data (in row-major) : RGB RGB RGB ....  Data size = width*height*3
    RgbImage(std::size_t _width, std::size_t _height, std::shared_ptr<const std::uint8_t> _data) : width(_width), height(_height),data(_data) {}
};

/**
 * @brief A color image given by #xv::ColorCamera
 */
struct ColorImage {
    enum class Codec { YUYV = 0, YUV420p, JPEG, NV12, BITSTREAM};
    Codec codec = Codec::YUYV;
    std::size_t width = 0; //!< width of the image (in pixel)
    std::size_t height = 0; //!< height of the image (in pixel)
    std::shared_ptr<const std::uint8_t> data = nullptr; //! image data
    unsigned int dataSize = 0;
    double hostTimestamp = std::numeric_limits<double>::infinity(); //!< host timestamp of the physical measurement (in second based on the `std::chrono::steady_clock`).
    std::int64_t edgeTimestampUs = (std::numeric_limits<std::int64_t>::min)(); //!< timestamp of the physical measurement (in microsecond based on edge clock).
    /**
     * @brief Convert to a #xv::RgbImage
     */
    RgbImage toRgb() const;
};

/**
 * @brief SGBM CONFIG STRUCT
*/
struct sgbm_config
    {
        int32_t enable_dewarp;
        float dewarp_zoom_factor;
        int32_t enable_disparity;
        int32_t enable_depth;
        int32_t enable_point_cloud;
        float baseline;
        float fov;
        uint8_t disparity_confidence_threshold;
        float homography[9];
        int32_t enable_gamma;
        float gamma_value;
        int32_t enable_gaussian;
        uint8_t mode;//standard 0 /Default:LRcheck 1 /Extended 2 /Subpixel 3
        uint16_t max_distance;//mm
        uint16_t min_distance;//mm

        inline bool operator==(const sgbm_config &cmp) const
        {
            return (enable_dewarp == cmp.enable_dewarp &&
                    dewarp_zoom_factor == cmp.dewarp_zoom_factor &&
                    enable_disparity == cmp.enable_disparity &&
                    enable_depth == cmp.enable_depth &&
                    enable_point_cloud == cmp.enable_point_cloud &&
                    baseline == cmp.baseline &&
                    fov == cmp.fov &&
                    disparity_confidence_threshold == cmp.disparity_confidence_threshold);
        }
        inline bool operator!=(const sgbm_config &cmp) const
        {
            return !(*this == cmp);
        }
    };

/**
 * @brief An image provided by a TOF camera.
 * @note  There are two manufacturers of TOF camera, Pmd and sony.
 *        Pmd TOF depth type is Depth_32，sony TOF depth type is Depth_16.
 *        Cloud type just use for Pmd point cloud,the coordinate system of the point cloud is the camera coordinate system, and the data unit is meters.
 *        Length, width and depth are in meters use Pmd TOF.
 *        Length, width and depth are in metmillimeterers use sony TOF. 
 */
struct DepthImage {
    enum class Type { Depth_16 = 0, Depth_32, IR, Cloud, Raw, Eeprom, IQ };
    Type type = Type::Depth_32;
    std::size_t width = 0; //!< width of the image (in pixel)
    std::size_t height = 0; //!< height of the image (in pixel)
    double  confidence = 0.0; //!< confidence of depth [0.0,1.0]
    std::shared_ptr<const std::uint8_t> data = nullptr; //! image of depth
    unsigned int dataSize = 0;
    double hostTimestamp = std::numeric_limits<double>::infinity(); //!< host timestamp of the physical measurement (in second based on the `std::chrono::steady_clock`).
    std::int64_t edgeTimestampUs = (std::numeric_limits<std::int64_t>::min)(); //!< timestamp of the physical measurement (in microsecond based on edge clock).
    /**
     * @brief Convert to a #xv::RgbImage
     */
    RgbImage toRgb() const;
};

struct DepthColorImage {
    std::size_t width = 0; //!< width of the image (in pixel)
    std::size_t height = 0; //!< height of the image (in pixel)
    std::shared_ptr<const std::uint8_t> data = nullptr; //! image data of RGB-D pixels : RGB (3 bytes) D (float 4bytes)
    double hostTimestamp = std::numeric_limits<double>::infinity(); //!< host timestamp of the physical measurement (in second based on the `std::chrono::steady_clock`).
};

/**
 * @brief A point cloud of 3D points.
 */
struct PointCloud {
    double hostTimestamp = std::numeric_limits<double>::infinity(); //!< host timestamp of ? (in second based on the `std::chrono::steady_clock`).
    std::int64_t edgeTimestampUs = (std::numeric_limits<std::int64_t>::min)(); //!< timestamp of ? (in microsecond based on edge clock).
    std::vector<Vector3f> points;
};

struct BeiDouGPSData
{
    uint8_t data_ready_flag; // 0:invalid   1:valid
    double lat_data; //Latitude data   
    uint8_t latdir;  //latdir == 1(lat_data  is north latitude)  latdir == 2(lat_data  is south latitude)
    double lon_data; //Longitude data
    uint8_t londir;  //londir == 1(lon_data is east longitude)  londir == 2(lon_data is west longitude) 
    int satellite_num;  //Number of satellites
    uint8_t mode;       //mode == 1 (GPS + BDS)  mode == 2(BDS)
};

enum class BeiDouGPSMode
{
    BDS_SINGLE = 0,
    BDS_GPS_MIX = 1
};

/**
 * @brief Object detection bounding box.
 */
struct Object {
    enum class Shape { BoundingBox = 0, Human, HandSkeleton };
    struct keypoint {
        double x = -1, y = -1, z = -1;
    };

    Shape shape = Shape::BoundingBox;

    int typeID = -1;
    std::string type = "";
    double x = 0;
    double y = 0;
    double width = 0;
    double height = 0;
    double confidence = 0.0f;

    std::vector<keypoint> keypoints;
};

/**
 * @brief Object detection bounding box. 2d
 */
struct Det2dObject{
    struct keypoint {
        double x = -1, y = -1, z = -1;
    };

    int idx;
    std::string name;
    float score;
    float left, top, width, height;
    std::vector<keypoint> keypoints;
    Det2dObject(){};
    Det2dObject(int i, const std::string& n, float s, float l, float t, float w, float h) : 
        idx(i), name(n), score(s), left(l), top(t), width(w), height(h) {}
    Det2dObject(int i, const std::string& n, float s, float l, float t, float w, float h, std::vector<keypoint> pts) : 
        idx(i), name(n), score(s), left(l), top(t), width(w), height(h), keypoints(pts){}
};

struct CnnRawWrapper{
    std::shared_ptr<float> raw_data = nullptr;
    unsigned int raw_data_length;
    double hostTimestamp;
    uint64_t edgeTimestamp;
};




struct ObjectDescriptor{
    std::string type;
    std::vector<std::string> classes;
    double threshold = 0.5;
    bool flipStereo = false;
    bool flipRgb = false;
    bool flipTof = false;
    bool enable_3dbbox = false;
    std::vector<double> prior_bbox_z;   
};

/**
 * @brief SGBM data.
 * @note Length, width and depth are in millimeters.
 */
struct SgbmImage {
    enum class Type { Disparity = 0, Depth, PointCloud, None};
    Type type;
    SgbmImage() : type(Type::None) {}
    explicit SgbmImage(Type t) : type(t) {}
    std::size_t width = 0; //!< width of the image (in pixel)
    std::size_t height = 0; //!< height of the image (in pixel)
    std::shared_ptr<const std::uint8_t> data = nullptr; //! data of SGBM
    unsigned int dataSize = 0;
    double hostTimestamp = std::numeric_limits<double>::infinity(); //!< host timestamp of the physical measurement (in second based on the `std::chrono::steady_clock`).
    std::int64_t edgeTimestampUs = (std::numeric_limits<std::int64_t>::min)(); //!< timestamp of the physical measurement (in microsecond based on edge clock).
    /**
     * @brief Convert to a #xv::RgbImage
     */
    RgbImage toRgb() const;
};

/**
 * @brief A color image given by #xv::ThermalCamera
 */
struct ThermalImage {
    enum class Codec {UYVY};
    const Codec codec;
    explicit ThermalImage(Codec t) : codec(t) {}
    std::size_t width = 0; //!< width of the image (in pixel)
    std::size_t height = 0; //!< height of the image (in pixel)
    std::shared_ptr<const std::uint8_t> data = nullptr; //! image data
    unsigned int dataSize = 0;
    double hostTimestamp = std::numeric_limits<double>::infinity(); //!< host timestamp of the physical measurement (in second based on the `std::chrono::steady_clock`).
    std::int64_t edgeTimestampUs = (std::numeric_limits<std::int64_t>::min)(); //!< timestamp of the physical measurement (in microsecond based on edge clock).
    /**
     * @brief Convert to a #xv::RgbImage
     */
    RgbImage toRgb() const;

    //  InfraredImage(std::shared_ptr<const std::uint8_t> ptr) :
    //     data(ptr)
    // {

    // }
    // std::size_t width = 0; //!< width of the image (in pixel)
    // std::size_t height = 0; //!< height of the image (in pixel)
    // std::shared_ptr<const std::uint8_t> data = nullptr; //! image of depth
    // unsigned int dataSize = 0;
    // double hostTimestamp = std::numeric_limits<double>::infinity(); //!< host timestamp of the physical measurement (in second based on the `std::chrono::steady_clock`).
    // std::int64_t edgeTimestampUs = (std::numeric_limits<std::int64_t>::min)(); //!< timestamp of the physical measurement (in microsecond based on edge clock).
};

struct IrTrackingImage {
    enum class Codec {UYVY};
    std::size_t width = 0; //!< width of the image (in pixel)
    std::size_t height = 0; //!< height of the image (in pixel)
    std::shared_ptr<const std::uint8_t> data;
    double hostTimestamp = std::numeric_limits<double>::infinity(); //!< host timestamp of the physical measurement (in second based on the `std::chrono::steady_clock`).
    std::int64_t deviceTimestamp = (std::numeric_limits<std::int64_t>::min)(); //!< timestamp of the physical measurement (in microsecond based on edge clock).
};

/**
 * @brief A color image given by #xv::EyetrackingCamera
 */
struct EyetrackingImage {
    double hostTimestamp = std::numeric_limits<double>::infinity(); //!< host timestamp of the physical measurement (in second based on the `std::chrono::steady_clock`), need to activate IMU stream (with SLAM or IMU callback) to have this value.
    std::int64_t edgeTimestampUs = (std::numeric_limits<std::int64_t>::min)(); //!< timestamp of the physical measurement (in microsecond based on edge clock).
    std::vector<GrayScaleImage> images; //! List of images (typically 2, first is left and second image is the right image)
};

struct MicData {
    double hostTimestamp = std::numeric_limits<double>::infinity(); //!< host timestamp of the physical measurement (in second based on the `std::chrono::steady_clock`).
    std::int64_t edgeTimestampUs = (std::numeric_limits<std::int64_t>::min)(); //!< timestamp of the physical measurement (in microsecond based on edge clock).
    std::shared_ptr<const std::uint8_t> data = nullptr; //! image data
    unsigned int dataSize = 0;
};

/**
 * @brief Gesture key point
 */
struct keypoint {
    float x = -1;
    float y = -1;
    float z = -1;
};

/**
 * @brief Gesture data
 */
struct GestureData {
    int index[2] = {-1,-1};//!<Index array for hands gesture, max size is two, default is -1 means invalid.
    keypoint position[2];//!<Position array for hand gesture,  max size is two, 2D points, z isn't used by default.
    keypoint slamPosition[2];//!<Convert rgb points into slam points, Position array for hand gesture,  max size is two.
    double hostTimestamp = std::numeric_limits<double>::infinity(); //!< host timestamp of the physical measurement (in second based on the `std::chrono::steady_clock`).
    std::int64_t edgeTimestampUs = (std::numeric_limits<std::int64_t>::min)(); //!< timestamp of the physical measurement (in microsecond based on edge clock).
    float distance;//!< reserved, dynamic gesture movement distance.
    float confidence;//!<reserved, gesture confidence.
};

enum class ResolutionMode{
    R_VGA,
    R_720P
};

/**
* @union xv_ETPoint2D
*/
typedef union XV_ET_POINT_2D
{
	struct {
		float x, y;
	};
	float seq[2];
}xv_ETPoint2D;

/**
* @union xv_ETPoint3D
*/
typedef union XV_ET_POINT_3D
{
    struct {
        float x, y, z;
    };
    float seq[3];
}xv_ETPoint3D;

/**
* @enum xv_ETEyeType
*/
enum XV_ET_EYE_TYPE {
    L_EYE = 1,//!<left eye
    R_EYE = 2//!<right eye
};
typedef XV_ET_EYE_TYPE xv_ETEyeType;

/**
* @enum xv_ETMode
*/
enum XV_ET_MODE {
	track = 3,//!<eye track mode
	iris = 5//!<iris identify mode    	        
};
typedef XV_ET_MODE xv_ETMode;

/**
* @struct xv_ETInitParam
*/
struct XV_ET_INIT_PARAM {
	xv_ETMode mode;//!< sdk mode, refer to xv_ETMode.
	char configPath[260];//!<config file path.
};
typedef XV_ET_INIT_PARAM xv_ETInitParam;

/**
* @struct xv_ETCoefficient
*/
typedef struct XV_ET_COEFFICIENT {
	unsigned char buf[1024];//!<calibration factor.
}xv_ETCoefficient;


enum XV_EyeGazeExDataValidity {
    EYE_GAZE_EXDATA_SCORE = 0,//!<gaze score.
};

/**
* @struct
* gaze struct
*/
struct XV_ET_GAZE_POINT
{
    unsigned int gazeBitMask;//!<gaze bit mask, identify the six data below are valid or invalid.    
    xv_ETPoint3D gazePoint;//!<gaze point, x and y are valid, z default value is 0, x and y scope are related to the input calibration point, not fixed.    
    xv_ETPoint3D rawPoint;//!<gaze point before smooth, x and y are valid, z default value is 0, x and y scope are as above.
    xv_ETPoint3D smoothPoint;//!<gaze point after smooth, x and y are valid, z default value is 0, x and y scope are as above.
    xv_ETPoint3D gazeOrigin;//!<origin gaze center coordinate.
    xv_ETPoint3D gazeDirection;//!<gaze direction.
    float re;//!<gaze re value, confidence level.
    unsigned int exDataBitMask;//!<reserved data.
    float exData[32];//!<reserved data.
};

/**
* @struct
* pupil struct
*/
struct XV_ET_PUPIL_INFO
{
    unsigned int pupilBitMask;//!<pupil bit mask, identify the six data below are valid or invalid.
    xv_ETPoint2D pupilCenter;//!<pupil center(0-1), the coordinate value of pupil center in the image, normalization value, image height and width is 1.
    float pupilDistance;//!<the distance between pupil and camera(mm)
    float pupilDiameter;//!<pupil diameter, pupil long axis value(0-1), the ratio of the pixel value of the long axis size of the pupil ellipse to the image width, normalization value.
    float pupilDiameterMM;//!<pupil diameter, pupil long axis value(mm).
    float pupilMinorAxis;//!<pupil diameter, pupil minor axis value(0-1), the ratio of the pixel value of the minor axis size of the pupil ellipse to the image width, normalization value.
    float pupilMinorAxisMM;//!<pupil diameter, pupil minor axis value(mm).
};

/**
* @struct
* eye extend data struct
*/
struct XV_ET_EYE_EXDATA
{
    unsigned int eyeDataExBitMask;//!<eye extend data bit mask, identify the four data below are valid or invalid.
    int blink;//!<blink data, 0-no blink, 1-start blinking, 2-closing process, 3-close eyes, 4-opening process, 5-finish blinking.
    float openness;//!<eye openness(0-100), 0-cloing, 100-opening normally, >100-opening on purpose.
    float eyelidUp;//!<up eyelid data(0-1), up eyelid's vertical position in the image, normalization value, image height is 1.
    float eyelidDown;//!<down eyelid data(0-1), down eyelid's vertical position in the image, normalization value, image height is 1.
};

/**
* @struct 
* eye data struct
*/
struct XV_ET_EYE_DATA_EX
{
    unsigned long long timestamp;//!<timestamp.
    int recommend;//!<whether if there has the recommend point. 0-no recommend point, 1-use left eye as recommend point, 2-use right eye as recomment point.                      
    XV_ET_GAZE_POINT recomGaze;//!<recommend gaze data
    XV_ET_GAZE_POINT leftGaze;//!<left eye gaze data
    XV_ET_GAZE_POINT rightGaze;//!<right eye gaze data

    XV_ET_PUPIL_INFO leftPupil;//!<left eye pupil data
    XV_ET_PUPIL_INFO rightPupil;//!<right eye pupil data

    XV_ET_EYE_EXDATA leftExData;//!<left eye extend data(include blink and eyelid data)
    XV_ET_EYE_EXDATA rightExData;//!<right eye extend data(include blink and eyelid data)

    int leftEyeMove;//!<0-Eye movement type is no-eye detected. 1-Eye movement type is blink. 2-Eye movement type is noraml.
    int rightEyeMove;//!<0-Eye movement type is no-eye detected. 1-Eye movement type is blink. 2-Eye movement type is noraml.
    float ipd;//!<The estimated interpupilary distance (IPD)
};

/** Status codes returned by the API. */
typedef enum GazeStatus {
    /** No error. */
    GAZE_STATUS_OK,

    /** Undefined error. */
    GAZE_STATUS_ERROR,

    /** Failed to initialize the API. */
    GAZE_STATUS_INITIALIZE_FAILED,

    /** Failed to terminate the API. */
    GAZE_STATUS_TERMINATE_FAILED,

    /** An invalid parameter was given. */
    GAZE_STATUS_INVALID_PARAMETER,

    /** The operation was invalid. */
    GAZE_STATUS_INVALID_OPERATION,

    /** The device is unavailable. */
    GAZE_STATUS_DEVICE_NOT_AVAILABLE,

    /** The operation timed out. */
    GAZE_STATUS_TIMED_OUT,

    /** Failed to allocate memory. */
    GAZE_STATUS_MEM_ALLOCATION_FAILED
}
GazeStatus;


typedef enum CalibrationApiStatus {
    /**
     * Special status for CalibrationSetup API.
     *   Indicates that this operation already performed 'automatically' inside the algorithm core.
     */
    CALIBRATION_API_STATUS_COMPLETE_AUTOMATICALLY,

    /** Indicates that this operation already completed by a previous explict call. */
    CALIBRATION_API_STATUS_COMPLETE_MANUALLY,

    /**
     * Indicate that the API is ready for calling.
     *   Or failing on a CalibrationSetup explicit call (special definition)
     */
    CALIBRATION_API_STATUS_ACCEPT_CALLING,

    /** Indicate that the API is NOT ready for calling, could lead to undefined behavior if calling the not-ready-API */
    CALIBRATION_API_STATUS_DO_NOT_CALL,

    /** Special status for CalibrationSetup API, since its internal computation may take longer time than the other APIs */
    CALIBRATION_API_STATUS_BUSY
}
CalibrationApiStatus;

/** A struct to represent the current calibration routine API status */
typedef struct CalibrationStatus {
    /** The status of API CalibrationEnter */
    CalibrationApiStatus enter_status;

    /** The status of API CalibrationCollect */
    CalibrationApiStatus collect_status;

    /** The status of API CalibrationSetup */
    CalibrationApiStatus setup_status;

    /** The status of API CalibrationComputeApply */
    CalibrationApiStatus compute_apply_status;

    /** The status of API CalibrationLeave */
    CalibrationApiStatus leave_status;

    /** The status of API CalibrationReset */
    CalibrationApiStatus reset_status;
}
CalibrationStatus;

/** A structure represents the calibration data used by the eye tracker. */
typedef struct GazeCalibrationData {
    /** The calibration data used by the eye tracker. */
    void* data = nullptr;

    /** The size of the calibration data used by the eye tracker. */
    size_t size;

    /** Default Destructor */
    ~GazeCalibrationData()
    {
        if (data != nullptr)
        {
            free(data);
            data = nullptr;
        }
    };
}
GazeCalibrationData;

struct XV_IRIS_DATA
{
    std::vector<char> name;
    std::vector<unsigned char> feature;
    int size;
    int error;
};

/**
* @struct 
* UTC time struct
*/
struct DateTime {
  int Y, M, D;//!<Year, month, and day
  int h, m;//!<Hour and minutes
  int s;//!<Seconds
};

/**
* @struct 
* GPS distance data struct
*/
struct GPSDistanceData {
  int           distance;//!<Distance
  int           signal;//!<Signal Intensity
  unsigned char sum;//!<Check sum
};

struct Triple
{
  Vector2<unsigned short> p2d;
  size_t i3d;
  Triple(Vector2<unsigned short> const& p2, size_t i3): p2d(p2), i3d(i3) {}
};
struct PointMatches
{
    FisheyeImages fisheyeImages;
    std::vector<std::vector<Triple>> matches;
};

struct TerrestrialMagnetismData
{
    float offset[3];
    float angles[3];
    float magnetic[3];
    int level;
};

/**
 * @brief new hand pose struct.
 */
struct HandPose{
    std::vector<Pose> pose; // 26 + 26
    float scale[2];   // 1 + 1
    int status[2] = {-1,-1};
    double timestamp[2];
    double fisheye_timestamp;
};

struct ExternalData {
    int             tv_sec;
    int             tv_msec;
    std::uint8_t    state;
    float           x;
    float           y;
    float           z;
    float           pitch;
    float           roll;
    float           yaw;
    double          hostTimestamp;
};

enum WirelessControllerDataType
{
    UNKNOW = 0,
    LEFT = 0xC4,
    RIGHT = 0xC5,
    ALL = 0xC6
};
struct WirelessControllerData
{
    WirelessControllerDataType type;
    Pose pose;
    uint8_t keyTrigger;
    uint8_t keySide;
    int16_t rocker_x;
    int16_t rocker_y;
    uint8_t key;
};

enum WirelessControllerSlamType 
{
    VIO = 1,
    CSLAM = 2,
    REAL_TIME_SHARED_MAP = 3
};

struct WirelessControllerState
{
    std::string name;
    std::string mac;
    int state;
};

struct WirelessControllerDeviceInformation
{
    uint8_t battery;            // 1 bytes
    uint8_t temp;               // 1 bytes
};

struct GazeConfigs
{
    int screenWidth;
    int screenHeight;
    int etWidth;
    int etHeight;
    float ipdDist;
    int srValue;
    int loplength;
    float etFoclen;
    float etOccupy;
    float ftFoclen;
    int hiValue;
};

struct ResolutionParam
{
    int width = -1;
    int height = -1;
};

struct RoiParam
{
    int x;
    int y;
    int width;
    int height;
};

struct ExposureParam
{
    unsigned int time; // The unit is microseconds.
    float gain;
};

struct IrTrackingTemperature
{
    int one;
    int two;
};

}
