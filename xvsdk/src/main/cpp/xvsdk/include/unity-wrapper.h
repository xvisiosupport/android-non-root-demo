#pragma once

#ifdef WIN32
#define EXPORT_API __declspec( dllexport )
#else
#define EXPORT_API __attribute__ ((visibility ("default")))
#endif

#include <string>

typedef void(*cb_data)(unsigned char *data, int len);

/** \cond */
struct Vector2
{
     float x;
     float y;
};

struct Vector3
{
     float x;
     float y;
     float z;
};

struct Matrix4x4
{
    float m[16];
};
/** \endcond */

/** \cond */
struct Quaternion
{
    double x,y,z,w;
};

struct Orientation {
    long long hostTimestamp = 0; //!<Timestamp in µs read on host
    long long deviceTimestamp = 0; //!<Timestamp in µs read on the device
    Quaternion quaternion; //!< Absolute quaternion (3DoF)
    double roll = 0.0; //!< Absolute roll euler angle (3DoF)
    double pitch = 0.0; //!< Absolute pitch euler angle (3DoF)
    double yaw = 0.0; //!< Absolute yaw euler angle (3DoF)
    double angularVelocity[3]; //!< Instantaneous angular velocity (radian/second)
};
/** \endcond */

/** \cond */

/**
 * @brief Rotation and translation structure
 */
struct transform
{
    double rotation[9]; //!< Rotation matrix (row major)
    double translation[3]; //!< Translation vector
};

/**
 * @brief Polynomial Distortion Model
 */
struct pdm
{
    double K[11];
/**
    Projection and raytrace formula can be found here:
    https://docs.opencv.org/3.4.0/d4/d94/tutorial_camera_calibration.html

    K[0] : fx
    K[1] : fy
    K[2] : u0
    K[3] : v0
    K[4] : k1
    K[5] : k2
    K[6] : p1
    K[7] : p2
    K[8] : k3
    K[9] : image width
    K[10] : image height
*/
};

/**
 * @brief Unified camera model
 */
struct unified
{
    double K[7];
/**
  Projection and raytrace formula can be found here:
  1.  C. Geyer and K. Daniilidis, “A unifying theory for central panoramic systems and practical applications,” in Proc. 6th Eur. Conf. Comput. Vis.
II (ECCV’00), Jul. 26, 2000, pp. 445–461
  or
  2. "J.P. Barreto. General central projection systems, modeling, calibration and visual
servoing. Ph.D., University of Coimbra, 2003". Section 2.2.2.

  K[0] : fx
  K[1] : fy
  K[2] : u0
  K[3] : v0
  K[4] : xi
  K[5] : image width
  K[6] : image height

  More details,
  Projection:
    The simplest camera model is represented by projection relation:    p = 1/z K X
    where p=(u v)^T is an image point, X = (x y z)^T is a spatial point to be projected
    and K is a projection matrix: K = (fx 0 u0; 0 fy v0).

    The distortion model is added in the following manner.
    First we project all the points onto the unit sphere S
        Qs = X / ||X|| = 1/rho (x y z)   where rho = sqrt(X^2+Y^2+Z^2)
    and then we apply the perspective projection with center (0 0 -xi)^T of Qs onto plan image
        p = 1/(z/rho + xi) K (x/rho  y/rho).

  Back-projection/raytrace:
    The normalized coordinate of a pixel is (x y 1)^1.
    We know that a line joining this normalized point and the projection center intersects the unit sphere
    at a point Qs. This point is defined as
        Qs = (eta*x  eta*y  eta-xi)
    where scale factor    eta = (xi + sqrt(1 + (x^2+y^2)(1-xi^2))) / (x^2+y^2+1).
*/
};

struct unified_calibration
{
    transform extrinsic;
    unified intrinsic;
};

struct stereo_fisheyes
{
    unified_calibration calibrations[2];
};

struct pdm_calibration
{
    transform extrinsic;
    pdm intrinsic;
};

struct stereo_pdm_calibration
{
    pdm_calibration calibrations[2];
};

struct rgb_calibration
{
    transform extrinsic;
    pdm intrinsic1080; //!< 1920x1080
    pdm intrinsic720; //!< 1280x720
    pdm intrinsic480; //!< 640x480
};

struct imu_bias
{
    double gyro_offset[3];
    double accel_offset[3];
};
/** \endcond */


extern "C" {

    /**
     * UnityWrapper provide C++ functions to Unity
     */
    namespace UnityWrapper {
        enum SlamType { Edge = 0, Mixed = 1 };
        enum RgbSource { UVC = 0, VSC = 1 };

        // Should same to XSlam::VSC::RgbResolution
        enum RgbResolution {
            UNDEF         = -1, ///< Undefined
            RGB_1920x1080 = 0,  ///< RGB 1080p
            RGB_1280x720  = 1,  ///< RGB 720p
            RGB_640x480   = 2,  ///< RGB 480p
            RGB_320x240   = 3,  ///< RGB QVGA
            RGB_2560x1920 = 4,  ///< RGB 5m
            TOF           = 5,  ///< TOF YUYV 224x172
        };

        // If not ALL, must specify channels and streams to use
        enum Component {
            COM_ALL    = 0xFFFF,
            COM_IMU    = 0x0001,
            COM_POSE   = 0x0002,
            COM_STEREO = 0x0004,
            COM_RGB    = 0x0008,
            COM_TOF    = 0x0010,
            COM_EVENTS = 0x0040,
            COM_CNN    = 0x0080,

            COM_HID    = 0x0100,
            COM_UVC    = 0x0200,
            COM_VSC    = 0x0400,
            COM_SLAM   = 0x0800,
            COM_EDGEP  = 0x1000,
        };

        EXPORT_API bool xslam_ready();

        EXPORT_API bool xslam_init();
        EXPORT_API bool xslam_init_with_fd( int fd );
        EXPORT_API bool xslam_init_components( int components );
        EXPORT_API bool xslam_init_components_with_fd( int fd, int components );
        EXPORT_API bool xslam_uninit();

        // SLAM
        EXPORT_API void xslam_slam_type( SlamType type );
        EXPORT_API bool xslam_get_6dof(Vector3 *position, Vector3 *orientation, long long *timestamp);
        EXPORT_API bool xslam_get_transform_matrix(float *matrix, long long *timestamp, int *status);
        EXPORT_API bool xslam_get_transform(Matrix4x4 *matrix, long long *timestamp, int *status);

        EXPORT_API bool xslam_reset_slam();

        // RGB
        EXPORT_API bool xslam_set_rgb_source( RgbSource source );
        EXPORT_API int xslam_get_rgb_width();
        EXPORT_API int xslam_get_rgb_height();
        EXPORT_API bool xslam_get_rgb_image_YUV( unsigned char *data, int *width, int *height, long long* timestamp );
        EXPORT_API bool xslam_get_rgb_image_RGBA( unsigned char *data, int width, int height, long long* timestamp );
        EXPORT_API bool xslam_get_rgb_image_RGB( unsigned char *data, int width, int height, long long* timestamp );

        // TOF
        EXPORT_API int xslam_get_tof_width();
        EXPORT_API int xslam_get_tof_height();
        EXPORT_API bool xslam_get_tof_image( unsigned char *data, int width, int height );
        EXPORT_API bool xslam_get_depth_data( float *data );
        EXPORT_API bool xslam_get_cloud_data( Vector3 *cloud );

        // Stereo
        EXPORT_API int xslam_get_stereo_width();
        EXPORT_API int xslam_get_stereo_height();
        EXPORT_API bool xslam_get_left_image( unsigned char *data, int width, int height );
        EXPORT_API bool xslam_get_right_image( unsigned char *data, int width, int height );

        EXPORT_API int xslam_get_stereo_max_points();
        EXPORT_API bool xslam_get_left_points( Vector2 *points, int *size );
        EXPORT_API bool xslam_get_right_points( Vector2 *points, int *size );

        // IMU
        EXPORT_API bool xslam_get_imu( Vector3 *accel, Vector3 *gyro, Vector3 *magn, long long *timestamp );
        EXPORT_API bool xslam_get_imu_array( Vector3 *imu, long long *timestamp );

        // 3DOF
        EXPORT_API bool xslam_get_3dof( Orientation *o );

        // Event
        EXPORT_API bool xslam_get_event( int *type, int *state, long long *timestamp );

        // Configuration
        EXPORT_API bool xslam_set_aec(int p1, int p2, int p3, int p4, int p5, int p6);
        EXPORT_API bool xslam_set_imu_configuration(int mode, int stereoOffsetUs, int edgePredUs, bool edgeImuFusion, bool imuSyncedWithinEdgePacket);
        EXPORT_API bool xslam_set_flip(bool flip);
        EXPORT_API bool xslam_set_post_filter(bool enabled, float rotationParam, float translationParam);
        EXPORT_API bool xslam_set_imu_fusion(int imuFusionMode, bool synced, float delay, float prediction);
        EXPORT_API bool xslam_set_rgb_resolution(RgbResolution res);

        // Switches
        EXPORT_API void xslam_start_rgb_stream();
        EXPORT_API void xslam_stop_rgb_stream();
        EXPORT_API void xslam_start_tof_stream();
        EXPORT_API void xslam_stop_tof_stream();
        EXPORT_API void xslam_start_stereo_stream();
        EXPORT_API void xslam_stop_stereo_stream();
        EXPORT_API void xslam_start_speaker_stream();
        EXPORT_API void xslam_stop_speaker_stream();

        // HID read and write
        EXPORT_API bool xslam_write_hid(unsigned char *wdata, int wlen);
        EXPORT_API bool xslam_write_and_read_hid(unsigned char *wdata, int wlen, unsigned char *rdata, int rlen);

        // read Calibrations
        EXPORT_API bool readIMUBias(imu_bias *bias);
        EXPORT_API bool readStereoFisheyesCalibration(stereo_fisheyes *calib, int *imu_fisheye_shift_us);
        EXPORT_API bool readDisplayCalibration(pdm_calibration *calib);
        EXPORT_API bool readToFCalibration(pdm_calibration *calib);
        EXPORT_API bool readRGBCalibration(rgb_calibration *calib);
        EXPORT_API bool readStereoFisheyesPDMCalibration(stereo_pdm_calibration *calib);
        EXPORT_API bool readStereoDisplayCalibration(stereo_pdm_calibration *calib);

        // CNN
        EXPORT_API bool xslam_set_cnn_model( const char *path );
        EXPORT_API bool xslam_set_cnn_descriptor( const  char* path );
        EXPORT_API bool xslam_set_cnn_model_s( const std::string &path );
        EXPORT_API bool xslam_set_cnn_descriptor_s( const std::string &path );
        EXPORT_API bool xslam_set_cnn_source( int source );

        // Audio
        EXPORT_API int xslam_transfer_speaker_buffer(const unsigned char *data, int len);
        EXPORT_API bool xslam_play_sound( const unsigned char *data, int len );
        EXPORT_API bool xslam_play_sound_file( const char *path );
        EXPORT_API bool xslam_is_playing( );
        EXPORT_API void xslam_stop_play( );
        EXPORT_API bool xslam_set_mic_callback( cb_data cb );
        EXPORT_API void xslam_unset_mic_callback();

		//planedetect_stereo
		EXPORT_API bool xslam_start_detect_plane_from_stereo();
		EXPORT_API bool xslam_get_plane_from_stereo(unsigned char *data, int *len);
		EXPORT_API bool xslam_stop_detect_plane_from_stereo();
		//planedetect_tof
		EXPORT_API bool xslam_start_detect_plane_from_tof();
		EXPORT_API bool xslam_get_plane_from_tof(unsigned char *data, int *len);
		EXPORT_API bool xslam_stop_detect_plane_from_tof();

	}
}

