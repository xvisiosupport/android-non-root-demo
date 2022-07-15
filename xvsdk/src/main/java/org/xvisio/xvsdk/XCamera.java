package org.xvisio.xvsdk;

import android.Manifest;
import android.app.Activity;
import android.content.Context;
import android.content.pm.PackageManager;
import android.os.Handler;
import android.util.Log;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class XCamera extends XVisioClass {
    private static final String TAG = XCamera.class.getSimpleName();

    public enum Stream {
        SLAM,
        IMU,
        RGB,
        TOF,
        STEREO,
        SGBM
    }

    private static DeviceWatcher mDeviceWatcher;
    private DeviceListener mListener;

    ExecutorService mService = Executors.newSingleThreadExecutor();

    private static final int PERMISSIONS_REQUEST_CAMERA = 0;

    public XCamera() {
        Log.d(TAG, "Creation");
    }

    private Context mContext;

    public void init(Context context) {
        mContext = context;

        if (mDeviceWatcher == null)
            mDeviceWatcher = new DeviceWatcher(context);

        initCallbacks();
    }

    static public void requestPermissions(Activity activity, Context context) {
        if (android.os.Build.VERSION.SDK_INT > android.os.Build.VERSION_CODES.O &&
                context.checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            activity.requestPermissions(new String[]{Manifest.permission.CAMERA}, PERMISSIONS_REQUEST_CAMERA);
            return;
        }
    }

    public void startStream(final Stream stream) {
        mService.execute(new Runnable() {
            @Override
            public void run() {
                switch (stream) {
                    case SLAM:
                        startSlamStream();
                        break;

                    case IMU:
                        startImuStream();
                        break;

                    case RGB:
                        startRgbStream();
                        break;

                    case TOF:
                        startTofStream();
                        break;

                    case STEREO:
                        startStereoStream();
                        break;

                    case SGBM:
                        startSgbmStream();
                        break;

                    default:
                        break;
                }
            }
        });
    }

    public void stopStream(final Stream stream) {
        mService.execute(new Runnable() {
            @Override
            public void run() {
                switch (stream) {
                    case SLAM:
                        stopSlamStream();
                        break;

                    case IMU:
                        stopImuStream();
                        break;

                    case RGB:
                        stopRgbStream();
                        break;

                    case TOF:
                        stopTofStream();
                        break;

                    case STEREO:
                        stopStereoStream();
                        break;

                    case SGBM:
                        stopSgbmStream();
                        break;

                    default:
                        break;
                }
            }
        });
    }

    public void stopStreams() {
        mService.execute(new Runnable() {
            @Override
            public void run() {
                stopCallbacks();
            }
        });
    }

    @Override
    public void close() {
        removeDevicesChangedCallback();
    }

    public synchronized void setDevicesChangedCallback(DeviceListener listener) {
        removeDevicesChangedCallback();
        mListener = listener;
        if (mDeviceWatcher != null)
            mDeviceWatcher.addListener(mListener);
    }

    public synchronized void removeDevicesChangedCallback() {
        if (mListener != null && mDeviceWatcher != null)
            mDeviceWatcher.removeListener(mListener);
    }

    public synchronized void setRgbSolution(int mode) {
        nSetRgbSolution(mode);
    }

    public synchronized void setSlamMode(int mode) {
        nSetSlamMode(mode);
    }

    static PoseListener mPoseListener;

    public synchronized void setPoseCallback(PoseListener listener) {
        mPoseListener = listener;
    }

    static ImuListener mImuListener;

    public synchronized void setImuCallback(ImuListener listener) {
        mImuListener = listener;
    }

    static StereoListener mStereoListener;

    public synchronized void setStereoCallback(StereoListener listener) {
        mStereoListener = listener;
    }

    static RgbListener mRgbListener;

    public synchronized void setRgbCallback(RgbListener listener) {
        mRgbListener = listener;
    }

    static SgbmListener mSgbmListener;

    public synchronized void setSgbmCallback(SgbmListener listener) {
        mSgbmListener = listener;
    }

    static TofListener mTofListener;

    public synchronized void setTofCallback(TofListener listener) {
        mTofListener = listener;
    }

    static TofIrListener mTofIrListener;

    public synchronized void setTofIrCallback(TofIrListener listener) {
        mTofIrListener = listener;
    }

    private static native void nSetRgbSolution(int mode);

    private static native void nSetSlamMode(int mode);

    private static native void initCallbacks();

    private static native void stopCallbacks();

    private static native void startSlamStream();

    private static native void stopSlamStream();

    private static native void startImuStream();

    private static native void stopImuStream();

    private static native void startRgbStream();

    private static native void stopRgbStream();

    private static native void startTofStream();

    private static native void stopTofStream();

    private static native void startStereoStream();

    private static native void stopStereoStream();

    private static native void startSgbmStream();

    private static native void stopSgbmStream();

    public static void poseCallback(double x, double y, double z, double roll, double pitch, double yaw) {
        if (mPoseListener != null) {
            mPoseListener.onPose(x, y, z, roll, pitch, yaw);
        }
    }

    public static void imuCallback(double x, double y, double z) {
        if (mImuListener != null) {
            mImuListener.onImu(x, y, z);
        }
    }


    public static void stereoCallback(int width, int height, int[] data) {
        if (mStereoListener != null) {
            mStereoListener.onStereo(width, height, data);
        }
    }

    public static void rgbCallback(int width, int height, int[] data) {
        if (mRgbListener != null) {
            mRgbListener.onRgb(width, height, data);
        }
    }

    public static void rgbFpsCallback(int fps) {
        if (mRgbListener != null) {
            mRgbListener.onFps(fps);
        }
    }

    public static void sgbmCallback(int width, int height, int[] data) {
        if (mSgbmListener != null) {
            mSgbmListener.onSgbm(width, height, data);
        }
    }

    public static void tofCallback(int width, int height, int[] data) {
        if (mTofListener != null) {
            mTofListener.onTof(width, height, data);
        }
    }

    public static void tofIrCallback(int width, int height, int[] data) {
        if (mTofListener != null) {
            mTofIrListener.onTofIr(width, height, data);
        }
    }

}
