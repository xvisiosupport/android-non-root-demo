package org.xvisio.xslam;

import org.xvisio.xvsdk.SgbmListener;
import org.xvisio.xvsdk.StreamData;
import org.xvisio.xvsdk.TofIrListener;
import org.xvisio.xvsdk.TofListener;
import org.xvisio.xvsdk.XCamera;
import org.xvisio.xvsdk.DeviceListener;
import org.xvisio.xvsdk.ImuListener;
import org.xvisio.xvsdk.StereoListener;
import org.xvisio.xvsdk.PoseListener;
import org.xvisio.xvsdk.RgbListener;

import android.Manifest;
import android.content.Context;
import android.content.DialogInterface;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.Message;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.ImageView;
import android.widget.RadioGroup;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;
import androidx.constraintlayout.widget.ConstraintLayout;

import com.hjq.permissions.OnPermissionCallback;
import com.hjq.permissions.Permission;
import com.hjq.permissions.XXPermissions;

import java.util.List;

public class MainActivity extends AppCompatActivity {
    private static final String TAG = "XVSDK Demo";

    private static final int PERMISSIONS_REQUEST_CAMERA = 0;
    private boolean mPermissionsGranted = false;

    private Context mAppContext = null;
    private XCamera mCamera = null;

    private boolean isMixedMode = true;

    int rgbSolution = 1;
    int cameraSelect = 0;
    int modeSelect = 0;

    Button mBtRgbSolution;
    ImageView mIvStream;
    TextView mTvSolution, mTvFps;
    private StreamHandler mMainHandler;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        mAppContext = getApplicationContext();
        mMainHandler = new StreamHandler(Looper.getMainLooper());
        setContentView(R.layout.activity_main);

        mBtRgbSolution = findViewById(R.id.button_rgb);
        mBtRgbSolution.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                showRgbSolutionDialog();
            }
        });

        RadioGroup radioSlam = findViewById(R.id.radio_slam_mode);
        modeSelect = radioSlam.getCheckedRadioButtonId();
        radioSlam.setOnCheckedChangeListener(mSlamModeListener);

        CheckBox checkBoxSlam = findViewById(R.id.checkbox_slam);
        checkBoxSlam.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton compoundButton, boolean b) {
                if(b) {
                    mCamera.startStream(XCamera.Stream.SLAM);
                } else {
                    mCamera.stopStreams();
                }
            }
        });

        CheckBox checkBoxImu = findViewById(R.id.checkbox_imu);
        checkBoxImu.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton compoundButton, boolean b) {
                if(b) {
                    mCamera.startStream(XCamera.Stream.IMU);
                } else {
                    mCamera.stopStream(XCamera.Stream.IMU);
                }
            }
        });

        mIvStream = findViewById(R.id.rgbView);
        mTvSolution = findViewById(R.id.tv_rgb_solution);
        mTvFps = findViewById(R.id.tv_rgb_fps);
        RadioGroup cameraRadio = findViewById(R.id.radio_camera);
        cameraSelect = cameraRadio.getCheckedRadioButtonId();
        cameraRadio.setOnCheckedChangeListener(mCemeraSelectListener);

        //uvc permission https://github.com/saki4510t/UVCPermissionTest
        XXPermissions.with(this)
                // 不适配 Android 11 可以这样写
                .permission(Permission.Group.STORAGE)
                // 适配 Android 11 需要这样写，这里无需再写 Permission.Group.STORAGE
                // .permission(Permission.MANAGE_EXTERNAL_STORAGE)
                .permission(Permission.CAMERA)
                .request(new OnPermissionCallback() {

                    @Override
                    public void onGranted(List<String> permissions, boolean all) {
                        if (all) {
                            mPermissionsGranted = true;
                            init();
                            Log.i("test", "startRecord 0=");

//							Toast.makeText(mActivityContext,"获取存储权限成功", (int)1000).show();
                        }
                    }

                    @Override
                    public void onDenied(List<String> permissions, boolean never) {
                        if (never) {
                            Toast.makeText(mAppContext, "被永久拒绝授权，请手动授予存储权限", (int) 1000).show();
                            // 如果是被永久拒绝就跳转到应用权限系统设置页面
                            XXPermissions.startPermissionActivity(mAppContext, permissions);
                        } else {
                            Toast.makeText(mAppContext, "获取存储权限失败", (int) 1000).show();
                        }
                    }
                });

        mPermissionsGranted = true;
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        if (checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(new String[]{Manifest.permission.CAMERA}, PERMISSIONS_REQUEST_CAMERA);
            return;
        }
        mPermissionsGranted = true;
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (mPermissionsGranted) {
            init();
        } else {
            Log.e(TAG, "missing permissions");
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        if (mCamera != null) {
            mCamera.stopStreams();
        }
    }

    private void init() {
        if (mCamera == null) {
            mCamera = new XCamera();
            mCamera.init(mAppContext);
            mCamera.setDevicesChangedCallback(mListener);
            mCamera.setImuCallback(mImuListener);
            mCamera.setStereoCallback(mStereoListener);
            mCamera.setRgbCallback(mRgbListener);
            mCamera.setTofCallback(mTofListener);
            mCamera.setTofIrCallback(mTofIrListener);
            mCamera.setSgbmCallback(mSgbmListener);
            mCamera.setPoseCallback(mPoseListener);
            mCamera.init(mAppContext);
        }
        mCamera.setRgbSolution(rgbSolution);
    }

    RadioGroup.OnCheckedChangeListener mSlamModeListener = new RadioGroup.OnCheckedChangeListener() {
        @Override
        public void onCheckedChanged(RadioGroup radioGroup, int i) {
            if (modeSelect == i) {
                return;
            }
            modeSelect = i;

            switch (i) {
                case R.id.radio_mixed:
                    isMixedMode = true;
                    break;

                case R.id.radio_edge:
                    isMixedMode = false;
                    break;

                default:
                    break;
            }

            mCamera.setSlamMode(isMixedMode ? 0 : 2);
        }
    };

    RadioGroup.OnCheckedChangeListener mCemeraSelectListener = new RadioGroup.OnCheckedChangeListener() {
        @Override
        public void onCheckedChanged(RadioGroup radioGroup, int i) {
            if (cameraSelect == i) {
                return;
            }
            cameraSelect = i;
            mCamera.stopStream(XCamera.Stream.RGB);
            mCamera.stopStream(XCamera.Stream.TOF);
            mCamera.stopStream(XCamera.Stream.STEREO);
            mCamera.stopStream(XCamera.Stream.SGBM);

            switch (i) {
                case R.id.radio_rgb:
                    mCamera.startStream(XCamera.Stream.RGB);
                    break;

                case R.id.radio_tof:
                    mCamera.startStream(XCamera.Stream.TOF);
                    break;

                case R.id.radio_stereo:
                    mCamera.startStream(XCamera.Stream.STEREO);
                    break;

                case R.id.radio_sgbm:
                    mCamera.startStream(XCamera.Stream.SGBM);
                    break;

                default:
                    break;
            }
        }
    };

    class StreamHandler extends Handler {

        public static final int RGB = 1;
        public static final int TOF = 2;
        public static final int STEREO = 3;
        public static final int SGBM = 4;

        public StreamHandler(Looper looper) {
            super(looper);
        }

        @Override
        public void handleMessage(Message msg) {
            Log.e(TAG, "handleMessage " + msg.what);
            switch (msg.what) {
                case RGB:
                    mBtRgbSolution.setVisibility(View.VISIBLE);
                    onRgbCallback((StreamData) msg.obj);
                    break;

                case TOF:
                    mBtRgbSolution.setVisibility(View.GONE);
                    onTofCallback((StreamData) msg.obj);
                    break;

                case STEREO:
                    mBtRgbSolution.setVisibility(View.GONE);
                    onStereoCallback((StreamData) msg.obj);
                    break;

                case SGBM:
                    mBtRgbSolution.setVisibility(View.GONE);
                    onSgbmCallback((StreamData) msg.obj);
                    break;

                default:
                    break;
            }
        }
    }

    public void showRgbSolutionDialog() {
        String[] rgbSolutionItems = {"1920x1080", "1280x720", "640x480"};
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setSingleChoiceItems(rgbSolutionItems, rgbSolution, new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                rgbSolution = which;
                if (mCamera != null) {
                    mCamera.setRgbSolution(which);
                }
                dialog.dismiss();
            }
        });
        builder.show();
    }

    private final DeviceListener mListener = new DeviceListener() {
        @Override
        public void onDeviceAttach() {
        }

        @Override
        public void onDeviceDetach() {
        }
    };

    private final PoseListener mPoseListener = new PoseListener() {
        @Override
        public void onPose(final double x, final double y, final double z, final double roll, final double pitch, final double yaw) {
            mMainHandler.post(new Runnable() {
                @Override
                public void run() {
                    PoseDisplay pose_display = findViewById(R.id.poseDisplay);
                    pose_display.setValue(x, y, z, roll, pitch, yaw);
                }
            });

        }
    };

    private final ImuListener mImuListener = new ImuListener() {
        @Override
        public void onImu(final double x, final double y, final double z) {
            mMainHandler.post(new Runnable() {
                @Override
                public void run() {
                    AccelerometerDisplay a = findViewById(R.id.accelerometerDisplay);
                    a.setValues(x, y, z);
                }
            });
        }
    };

    private void onStereoCallback(StreamData data) {
        mTvSolution.setText(data.getWidth() + "X" + data.getHeight());
        mTvFps.setText("");
        Bitmap bitmap = Bitmap.createBitmap(data.getWidth(), data.getHeight(), Bitmap.Config.ARGB_8888);
        bitmap.setPixels(data.getPixels(), 0, data.getWidth(), 0, 0, data.getWidth(), data.getHeight());
        mIvStream.setImageBitmap(bitmap);
    }

    private final StereoListener mStereoListener = new StereoListener() {
        @Override
        public void onStereo(final int width, final int height, final int[] pixels) {
            sendStreamMessage(StreamHandler.STEREO, new StreamData(width, height, pixels));
        }
    };

    private void sendStreamMessage(int what, StreamData data) {
        mMainHandler.obtainMessage(what, data).sendToTarget();
    }

    private void onRgbCallback(StreamData data) {
        mTvSolution.setText(data.getWidth() + "X" + data.getHeight());
        Bitmap bitmap = Bitmap.createBitmap(data.getWidth(), data.getHeight(), Bitmap.Config.ARGB_8888);
        bitmap.setPixels(data.getPixels(), 0, data.getWidth(), 0, 0, data.getWidth(), data.getHeight());

        if (data.getHeight() == 480) {
            mIvStream.setImageBitmap(bitmap);
            return;
        }

        float scale = data.getHeight() == 720 ? 0.75f : 0.5f;
        Bitmap scaled = Bitmap.createScaledBitmap(bitmap, (int) (data.getWidth() * scale), (int) (data.getHeight() * scale), false);
        bitmap.recycle();
        mIvStream.setImageBitmap(scaled);
    }

    private final RgbListener mRgbListener = new RgbListener() {
        @Override
        public void onFps(final int fps) {
            mMainHandler.post(new Runnable() {
                @Override
                public void run() {
                    mTvFps.setVisibility(View.VISIBLE);
                    mTvFps.setText("FPS:  " + fps);
                }
            });
        }

        @Override
        public void onRgb(final int width, final int height, final int[] pixels) {
            sendStreamMessage(StreamHandler.RGB, new StreamData(width, height, pixels));
        }
    };

    private void onSgbmCallback(StreamData data) {
        mTvSolution.setText(data.getWidth() + "X" + data.getHeight());
        mTvFps.setText("");
        Bitmap bitmap = Bitmap.createBitmap(data.getWidth(), data.getHeight(), Bitmap.Config.ARGB_8888);
        bitmap.setPixels(data.getPixels(), 0, data.getWidth(), 0, 0, data.getWidth(), data.getHeight());
        mIvStream.setImageBitmap(bitmap);
    }

    private final SgbmListener mSgbmListener = new SgbmListener() {
        @Override
        public void onSgbm(int width, int height, int[] pixels) {
            sendStreamMessage(StreamHandler.SGBM, new StreamData(width, height, pixels));
        }
    };

    private void onTofCallback(StreamData data) {
        mTvSolution.setText(data.getWidth() + "X" + data.getHeight());
        mTvFps.setText("");
        Bitmap bitmap = Bitmap.createBitmap(data.getWidth(), data.getHeight(), Bitmap.Config.ARGB_8888);
        bitmap.setPixels(data.getPixels(), 0, data.getWidth(), 0, 0, data.getWidth(), data.getHeight());
        mIvStream.setImageBitmap(bitmap);
    }

    private final TofListener mTofListener = new TofListener() {
        @Override
        public void onTof(int width, int height, int[] pixels) {
            sendStreamMessage(StreamHandler.TOF, new StreamData(width, height, pixels));
        }
    };

    private final TofIrListener mTofIrListener = new TofIrListener() {
        @Override
        public void onTofIr(int width, int height, int[] pixels) {

        }
    };
}
