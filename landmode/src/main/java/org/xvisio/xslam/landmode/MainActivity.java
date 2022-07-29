package org.xvisio.xslam.landmode;

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
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;

import com.hjq.permissions.OnPermissionCallback;
import com.hjq.permissions.Permission;
import com.hjq.permissions.XXPermissions;

import org.xvisio.xvsdk.DeviceListener;
import org.xvisio.xvsdk.ImuListener;
import org.xvisio.xvsdk.PoseListener;
import org.xvisio.xvsdk.RgbListener;
import org.xvisio.xvsdk.StereoListener;
import org.xvisio.xvsdk.StreamData;
import org.xvisio.xvsdk.TofIrListener;
import org.xvisio.xvsdk.TofListener;
import org.xvisio.xvsdk.XCamera;

import java.util.List;

public class MainActivity extends AppCompatActivity {
    private static final String TAG = "XVSDK Demo";

    private static final int PERMISSIONS_REQUEST_CAMERA = 0;
    private boolean mPermissionsGranted = false;

    private Context mAppContext = null;
    public XCamera mCamera = null;
    private final boolean mIsRecord = false;
    private boolean isMixedMode = true;
    Button modeButton;
    ImageView iv;

    TextView mTvSolution;
    TextView mTvFps;
    private StreamHandler mMainHandler;

    int rgbSolution = 1;
    int mRgbFps = 0;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        mAppContext = getApplicationContext();

        setContentView(R.layout.activity_main);
        mMainHandler = new StreamHandler(Looper.getMainLooper());
        modeButton = findViewById(R.id.button_mode);
        modeButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if (mCamera != null) {
                    isMixedMode = !isMixedMode;
                    mCamera.setSlamMode(isMixedMode ? 0 : 2);
                    modeButton.setText(isMixedMode ? "[mixed] switch to edge+ mode" : "[edge] switch to mixed mode");
                }
            }
        });
        iv = findViewById(R.id.imageView);

        mTvSolution = findViewById(R.id.tv_solution);
        mTvFps = findViewById(R.id.tv_fps);

        Button rgbButton = findViewById(R.id.button_rgb);
        rgbButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                showRgbSolutionDialog();
            }
        });

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
                            Toast.makeText(mAppContext, "被永久拒绝授权，请手动授予存储权限", 1000).show();
                            // 如果是被永久拒绝就跳转到应用权限系统设置页面
                            XXPermissions.startPermissionActivity(mAppContext, permissions);
                        } else {
                            Toast.makeText(mAppContext, "获取存储权限失败", 1000).show();
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
            mCamera.setPoseCallback(mPoseListener);
            mCamera.init(mAppContext);
        }
        mCamera.setRgbSolution(rgbSolution);
    }


    class StreamHandler extends Handler {

        public static final int RGB = 1;
        public static final int TOF = 2;
        public static final int TOF_IR = 3;

        public StreamHandler(Looper looper) {
            super(looper);
        }

        @Override
        public void handleMessage(Message msg) {
            switch (msg.what) {
                case RGB:
                    onRgbCallback((StreamData) msg.obj);
                    break;

                case TOF:
                    onTofCallback((StreamData) msg.obj);
                    break;

                case TOF_IR:
                    onTofIrCallback((StreamData) msg.obj);
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
            mCamera.startStream(XCamera.Stream.RGB);
            mCamera.startStream(XCamera.Stream.TOF);
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


    private final StereoListener mStereoListener = new StereoListener() {
        @Override
        public void onStereo(final int width, final int height, final int[] pixels) {

        }
    };

    private void sendStreamMessage(int what, StreamData data) {
        mMainHandler.obtainMessage(what, data).sendToTarget();
    }

    private void onRgbCallback(StreamData data) {
        int width = data.getWidth();
        int height = data.getHeight();

        mTvSolution.setText(width + "X" + height);
        mTvFps.setText("" + mRgbFps);

        ImageView s = findViewById(R.id.rgbView);
        Bitmap bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        bitmap.setPixels(data.getPixels(), 0, width, 0, 0, width, height);

        if (height > width) {
            if (rgbSolution == 2) {
                s.setImageBitmap(bitmap);
            } else {
                Bitmap scaled = Bitmap.createScaledBitmap(bitmap, (int) (width * 0.5f), (int) (height * 0.5f), false);
                bitmap.recycle();
                s.setImageBitmap(scaled);
            }
        } else {
            if (rgbSolution == 2) {
                s.setImageBitmap(bitmap);
            } else {
                float scale = rgbSolution == 0 ? 0.25f : 0.5f;
                Bitmap scaled = Bitmap.createScaledBitmap(bitmap, (int) (width * scale), (int) (height * scale), false);
                bitmap.recycle();
                s.setImageBitmap(scaled);
            }
        }
    }


    private final RgbListener mRgbListener = new RgbListener() {
        @Override
        public void onFps(final int fps) {
            mRgbFps = fps;
        }

        @Override
        public void onRgb(final int width, final int height, final int[] pixels) {
            sendStreamMessage(StreamHandler.RGB, new StreamData(width, height, pixels));
        }
    };


    private void onTofCallback(StreamData data) {
        int width = data.getWidth();
        int height = data.getHeight();
        TextView solution = findViewById(R.id.tv_tof_solution);
        solution.setText(width + "X" + height);
        ImageView s = findViewById(R.id.tofView);
        Bitmap bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        bitmap.setPixels(data.getPixels(), 0, width, 0, 0, width, height);

        if (height > width) {
            Bitmap scaled = Bitmap.createScaledBitmap(bitmap, (int) (width * 1.4f), (int) (height * 1.4f), false);
            bitmap.recycle();
            s.setImageBitmap(scaled);
        } else {
            s.setImageBitmap(bitmap);
        }
    }

    private final TofListener mTofListener = new TofListener() {
        @Override
        public void onTof(int width, int height, int[] pixels) {
            sendStreamMessage(StreamHandler.TOF, new StreamData(width, height, pixels));
        }
    };

    private void onTofIrCallback(StreamData data) {
        int width = data.getWidth();
        int height = data.getHeight();
        TextView solution = findViewById(R.id.tv_ir_solution);
        solution.setText(width + "X" + height);

        ImageView s = findViewById(R.id.tofIrView);
        Bitmap bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        bitmap.setPixels(data.getPixels(), 0, width, 0, 0, width, height);

        if (height > width) {
            Bitmap scaled = Bitmap.createScaledBitmap(bitmap, (int) (width * 1.4f), (int) (height * 1.4f), false);
            bitmap.recycle();
            s.setImageBitmap(scaled);
        } else {
            s.setImageBitmap(bitmap);
        }
    }

    private final TofIrListener mTofIrListener = new TofIrListener() {
        @Override
        public void onTofIr(int width, int height, int[] pixels) {
            sendStreamMessage(StreamHandler.TOF_IR, new StreamData(width, height, pixels));
        }
    };
}
