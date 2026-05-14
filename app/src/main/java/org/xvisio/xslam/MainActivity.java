package org.xvisio.xslam;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.storage.StorageManager;
import android.os.storage.StorageVolume;
import android.util.Log;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

import org.xvisio.xvsdk.XCamera;

import java.nio.ByteBuffer;
import java.util.List;

public class MainActivity extends AppCompatActivity {
    private static final String TAG = "XVSDK Demo";

    private static final int PERMISSIONS_REQUEST_CAMERA = 0;
    private boolean mPermissionsGranted = false;

    private Context mAppContext = null;
    private XCamera mCamera = null;
    private Handler mHandler;

    CheckBox m_CheckBoxSave;
    TextView m_tvSaveTime;
    TextView m_tvSlamFps;
    TextView m_tvFisheyeFps;
    TextView m_tvRgb1Fps;
    TextView m_tvRgb2Fps;
    TextView m_tvGestureFps;
    TextView m_tvPose;
    TextView m_tvGestureData;

    ImageView m_ivFisheye;
    ImageView m_ivRgb1;
    ImageView m_ivRgb2;

    String mSdcardPath = "";
    int mUpdateCount = 0;

    static final int IMAGE_WIDTH = 640;
    static final int IMAGE_HEIGHT = 480;
    ByteBuffer m_fisheyeBuffer = ByteBuffer.allocateDirect(IMAGE_WIDTH * IMAGE_HEIGHT);
    ByteBuffer m_rgb1Buffer = ByteBuffer.allocateDirect(IMAGE_WIDTH * IMAGE_HEIGHT * 4);
    ByteBuffer m_rgb2Buffer = ByteBuffer.allocateDirect(IMAGE_WIDTH * IMAGE_HEIGHT * 4);

    String getSdcardPath() {
        try {
            StorageManager storageManager = (StorageManager) getSystemService(Context.STORAGE_SERVICE);
            List<StorageVolume> storageVolumes = storageManager.getStorageVolumes();

            for (StorageVolume volume : storageVolumes) {
                if (volume.isRemovable()) {
                    String path = null;
                    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
                        path = volume.getDirectory().getAbsolutePath();
                    }
                    if (!path.isEmpty()) {
                        return path;
                    }
                }
            }
        } catch (Throwable e) {
            e.printStackTrace();
        }

        return null;
    }

    Runnable m_updateRunnable = new Runnable() {
        @Override
        public void run() {
            mUpdateCount++;
            updateUI();
            mHandler.postDelayed(m_updateRunnable, 33);
        }
    };

    void updateUI() {
        if (mUpdateCount % 3 == 0) {
            m_CheckBoxSave.setTextColor(XCamera.isReady() ? Color.GREEN : Color.RED);
            m_CheckBoxSave.setEnabled(XCamera.isReady());
            int seconds = XCamera.getRecordTime();
            String time = String.format("time: %02d:%02d", seconds / 60, seconds % 60);
            m_tvSaveTime.setText(time);
        }

        if (mUpdateCount % 15 == 0) {
            String slamFps = "slam: " + XCamera.getFps(1) + "fps";
            m_tvSlamFps.setText(slamFps);

            String fisheyeFps = "fisheye: " + XCamera.getFps(2) + "fps";
            m_tvFisheyeFps.setText(fisheyeFps);

            String rgb1Fps = "rgb1: " + XCamera.getFps(3) + "fps";
            m_tvRgb1Fps.setText(rgb1Fps);

            String rgb2Fps = "rgb2: " + XCamera.getFps(4) + "fps";
            m_tvRgb2Fps.setText(rgb2Fps);

            String gestureFps = "gesture: " + XCamera.getFps(5) + "fps";
            m_tvGestureFps.setText(gestureFps);
        }

        String pose = "pose: " + XCamera.getPose();
        m_tvPose.setText(pose);

        String gesture = "gesture: " + XCamera.getGesture();
        m_tvGestureData.setText(gesture);

        if (XCamera.getFisheyeImage(m_fisheyeBuffer) > 0) {
            drawGrayImage(m_ivFisheye, m_fisheyeBuffer);
        }

        if (XCamera.getRgb1Image(m_rgb1Buffer) > 0) {
            drawRgbImage(m_ivRgb1, m_rgb1Buffer);
        }

        if (XCamera.getRgb2Image(m_rgb2Buffer) > 0) {
            drawRgbImage(m_ivRgb2, m_rgb2Buffer);
        }
    }

    void drawGrayImage(ImageView imageView, ByteBuffer buffer) {
        int width = imageView.getWidth();
        int height = imageView.getHeight();
        Bitmap bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        int[] data = new int[width * height];
        for (int i = 0; i < data.length; i++) {
            byte v = buffer.array()[i];
            data[i] = 0xFF000000 + (v << 16 & 0xFF0000) + (v << 8 & 0xFF00) + (v & 0xFF);
        }
        bitmap.setPixels(data, 0, width, 0, 0, width, height);
        imageView.setImageBitmap(bitmap);
    }

    void drawJpgImage(ImageView imageView, ByteBuffer buffer, int len) {
        BitmapFactory.Options options = new BitmapFactory.Options();
        options.inJustDecodeBounds = false;
        options.inPreferredConfig = Bitmap.Config.RGB_565;
        Bitmap bitmap = BitmapFactory.decodeByteArray(buffer.array(), 0, len, options);
        imageView.setImageBitmap(bitmap);
    }

    void drawRgbImage(ImageView imageView, ByteBuffer buffer) {
        int width = imageView.getWidth();
        int height = imageView.getHeight();
        int[] colors = new int[width * height];
        for (int i = 0; i < colors.length; i++) {
            int r = buffer.array()[i * 4] & 0xFF;
            int g = buffer.array()[i * 4 + 1] & 0xFF;
            int b = buffer.array()[i * 4 + 2] & 0xFF;
            int a = buffer.array()[i * 4 + 3] & 0xFF;
            colors[i] = (a << 24) | (r << 16) | (g << 8) | b;
        }
        Bitmap bitmap = Bitmap.createBitmap(colors, width, height, Bitmap.Config.ARGB_8888);
        imageView.setImageBitmap(bitmap);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        mAppContext = getApplicationContext();
        mHandler = new Handler(Looper.getMainLooper());
        setContentView(R.layout.activity_save);
        mSdcardPath = getSdcardPath();
        Log.i("MainActivity", "sdcard:" + mSdcardPath);
        m_tvSaveTime = findViewById(R.id.tv_time);
        m_tvSlamFps = findViewById(R.id.tv_slam);
        m_tvFisheyeFps = findViewById(R.id.tv_fisheye);
        m_tvRgb1Fps = findViewById(R.id.tv_rgb1);
        m_tvRgb2Fps = findViewById(R.id.tv_rgb2);
        m_tvGestureFps = findViewById(R.id.tv_gesture);
        m_tvPose = findViewById(R.id.tv_pose);
        m_tvGestureData = findViewById(R.id.tv_gesture_data);
        m_ivFisheye = findViewById(R.id.iv_fisheye);
        m_ivRgb1 = findViewById(R.id.iv_rgb1);
        m_ivRgb2 = findViewById(R.id.iv_rgb2);
        m_CheckBoxSave = findViewById(R.id.checkbox_save);
        TextView tvSdcard = findViewById(R.id.tv_sdcard);
        if (!mSdcardPath.isEmpty()) {
            String path = "path:" + mSdcardPath + "/xv_save";
            tvSdcard.setText(path);
        }

        m_CheckBoxSave.setEnabled(false);
        m_CheckBoxSave.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton compoundButton, boolean b) {
                if (!XCamera.isReady()) {
                    Toast.makeText(mAppContext, "device Not ready", Toast.LENGTH_SHORT).show();
                    compoundButton.setChecked(false);
                    return;
                }

                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        XCamera.nSaveData(mSdcardPath, b);
                    }
                }).start();
            }
        });


        String[] permissions = new String[]{Manifest.permission.READ_EXTERNAL_STORAGE, Manifest.permission.WRITE_EXTERNAL_STORAGE, Manifest.permission.CAMERA};
        requestPermissions(permissions, 101);
        mHandler.postDelayed(m_updateRunnable, 50);
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        if (checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(new String[]{Manifest.permission.CAMERA}, PERMISSIONS_REQUEST_CAMERA);
            return;
        }
        mPermissionsGranted = true;
        init();
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
    }

    private void init() {
        if (mCamera == null) {
            mCamera = new XCamera();
            mCamera.init(mAppContext);
        }
    }
}
