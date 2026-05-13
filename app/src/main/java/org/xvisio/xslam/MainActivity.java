package org.xvisio.xslam;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
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
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

import org.xvisio.xvsdk.XCamera;

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

    String mSdcardPath = "";

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
            updateUI();
            mHandler.postDelayed(m_updateRunnable, 50);
        }
    };

    void updateUI() {
        m_CheckBoxSave.setTextColor(XCamera.isReady() ? Color.GREEN : Color.RED);
        m_CheckBoxSave.setEnabled(XCamera.isReady());
        int seconds = XCamera.getRecordTime();
        String time = String.format("time: %02d:%02d", seconds/60, seconds%60);
        m_tvSaveTime.setText(time);

        String slamFps = "slam: " + XCamera.getFps(1) + "fps";
        m_tvSlamFps.setText(slamFps);

        String fisheyeFps = "fisheye: " + XCamera.getFps(2) + "fps";
        m_tvFisheyeFps.setText(fisheyeFps);

        String rgb1Fps = "rgb1: " + XCamera.getFps(3) + "fps";
        m_tvRgb1Fps.setText(rgb1Fps);

        String rgb2Fps = "rgb2: " + XCamera.getFps(4) + "fps";
        m_tvRgb2Fps.setText(rgb2Fps);
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
        m_CheckBoxSave = findViewById(R.id.checkbox_save);
        TextView tvSdcard = findViewById(R.id.tv_sdcard);
        if(!mSdcardPath.isEmpty()) {
            String path = "path:" + mSdcardPath + "/xv_save";
            tvSdcard.setText(path);
        }

        m_CheckBoxSave.setEnabled(false);
        m_CheckBoxSave.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton compoundButton, boolean b) {
                if(!XCamera.isReady()) {
                    Toast.makeText(mAppContext, "device Not ready", Toast.LENGTH_SHORT).show();
                    compoundButton.setChecked(false);
                    return;
                }
                boolean ret = XCamera.nSaveData(mSdcardPath, b);
                compoundButton.setChecked(ret);
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
