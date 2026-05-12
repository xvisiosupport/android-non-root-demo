package org.xvisio.xslam;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.os.storage.StorageManager;
import android.os.storage.StorageVolume;
import android.util.Log;
import android.widget.CheckBox;
import android.widget.CompoundButton;

import androidx.appcompat.app.AppCompatActivity;

import org.xvisio.xvsdk.XCamera;

import java.util.List;

public class MainActivity extends AppCompatActivity {
    private static final String TAG = "XVSDK Demo";

    private static final int PERMISSIONS_REQUEST_CAMERA = 0;
    private boolean mPermissionsGranted = false;

    private Context mAppContext = null;
    private XCamera mCamera = null;

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

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        mAppContext = getApplicationContext();
        setContentView(R.layout.activity_save);
        mSdcardPath = getSdcardPath();
        Log.i("MainActivity", "sdcard:" + mSdcardPath);
        CheckBox checkBoxSave = findViewById(R.id.checkbox_save);
        checkBoxSave.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton compoundButton, boolean b) {
                boolean ret = XCamera.nSaveData(mSdcardPath, b);
                compoundButton.setChecked(ret);
            }
        });


        String[] permissions = new String[]{Manifest.permission.READ_EXTERNAL_STORAGE, Manifest.permission.WRITE_EXTERNAL_STORAGE, Manifest.permission.CAMERA};
        requestPermissions(permissions, 101);
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
