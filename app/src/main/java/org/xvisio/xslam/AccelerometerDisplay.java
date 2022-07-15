package org.xvisio.xslam;

import android.content.Context;
import android.graphics.Color;
import android.util.AttributeSet;
import android.util.Log;
import android.widget.LinearLayout;

public class AccelerometerDisplay extends LinearLayout {
    public AccelerometerDisplay(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    @Override
    public void onFinishInflate() {
        super.onFinishInflate();
        ImuDisplay ix = findViewById(R.id.imuX);
        ix.setColor(Color.RED);
        ImuDisplay iy = findViewById(R.id.imuY);
        iy.setColor(Color.GREEN);
        ImuDisplay iz = findViewById(R.id.imuZ);
        iz.setColor(Color.BLUE);
    }

    public void setValues(double x, double y, double z) {
        ImuDisplay ix = findViewById(R.id.imuX);
        ix.setValue(x);
        ImuDisplay iy = findViewById(R.id.imuY);
        iy.setValue(y);
        ImuDisplay iz = findViewById(R.id.imuZ);
        iz.setValue(z);
    }

}
