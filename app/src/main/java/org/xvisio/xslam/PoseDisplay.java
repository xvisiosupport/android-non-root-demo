package org.xvisio.xslam;

import android.content.Context;
import android.util.AttributeSet;
import android.widget.LinearLayout;
import android.widget.TextView;

public class PoseDisplay extends LinearLayout {
    public PoseDisplay(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    public void setValue(double x, double y, double z, double roll, double pitch, double yaw) {
        TextView x_view = findViewById(R.id.xValue);
        x_view.setText(String.format("%.3f", x));
        TextView y_view = findViewById(R.id.yValue);
        y_view.setText(String.format("%.3f", y));
        TextView z_view = findViewById(R.id.zValue);
        z_view.setText(String.format("%.3f", z));

        TextView roll_view = findViewById(R.id.rollValue);
        roll_view.setText(String.format("%.3f", roll));
        TextView pitch_view = findViewById(R.id.pitchValue);
        pitch_view.setText(String.format("%.3f", pitch));
        TextView yaw_view = findViewById(R.id.yawValue);
        yaw_view.setText(String.format("%.3f", yaw));
    }
}
