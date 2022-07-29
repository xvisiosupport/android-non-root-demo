package org.xvisio.xslam.landmode;

import android.content.Context;
import android.content.res.ColorStateList;
import android.util.AttributeSet;
import android.widget.LinearLayout;
import android.widget.ProgressBar;

public class ImuDisplay extends LinearLayout {
    public ImuDisplay(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    public void setValue(double value) {
        ProgressBar nb = findViewById(R.id.negativeBar);
        ProgressBar pb = findViewById(R.id.positiveBar);
        if (value < 0) {
            nb.setProgress((int) (-value * 100));
            pb.setProgress(0);
        } else {
            nb.setProgress(0);
            pb.setProgress((int) (value * 100));
        }
    }

    public void setColor(int color) {
        ProgressBar nb = findViewById(R.id.negativeBar);
        ProgressBar pb = findViewById(R.id.positiveBar);

        nb.setProgressTintList(ColorStateList.valueOf(color));
        pb.setProgressTintList(ColorStateList.valueOf(color));
    }
}
