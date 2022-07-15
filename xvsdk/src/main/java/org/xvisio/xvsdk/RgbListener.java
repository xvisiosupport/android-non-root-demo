package org.xvisio.xvsdk;

public interface RgbListener {
    void onRgb(int width, int height, int[] data);

    void onFps(int fps);
}
