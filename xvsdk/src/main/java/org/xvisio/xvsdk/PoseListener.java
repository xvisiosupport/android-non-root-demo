package org.xvisio.xvsdk;

public interface PoseListener {
    void onPose(double x, double y, double z, double roll, double pitch, double yaw);
}
