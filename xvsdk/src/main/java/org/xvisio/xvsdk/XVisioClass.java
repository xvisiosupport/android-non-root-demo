package org.xvisio.xvsdk;

abstract class XVisioClass implements AutoCloseable {
    static {
        System.loadLibrary("xslam_wrapper");
    }

    protected long mHandle = 0;
    protected boolean mOwner = true;

    public long getHandle() {
        return mHandle;
    }
}
