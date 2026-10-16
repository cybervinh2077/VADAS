package com.vadas.core.camera.model;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.nio.ByteBuffer;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * High-performance, zero-copy camera frame container designed for real-time ADAS processing.
 * Supports reference counting to safely recycle ByteBuffers without triggering Java GC pauses.
 */
public final class CameraFrame {

    public enum PixelFormat {
        NV21,
        YUV_420_888,
        RGBA_8888
    }

    private final ByteBuffer buffer;
    private final int width;
    private final int height;
    private final int rotationDegrees;
    private final long timestampNs;
    private final PixelFormat format;

    private final AtomicInteger refCount = new AtomicInteger(1);
    @Nullable
    private final android.graphics.Bitmap bitmap;
    @Nullable
    private final FrameRecycler recycler;

    public interface FrameRecycler {
        void recycle(@NonNull CameraFrame frame);
    }

    public CameraFrame(@NonNull ByteBuffer buffer,
                       int width,
                       int height,
                       int rotationDegrees,
                       long timestampNs,
                       @NonNull PixelFormat format,
                       @Nullable FrameRecycler recycler) {
        this(buffer, null, width, height, rotationDegrees, timestampNs, format, recycler);
    }

    public CameraFrame(@NonNull android.graphics.Bitmap bitmap,
                       int width,
                       int height,
                       int rotationDegrees,
                       long timestampNs,
                       @NonNull PixelFormat format,
                       @Nullable FrameRecycler recycler) {
        this(ByteBuffer.allocate(0), bitmap, width, height, rotationDegrees, timestampNs, format, recycler);
    }

    public CameraFrame(@NonNull ByteBuffer buffer,
                       @Nullable android.graphics.Bitmap bitmap,
                       int width,
                       int height,
                       int rotationDegrees,
                       long timestampNs,
                       @NonNull PixelFormat format,
                       @Nullable FrameRecycler recycler) {
        this.buffer = buffer;
        this.bitmap = bitmap;
        this.width = width;
        this.height = height;
        this.rotationDegrees = rotationDegrees;
        this.timestampNs = timestampNs;
        this.format = format;
        this.recycler = recycler;
    }

    @Nullable
    public android.graphics.Bitmap getBitmap() {
        return bitmap;
    }

    @NonNull
    public ByteBuffer getBuffer() {
        return buffer;
    }

    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }

    public int getRotationDegrees() {
        return rotationDegrees;
    }

    public long getTimestampNs() {
        return timestampNs;
    }

    @NonNull
    public PixelFormat getFormat() {
        return format;
    }

    /**
     * Increments the reference count when a pipeline branch processes this frame.
     */
    public CameraFrame retain() {
        refCount.incrementAndGet();
        return this;
    }

    /**
     * Decrements the reference count. When count drops to 0, recycling callback is invoked.
     */
    public void release() {
        if (refCount.decrementAndGet() == 0) {
            if (recycler != null) {
                recycler.recycle(this);
            }
        }
    }

    public boolean isAlive() {
        return refCount.get() > 0;
    }
}
