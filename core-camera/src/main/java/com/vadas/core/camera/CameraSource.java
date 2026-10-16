package com.vadas.core.camera;

import androidx.annotation.NonNull;

import com.vadas.core.camera.pipeline.FrameConsumer;

/**
 * Common abstraction for video frame sources (local CameraX or network RTSP Dashcam).
 */
public interface CameraSource {

    enum SourceType {
        LOCAL_DEVICE_CAMERAX,
        NETWORK_RTSP_DASHCAM,
        VIDEO_FILE
    }

    @NonNull
    SourceType getType();

    void start(@NonNull FrameConsumer consumer);

    void stop();

    boolean isStreaming();
}
