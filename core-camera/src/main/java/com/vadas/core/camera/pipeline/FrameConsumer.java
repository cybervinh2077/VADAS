package com.vadas.core.camera.pipeline;

import androidx.annotation.NonNull;

import com.vadas.core.camera.model.CameraFrame;

/**
 * Interface for consuming real-time camera frames in the perception pipeline.
 * Implementations MUST call {@link CameraFrame#release()} once processing completes,
 * or {@link CameraFrame#retain()} if handing off to an asynchronous executor thread.
 */
@FunctionalInterface
public interface FrameConsumer {

    /**
     * Called when a new frame is captured from the camera or RTSP stream.
     *
     * @param frame the live frame ready for processing.
     */
    void onFrameAvailable(@NonNull CameraFrame frame);
}
