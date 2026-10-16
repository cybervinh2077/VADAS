package com.vadas.core.vision;

import android.content.Context;

import androidx.annotation.NonNull;

import com.vadas.core.camera.model.CameraFrame;
import com.vadas.core.vision.model.VisionPerceptionResult;

/**
 * On-device neural vision inference engine abstraction.
 * Executes object detection (vehicles/motorcycles), lane departure, and traffic light models.
 */
public interface VisionEngine {

    void initialize(@NonNull Context context) throws Exception;

    @NonNull
    VisionPerceptionResult processFrame(@NonNull CameraFrame frame);

    void close();
}
