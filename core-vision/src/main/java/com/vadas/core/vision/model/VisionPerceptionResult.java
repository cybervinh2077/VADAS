package com.vadas.core.vision.model;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.Collections;
import java.util.List;

/**
 * Perception output aggregated across TFLite detection heads.
 */
public final class VisionPerceptionResult {

    private final List<DetectedBox> detectedObjects;
    @Nullable
    private final DetectedBox primaryLeadVehicle;
    private final boolean laneDepartureDetected;
    private final boolean redLightViolationRisk;
    private final long timestampNs;

    public VisionPerceptionResult(@NonNull List<DetectedBox> detectedObjects,
                                  @Nullable DetectedBox primaryLeadVehicle,
                                  boolean laneDepartureDetected,
                                  boolean redLightViolationRisk,
                                  long timestampNs) {
        this.detectedObjects = Collections.unmodifiableList(detectedObjects);
        this.primaryLeadVehicle = primaryLeadVehicle;
        this.laneDepartureDetected = laneDepartureDetected;
        this.redLightViolationRisk = redLightViolationRisk;
        this.timestampNs = timestampNs;
    }

    @NonNull
    public List<DetectedBox> getDetectedObjects() {
        return detectedObjects;
    }

    @Nullable
    public DetectedBox getPrimaryLeadVehicle() {
        return primaryLeadVehicle;
    }

    public boolean isLaneDepartureDetected() {
        return laneDepartureDetected;
    }

    public boolean isRedLightViolationRisk() {
        return redLightViolationRisk;
    }

    public long getTimestampNs() {
        return timestampNs;
    }
}
