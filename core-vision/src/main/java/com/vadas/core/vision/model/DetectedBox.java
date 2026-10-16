package com.vadas.core.vision.model;

import androidx.annotation.NonNull;

/**
 * Bounding box detection with estimated distance in 3D camera coordinates.
 */
public final class DetectedBox {

    public enum ObjectClass {
        VEHICLE,
        CAR,
        MOTORCYCLE,
        BUS_TRUCK,
        PEDESTRIAN,
        TRAFFIC_LIGHT_RED,
        TRAFFIC_LIGHT_GREEN,
        TRAFFIC_LIGHT_YELLOW,
        UNKNOWN
    }

    private final float left;
    private final float top;
    private final float right;
    private final float bottom;
    private final float confidence;
    private final ObjectClass objectClass;
    private final double estimatedDistanceMeters;

    public DetectedBox(float left, float top, float right, float bottom,
                       float confidence, @NonNull ObjectClass objectClass,
                       double estimatedDistanceMeters) {
        this.left = left;
        this.top = top;
        this.right = right;
        this.bottom = bottom;
        this.confidence = confidence;
        this.objectClass = objectClass;
        this.estimatedDistanceMeters = estimatedDistanceMeters;
    }

    public float getLeft() {
        return left;
    }

    public float getTop() {
        return top;
    }

    public float getRight() {
        return right;
    }

    public float getBottom() {
        return bottom;
    }

    public float getConfidence() {
        return confidence;
    }

    @NonNull
    public ObjectClass getObjectClass() {
        return objectClass;
    }

    public double getEstimatedDistanceMeters() {
        return estimatedDistanceMeters;
    }
}
