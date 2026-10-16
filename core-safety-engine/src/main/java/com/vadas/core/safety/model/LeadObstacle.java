package com.vadas.core.safety.model;

import androidx.annotation.NonNull;

/**
 * Representation of the leading vehicle or obstacle detected by perception systems.
 */
public final class LeadObstacle {

    public enum ObstacleType {
        CAR,
        MOTORCYCLE,
        TRUCK_BUS,
        PEDESTRIAN,
        UNKNOWN
    }

    private final double distanceMeters;
    private final double relativeSpeedKmh; // Positive when ego vehicle is closing in
    private final ObstacleType type;
    private final float confidence;

    public LeadObstacle(double distanceMeters,
                        double relativeSpeedKmh,
                        @NonNull ObstacleType type,
                        float confidence) {
        this.distanceMeters = Math.max(0.0, distanceMeters);
        this.relativeSpeedKmh = relativeSpeedKmh;
        this.type = type;
        this.confidence = confidence;
    }

    public double getDistanceMeters() {
        return distanceMeters;
    }

    public double getRelativeSpeedKmh() {
        return relativeSpeedKmh;
    }

    @NonNull
    public ObstacleType getType() {
        return type;
    }

    public float getConfidence() {
        return confidence;
    }

    @Override
    public String toString() {
        return "LeadObstacle{" +
                "distanceMeters=" + distanceMeters +
                ", relativeSpeedKmh=" + relativeSpeedKmh +
                ", type=" + type +
                ", confidence=" + confidence +
                '}';
    }
}
