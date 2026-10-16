package com.vadas.core.vision.model;

import androidx.annotation.NonNull;

import java.util.Arrays;

/**
 * Geometric representation of a detected lane line boundary (OpenPilot format).
 * Points are represented in vehicle reference frame:
 * - X: longitudinal distance ahead (meters, positive forward)
 * - Y: lateral displacement (meters, negative = left of ego-center, positive = right of ego-center)
 */
public final class LaneLine {

    private final double[] longitudinalX; // Distance points ahead (e.g. 0m to 100m)
    private final double[] lateralY;       // Lateral position at each distance point
    private final float probability;      // Model confidence that this lane exists (0.0 to 1.0)
    private final boolean isEgoCrossing;  // True if vehicle tires are departing across this line

    public LaneLine(double[] longitudinalX,
                    double[] lateralY,
                    float probability,
                    boolean isEgoCrossing) {
        this.longitudinalX = longitudinalX;
        this.lateralY = lateralY;
        this.probability = probability;
        this.isEgoCrossing = isEgoCrossing;
    }

    public double[] getLongitudinalX() {
        return longitudinalX;
    }

    public double[] getLateralY() {
        return lateralY;
    }

    public float getProbability() {
        return probability;
    }

    public boolean isEgoCrossing() {
        return isEgoCrossing;
    }

    /**
     * Estimated lateral distance from vehicle center to this lane line at the front bumper (X = 0m).
     */
    public double getBumperLateralDistance() {
        if (lateralY != null && lateralY.length > 0) {
            return lateralY[0];
        }
        return 0.0;
    }

    @NonNull
    @Override
    public String toString() {
        return "LaneLine{" +
                "bumperY=" + String.format("%.2f", getBumperLateralDistance()) + "m" +
                ", prob=" + String.format("%.2f", probability) +
                ", crossing=" + isEgoCrossing +
                '}';
    }
}
