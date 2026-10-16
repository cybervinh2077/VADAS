package com.vadas.core.vietmap.model;

import androidx.annotation.NonNull;

/**
 * Real-time navigation speed information retrieved from Vietmap SDK.
 */
public final class VietmapSpeedInfo {

    private final double currentSpeedKmh;
    private final int speedLimitKmh;
    private final String roadName;
    private final boolean isOverSpeed;

    public VietmapSpeedInfo(double currentSpeedKmh,
                            int speedLimitKmh,
                            @NonNull String roadName) {
        this.currentSpeedKmh = currentSpeedKmh;
        this.speedLimitKmh = speedLimitKmh;
        this.roadName = roadName;
        this.isOverSpeed = speedLimitKmh > 0 && currentSpeedKmh > speedLimitKmh;
    }

    public double getCurrentSpeedKmh() {
        return currentSpeedKmh;
    }

    public int getSpeedLimitKmh() {
        return speedLimitKmh;
    }

    @NonNull
    public String getRoadName() {
        return roadName;
    }

    public boolean isOverSpeed() {
        return isOverSpeed;
    }

    @Override
    public String toString() {
        return "VietmapSpeedInfo{" +
                "currentSpeedKmh=" + currentSpeedKmh +
                ", speedLimitKmh=" + speedLimitKmh +
                ", roadName='" + roadName + '\'' +
                ", isOverSpeed=" + isOverSpeed +
                '}';
    }
}
