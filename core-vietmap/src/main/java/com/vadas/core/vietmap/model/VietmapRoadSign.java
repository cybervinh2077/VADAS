package com.vadas.core.vietmap.model;

import androidx.annotation.NonNull;

/**
 * Approaching road sign alert provided by Vietmap SDK.
 */
public final class VietmapRoadSign {

    public enum SignType {
        SPEED_LIMIT,
        SPEED_CAMERA,
        SCHOOL_ZONE,
        NO_OVERTAKING,
        RESIDENTIAL_AREA_ENTRY,
        RESIDENTIAL_AREA_EXIT,
        RAILWAY_CROSSING
    }

    private final SignType type;
    private final int value; // e.g. 60 for speed limit 60
    private final double distanceToSignMeters;
    private final String description;

    public VietmapRoadSign(@NonNull SignType type,
                           int value,
                           double distanceToSignMeters,
                           @NonNull String description) {
        this.type = type;
        this.value = value;
        this.distanceToSignMeters = distanceToSignMeters;
        this.description = description;
    }

    @NonNull
    public SignType getType() {
        return type;
    }

    public int getValue() {
        return value;
    }

    public double getDistanceToSignMeters() {
        return distanceToSignMeters;
    }

    @NonNull
    public String getDescription() {
        return description;
    }

    @Override
    public String toString() {
        return "VietmapRoadSign{" +
                "type=" + type +
                ", value=" + value +
                ", distanceToSignMeters=" + distanceToSignMeters +
                ", description='" + description + '\'' +
                '}';
    }
}
