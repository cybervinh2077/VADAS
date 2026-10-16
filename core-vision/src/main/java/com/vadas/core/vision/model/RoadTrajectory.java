package com.vadas.core.vision.model;

import androidx.annotation.NonNull;

/**
 * OpenPilot predicted path trajectory for vehicle driving guidance.
 */
public final class RoadTrajectory {

    private final double[] pathX; // Forward distance points (m)
    private final double[] pathY; // Lateral offsets (m)
    private final double[] pathZ; // Vertical elevation / pitch offsets (m)
    private final float curvature; // Road curvature (1/R)

    public RoadTrajectory(double[] pathX, double[] pathY, double[] pathZ, float curvature) {
        this.pathX = pathX;
        this.pathY = pathY;
        this.pathZ = pathZ;
        this.curvature = curvature;
    }

    public double[] getPathX() {
        return pathX;
    }

    public double[] getPathY() {
        return pathY;
    }

    public double[] getPathZ() {
        return pathZ;
    }

    public float getCurvature() {
        return curvature;
    }

    @NonNull
    @Override
    public String toString() {
        return "RoadTrajectory{" +
                "points=" + (pathX != null ? pathX.length : 0) +
                ", curvature=" + curvature +
                '}';
    }
}
