package com.vadas.core.vision.pinhole;

import androidx.annotation.NonNull;

import com.vadas.core.vision.model.DetectedBox;

/**
 * Real-world dimensional priors (average height, width, length) for Vietnamese road traffic vehicles.
 */
public final class VehiclePrior {

    private final double heightMeters;
    private final double widthMeters;

    public VehiclePrior(double heightMeters, double widthMeters) {
        this.heightMeters = heightMeters;
        this.widthMeters = widthMeters;
    }

    public double getHeightMeters() {
        return heightMeters;
    }

    public double getWidthMeters() {
        return widthMeters;
    }

    /**
     * Retrieves typical vehicle dimension prior based on detected object classification.
     */
    @NonNull
    public static VehiclePrior getPriorForClass(@NonNull DetectedBox.ObjectClass objectClass) {
        switch (objectClass) {
            case VEHICLE:
            case CAR:
                // Standard sedan / hatchback / compact crossover (Vios, Accent, Fadil, CX-5)
                return new VehiclePrior(1.50, 1.78);
            case MOTORCYCLE:
                // Vietnamese scooters / commuter motorbikes (Wave, Vision, Lead, SH)
                return new VehiclePrior(1.20, 0.70);
            case BUS_TRUCK:
                // Heavy commercial trucks, container trucks, city buses (Hyundai, Hino, Thaco)
                return new VehiclePrior(3.20, 2.50);
            case PEDESTRIAN:
                return new VehiclePrior(1.65, 0.50);
            default:
                return new VehiclePrior(1.50, 1.75);
        }
    }
}
