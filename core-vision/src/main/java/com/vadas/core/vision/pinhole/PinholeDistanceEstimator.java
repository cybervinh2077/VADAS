package com.vadas.core.vision.pinhole;

import androidx.annotation.NonNull;

import com.vadas.core.vision.model.DetectedBox;

/**
 * High-precision monocular distance and 3D vehicle position estimator using the Pinhole Camera Model
 * combined with flat road ground-plane projection and vehicle dimensional priors.
 *
 * Provides two complementary geometric estimation pathways:
 * 1. Road Contact Point (Ground Plane Projection):
 *    Z_ground = H_cam / tan(theta_pitch + arctan((y_bottom - cy) / fy))
 * 2. Apparent Dimension Ratio (Pinhole Projection):
 *    Z_height = (fy * H_real) / h_pixels
 *    Z_width  = (fx * W_real) / w_pixels
 *
 * Also computes lateral displacement (X meters) to filter vehicles strictly within ego-lane.
 */
public final class PinholeDistanceEstimator {

    private final PinholeCameraParams params;

    public static final class SpatialVehicleEstimate {
        private final double distanceZ;     // Longitudinal distance ahead (meters)
        private final double lateralX;      // Lateral offset relative to camera optical axis (meters)
        private final double groundDistanceZ; // Distance from road contact point
        private final double apparentDistanceZ; // Distance from bounding box height
        private final boolean inEgoLane;    // Whether vehicle is inside ego-lane bounds (~[-1.6m, +1.6m])

        public SpatialVehicleEstimate(double distanceZ,
                                      double lateralX,
                                      double groundDistanceZ,
                                      double apparentDistanceZ,
                                      boolean inEgoLane) {
            this.distanceZ = distanceZ;
            this.lateralX = lateralX;
            this.groundDistanceZ = groundDistanceZ;
            this.apparentDistanceZ = apparentDistanceZ;
            this.inEgoLane = inEgoLane;
        }

        public double getDistanceZ() {
            return distanceZ;
        }

        public double getLateralX() {
            return lateralX;
        }

        public double getGroundDistanceZ() {
            return groundDistanceZ;
        }

        public double getApparentDistanceZ() {
            return apparentDistanceZ;
        }

        public boolean isInEgoLane() {
            return inEgoLane;
        }

        @Override
        public String toString() {
            return String.format("Z=%.1fm, X=%.2fm (%s)",
                    distanceZ, lateralX, inEgoLane ? "Ego Lane" : "Neighboring Lane");
        }
    }

    public PinholeDistanceEstimator(@NonNull PinholeCameraParams params) {
        this.params = params;
    }

    /**
     * Estimates 3D spatial position (Z distance ahead, X lateral offset) from a 2D bounding box.
     *
     * @param box detected bounding box with normalized or absolute pixel coordinates.
     * @return {@link SpatialVehicleEstimate} containing fused distance and lane position.
     */
    @NonNull
    public SpatialVehicleEstimate estimateDistance(@NonNull DetectedBox box) {
        // Convert normalized coordinates [0.0 - 1.0] to absolute image pixels
        double leftPx = box.getLeft() * params.getImageWidth();
        double topPx = box.getTop() * params.getImageHeight();
        double rightPx = box.getRight() * params.getImageWidth();
        double bottomPx = box.getBottom() * params.getImageHeight();

        double boxWidthPx = Math.max(2.0, rightPx - leftPx);
        double boxHeightPx = Math.max(2.0, bottomPx - topPx);
        double boxCenterX = (leftPx + rightPx) / 2.0;

        VehiclePrior prior = VehiclePrior.getPriorForClass(box.getObjectClass());

        // 1. Apparent Height & Width Estimation
        double zHeight = (params.getFocalLengthY() * prior.getHeightMeters()) / boxHeightPx;
        double zWidth = (params.getFocalLengthX() * prior.getWidthMeters()) / boxWidthPx;
        // Prioritize height since aspect ratios vary less along height for common cars
        double zApparent = (0.75 * zHeight) + (0.25 * zWidth);

        // 2. Ground Plane Projection from Road Contact Point (y_bottom)
        double deltaY = bottomPx - params.getPrincipalPointY();
        double alphaAngle = Math.atan(deltaY / params.getFocalLengthY());
        double totalPitch = params.getPitchAngleRadians() + alphaAngle;

        double zGround;
        if (totalPitch > 0.01) { // Ensure ground contact point is strictly below camera horizon
            zGround = params.getCameraMountHeightMeters() / Math.tan(totalPitch);
            // Clamp against unrealistic values
            if (zGround < 1.0 || zGround > 150.0) {
                zGround = zApparent;
            }
        } else {
            // Above or at horizon: fallback to apparent dimension
            zGround = zApparent;
        }

        // 3. Adaptive Distance Fusion
        // Close range (< 25m): road contact point is very reliable (sharp tire contact)
        // Long range (> 40m): contact point sensitive to horizon pitch noise; blend towards apparent height
        double fusedZ;
        if (zGround < 25.0) {
            fusedZ = (0.70 * zGround) + (0.30 * zApparent);
        } else if (zGround < 50.0) {
            fusedZ = (0.50 * zGround) + (0.50 * zApparent);
        } else {
            fusedZ = (0.30 * zGround) + (0.70 * zApparent);
        }

        // 4. Lateral Offset (X position in meters)
        // X = Z * (x_center - cx) / fx
        double lateralX = fusedZ * (boxCenterX - params.getPrincipalPointX()) / params.getFocalLengthX();

        // Standard Vietnamese lane width is approximately 3.2m - 3.75m.
        // A vehicle within lateral offset +/- 1.35m is considered directly in the ego lane.
        boolean inEgoLane = Math.abs(lateralX) <= 1.35;

        return new SpatialVehicleEstimate(fusedZ, lateralX, zGround, zApparent, inEgoLane);
    }
}
