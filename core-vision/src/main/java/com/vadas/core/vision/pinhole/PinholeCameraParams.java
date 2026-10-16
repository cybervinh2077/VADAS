package com.vadas.core.vision.pinhole;

/**
 * Camera intrinsic and extrinsic calibration parameters for ADAS monocular distance estimation.
 */
public final class PinholeCameraParams {

    // Intrinsic parameters (in pixels)
    private final double focalLengthX;       // fx (pixels)
    private final double focalLengthY;       // fy (pixels)
    private final double principalPointX;    // cx (image center x, pixels)
    private final double principalPointY;    // cy (image center y, pixels)
    private final int imageWidth;
    private final int imageHeight;

    // Extrinsic parameters (in vehicle coordinate frame)
    private final double cameraMountHeightMeters; // Height from ground to camera lens (m)
    private final double pitchAngleRadians;        // Tilt down angle (positive when looking slightly down)
    private final double yawAngleRadians;          // Horizontal mounting offset angle
    private final double rollAngleRadians;         // Camera roll relative to horizon

    public PinholeCameraParams(double focalLengthX,
                               double focalLengthY,
                               double principalPointX,
                               double principalPointY,
                               int imageWidth,
                               int imageHeight,
                               double cameraMountHeightMeters,
                               double pitchAngleDegrees) {
        this(focalLengthX, focalLengthY, principalPointX, principalPointY,
                imageWidth, imageHeight, cameraMountHeightMeters,
                Math.toRadians(pitchAngleDegrees), 0.0, 0.0);
    }

    public PinholeCameraParams(double focalLengthX,
                               double focalLengthY,
                               double principalPointX,
                               double principalPointY,
                               int imageWidth,
                               int imageHeight,
                               double cameraMountHeightMeters,
                               double pitchAngleRadians,
                               double yawAngleRadians,
                               double rollAngleRadians) {
        this.focalLengthX = focalLengthX;
        this.focalLengthY = focalLengthY;
        this.principalPointX = principalPointX;
        this.principalPointY = principalPointY;
        this.imageWidth = imageWidth;
        this.imageHeight = imageHeight;
        this.cameraMountHeightMeters = cameraMountHeightMeters;
        this.pitchAngleRadians = pitchAngleRadians;
        this.yawAngleRadians = yawAngleRadians;
        this.rollAngleRadians = rollAngleRadians;
    }

    /**
     * Creates typical default parameters for standard smartphone or dashcam (FOV ~ 70-80 degrees, 1080p,
     * mounted behind rearview mirror ~1.3m above ground).
     */
    public static PinholeCameraParams createDefaultDashcam1080p() {
        int width = 1920;
        int height = 1080;
        // FOV ~ 75 degrees horizontal -> fx = (W / 2) / tan(FOV_h / 2) ~ 1250 pixels
        double fx = 1250.0;
        double fy = 1250.0;
        double cx = width / 2.0;
        double cy = height / 2.0;
        double cameraHeight = 1.30; // 1.3 meters above road
        double pitchAngleDeg = 3.0; // 3 degrees slight downward tilt towards road horizon
        return new PinholeCameraParams(fx, fy, cx, cy, width, height, cameraHeight, pitchAngleDeg);
    }

    public double getFocalLengthX() {
        return focalLengthX;
    }

    public double getFocalLengthY() {
        return focalLengthY;
    }

    public double getPrincipalPointX() {
        return principalPointX;
    }

    public double getPrincipalPointY() {
        return principalPointY;
    }

    public int getImageWidth() {
        return imageWidth;
    }

    public int getImageHeight() {
        return imageHeight;
    }

    public double getCameraMountHeightMeters() {
        return cameraMountHeightMeters;
    }

    public double getPitchAngleRadians() {
        return pitchAngleRadians;
    }

    public double getYawAngleRadians() {
        return yawAngleRadians;
    }

    public double getRollAngleRadians() {
        return rollAngleRadians;
    }
}
