package com.vadas.core.vision.openpilot;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.vadas.core.vision.model.LaneLine;
import com.vadas.core.vision.model.RoadTrajectory;

/**
 * Output data contract parsed from OpenPilot's supercombo neural network.
 * Provides 4 detected lane lines (Left-Left, Left, Right, Right-Right),
 * 2D normalized screen coordinates for dynamic AR HUD rendering,
 * 3D path trajectory, lead obstacle, and camera pitch/yaw calibration telemetry.
 */
public final class SupercomboOutput {

    @Nullable
    private final LaneLine leftLane;
    @Nullable
    private final LaneLine rightLane;
    @Nullable
    private final LaneLine leftLeftLane;
    @Nullable
    private final LaneLine rightRightLane;
    @Nullable
    private final RoadTrajectory pathTrajectory;

    // Supercombo lead vehicle prediction
    private final double leadDistanceMeters;
    private final double leadRelativeSpeedMps;
    private final float leadProbability;

    // Derived alerts & camera calibration telemetry
    private final boolean laneDepartureWarningLeft;
    private final boolean laneDepartureWarningRight;
    private final float horizonYNorm;
    private final float roadCurvature;
    private final float pitchDegrees;
    private final float yawDegrees;
    private final int calibrationPercent;

    // Real-time 2D screen coordinate paths (normalized 0.0 - 1.0)
    @Nullable
    private final float[] leftLaneNormX;
    @Nullable
    private final float[] rightLaneNormX;
    @Nullable
    private final float[] adjLeftLaneNormX;
    @Nullable
    private final float[] adjRightLaneNormX;
    @Nullable
    private final float[] scanElevationNormY;

    // Arbitrary number of white road boundaries & adjacent lane lines
    @NonNull
    private final java.util.List<float[]> allWhiteLanesNormX;

    // Direct raw detected lane models from AI vision engine
    @NonNull
    private final java.util.List<com.vadas.core.vision.lane.LaneAnalyzer.Lane> detectedLanes;

    public enum LaneChangeStatus {
        KEEPING_LANE,
        CHANGING_LEFT,
        CHANGING_RIGHT
    }

    @NonNull
    private final LaneChangeStatus laneChangeStatus;
    private final float lateralOffsetMeters;

    public SupercomboOutput(@Nullable LaneLine leftLane,
                            @Nullable LaneLine rightLane,
                            @Nullable LaneLine leftLeftLane,
                            @Nullable LaneLine rightRightLane,
                            @Nullable RoadTrajectory pathTrajectory,
                            double leadDistanceMeters,
                            double leadRelativeSpeedMps,
                            float leadProbability,
                            boolean laneDepartureWarningLeft,
                            boolean laneDepartureWarningRight,
                            float horizonYNorm,
                            float roadCurvature,
                            float pitchDegrees,
                            float yawDegrees,
                            int calibrationPercent,
                            @Nullable float[] leftLaneNormX,
                            @Nullable float[] rightLaneNormX,
                            @Nullable float[] adjLeftLaneNormX,
                            @Nullable float[] adjRightLaneNormX,
                            @Nullable float[] scanElevationNormY,
                            @Nullable java.util.List<float[]> allWhiteLanesNormX,
                            @Nullable LaneChangeStatus laneChangeStatus,
                            float lateralOffsetMeters,
                            @Nullable java.util.List<com.vadas.core.vision.lane.LaneAnalyzer.Lane> detectedLanes) {
        this.leftLane = leftLane;
        this.rightLane = rightLane;
        this.leftLeftLane = leftLeftLane;
        this.rightRightLane = rightRightLane;
        this.pathTrajectory = pathTrajectory;
        this.leadDistanceMeters = leadDistanceMeters;
        this.leadRelativeSpeedMps = leadRelativeSpeedMps;
        this.leadProbability = leadProbability;
        this.laneDepartureWarningLeft = laneDepartureWarningLeft;
        this.laneDepartureWarningRight = laneDepartureWarningRight;
        this.horizonYNorm = horizonYNorm;
        this.roadCurvature = roadCurvature;
        this.pitchDegrees = pitchDegrees;
        this.yawDegrees = yawDegrees;
        this.calibrationPercent = calibrationPercent;
        this.leftLaneNormX = leftLaneNormX;
        this.rightLaneNormX = rightLaneNormX;
        this.adjLeftLaneNormX = adjLeftLaneNormX;
        this.adjRightLaneNormX = adjRightLaneNormX;
        this.scanElevationNormY = scanElevationNormY;
        this.allWhiteLanesNormX = allWhiteLanesNormX != null ? allWhiteLanesNormX : java.util.Collections.emptyList();
        this.laneChangeStatus = laneChangeStatus != null ? laneChangeStatus : LaneChangeStatus.KEEPING_LANE;
        this.lateralOffsetMeters = lateralOffsetMeters;
        this.detectedLanes = detectedLanes != null ? detectedLanes : java.util.Collections.emptyList();
    }

    public SupercomboOutput(@Nullable LaneLine leftLane,
                            @Nullable LaneLine rightLane,
                            @Nullable LaneLine leftLeftLane,
                            @Nullable LaneLine rightRightLane,
                            @Nullable RoadTrajectory pathTrajectory,
                            double leadDistanceMeters,
                            double leadRelativeSpeedMps,
                            float leadProbability,
                            boolean laneDepartureWarningLeft,
                            boolean laneDepartureWarningRight,
                            float horizonYNorm,
                            float roadCurvature,
                            float pitchDegrees,
                            float yawDegrees,
                            int calibrationPercent,
                            @Nullable float[] leftLaneNormX,
                            @Nullable float[] rightLaneNormX,
                            @Nullable float[] adjLeftLaneNormX,
                            @Nullable float[] adjRightLaneNormX,
                            @Nullable float[] scanElevationNormY,
                            @Nullable java.util.List<float[]> allWhiteLanesNormX,
                            @Nullable LaneChangeStatus laneChangeStatus,
                            float lateralOffsetMeters) {
        this(leftLane, rightLane, leftLeftLane, rightRightLane, pathTrajectory,
                leadDistanceMeters, leadRelativeSpeedMps, leadProbability,
                laneDepartureWarningLeft, laneDepartureWarningRight,
                horizonYNorm, roadCurvature, pitchDegrees, yawDegrees, calibrationPercent,
                leftLaneNormX, rightLaneNormX, adjLeftLaneNormX, adjRightLaneNormX, scanElevationNormY,
                allWhiteLanesNormX, laneChangeStatus, lateralOffsetMeters, null);
    }

    public SupercomboOutput(@Nullable LaneLine leftLane,
                            @Nullable LaneLine rightLane,
                            @Nullable LaneLine leftLeftLane,
                            @Nullable LaneLine rightRightLane,
                            @Nullable RoadTrajectory pathTrajectory,
                            double leadDistanceMeters,
                            double leadRelativeSpeedMps,
                            float leadProbability,
                            boolean laneDepartureWarningLeft,
                            boolean laneDepartureWarningRight,
                            float horizonYNorm,
                            float roadCurvature,
                            float pitchDegrees,
                            float yawDegrees,
                            int calibrationPercent,
                            @Nullable float[] leftLaneNormX,
                            @Nullable float[] rightLaneNormX,
                            @Nullable float[] adjLeftLaneNormX,
                            @Nullable float[] adjRightLaneNormX,
                            @Nullable float[] scanElevationNormY) {
        this(leftLane, rightLane, leftLeftLane, rightRightLane, pathTrajectory,
                leadDistanceMeters, leadRelativeSpeedMps, leadProbability,
                laneDepartureWarningLeft, laneDepartureWarningRight,
                horizonYNorm, roadCurvature, pitchDegrees, yawDegrees, calibrationPercent,
                leftLaneNormX, rightLaneNormX, adjLeftLaneNormX, adjRightLaneNormX, scanElevationNormY,
                null, LaneChangeStatus.KEEPING_LANE, 0.0f);
    }

    public SupercomboOutput(@Nullable LaneLine leftLane,
                            @Nullable LaneLine rightLane,
                            @Nullable LaneLine leftLeftLane,
                            @Nullable LaneLine rightRightLane,
                            @Nullable RoadTrajectory pathTrajectory,
                            double leadDistanceMeters,
                            double leadRelativeSpeedMps,
                            float leadProbability,
                            boolean laneDepartureWarningLeft,
                            boolean laneDepartureWarningRight,
                            float horizonYNorm,
                            float roadCurvature,
                            float pitchDegrees,
                            float yawDegrees,
                            int calibrationPercent) {
        this(leftLane, rightLane, leftLeftLane, rightRightLane, pathTrajectory,
                leadDistanceMeters, leadRelativeSpeedMps, leadProbability,
                laneDepartureWarningLeft, laneDepartureWarningRight,
                horizonYNorm, roadCurvature, pitchDegrees, yawDegrees, calibrationPercent,
                null, null, null, null, null);
    }

    public SupercomboOutput(@Nullable LaneLine leftLane,
                            @Nullable LaneLine rightLane,
                            @Nullable RoadTrajectory pathTrajectory,
                            double leadDistanceMeters,
                            double leadRelativeSpeedMps,
                            float leadProbability,
                            boolean laneDepartureWarningLeft,
                            boolean laneDepartureWarningRight) {
        this(leftLane, rightLane, null, null, pathTrajectory,
                leadDistanceMeters, leadRelativeSpeedMps, leadProbability,
                laneDepartureWarningLeft, laneDepartureWarningRight,
                0.46f, 0.0f, -6.47f, -2.17f, 86);
    }

    @Nullable
    public LaneLine getLeftLane() {
        return leftLane;
    }

    @Nullable
    public LaneLine getRightLane() {
        return rightLane;
    }

    @Nullable
    public LaneLine getLeftLeftLane() {
        return leftLeftLane;
    }

    @Nullable
    public LaneLine getRightRightLane() {
        return rightRightLane;
    }

    @Nullable
    public RoadTrajectory getPathTrajectory() {
        return pathTrajectory;
    }

    public double getLeadDistanceMeters() {
        return leadDistanceMeters;
    }

    public double getLeadRelativeSpeedMps() {
        return leadRelativeSpeedMps;
    }

    public float getLeadProbability() {
        return leadProbability;
    }

    public boolean isLaneDepartureWarningLeft() {
        return laneDepartureWarningLeft;
    }

    public boolean isLaneDepartureWarningRight() {
        return laneDepartureWarningRight;
    }

    public boolean hasLaneDeparture() {
        return laneDepartureWarningLeft || laneDepartureWarningRight;
    }

    public float getHorizonYNorm() {
        return horizonYNorm;
    }

    public float getRoadCurvature() {
        return roadCurvature;
    }

    public float getPitchDegrees() {
        return pitchDegrees;
    }

    public float getYawDegrees() {
        return yawDegrees;
    }

    public int getCalibrationPercent() {
        return calibrationPercent;
    }

    @Nullable
    public float[] getLeftLaneNormX() {
        return leftLaneNormX;
    }

    @Nullable
    public float[] getRightLaneNormX() {
        return rightLaneNormX;
    }

    @Nullable
    public float[] getAdjLeftLaneNormX() {
        return adjLeftLaneNormX;
    }

    @Nullable
    public float[] getAdjRightLaneNormX() {
        return adjRightLaneNormX;
    }

    @Nullable
    public float[] getScanElevationNormY() {
        return scanElevationNormY;
    }

    /**
     * Estimated lane width between left and right detected lines.
     */
    public double getEstimatedLaneWidth() {
        if (leftLane != null && rightLane != null) {
            return Math.abs(rightLane.getBumperLateralDistance() - leftLane.getBumperLateralDistance());
        }
        return 3.5; // Standard 3.5m default
    }

    @NonNull
    public java.util.List<float[]> getAllWhiteLanesNormX() {
        return allWhiteLanesNormX;
    }

    @NonNull
    public LaneChangeStatus getLaneChangeStatus() {
        return laneChangeStatus;
    }

    public float getLateralOffsetMeters() {
        return lateralOffsetMeters;
    }

    @NonNull
    public java.util.List<com.vadas.core.vision.lane.LaneAnalyzer.Lane> getDetectedLanes() {
        return detectedLanes;
    }
}
