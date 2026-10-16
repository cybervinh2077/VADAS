package com.vadas.core.vision.openpilot;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.PointF;
import android.os.SystemClock;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.vadas.core.camera.model.CameraFrame;
import com.vadas.core.vision.VisionEngine;
import com.vadas.core.vision.horizon.DynamicHorizonEstimator;
import com.vadas.core.vision.model.DetectedBox;
import com.vadas.core.vision.model.VisionPerceptionResult;
import com.vadas.core.vision.pinhole.PinholeCameraParams;
import com.vadas.core.vision.pinhole.PinholeDistanceEstimator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Perception Engine combining:
 * 1. OpenPilot supercombo perspective road & lane tracking (Single Parametric Perspective Road Model).
 * 2. Real-time vision-based vehicle detection analyzing camera frames (tire shadows + vehicle body contrast).
 * 3. Monocular Pinhole Camera Model for distance and lateral lane position estimation.
 * 4. Dynamic Horizon Line & Camera Pitch Estimator with dual-cue vision fusion.
 * 5. Robust driver steering & vehicle dodging compensation: maintains stable lane tracking during maneuvers.
 */
public class SupercomboVisionEngine implements VisionEngine {

    private static final String TAG = "SupercomboVisionEngine";

    private final PinholeDistanceEstimator distanceEstimator;
    private final OpenPilotSupercomboParser supercomboParser;
    private final DynamicHorizonEstimator horizonEstimator;

    // Latest parsed lane state
    private SupercomboOutput latestSupercomboOutput;
    private float latestHorizonYNorm = 0.440f;
    private float latestPitchDegrees = 2.98f;

    // Reusable buffers
    private final com.vadas.core.vision.vehicle.VehicleDetector vehicleDetector = new com.vadas.core.vision.vehicle.VehicleDetector();

    public SupercomboVisionEngine() {
        this(PinholeCameraParams.createDefaultDashcam1080p());
    }

    public SupercomboVisionEngine(@NonNull PinholeCameraParams cameraParams) {
        this.distanceEstimator = new PinholeDistanceEstimator(cameraParams);
        this.supercomboParser = new OpenPilotSupercomboParser();
        this.horizonEstimator = new DynamicHorizonEstimator();
        this.latestSupercomboOutput = supercomboParser.createDefaultFallbackOutput();
    }

    @Override
    public void initialize(@NonNull Context context) {
        Log.i(TAG, "Initializing Supercombo Vision Engine with Dynamic Horizon & Pinhole Model");
        vehicleDetector.initialize(context);
    }

    @NonNull
    public com.vadas.core.vision.vehicle.VehicleDetector getVehicleDetector() {
        return vehicleDetector;
    }

    @Override
    public void close() {
        laneModelExecutor.shutdown();
        // vehicleDetector released
    }

    @NonNull
    @Override
    public VisionPerceptionResult processFrame(@NonNull CameraFrame frame) {
        // 1. Detect actual vehicles from frame bitmap FIRST so lane processor has vehicle occlusion context
        List<DetectedBox> detectedObjects = detectVehiclesFromFrame(frame);

        // 2. Process OpenPilot lane detection with dynamic horizon and vehicle occlusion filtering
        SupercomboOutput supercomboResult = processLanes(frame, detectedObjects);
        this.latestSupercomboOutput = supercomboResult;

        // 3. Apply Pinhole Camera Model to calculate distances & find primary lead vehicle in ego-lane
        DetectedBox primaryLead = null;
        double minLeadDistance = Double.POSITIVE_INFINITY;

        List<DetectedBox> positionedObjects = new ArrayList<>(detectedObjects.size());
        for (DetectedBox rawBox : detectedObjects) {
            PinholeDistanceEstimator.SpatialVehicleEstimate estimate = distanceEstimator.estimateDistance(rawBox);

            DetectedBox calculatedBox = new DetectedBox(
                    rawBox.getLeft(),
                    rawBox.getTop(),
                    rawBox.getRight(),
                    rawBox.getBottom(),
                    rawBox.getConfidence(),
                    rawBox.getObjectClass(),
                    estimate.getDistanceZ()
            );
            positionedObjects.add(calculatedBox);

            // Filter for closest vehicle inside ego-lane bounds (|X| <= 1.4m)
            if (estimate.isInEgoLane() && estimate.getDistanceZ() < minLeadDistance) {
                minLeadDistance = estimate.getDistanceZ();
                primaryLead = calculatedBox;
            }
        }

        boolean laneDeparture = supercomboResult.hasLaneDeparture();
        boolean redLightRisk = false;

        return new VisionPerceptionResult(
                positionedObjects,
                primaryLead,
                laneDeparture,
                redLightRisk,
                frame.getTimestampNs()
        );
    }

    private List<DetectedBox> detectVehiclesFromFrame(@NonNull CameraFrame frame) {
        Bitmap bitmap = frame.getBitmap();
        if (bitmap == null || bitmap.isRecycled()) {
            return Collections.emptyList();
        }
        vehicleDetector.setHorizonYNorm(latestHorizonYNorm);
        return vehicleDetector.detectVehicles(bitmap);
    }

    @NonNull
    public SupercomboOutput getLatestSupercomboOutput() {
        return latestSupercomboOutput;
    }

    @NonNull
    public PinholeDistanceEstimator getDistanceEstimator() {
        return distanceEstimator;
    }

    @Nullable
    private com.vadas.core.vision.lane.LaneAnalyzer laneAnalyzer = null;

    // Asynchronous AI Lane Model Worker
    private final ExecutorService laneModelExecutor = Executors.newSingleThreadExecutor(r -> {
        Thread t = new Thread(r, "VADAS-LaneModelWorker");
        t.setPriority(Thread.NORM_PRIORITY);
        return t;
    });
    private final AtomicBoolean isLaneModelRunning = new AtomicBoolean(false);
    private volatile List<com.vadas.core.vision.lane.LaneAnalyzer.Lane> latestAiLanes = Collections.emptyList();

    public void setLaneAnalyzer(@Nullable com.vadas.core.vision.lane.LaneAnalyzer laneAnalyzer) {
        this.laneAnalyzer = laneAnalyzer;
        this.latestAiLanes = Collections.emptyList();
    }

    // Dynamic 9 elevation slices adapting to real-time horizon
    public static final int NUM_SLICES = 9;
    public static final float HOOD_Y = 0.960f;
    public static final float[] SCAN_ELEVATIONS_NORM_Y = new float[]{
            0.960f, 0.895f, 0.830f, 0.765f, 0.700f, 0.635f, 0.570f, 0.505f, 0.440f
    };

    private final float[] dynamicScanElevations = new float[]{
            0.960f, 0.895f, 0.830f, 0.765f, 0.700f, 0.635f, 0.570f, 0.505f, 0.440f
    };

    private final float[] currentNominalLeftX = new float[]{
            0.200f, 0.235f, 0.270f, 0.305f, 0.340f, 0.375f, 0.410f, 0.445f, 0.480f
    };
    private final float[] currentNominalRightX = new float[]{
            0.800f, 0.765f, 0.730f, 0.695f, 0.660f, 0.625f, 0.590f, 0.555f, 0.520f
    };

    // Stateful ego lane tracking with hysteresis memory (handles driver steering & vehicle dodging)
    private float trackedEgoLeftBaseX = 0.200f;
    private float trackedEgoRightBaseX = 0.800f;
    private boolean hasTrackedEgoHistory = false;

    // Dynamic lane tracking state with temporal smoothing (9 slices)
    private final float[] smoothedLeftX = new float[]{
            0.200f, 0.235f, 0.270f, 0.305f, 0.340f, 0.375f, 0.410f, 0.445f, 0.480f
    };
    private final float[] smoothedRightX = new float[]{
            0.800f, 0.765f, 0.730f, 0.695f, 0.660f, 0.625f, 0.590f, 0.555f, 0.520f
    };
    private final float[] smoothedAdjLeftX = new float[]{
            -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f
    };
    private final float[] smoothedAdjRightX = new float[]{
            -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f
    };
    private long lastAiModelRunMs = 0;

    private float targetCurvature = 0.0f;
    private float targetDrift = 0.0f;
    private float smoothedCurvature = 0.0f;
    private float smoothedDrift = 0.0f;

    private SupercomboOutput processLanes(@NonNull CameraFrame frame) {
        return processLanes(frame, Collections.emptyList());
    }

    private SupercomboOutput processLanes(@NonNull CameraFrame frame, @NonNull List<DetectedBox> detectedVehicles) {
        // 0. Real-time vision-based Horizon Line & Camera Pitch Estimation
        DynamicHorizonEstimator.HorizonEstimate horizonEst = horizonEstimator.estimateHorizon(
                frame, smoothedLeftX, smoothedRightX, dynamicScanElevations
        );
        latestHorizonYNorm = horizonEst.getHorizonYNorm();
        latestPitchDegrees = horizonEst.getPitchDegrees();
        float vpX = horizonEst.getVanishingPointX();

        // Dynamically compute the NUM_SLICES elevation slices from hood (0.960) up to near horizon
        float yHood = HOOD_Y;
        float yTop = Math.max(latestHorizonYNorm + 0.025f, 0.420f);
        for (int i = 0; i < NUM_SLICES; i++) {
            dynamicScanElevations[i] = yHood - (i / (float) (NUM_SLICES - 1)) * (yHood - yTop);
        }

        Bitmap bitmap = frame.getBitmap();

        // 1. AI Lane Model Execution & Spatial Association
        boolean modelDetected = false;
        List<float[]> allWhiteLanesList = new ArrayList<>();
        float cameraCenterVpX = 0.50f;
        float horizonY = latestHorizonYNorm > 0 ? latestHorizonYNorm : 0.440f;

        if (laneAnalyzer != null) {
            try {
                if (latestAiLanes.isEmpty()) {
                    List<com.vadas.core.vision.lane.LaneAnalyzer.Lane> initialLanes = laneAnalyzer.analyzeBitmap(bitmap);
                    if (initialLanes != null && !initialLanes.isEmpty()) {
                        latestAiLanes = initialLanes;
                    }
                } else if (bitmap != null && !bitmap.isRecycled() && (SystemClock.elapsedRealtime() - lastAiModelRunMs >= 160) && isLaneModelRunning.compareAndSet(false, true)) {
                    lastAiModelRunMs = SystemClock.elapsedRealtime();
                    final Bitmap frameCopy = bitmap.copy(Bitmap.Config.ARGB_8888, false);
                    if (frameCopy != null) {
                        laneModelExecutor.execute(() -> {
                            try {
                                List<com.vadas.core.vision.lane.LaneAnalyzer.Lane> lanes = laneAnalyzer.analyzeBitmap(frameCopy);
                                if (lanes != null && !lanes.isEmpty()) {
                                    latestAiLanes = lanes;
                                }
                            } catch (Exception e) {
                                Log.w(TAG, "Async lane model error: " + e.getMessage());
                            } finally {
                                if (!frameCopy.isRecycled()) {
                                    frameCopy.recycle();
                                }
                                isLaneModelRunning.set(false);
                            }
                        });
                    } else {
                        isLaneModelRunning.set(false);
                    }
                }

                List<com.vadas.core.vision.lane.LaneAnalyzer.Lane> aiLanes = latestAiLanes;
                if (aiLanes != null && !aiLanes.isEmpty()) {
                    EgoLaneSelection selection = selectEgoAndAdjacentLanes(aiLanes, yHood, detectedVehicles);
                    com.vadas.core.vision.lane.LaneAnalyzer.Lane egoLeft = selection.egoLeft;
                    com.vadas.core.vision.lane.LaneAnalyzer.Lane egoRight = selection.egoRight;
                    List<com.vadas.core.vision.lane.LaneAnalyzer.Lane> adjacentLanes = selection.allAdjacentLanes;

                    // Extract base positions at hood
                    float detectedBaseL = -1f;
                    float detectedBaseR = -1f;
                    if (egoLeft != null && egoLeft.points.size() >= 3) {
                        detectedBaseL = interpolateXFromPoints(egoLeft.points, yHood, cameraCenterVpX, horizonY);
                    }
                    if (egoRight != null && egoRight.points.size() >= 3) {
                        detectedBaseR = interpolateXFromPoints(egoRight.points, yHood, cameraCenterVpX, horizonY);
                    }

                    final float NOMINAL_LANE_WIDTH = hasTrackedEgoHistory ?
                            Math.max(0.24f, Math.min(0.90f, trackedEgoRightBaseX - trackedEgoLeftBaseX)) : 0.600f;
                    float rawBaseL;
                    float rawBaseR;

                    if (detectedBaseL > -0.30f && detectedBaseR > 0.20f) {
                        float measuredWidth = detectedBaseR - detectedBaseL;
                        if (measuredWidth >= 0.15f && measuredWidth <= 1.10f) {
                            rawBaseL = detectedBaseL;
                            rawBaseR = detectedBaseR;
                        } else {
                            float distL = Math.abs(detectedBaseL - trackedEgoLeftBaseX);
                            float distR = Math.abs(detectedBaseR - trackedEgoRightBaseX);
                            if (distL <= distR) {
                                rawBaseL = detectedBaseL;
                                rawBaseR = rawBaseL + NOMINAL_LANE_WIDTH;
                            } else {
                                rawBaseR = detectedBaseR;
                                rawBaseL = rawBaseR - NOMINAL_LANE_WIDTH;
                            }
                        }
                    } else if (detectedBaseL > -0.30f) {
                        rawBaseL = detectedBaseL;
                        rawBaseR = rawBaseL + NOMINAL_LANE_WIDTH;
                    } else if (detectedBaseR > 0.20f) {
                        rawBaseR = detectedBaseR;
                        rawBaseL = rawBaseR - NOMINAL_LANE_WIDTH;
                    } else {
                        rawBaseL = trackedEgoLeftBaseX;
                        rawBaseR = trackedEgoRightBaseX;
                    }

                    // Smooth base positions (EMA): allows driver lateral movement while keeping continuity
                    float baseAlpha = 0.22f;
                    trackedEgoLeftBaseX += baseAlpha * (rawBaseL - trackedEgoLeftBaseX);
                    trackedEgoRightBaseX += baseAlpha * (rawBaseR - trackedEgoRightBaseX);
                    trackedEgoLeftBaseX = Math.max(-0.35f, Math.min(0.60f, trackedEgoLeftBaseX));
                    trackedEgoRightBaseX = Math.max(0.35f, Math.min(1.35f, trackedEgoRightBaseX));

                    // Cross-Validated Curvature: both left and right must agree on curve direction
                    float curvL = (egoLeft != null && egoLeft.points.size() >= 3) ? extractSagittaCurvature(egoLeft.points) : Float.NaN;
                    float curvR = (egoRight != null && egoRight.points.size() >= 3) ? extractSagittaCurvature(egoRight.points) : Float.NaN;
                    float verifiedCurvature = 0.0f;

                    if (!Float.isNaN(curvL) && !Float.isNaN(curvR)) {
                        if (curvL * curvR > 0) {
                            verifiedCurvature = (curvL + curvR) / 2.0f;
                        } else {
                            verifiedCurvature = Math.abs(curvL) > Math.abs(curvR) ? curvL : curvR;
                        }
                    } else if (!Float.isNaN(curvL)) {
                        verifiedCurvature = curvL;
                    } else if (!Float.isNaN(curvR)) {
                        verifiedCurvature = curvR;
                    }

                    if (Math.abs(verifiedCurvature) < 0.012f) {
                        verifiedCurvature = 0.0f;
                    }
                    targetCurvature = verifiedCurvature;
                    modelDetected = true;

                    // Extract actual adjacent lanes (that do not collide with vehicles)
                    for (com.vadas.core.vision.lane.LaneAnalyzer.Lane adjLane : adjacentLanes) {
                        if (adjLane.points == null || adjLane.points.size() < 3 || adjLane.confidence < 0.60f) continue;
                        float[] whiteSlice = new float[NUM_SLICES];
                        for (int i = 0; i < NUM_SLICES; i++) {
                            whiteSlice[i] = interpolateXFromPoints(adjLane.points, dynamicScanElevations[i], cameraCenterVpX, horizonY);
                        }
                        if (whiteSlice[0] < trackedEgoLeftBaseX - 0.05f || whiteSlice[0] > trackedEgoRightBaseX + 0.05f) {
                            allWhiteLanesList.add(whiteSlice);
                        }
                    }

                    Collections.sort(allWhiteLanesList, (a, b) -> Float.compare(a[0], b[0]));

                    // Update smoothedAdjLeftX and smoothedAdjRightX ONLY if real non-occluded line exists
                    boolean foundAdjL = false;
                    for (float[] ws : allWhiteLanesList) {
                        if (ws[0] < trackedEgoLeftBaseX - 0.05f) {
                            for (int i = 0; i < NUM_SLICES; i++) {
                                if (smoothedAdjLeftX[i] < 0) smoothedAdjLeftX[i] = ws[i];
                                else smoothedAdjLeftX[i] += 0.30f * (ws[i] - smoothedAdjLeftX[i]);
                            }
                            foundAdjL = true;
                            break;
                        }
                    }
                    if (!foundAdjL) {
                        Arrays.fill(smoothedAdjLeftX, -1.0f);
                    }

                    boolean foundAdjR = false;
                    for (float[] ws : allWhiteLanesList) {
                        if (ws[0] > trackedEgoRightBaseX + 0.05f) {
                            for (int i = 0; i < NUM_SLICES; i++) {
                                if (smoothedAdjRightX[i] < 0) smoothedAdjRightX[i] = ws[i];
                                else smoothedAdjRightX[i] += 0.30f * (ws[i] - smoothedAdjRightX[i]);
                            }
                            foundAdjR = true;
                            break;
                        }
                    }
                    if (!foundAdjR) {
                        Arrays.fill(smoothedAdjRightX, -1.0f);
                    }

                    float rawDrift = ((trackedEgoLeftBaseX + trackedEgoRightBaseX) / 2.0f) - 0.50f;
                    if (Math.abs(rawDrift) < 0.035f) {
                        rawDrift = 0.0f;
                    }
                    targetDrift = Math.max(-0.25f, Math.min(0.25f, rawDrift));
                }
            } catch (Exception e) {
                Log.w(TAG, "Lane model execution note: " + e.getMessage());
            }
        }

        // 2. Curvature & Drift temporal smoothing
        float diffC = targetCurvature - smoothedCurvature;
        smoothedCurvature += diffC * 0.20f;
        if (Math.abs(smoothedCurvature) < 0.008f) {
            smoothedCurvature = 0.0f;
        }
        smoothedCurvature = Math.max(-0.25f, Math.min(0.25f, smoothedCurvature));

        float diffD = targetDrift - smoothedDrift;
        smoothedDrift += diffD * 0.18f;
        smoothedDrift = Math.max(-0.25f, Math.min(0.25f, smoothedDrift));

        // 3. PROJECTION CÁC ĐƯỜNG LÀN XE CHỦ TỪ DỮ LIỆU THỰC TẾ CỦA MÔ HÌNH THỊ GIÁC
        if (modelDetected && latestAiLanes != null && !latestAiLanes.isEmpty()) {
            EgoLaneSelection activeSelection = selectEgoAndAdjacentLanes(latestAiLanes, yHood, detectedVehicles);
            com.vadas.core.vision.lane.LaneAnalyzer.Lane egoLeft = activeSelection.egoLeft;
            com.vadas.core.vision.lane.LaneAnalyzer.Lane egoRight = activeSelection.egoRight;
            float trackedW = Math.max(0.20f, trackedEgoRightBaseX - trackedEgoLeftBaseX);

            for (int i = 0; i < NUM_SLICES; i++) {
                float y = dynamicScanElevations[i];
                float tau = (y - horizonY) / (yHood - horizonY);
                tau = Math.max(0.05f, Math.min(1.0f, tau));

                float targetL;
                float targetR;

                if (egoLeft != null && egoRight != null) {
                    targetL = interpolateXFromPoints(egoLeft.points, y, vpX, horizonY);
                    targetR = interpolateXFromPoints(egoRight.points, y, vpX, horizonY);
                    if (targetR <= targetL + 0.020f) {
                        float center = (targetL + targetR) / 2.0f;
                        targetL = center - 0.010f;
                        targetR = center + 0.010f;
                    }
                } else if (egoLeft != null) {
                    targetL = interpolateXFromPoints(egoLeft.points, y, vpX, horizonY);
                    targetR = targetL + (trackedW * tau);
                } else if (egoRight != null) {
                    targetR = interpolateXFromPoints(egoRight.points, y, vpX, horizonY);
                    targetL = targetR - (trackedW * tau);
                } else {
                    float laneWidthHood = Math.max(0.24f, Math.min(0.70f, trackedEgoRightBaseX - trackedEgoLeftBaseX));
                    float laneCenterHood = (trackedEgoLeftBaseX + trackedEgoRightBaseX) / 2.0f;
                    float curveVpX = cameraCenterVpX + (smoothedCurvature * 2.8f);
                    float widthAtSlice = laneWidthHood * tau;
                    float centerAtSlice = curveVpX + (laneCenterHood - curveVpX) * tau + (smoothedCurvature * tau * (1.0f - tau) * 0.7f);
                    targetL = centerAtSlice - (widthAtSlice / 2.0f);
                    targetR = centerAtSlice + (widthAtSlice / 2.0f);
                }

                smoothedLeftX[i] += 0.35f * (targetL - smoothedLeftX[i]);
                smoothedRightX[i] += 0.35f * (targetR - smoothedRightX[i]);
            }
        } else {
            // Fallback khi không phát hiện vạch: Mô hình phối cảnh tham số chuẩn
            float laneWidthHood = Math.max(0.24f, Math.min(0.70f, trackedEgoRightBaseX - trackedEgoLeftBaseX));
            float laneCenterHood = (trackedEgoLeftBaseX + trackedEgoRightBaseX) / 2.0f;
            float curveVpX = cameraCenterVpX + (smoothedCurvature * 2.8f);

            for (int i = 0; i < NUM_SLICES; i++) {
                float y = dynamicScanElevations[i];
                float tau = (y - horizonY) / (yHood - horizonY);
                tau = Math.max(0.05f, Math.min(1.0f, tau));

                float widthAtSlice = laneWidthHood * tau;
                float centerAtSlice = curveVpX + (laneCenterHood - curveVpX) * tau + (smoothedCurvature * tau * (1.0f - tau) * 0.7f);

                float targetL = centerAtSlice - (widthAtSlice / 2.0f);
                float targetR = centerAtSlice + (widthAtSlice / 2.0f);

                smoothedLeftX[i] += 0.35f * (targetL - smoothedLeftX[i]);
                smoothedRightX[i] += 0.35f * (targetR - smoothedRightX[i]);
            }
        }

        if (!modelDetected) {
            Arrays.fill(smoothedAdjLeftX, -1.0f);
            Arrays.fill(smoothedAdjRightX, -1.0f);
        }

        // OpenPilot metric world coordinates
        double[] xPoints = OpenPilotSupercomboParser.DEFAULT_X_POINTS;
        double[] leftY = new double[xPoints.length];
        double[] rightY = new double[xPoints.length];
        double[] leftLeftY = new double[xPoints.length];
        double[] rightRightY = new double[xPoints.length];
        double[] pathY = new double[xPoints.length];
        double[] pathZ = new double[xPoints.length];

        for (int i = 0; i < xPoints.length; i++) {
            double dist = xPoints[i];
            double curveOffset = 0.5 * smoothedCurvature * (dist * dist / 50.0);
            double driftOffset = smoothedDrift * 1.2;

            leftY[i] = -1.75 + driftOffset + curveOffset;
            rightY[i] = 1.75 + driftOffset + curveOffset;
            leftLeftY[i] = -5.25 + driftOffset + curveOffset;
            rightRightY[i] = 5.25 + driftOffset + curveOffset;
            pathY[i] = driftOffset + curveOffset;
            pathZ[i] = 0.0;
        }

        boolean ldwLeft = smoothedDrift < -0.45f;
        boolean ldwRight = smoothedDrift > 0.45f;

        com.vadas.core.vision.model.LaneLine leftLane = new com.vadas.core.vision.model.LaneLine(xPoints, leftY, 0.96f, ldwLeft);
        com.vadas.core.vision.model.LaneLine rightLane = new com.vadas.core.vision.model.LaneLine(xPoints, rightY, 0.96f, ldwRight);
        com.vadas.core.vision.model.LaneLine leftLeftLane = new com.vadas.core.vision.model.LaneLine(xPoints, leftLeftY, 0.88f, false);
        com.vadas.core.vision.model.LaneLine rightRightLane = new com.vadas.core.vision.model.LaneLine(xPoints, rightRightY, 0.88f, false);
        com.vadas.core.vision.model.RoadTrajectory trajectory = new com.vadas.core.vision.model.RoadTrajectory(xPoints, pathY, pathZ, smoothedCurvature);

        float pitch = latestPitchDegrees;
        float yaw = -2.17f + (smoothedCurvature * 30.0f) + (smoothedDrift * 2.0f);

        // Determine Lane Change Status and Lateral Offset
        float egoCenter = 0.50f;
        float xL = smoothedLeftX[0];
        float xR = smoothedRightX[0];
        float laneW = Math.max(0.18f, xR - xL);
        float laneCenter = (xL + xR) / 2.0f;
        float lateralOffsetMeters = (egoCenter - laneCenter) * (3.5f / laneW);

        SupercomboOutput.LaneChangeStatus lcStatus;
        if (egoCenter - xL < 0.08f || lateralOffsetMeters <= -0.65f) {
            lcStatus = SupercomboOutput.LaneChangeStatus.CHANGING_LEFT;
        } else if (xR - egoCenter < 0.08f || lateralOffsetMeters >= 0.65f) {
            lcStatus = SupercomboOutput.LaneChangeStatus.CHANGING_RIGHT;
        } else {
            lcStatus = SupercomboOutput.LaneChangeStatus.KEEPING_LANE;
        }

        boolean hasValidAdjL = smoothedAdjLeftX[0] > 0.01f;
        boolean hasValidAdjR = smoothedAdjRightX[0] > 0.01f;

        return new SupercomboOutput(
                leftLane,
                rightLane,
                leftLeftLane,
                rightRightLane,
                trajectory,
                58.5,
                -1.2,
                0.91f,
                ldwLeft,
                ldwRight,
                latestHorizonYNorm,
                smoothedCurvature,
                pitch,
                yaw,
                86,
                smoothedLeftX.clone(),
                smoothedRightX.clone(),
                smoothedAdjLeftX.clone(),
                smoothedAdjRightX.clone(),
                dynamicScanElevations.clone(),
                allWhiteLanesList,
                lcStatus,
                lateralOffsetMeters,
                latestAiLanes != null ? new ArrayList<>(latestAiLanes) : Collections.emptyList()
        );
    }

    @NonNull
    public List<com.vadas.core.vision.lane.LaneAnalyzer.Lane> getLatestDetectedLanes() {
        return latestAiLanes != null ? new ArrayList<>(latestAiLanes) : Collections.emptyList();
    }

    private static class EgoLaneSelection {
        final com.vadas.core.vision.lane.LaneAnalyzer.Lane egoLeft;
        final com.vadas.core.vision.lane.LaneAnalyzer.Lane egoRight;
        final List<com.vadas.core.vision.lane.LaneAnalyzer.Lane> allAdjacentLanes;

        EgoLaneSelection(com.vadas.core.vision.lane.LaneAnalyzer.Lane egoLeft,
                         com.vadas.core.vision.lane.LaneAnalyzer.Lane egoRight,
                         List<com.vadas.core.vision.lane.LaneAnalyzer.Lane> allAdjacentLanes) {
            this.egoLeft = egoLeft;
            this.egoRight = egoRight;
            this.allAdjacentLanes = allAdjacentLanes != null ? allAdjacentLanes : new ArrayList<>();
        }
    }

    private static class LaneCandidate {
        final com.vadas.core.vision.lane.LaneAnalyzer.Lane lane;
        final float baseX;

        LaneCandidate(com.vadas.core.vision.lane.LaneAnalyzer.Lane lane, float baseX) {
            this.lane = lane;
            this.baseX = baseX;
        }
    }

    private EgoLaneSelection selectEgoAndAdjacentLanes(
            @NonNull List<com.vadas.core.vision.lane.LaneAnalyzer.Lane> lanes,
            float yHood,
            @NonNull List<DetectedBox> vehicles) {

        List<LaneCandidate> candidates = new ArrayList<>();
        for (com.vadas.core.vision.lane.LaneAnalyzer.Lane l : lanes) {
            if (l.points == null || l.points.size() < 3) continue;
            float bX = interpolateXFromPoints(l.points, yHood);
            if (bX < -0.40f || bX > 1.40f) continue;
            candidates.add(new LaneCandidate(l, bX));
        }

        if (candidates.isEmpty()) {
            return new EgoLaneSelection(null, null, new ArrayList<>());
        }

        Collections.sort(candidates, (a, b) -> Float.compare(a.baseX, b.baseX));

        final float EGO_CENTER_X = 0.50f;
        LaneCandidate bestLeft = null;
        LaneCandidate bestRight = null;

        // 1. Tìm cặp vạch bao quanh đường chạy xe (Ego Pair)
        float bestScore = Float.MAX_VALUE;
        for (int i = 0; i < candidates.size() - 1; i++) {
            LaneCandidate cL = candidates.get(i);
            for (int j = i + 1; j < candidates.size(); j++) {
                LaneCandidate cR = candidates.get(j);
                float width = cR.baseX - cL.baseX;

                // Hỗ trợ dải bề rộng làn nhìn phối cảnh: [0.15, 1.10]
                if (cL.baseX < cR.baseX && width >= 0.15f && width <= 1.10f &&
                    cL.baseX <= 0.65f && cR.baseX >= 0.35f) {

                    float laneMid = (cL.baseX + cR.baseX) / 2.0f;
                    float score;
                    if (hasTrackedEgoHistory) {
                        float trackedW = trackedEgoRightBaseX - trackedEgoLeftBaseX;
                        float historyCost = Math.abs(cL.baseX - trackedEgoLeftBaseX) + Math.abs(cR.baseX - trackedEgoRightBaseX);
                        float widthCost = Math.abs(width - trackedW);
                        score = historyCost * 1.5f + widthCost * 1.5f;
                    } else {
                        float dist = Math.abs(laneMid - EGO_CENTER_X);
                        float widthPenalty = Math.abs(width - 0.50f);
                        // Ưu tiên cao cho cặp vạch kẹp giữa tâm xe chủ (0.50)
                        float bracketPenalty = (cL.baseX <= 0.52f && cR.baseX >= 0.48f) ? 0.0f : 0.40f;
                        score = dist * 1.0f + widthPenalty * 0.5f + bracketPenalty;
                    }

                    if (score < bestScore) {
                        bestScore = score;
                        bestLeft = cL;
                        bestRight = cR;
                    }
                }
            }
        }

        // 2. Single-boundary fallback if other side is occluded
        if (bestLeft == null && bestRight == null) {
            for (LaneCandidate c : candidates) {
                if (c.baseX <= 0.62f && c.baseX >= -0.30f) {
                    if (bestLeft == null || (hasTrackedEgoHistory ?
                            Math.abs(c.baseX - trackedEgoLeftBaseX) < Math.abs(bestLeft.baseX - trackedEgoLeftBaseX) :
                            c.baseX > bestLeft.baseX)) {
                        bestLeft = c;
                    }
                }
            }
            for (LaneCandidate c : candidates) {
                if (c.baseX >= 0.38f && c.baseX <= 1.30f) {
                    if (bestLeft != null && c.lane == bestLeft.lane) continue;
                    if (bestRight == null || (hasTrackedEgoHistory ?
                            Math.abs(c.baseX - trackedEgoRightBaseX) < Math.abs(bestRight.baseX - trackedEgoRightBaseX) :
                            c.baseX < bestRight.baseX)) {
                        bestRight = c;
                    }
                }
            }

            if (bestLeft != null && bestRight != null) {
                float width = bestRight.baseX - bestLeft.baseX;
                if (width < 0.15f || width > 1.10f) {
                    float distL = hasTrackedEgoHistory ? Math.abs(bestLeft.baseX - trackedEgoLeftBaseX) : Math.abs(bestLeft.baseX - 0.28f);
                    float distR = hasTrackedEgoHistory ? Math.abs(bestRight.baseX - trackedEgoRightBaseX) : Math.abs(bestRight.baseX - 0.72f);
                    if (distL <= distR) {
                        bestRight = null;
                    } else {
                        bestLeft = null;
                    }
                }
            }
        }

        // Update temporal tracking memory
        float trackedWidth = hasTrackedEgoHistory ? Math.max(0.24f, Math.min(0.90f, trackedEgoRightBaseX - trackedEgoLeftBaseX)) : 0.60f;
        if (bestLeft != null && bestRight != null) {
            trackedEgoLeftBaseX = 0.75f * trackedEgoLeftBaseX + 0.25f * bestLeft.baseX;
            trackedEgoRightBaseX = 0.75f * trackedEgoRightBaseX + 0.25f * bestRight.baseX;
            hasTrackedEgoHistory = true;
        } else if (bestLeft != null) {
            trackedEgoLeftBaseX = 0.75f * trackedEgoLeftBaseX + 0.25f * bestLeft.baseX;
            trackedEgoRightBaseX = trackedEgoLeftBaseX + trackedWidth;
            hasTrackedEgoHistory = true;
        } else if (bestRight != null) {
            trackedEgoRightBaseX = 0.75f * trackedEgoRightBaseX + 0.25f * bestRight.baseX;
            trackedEgoLeftBaseX = trackedEgoRightBaseX - trackedWidth;
            hasTrackedEgoHistory = true;
        }

        // 3. Adjacent lines: strictly filter out any lines occluded by detected vehicles
        List<com.vadas.core.vision.lane.LaneAnalyzer.Lane> allAdjacent = new ArrayList<>();
        for (LaneCandidate c : candidates) {
            if ((bestLeft == null || c.lane != bestLeft.lane) &&
                (bestRight == null || c.lane != bestRight.lane)) {
                if (c.lane.confidence >= 0.60f && c.lane.points != null && c.lane.points.size() >= 3) {
                    boolean isOccludedByVehicle = false;
                    for (DetectedBox box : vehicles) {
                        if (box.getConfidence() < 0.45f) continue;
                        int occCount = 0;
                        for (PointF pt : c.lane.points) {
                            if (pt.x >= box.getLeft() && pt.x <= box.getRight() &&
                                pt.y >= box.getTop() && pt.y <= box.getBottom() + 0.02f) {
                                occCount++;
                            }
                        }
                        if (occCount >= 3 || (float) occCount / c.lane.points.size() > 0.40f) {
                            isOccludedByVehicle = true;
                            break;
                        }
                    }

                    if (!isOccludedByVehicle) {
                        if (bestLeft != null && c.baseX < bestLeft.baseX - 0.05f) {
                            allAdjacent.add(c.lane);
                        } else if (bestRight != null && c.baseX > bestRight.baseX + 0.05f) {
                            allAdjacent.add(c.lane);
                        }
                    }
                }
            }
        }

        if (bestLeft == null && bestRight == null) {
            com.vadas.core.vision.lane.LaneAnalyzer.Lane fbLeft = null;
            com.vadas.core.vision.lane.LaneAnalyzer.Lane fbRight = null;
            List<com.vadas.core.vision.lane.LaneAnalyzer.Lane> fbAdj = new ArrayList<>();
            for (com.vadas.core.vision.lane.LaneAnalyzer.Lane l : lanes) {
                if (l.id == 1) fbLeft = l;
                else if (l.id == 2) fbRight = l;
                else fbAdj.add(l);
            }
            return new EgoLaneSelection(fbLeft, fbRight, fbAdj);
        }

        return new EgoLaneSelection(
                bestLeft != null ? bestLeft.lane : null,
                bestRight != null ? bestRight.lane : null,
                allAdjacent
        );
    }

    private float extractSagittaCurvature(@NonNull List<PointF> pts) {
        if (pts.size() < 3) return Float.NaN;
        PointF p1 = pts.get(0);
        PointF p2 = pts.get(pts.size() - 1);
        PointF pTop = p1.y < p2.y ? p1 : p2;
        PointF pBot = p1.y < p2.y ? p2 : p1;
        PointF pMid = pts.get(pts.size() / 2);

        float dy = pBot.y - pTop.y;
        if (dy <= 0.04f) return Float.NaN;

        float chordX = pTop.x + ((pMid.y - pTop.y) / dy) * (pBot.x - pTop.x);
        float sagitta = pMid.x - chordX;
        if (Math.abs(sagitta) < 0.015f) {
            return 0.0f;
        }
        return sagitta * 1.4f;
    }

    private float interpolateXFromPoints(@NonNull List<PointF> pts, float targetY) {
        return interpolateXFromPoints(pts, targetY, 0.50f, 0.44f);
    }

    private float interpolateXFromPoints(@NonNull List<PointF> pts, float targetY, float vpX, float horizonY) {
        if (pts.isEmpty()) return -1f;
        if (pts.size() == 1) return pts.get(0).x;

        PointF pFirst = pts.get(0);
        PointF pLast = pts.get(pts.size() - 1);
        boolean ascending = pFirst.y <= pLast.y;
        PointF pTop = ascending ? pFirst : pLast;
        PointF pBot = ascending ? pLast : pFirst;

        float minY = pTop.y;
        float maxY = pBot.y;

        // 1. Phân đoạn nội suy mượt mà bên trong phạm vi điểm thực tế
        if (targetY >= minY && targetY <= maxY) {
            for (int i = 0; i < pts.size() - 1; i++) {
                PointF p1 = pts.get(i);
                PointF p2 = pts.get(i + 1);
                float segMinY = Math.min(p1.y, p2.y);
                float segMaxY = Math.max(p1.y, p2.y);
                if (targetY >= segMinY && targetY <= segMaxY && Math.abs(p2.y - p1.y) > 1e-4f) {
                    float t = (targetY - p1.y) / (p2.y - p1.y);
                    return p1.x + t * (p2.x - p1.x);
                }
            }
            return pBot.x;
        }

        // 2. Ngoại suy lên phía chân trời (Perspective Convergence Ray hội tụ chuẩn xác về vpX):
        // Tuyệt đối không dùng tiếp tuyến vi phân (dy/dx) 2 điểm để tránh đâm chéo hoặc cong vẹo lên trời
        if (targetY < minY) {
            float denom = Math.max(0.01f, minY - horizonY);
            float t = Math.max(0.0f, Math.min(1.0f, (targetY - horizonY) / denom));
            return vpX + t * (pTop.x - vpX);
        }

        // 3. Ngoại suy xuống phía nắp ca-pô (Perspective Divergence Ray mở rộng tự nhiên từ vpX):
        // Giữ đúng góc nhìn phối cảnh của toàn bộ làn đường, triệt tiêu hoàn toàn vạch đâm ngang đáy
        float denom = Math.max(0.01f, maxY - horizonY);
        float t = (targetY - horizonY) / denom;
        t = Math.max(1.0f, Math.min(1.35f, t));
        return vpX + t * (pBot.x - vpX);
    }
}