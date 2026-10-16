package com.vadas.core.vision.horizon;

import android.graphics.Bitmap;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.vadas.core.camera.model.CameraFrame;

/**
 * Real-time vision-based Horizon Line (Đường chân trời) & Camera Pitch Estimator.
 *
 * Implements a dual-cue fusion algorithm:
 * 1. Cue A (Vanishing Point Convergence):
 *    Calculates 2D projective intersection (VP_x, VP_y) of converging road lane boundaries.
 * 2. Cue B (High-Frequency Perspective Texture Peak & Pavement Contrast):
 *    Evaluates horizontal spatial gradient energy across vertical candidate elevations.
 *    Smooth foreground asphalt transitions into high-frequency perspective compression at the horizon.
 * 3. Temporal Rate-Limited Filter:
 *    Guarantees silky-smooth continuous tracking across frames without visual jitter (delta <= 0.003/frame).
 * 4. Pinhole Model Telemetry:
 *    Calculates physical camera pitch angle: theta = atan2(c_y - Y_horizon, f_y).
 */
public final class DynamicHorizonEstimator {

    public static final class HorizonEstimate {
        private final float horizonYNorm;      // Normalized vertical elevation (0.0 = top, 1.0 = bottom)
        private final float pitchDegrees;      // Vehicle pitch angle relative to flat ground (degrees)
        private final float vanishingPointX;   // Normalized horizontal vanishing point (0.0 to 1.0)
        private final float confidence;        // Detection confidence (0.0 to 1.0)

        public HorizonEstimate(float horizonYNorm, float pitchDegrees, float vanishingPointX, float confidence) {
            this.horizonYNorm = horizonYNorm;
            this.pitchDegrees = pitchDegrees;
            this.vanishingPointX = vanishingPointX;
            this.confidence = confidence;
        }

        public float getHorizonYNorm() {
            return horizonYNorm;
        }

        public float getPitchDegrees() {
            return pitchDegrees;
        }

        public float getVanishingPointX() {
            return vanishingPointX;
        }

        public float getConfidence() {
            return confidence;
        }
    }

    // Normalized intrinsic camera priors for standard dashcam (FOV ~70 deg, center = 0.50)
    private static final float CY_NORM = 0.500f;
    private static final float FY_NORM = 1.150f;

    // Search window for candidate horizon elevations
    private static final float SEARCH_Y_MIN = 0.280f;
    private static final float SEARCH_Y_MAX = 0.550f;

    // Temporal filter state
    private boolean initialized = false;
    private float smoothedHorizonY = 0.440f;
    private float smoothedVpX = 0.500f;

    // Reusable buffers to guarantee ZERO frame allocations in the 60 FPS loop
    private int[] rowPixelsBuffer;
    private float[] gradRowScores;

    public DynamicHorizonEstimator() {
        this(0.440f);
    }

    public DynamicHorizonEstimator(float initialHorizonY) {
        this.smoothedHorizonY = initialHorizonY;
        this.smoothedVpX = 0.500f;
    }

    /**
     * Resets the estimator state.
     */
    public void reset() {
        this.initialized = false;
        this.smoothedHorizonY = 0.440f;
        this.smoothedVpX = 0.500f;
    }

    /**
     * Estimates horizon from camera frame pixels and current tracked lane line points.
     *
     * @param frame CameraFrame containing frame bitmap
     * @param trackedLeftX Array of normalized X positions for left lane
     * @param trackedRightX Array of normalized X positions for right lane
     * @param scanElevationsNormY Array of normalized Y elevations corresponding to lane points
     * @return HorizonEstimate with live horizon Y, pitch angle, and VP X
     */
    @NonNull
    public HorizonEstimate estimateHorizon(@NonNull CameraFrame frame,
                                           @Nullable float[] trackedLeftX,
                                           @Nullable float[] trackedRightX,
                                           @Nullable float[] scanElevationsNormY) {
        Bitmap bitmap = frame.getBitmap();
        float rawCueY = smoothedHorizonY;
        float rawVpX = smoothedVpX;
        float confidence = 0.5f;

        // 1. Cue B: Visual Texture / Pavement Contrast Peak Scan
        if (bitmap != null && !bitmap.isRecycled()) {
            float visHorizon = detectVisualTextureHorizon(bitmap);
            if (visHorizon >= SEARCH_Y_MIN && visHorizon <= SEARCH_Y_MAX) {
                rawCueY = visHorizon;
                confidence = 0.75f;
            }
        }

        // 2. Cue A: Vanishing Point from Lane Boundary Convergence
        if (trackedLeftX != null && trackedRightX != null && scanElevationsNormY != null
                && trackedLeftX.length >= 3 && trackedRightX.length >= 3) {
            int n = trackedLeftX.length;
            // Use bottom slice (hood) and middle slice to calculate convergence
            float y0 = scanElevationsNormY[0];
            float y1 = scanElevationsNormY[Math.min(2, n - 1)];
            float dy = y1 - y0;

            if (Math.abs(dy) > 0.04f) {
                float mL = (trackedLeftX[Math.min(2, n - 1)] - trackedLeftX[0]) / dy;
                float mR = (trackedRightX[Math.min(2, n - 1)] - trackedRightX[0]) / dy;
                float dSlope = mL - mR;

                // Converging inward: left slopes right (mL < -0.1), right slopes left (mR > 0.1)
                if (dSlope < -0.3f) {
                    float laneVpY = y0 + (trackedRightX[0] - trackedLeftX[0]) / dSlope;
                    float laneVpX = trackedLeftX[0] + mL * (laneVpY - y0);

                    if (laneVpY >= SEARCH_Y_MIN && laneVpY <= SEARCH_Y_MAX) {
                        // Only fuse if consistent with visual horizon cue (within 0.06)
                        if (Math.abs(laneVpY - rawCueY) < 0.06f) {
                            rawCueY = 0.60f * rawCueY + 0.40f * laneVpY;
                            if (laneVpX >= 0.32f && laneVpX <= 0.65f) {
                                rawVpX = laneVpX;
                            }
                            confidence = 0.92f;
                        }
                    }
                }
            }
        }

        // Clamp raw estimate to physically realistic dashcam horizon range (broadened VP X for turns)
        rawCueY = Math.max(SEARCH_Y_MIN, Math.min(SEARCH_Y_MAX, rawCueY));
        rawVpX = Math.max(0.35f, Math.min(0.62f, rawVpX));

        // 3. Temporal Rate-Limited Smoothing Filter
        if (!initialized) {
            smoothedHorizonY = rawCueY;
            smoothedVpX = rawVpX;
            initialized = true;
        } else {
            // Vertical horizon rate limit: maximum adjustment per frame to eliminate any jitter or twitching (<= 0.003/frame)
            float maxStepY = 0.003f;
            float diffY = rawCueY - smoothedHorizonY;
            smoothedHorizonY += Math.max(-maxStepY, Math.min(maxStepY, diffY * 0.25f));

            // Horizontal vanishing point rate limit: responsive to vehicle steering into curves (<= 0.008/frame)
            float maxStepX = 0.008f;
            float diffX = rawVpX - smoothedVpX;
            smoothedVpX += Math.max(-maxStepX, Math.min(maxStepX, diffX * 0.35f));
        }

        // 4. Calculate Camera Pitch Angle using Pinhole Projection
        // theta_pitch = atan2(c_y - Y_horizon, f_y) * (180 / pi)
        // Positive pitch = camera tilted downward toward road
        float pitchDegrees = (float) Math.toDegrees(Math.atan2(CY_NORM - smoothedHorizonY, FY_NORM));

        return new HorizonEstimate(smoothedHorizonY, pitchDegrees, smoothedVpX, confidence);
    }

    /**
     * Fast vision-based texture peak detector analyzing the central column ROI.
     */
    private float detectVisualTextureHorizon(@NonNull Bitmap bitmap) {
        int w = bitmap.getWidth();
        int h = bitmap.getHeight();

        if (w <= 0 || h <= 0) return smoothedHorizonY;

        // ROI: Central 40% horizontal band [0.30*w, 0.70*w]
        int xMin = (int) (0.30f * w);
        int xMax = (int) (0.70f * w);
        int roiWidth = xMax - xMin;
        if (roiWidth <= 10) return smoothedHorizonY;

        if (rowPixelsBuffer == null || rowPixelsBuffer.length < roiWidth) {
            rowPixelsBuffer = new int[roiWidth];
        }

        int yMinPx = (int) (SEARCH_Y_MIN * h);
        int yMaxPx = (int) (SEARCH_Y_MAX * h);
        int numCandidateRows = yMaxPx - yMinPx + 1;

        if (gradRowScores == null || gradRowScores.length < numCandidateRows) {
            gradRowScores = new float[numCandidateRows];
        }

        // Subsample rows by step of 2 for ultra-fast performance (<0.5ms)
        int step = Math.max(1, h / 200);

        for (int i = 0; i < numCandidateRows; i += step) {
            int y = yMinPx + i;
            if (y >= h) break;

            try {
                bitmap.getPixels(rowPixelsBuffer, 0, roiWidth, xMin, y, roiWidth, 1);
            } catch (Exception e) {
                return smoothedHorizonY;
            }

            // Compute horizontal gradient across sampled row: |I(x+2) - I(x-2)|
            float gradSum = 0f;
            int samples = 0;
            for (int x = 2; x < roiWidth - 2; x += 3) {
                int p1 = rowPixelsBuffer[x - 2];
                int p2 = rowPixelsBuffer[x + 2];

                int lum1 = (((p1 >> 16) & 0xFF) * 299 + ((p1 >> 8) & 0xFF) * 587 + (p1 & 0xFF) * 114) / 1000;
                int lum2 = (((p2 >> 16) & 0xFF) * 299 + ((p2 >> 8) & 0xFF) * 587 + (p2 & 0xFF) * 114) / 1000;

                gradSum += Math.abs(lum2 - lum1);
                samples++;
            }

            float rowScore = samples > 0 ? (gradSum / samples) : 0f;
            gradRowScores[i] = rowScore;
            // Fill intermediate step if step > 1
            for (int s = 1; s < step && (i + s) < numCandidateRows; s++) {
                gradRowScores[i + s] = rowScore;
            }
        }

        // Find the peak in gradRowScores where high-frequency horizon texture meets smooth road pavement
        int bestIdx = -1;
        float bestScore = -1e6f;
        int paveOffset = Math.max(4, (int) (0.05f * h)); // Pavement sample below candidate
        int skyOffset = Math.max(4, (int) (0.05f * h));  // Sky sample above candidate

        for (int i = 2; i < numCandidateRows - 2; i++) {
            // 5-tap Gaussian-like smoothed score
            float gCenter = (gradRowScores[i - 2] + 2f * gradRowScores[i - 1] + 4f * gradRowScores[i]
                    + 2f * gradRowScores[i + 1] + gradRowScores[i + 2]) / 10f;

            int pIdx = Math.min(numCandidateRows - 1, i + paveOffset);
            float gPave = gradRowScores[pIdx];

            int sIdx = Math.max(0, i - skyOffset);
            float gSky = gradRowScores[sIdx];

            // Peak contrast against pavement and sky
            float peakness = (gCenter - gPave) + 0.5f * (gCenter - gSky);

            // Temporal distance penalty to avoid jumping due to isolated wiper drops
            float curNormY = (yMinPx + i) / (float) h;
            float distPen = Math.abs(curNormY - smoothedHorizonY) * 3.5f;

            float totalScore = peakness - distPen;
            if (totalScore > bestScore) {
                bestScore = totalScore;
                bestIdx = i;
            }
        }

        if (bestIdx >= 0) {
            return (yMinPx + bestIdx) / (float) h;
        }

        return smoothedHorizonY;
    }
}
