package com.vadas.core.vision.pinhole;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.vadas.core.vision.model.DetectedBox;

import org.junit.Before;
import org.junit.Test;

public class PinholeDistanceEstimatorTest {

    private PinholeDistanceEstimator estimator;

    @Before
    public void setUp() {
        PinholeCameraParams params = PinholeCameraParams.createDefaultDashcam1080p();
        estimator = new PinholeDistanceEstimator(params);
    }

    @Test
    public void testForwardCarDistanceEstimation() {
        // A sedan (1.5m height) at ~20m distance ahead in 1080p frame (H=1080, fy=1250)
        // h_pixels = (fy * H_real) / Z = (1250 * 1.5) / 20 = ~93.75 pixels -> ~0.0868 in normalized height
        // Centered horizontally: cx = 0.50
        float centerNorm = 0.50f;
        float widthNorm = 0.10f; // ~192 pixels width
        float heightNorm = 0.0868f; // ~93.75px height for 1.5m car at 20m

        // At Z = 20m with H_cam=1.3m and pitch=3 deg:
        // bottom contact point: alpha = atan(1.3 / 20) - rad(3 deg) = 0.0126 rad -> deltaY ~ 15.8px
        // bottomPx = 540 + 15.8 = 555.8px -> bottomNorm ~ 0.5146f
        float bottomNorm = 0.515f;
        float topNorm = bottomNorm - heightNorm;

        DetectedBox box = new DetectedBox(
                centerNorm - (widthNorm / 2),
                topNorm,
                centerNorm + (widthNorm / 2),
                bottomNorm,
                0.95f,
                DetectedBox.ObjectClass.CAR,
                0.0
        );

        PinholeDistanceEstimator.SpatialVehicleEstimate estimate = estimator.estimateDistance(box);

        // Distance should be close to 20m (+/- 3m due to ground/apparent fusion)
        assertTrue(estimate.getDistanceZ() > 15.0 && estimate.getDistanceZ() < 25.0);
        // Lateral offset should be centered in ego-lane (X close to 0)
        assertEquals(0.0, estimate.getLateralX(), 0.2);
        assertTrue(estimate.isInEgoLane());
    }

    @Test
    public void testNeighboringLaneVehicleDetection() {
        // Vehicle in adjacent right lane: normalized X center = 0.70 (shifted right)
        float leftNorm = 0.65f;
        float rightNorm = 0.75f;
        float topNorm = 0.54f;
        float bottomNorm = 0.62f;

        DetectedBox box = new DetectedBox(
                leftNorm,
                topNorm,
                rightNorm,
                bottomNorm,
                0.90f,
                DetectedBox.ObjectClass.CAR,
                0.0
        );

        PinholeDistanceEstimator.SpatialVehicleEstimate estimate = estimator.estimateDistance(box);

        // Vehicle is shifted to the right lane (X > 1.35m)
        assertTrue(estimate.getLateralX() > 1.35);
        assertFalse(estimate.isInEgoLane());
    }
}
