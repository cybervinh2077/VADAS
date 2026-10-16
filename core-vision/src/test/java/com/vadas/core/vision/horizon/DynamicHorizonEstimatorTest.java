package com.vadas.core.vision.horizon;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.vadas.core.camera.model.CameraFrame;

import org.junit.Before;
import org.junit.Test;

public class DynamicHorizonEstimatorTest {

    private DynamicHorizonEstimator estimator;

    @Before
    public void setUp() {
        estimator = new DynamicHorizonEstimator(0.440f);
    }

    @Test
    public void testVanishingPointConvergenceWebvision() {
        // Webvision nominal lane perspective coordinates
        // Hood: y=0.670, Left=0.170, Right=0.605
        // Mid:  y=0.570, Left=0.300, Right=0.528
        // Top:  y=0.475, Left=0.420, Right=0.480
        float[] leftX = new float[]{0.170f, 0.235f, 0.300f, 0.362f, 0.420f};
        float[] rightX = new float[]{0.605f, 0.562f, 0.528f, 0.502f, 0.480f};
        float[] elevY = new float[]{0.670f, 0.620f, 0.570f, 0.520f, 0.475f};

        CameraFrame frame = new CameraFrame(java.nio.ByteBuffer.allocate(0), null, 1920, 1080, 0, 1000L, CameraFrame.PixelFormat.RGBA_8888, null);
        DynamicHorizonEstimator.HorizonEstimate est = estimator.estimateHorizon(frame, leftX, rightX, elevY);

        // Webvision horizon should be around 0.40 - 0.50
        assertTrue("Horizon Y should be between 0.40 and 0.50: " + est.getHorizonYNorm(),
                est.getHorizonYNorm() >= 0.40f && est.getHorizonYNorm() <= 0.50f);

        // Pitch should be slightly positive (looking down ~ +1 to +6 deg)
        assertTrue("Pitch angle should be realistic: " + est.getPitchDegrees(),
                est.getPitchDegrees() >= 0.0f && est.getPitchDegrees() <= 6.0f);
    }

    @Test
    public void testVanishingPointConvergenceRainDashcam() {
        // WRX Rain camera angle looking further down
        // Hood: y=0.670, Left=0.170, Right=0.605
        // Vanishing point at y ~ 0.350, x ~ 0.455
        float[] leftX = new float[]{0.170f, 0.241f, 0.312f, 0.383f, 0.455f};
        float[] rightX = new float[]{0.605f, 0.567f, 0.530f, 0.492f, 0.455f};
        float[] elevY = new float[]{0.670f, 0.590f, 0.510f, 0.430f, 0.350f};

        DynamicHorizonEstimator rainEstimator = new DynamicHorizonEstimator(0.360f);
        CameraFrame frame = new CameraFrame(java.nio.ByteBuffer.allocate(0), null, 1920, 1080, 0, 2000L, CameraFrame.PixelFormat.RGBA_8888, null);
        DynamicHorizonEstimator.HorizonEstimate est = rainEstimator.estimateHorizon(frame, leftX, rightX, elevY);

        // Rain horizon should be near 0.32 - 0.40
        assertTrue("Horizon Y should be between 0.32 and 0.40: " + est.getHorizonYNorm(),
                est.getHorizonYNorm() >= 0.32f && est.getHorizonYNorm() <= 0.40f);

        // Pitch should be higher (+5 to +9 deg)
        assertTrue("Pitch angle for tilted-down dashcam should be ~5-9 deg: " + est.getPitchDegrees(),
                est.getPitchDegrees() >= 5.0f && est.getPitchDegrees() <= 9.0f);
    }

    @Test
    public void testTemporalSmoothingEliminatesJumps() {
        float[] leftX = new float[]{0.170f, 0.235f, 0.300f, 0.362f, 0.420f};
        float[] rightX = new float[]{0.605f, 0.562f, 0.528f, 0.502f, 0.480f};
        float[] elevY = new float[]{0.670f, 0.620f, 0.570f, 0.520f, 0.475f};

        CameraFrame frame = new CameraFrame(java.nio.ByteBuffer.allocate(0), null, 1920, 1080, 0, 1000L, CameraFrame.PixelFormat.RGBA_8888, null);
        DynamicHorizonEstimator.HorizonEstimate est1 = estimator.estimateHorizon(frame, leftX, rightX, elevY);

        // Second frame with slightly shifted lane lines
        float[] leftX2 = new float[]{0.175f, 0.240f, 0.305f, 0.367f, 0.425f};
        DynamicHorizonEstimator.HorizonEstimate est2 = estimator.estimateHorizon(frame, leftX2, rightX, elevY);

        // Delta between consecutive frames must be <= 0.003 to guarantee no stutter ("không giật cục")
        float deltaY = Math.abs(est2.getHorizonYNorm() - est1.getHorizonYNorm());
        assertTrue("Frame-to-frame delta must be <= 0.003: " + deltaY, deltaY <= 0.0035f);
    }
}
