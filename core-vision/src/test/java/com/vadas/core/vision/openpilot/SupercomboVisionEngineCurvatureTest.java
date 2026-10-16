package com.vadas.core.vision.openpilot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import android.graphics.Bitmap;
import android.graphics.PointF;

import com.vadas.core.camera.model.CameraFrame;
import com.vadas.core.vision.lane.LaneAnalyzer;
import com.vadas.core.vision.pinhole.PinholeCameraParams;

import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SupercomboVisionEngineCurvatureTest {

    private SupercomboVisionEngine engine;

    @Before
    public void setUp() {
        engine = new SupercomboVisionEngine(PinholeCameraParams.createDefaultDashcam1080p());
    }

    @Test
    public void testNineSlicesInitializedProperly() {
        SupercomboOutput output = engine.getLatestSupercomboOutput();
        assertNotNull(output);
        float[] leftX = output.getLeftLaneNormX();
        float[] rightX = output.getRightLaneNormX();
        float[] elevY = output.getScanElevationNormY();

        assertEquals(9, leftX.length);
        assertEquals(9, rightX.length);
        assertEquals(9, elevY.length);

        // Verify descending elevation from hood (~0.67) to horizon (~0.40)
        assertTrue("Hood slice Y should be >= 0.65", elevY[0] >= 0.65f);
        assertTrue("Horizon slice Y should be <= 0.45", elevY[8] <= 0.45f);
        assertTrue("Elevations must be monotonically decreasing", elevY[0] > elevY[4] && elevY[4] > elevY[8]);
    }

    private static PointF pt(float x, float y) {
        PointF p = new PointF();
        p.x = x;
        p.y = y;
        return p;
    }

    @Test
    public void testCurvatureBendsLanesWhenTurningLeft() {
        List<PointF> leftPoints = new ArrayList<>();
        leftPoints.add(pt(0.385f, 0.670f));
        leftPoints.add(pt(0.320f, 0.600f));
        leftPoints.add(pt(0.260f, 0.535f));
        leftPoints.add(pt(0.225f, 0.465f));
        leftPoints.add(pt(0.200f, 0.400f));

        List<PointF> rightPoints = new ArrayList<>();
        rightPoints.add(pt(0.740f, 0.670f));
        rightPoints.add(pt(0.660f, 0.600f));
        rightPoints.add(pt(0.570f, 0.535f));
        rightPoints.add(pt(0.480f, 0.465f));
        rightPoints.add(pt(0.380f, 0.400f));

        LaneAnalyzer.Lane egoLeft = new LaneAnalyzer.Lane(1, leftPoints, 0.95f);
        LaneAnalyzer.Lane egoRight = new LaneAnalyzer.Lane(2, rightPoints, 0.95f);
        List<LaneAnalyzer.Lane> mockLanes = Arrays.asList(egoLeft, egoRight);

        // Feed mock AI lanes into engine
        engine.setLaneAnalyzer(new LaneAnalyzer(null, (l, f, lat, th) -> {}) {
            @Override
            public synchronized List<LaneAnalyzer.Lane> analyzeBitmap(Bitmap bitmap) {
                return mockLanes;
            }
        });

        CameraFrame frame = new CameraFrame(java.nio.ByteBuffer.allocate(0), null, 1920, 1080, 0, 1000L, CameraFrame.PixelFormat.RGBA_8888, null);

        // Simulate 15 consecutive frames turning left
        SupercomboOutput latest = null;
        for (int i = 0; i < 15; i++) {
            engine.processFrame(frame);
            latest = engine.getLatestSupercomboOutput();
        }

        assertNotNull(latest);
        // Curvature should be negative (turning LEFT)
        assertTrue("Curvature must be negative for left turn: " + latest.getRoadCurvature(),
                latest.getRoadCurvature() < -0.01f);

        // Yaw degrees should tilt into the turn (more negative than nominal -2.17 deg)
        assertTrue("Yaw degrees should reflect vehicle turning left: " + latest.getYawDegrees(),
                latest.getYawDegrees() < -2.17f);

        // Slices must visibly bend left: mid slice X should be shifted left relative to straight line
        float[] leftX = latest.getLeftLaneNormX();
        assertTrue("Left lane top slice should sweep left: " + leftX[8], leftX[8] < 0.40f);
    }

    @Test
    public void testCurvatureBendsLanesWhenTurningRight() {
        // Construct curved right lane with positive sagitta (curving toward right)
        List<PointF> leftPoints = new ArrayList<>();
        leftPoints.add(pt(0.385f, 0.670f));
        leftPoints.add(pt(0.450f, 0.600f));
        leftPoints.add(pt(0.510f, 0.535f));
        leftPoints.add(pt(0.540f, 0.465f));
        leftPoints.add(pt(0.560f, 0.400f));

        List<PointF> rightPoints = new ArrayList<>();
        rightPoints.add(pt(0.740f, 0.670f));
        rightPoints.add(pt(0.780f, 0.600f));
        rightPoints.add(pt(0.815f, 0.535f));
        rightPoints.add(pt(0.820f, 0.465f));
        rightPoints.add(pt(0.820f, 0.400f));

        LaneAnalyzer.Lane egoLeft = new LaneAnalyzer.Lane(1, leftPoints, 0.95f);
        LaneAnalyzer.Lane egoRight = new LaneAnalyzer.Lane(2, rightPoints, 0.95f);
        List<LaneAnalyzer.Lane> mockLanes = Arrays.asList(egoLeft, egoRight);

        engine.setLaneAnalyzer(new LaneAnalyzer(null, (l, f, lat, th) -> {}) {
            @Override
            public synchronized List<LaneAnalyzer.Lane> analyzeBitmap(Bitmap bitmap) {
                return mockLanes;
            }
        });

        CameraFrame frame = new CameraFrame(java.nio.ByteBuffer.allocate(0), null, 1920, 1080, 0, 1000L, CameraFrame.PixelFormat.RGBA_8888, null);

        // Simulate 15 consecutive frames turning right
        SupercomboOutput latest = null;
        for (int i = 0; i < 15; i++) {
            engine.processFrame(frame);
            latest = engine.getLatestSupercomboOutput();
        }

        assertNotNull(latest);
        // Curvature should be positive (turning RIGHT)
        assertTrue("Curvature must be positive for right turn: " + latest.getRoadCurvature(),
                latest.getRoadCurvature() > +0.01f);

        // Yaw degrees should tilt to the right (greater than nominal -2.17 deg)
        assertTrue("Yaw degrees should reflect vehicle turning right: " + latest.getYawDegrees(),
                latest.getYawDegrees() > -2.17f);

        // Right lane top slice should sweep right
        float[] rightX = latest.getRightLaneNormX();
        assertTrue("Right lane top slice should sweep right: " + rightX[8], rightX[8] > 0.52f);
    }

    @Test
    public void testSpatialEgoLaneAssociationWithMultipleLanes() {
        // Road has 3 lanes visible:
        // Lane 1: left curb / outer line (x ~ 0.04 to 0.28)
        // Lane 2: dashed center line (x ~ 0.46 to 0.49)
        // Lane 3: right edge / parking line (x ~ 0.88 to 0.52)
        // Vehicle is driving in the middle lane (center 0.50).
        // EgoLeft must be Lane 2 (~0.46), NOT Lane 1 (~0.04)!
        List<PointF> curbPoints = new ArrayList<>();
        curbPoints.add(pt(0.04f, 0.670f));
        curbPoints.add(pt(0.12f, 0.535f));
        curbPoints.add(pt(0.24f, 0.400f));

        List<PointF> centerDashedPoints = new ArrayList<>();
        centerDashedPoints.add(pt(0.46f, 0.670f));
        centerDashedPoints.add(pt(0.475f, 0.535f));
        centerDashedPoints.add(pt(0.49f, 0.400f));

        List<PointF> rightPoints = new ArrayList<>();
        rightPoints.add(pt(0.88f, 0.670f));
        rightPoints.add(pt(0.70f, 0.535f));
        rightPoints.add(pt(0.52f, 0.400f));

        // Even if model assigned IDs 1, 2, 3 in arbitrary order:
        LaneAnalyzer.Lane laneCurb = new LaneAnalyzer.Lane(1, curbPoints, 0.90f);
        LaneAnalyzer.Lane laneCenter = new LaneAnalyzer.Lane(2, centerDashedPoints, 0.95f);
        LaneAnalyzer.Lane laneRight = new LaneAnalyzer.Lane(3, rightPoints, 0.95f);
        List<LaneAnalyzer.Lane> mockLanes = Arrays.asList(laneCurb, laneCenter, laneRight);

        engine.setLaneAnalyzer(new LaneAnalyzer(null, (l, f, lat, th) -> {}) {
            @Override
            public synchronized List<LaneAnalyzer.Lane> analyzeBitmap(Bitmap bitmap) {
                return mockLanes;
            }
        });

        CameraFrame frame = new CameraFrame(java.nio.ByteBuffer.allocate(0), null, 1920, 1080, 0, 1000L, CameraFrame.PixelFormat.RGBA_8888, null);

        SupercomboOutput latest = null;
        for (int i = 0; i < 15; i++) {
            engine.processFrame(frame);
            latest = engine.getLatestSupercomboOutput();
        }

        assertNotNull(latest);
        float[] leftX = latest.getLeftLaneNormX();
        float[] rightX = latest.getRightLaneNormX();
        float[] adjLeftX = latest.getAdjLeftLaneNormX();

        // Left boundary of corridor at hood must be close to the center dashed line (~0.46), NOT the curb (~0.04)
        assertTrue("Ego Left lane base must be near center dashed line (> 0.40): " + leftX[0], leftX[0] > 0.40f);
        assertTrue("Ego Left lane base must be <= 0.50: " + leftX[0], leftX[0] <= 0.50f);

        // Right boundary of corridor at hood must be near right line (~0.88)
        assertTrue("Ego Right lane base must be > 0.75: " + rightX[0], rightX[0] > 0.75f);

        // Adjacent left lane should track the curb (~0.04)
        assertTrue("Adjacent left lane must track the outer curb (< 0.20): " + adjLeftX[0], adjLeftX[0] < 0.20f);
    }
}
