package com.vadas.core.vision.lane;

import android.graphics.PointF;

import com.vadas.core.vision.pinhole.PinholeCameraParams;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

public class LaneAlgorithmTest {

    private LanePostProcessor postProcessor;
    private LaneDepartureWarning ldwEngine;

    @Before
    public void setUp() {
        postProcessor = new LanePostProcessor();
        PinholeCameraParams params = PinholeCameraParams.createDefaultDashcam1080p();
        ldwEngine = new LaneDepartureWarning(params, 0.0, 0.88f);
    }

    @Test
    public void testPolynomialFitQuadratic() {
        // Tạo tập điểm theo hàm chính xác x = 0.5 + 0.1 * y - 0.2 * y^2
        double[] yVals = new double[]{0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90};
        double[] xVals = new double[yVals.length];
        for (int i = 0; i < yVals.length; i++) {
            double y = yVals[i];
            xVals[i] = 0.5 + (0.1 * y) - (0.2 * y * y);
        }

        LanePostProcessor.PolynomialFitResult fit = LanePostProcessor.fitPolynomial(xVals, yVals, 2);
        Assert.assertNotNull(fit);
        Assert.assertNotNull(fit.coefficients);
        Assert.assertEquals(3, fit.coefficients.length);

        // Kiểm tra độ khớp R^2 > 0.999
        Assert.assertTrue("R^2 phải xấp xỉ 1.0 (thực tế: " + fit.rSquared + ")", fit.rSquared > 0.999);

        // Kiểm tra các hệ số hồi quy khớp với dữ liệu sinh [0.5, 0.1, -0.2]
        Assert.assertEquals(0.5, fit.coefficients[0], 0.02);
        Assert.assertEquals(0.1, fit.coefficients[1], 0.05);
        Assert.assertEquals(-0.2, fit.coefficients[2], 0.05);
    }

    @Test
    public void testSmoothedCurveGeneration() {
        double[] coefs = new double[]{0.50, -0.15, -0.08};
        List<PointF> smooth = LanePostProcessor.generateSmoothedCurve(0.40f, 0.90f, coefs, 25);

        Assert.assertNotNull(smooth);
        Assert.assertEquals(25, smooth.size());
    }

    @Test
    public void testLaneDepartureWarningCentered() {
        // Giả lập xe đi chính giữa làn (Ego Left ở 0.35, Ego Right ở 0.65 -> Tâm làn = 0.50 = Tâm xe)
        List<LanePostProcessor.LaneResult> lanes = new ArrayList<>();

        double[] leftCoef = new double[]{0.35, 0.0, 0.0};
        lanes.add(new LanePostProcessor.LaneResult(1, new ArrayList<>(), new ArrayList<>(), leftCoef, 0.95f, 0.99));

        double[] rightCoef = new double[]{0.65, 0.0, 0.0};
        lanes.add(new LanePostProcessor.LaneResult(2, new ArrayList<>(), new ArrayList<>(), rightCoef, 0.95f, 0.99));

        LaneDepartureWarning.LDWResult result = ldwEngine.evaluate(lanes, 60.0);
        Assert.assertNotNull(result);
        Assert.assertEquals(LaneDepartureWarning.LDWStatus.NORMAL, result.status);
        Assert.assertEquals(0.0, result.lateralOffsetMeters, 0.15); // Lệch tâm ~0 mét
        Assert.assertTrue(result.distanceToLeftMeters > 1.5);
        Assert.assertTrue(result.distanceToRightMeters > 1.5);
    }

    @Test
    public void testLaneDepartureWarningLeftDrift() {
        // Giả lập xe lệch mạnh sang trái (Left lane sát tâm xe: Left ở 0.45, Right ở 0.75 -> Tâm làn = 0.60, xe ở 0.50)
        List<LanePostProcessor.LaneResult> lanes = new ArrayList<>();

        double[] leftCoef = new double[]{0.45, 0.0, 0.0};
        lanes.add(new LanePostProcessor.LaneResult(1, new ArrayList<>(), new ArrayList<>(), leftCoef, 0.95f, 0.99));

        double[] rightCoef = new double[]{0.75, 0.0, 0.0};
        lanes.add(new LanePostProcessor.LaneResult(2, new ArrayList<>(), new ArrayList<>(), rightCoef, 0.95f, 0.99));

        LaneDepartureWarning.LDWResult result = ldwEngine.evaluate(lanes, 60.0);
        Assert.assertNotNull(result);

        // Khoảng cách mép lốp trái phải rất nhỏ hoặc âm
        Assert.assertTrue("Phải kích hoạt cảnh báo trái hoặc đè vạch (trạng thái: " + result.status + ")",
                result.status == LaneDepartureWarning.LDWStatus.WARNING_LEFT ||
                result.status == LaneDepartureWarning.LDWStatus.DEPARTURE_LEFT);
    }

    @Test
    public void testSpeedGatingSuppression() {
        // Tốc độ xe quá thấp (10 km/h < 25 km/h) -> Cảnh báo phải được kìm hãm để không gây phiền khi đỗ xe
        List<LanePostProcessor.LaneResult> lanes = new ArrayList<>();
        double[] leftCoef = new double[]{0.48, 0.0, 0.0};
        double[] rightCoef = new double[]{0.78, 0.0, 0.0};
        lanes.add(new LanePostProcessor.LaneResult(1, new ArrayList<>(), new ArrayList<>(), leftCoef, 0.9f, 0.99));
        lanes.add(new LanePostProcessor.LaneResult(2, new ArrayList<>(), new ArrayList<>(), rightCoef, 0.9f, 0.99));

        LaneDepartureWarning.LDWResult result = ldwEngine.evaluate(lanes, 10.0);
        Assert.assertEquals(LaneDepartureWarning.LDWStatus.NORMAL, result.status);
    }
}
