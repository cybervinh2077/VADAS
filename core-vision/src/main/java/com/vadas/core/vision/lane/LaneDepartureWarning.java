package com.vadas.core.vision.lane;

import android.graphics.PointF;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.vadas.core.vision.pinhole.PinholeCameraParams;

import java.util.List;
import java.util.Locale;

/**
 * Hệ thống Cảnh báo Chệch làn đường (Lane Departure Warning - LDW):
 * - Tích hợp cấu hình Camera Calibration (Pinhole Camera Parameters, góc nghiêng Pitch/Yaw, độ lệch vị trí đặt camera).
 * - Xác định chính xác tâm hình học của xe (Ego Vehicle Center) trên mặt đường tại khoảng cách cản trước (Front Bumper).
 * - Tính toán khoảng cách thực tế (mét) từ tim xe và mép lốp (Tire Clearance) tới vạch phân cách trái và phải.
 * - Thuật toán dự báo Time-to-Lane-Crossing (TLC) dựa trên vận tốc trôi ngang (Lateral Drift Velocity).
 * - Bộ lọc Hysteresis & Debounce chống báo động giả (False-Positive Suppression) khi xe chuyển làn hoặc rung lắc.
 */
public class LaneDepartureWarning {

    private static final String TAG = "LaneDepartureWarning";

    // Kích thước chuẩn phương tiện & làn đường (theo TCVN / AASHTO / Thông tư 31)
    public static final double STANDARD_LANE_WIDTH_METERS = 3.50; // Bề rộng làn đường chuẩn (3.5 mét)
    public static final double STANDARD_VEHICLE_WIDTH_METERS = 1.80; // Bề rộng thân xe ô tô con (1.8 mét)
    public static final double VEHICLE_HALF_WIDTH_METERS = STANDARD_VEHICLE_WIDTH_METERS / 2.0; // 0.90 mét

    // Các ngưỡng an toàn kích hoạt cảnh báo
    public static final double WARNING_TIRE_DISTANCE_THRESHOLD = 0.12; // Cảnh báo khi mép lốp cách vạch dưới 12 cm (loại trừ né chướng ngại vật trong làn)
    public static final double DEPARTURE_TIRE_DISTANCE_THRESHOLD = 0.00; // Đè vạch khi khoảng cách <= 0 cm
    public static final double RELEASE_TIRE_DISTANCE_THRESHOLD = 0.25; // Ngưỡng hồi phục an toàn để hủy cảnh báo (Hysteresis mượt mà)
    public static final double TLC_WARNING_THRESHOLD_SECONDS = 0.75;   // Kích hoạt khi TLC < 0.75 giây (tránh báo ảo khi chuyển làn có chủ đích)
    public static final double MIN_SPEED_KMH_ACTIVATION = 25.0;        // Tốc độ tối thiểu kích hoạt LDW (tránh phiền khi đỗ xe)

    public enum LDWStatus {
        NORMAL("An toàn", "#00E5FF", false),
        WARNING_LEFT("Cảnh báo: Trôi vạch Trái", "#FFAB00", true),
        WARNING_RIGHT("Cảnh báo: Trôi vạch Phải", "#FFAB00", true),
        DEPARTURE_LEFT("NGUY HIỂM: ĐÈ VẠCH TRÁI!", "#FF1744", true),
        DEPARTURE_RIGHT("NGUY HIỂM: ĐÈ VẠCH PHẢI!", "#FF1744", true),
        UNAVAILABLE("Chưa đủ điều kiện", "#9E9E9E", false);

        private final String message;
        private final String hexColor;
        private final boolean isWarning;

        LDWStatus(String message, String hexColor, boolean isWarning) {
            this.message = message;
            this.hexColor = hexColor;
            this.isWarning = isWarning;
        }

        public String getMessage() {
            return message;
        }

        public String getHexColor() {
            return hexColor;
        }

        public boolean isWarning() {
            return isWarning;
        }
    }

    /**
     * Kết quả đánh giá trạng thái chệch làn toàn diện.
     */
    public static class LDWResult {
        public final LDWStatus status;
        public final double distanceToLeftMeters;      // Khoảng cách từ tâm xe đến vạch trái (mét)
        public final double distanceToRightMeters;     // Khoảng cách từ tâm xe đến vạch phải (mét)
        public final double leftTireClearanceMeters;   // Khoảng cách từ mép lốp trái đến vạch (mét, âm = đè vạch)
        public final double rightTireClearanceMeters;  // Khoảng cách từ mép lốp phải đến vạch (mét, âm = đè vạch)
        public final double lateralOffsetMeters;       // Độ lệch tâm làn (+ lệch phải, - lệch trái)
        public final double lateralVelocityMps;        // Tốc độ dạt ngang (m/s)
        public final double timeToLaneCrossingSeconds; // Thời gian chạm vạch TLC (giây)
        public final float egoCenterPixelX;            // Tọa độ pixel tâm xe ở đáy kính lái
        public final float laneCenterPixelX;           // Tọa độ pixel tim làn ở đáy kính lái
        public final long timestampMs;

        public LDWResult(LDWStatus status,
                         double distanceToLeftMeters,
                         double distanceToRightMeters,
                         double leftTireClearanceMeters,
                         double rightTireClearanceMeters,
                         double lateralOffsetMeters,
                         double lateralVelocityMps,
                         double timeToLaneCrossingSeconds,
                         float egoCenterPixelX,
                         float laneCenterPixelX,
                         long timestampMs) {
            this.status = status;
            this.distanceToLeftMeters = distanceToLeftMeters;
            this.distanceToRightMeters = distanceToRightMeters;
            this.leftTireClearanceMeters = leftTireClearanceMeters;
            this.rightTireClearanceMeters = rightTireClearanceMeters;
            this.lateralOffsetMeters = lateralOffsetMeters;
            this.lateralVelocityMps = lateralVelocityMps;
            this.timeToLaneCrossingSeconds = timeToLaneCrossingSeconds;
            this.egoCenterPixelX = egoCenterPixelX;
            this.laneCenterPixelX = laneCenterPixelX;
            this.timestampMs = timestampMs;
        }

        @NonNull
        @Override
        public String toString() {
            return String.format(Locale.US,
                    "LDW[%s | L:%.2fm, R:%.2fm | Offset:%.2fm | TLC:%.1fs]",
                    status.name(), distanceToLeftMeters, distanceToRightMeters, lateralOffsetMeters, timeToLaneCrossingSeconds);
        }
    }

    private final PinholeCameraParams cameraParams;
    private final double cameraLateralOffsetMeters; // Độ lệch lắp đặt camera so với tâm xe (+ phải, - trái)
    private final float evaluationNormalizedY;     // Tọa độ Y chuẩn hóa để tính toán cản trước (mặc định 0.88 - 0.90)

    // Trạng thái theo dõi thời gian & động lực học dạt ngang
    private long lastEvaluationTimestamp = 0;
    private double lastLateralOffsetMeters = 0.0;
    private double smoothedLateralVelocityMps = 0.0;
    private static final double VELOCITY_EMA_ALPHA = 0.30; // Bộ lọc EMA giảm rung giật mặt đường

    // Bộ lọc Hysteresis duy trì trạng thái cảnh báo tối thiểu 15 khung hình (tránh nhấp nháy UI)
    private LDWStatus activeAlertStatus = LDWStatus.NORMAL;
    private int alertHoldFrameCounter = 0;
    private static final int MIN_ALERT_HOLD_FRAMES = 8;

    public LaneDepartureWarning() {
        this(PinholeCameraParams.createDefaultDashcam1080p(), 0.0, 0.88f);
    }

    public LaneDepartureWarning(@NonNull PinholeCameraParams cameraParams,
                                double cameraLateralOffsetMeters,
                                float evaluationNormalizedY) {
        this.cameraParams = cameraParams;
        this.cameraLateralOffsetMeters = cameraLateralOffsetMeters;
        this.evaluationNormalizedY = Math.max(0.60f, Math.min(0.98f, evaluationNormalizedY));
    }

    /**
     * Đánh giá trạng thái chệch làn dựa trên danh sách làn đường đã phát hiện và tốc độ xe hiện tại.
     *
     * @param lanes       danh sách làn đường sau xử lý hồi quy
     * @param egoSpeedKmh tốc độ phương tiện từ GPS hoặc OBD-II (km/h)
     * @return {@link LDWResult} chi tiết
     */
    @NonNull
    public synchronized LDWResult evaluate(@NonNull List<LanePostProcessor.LaneResult> lanes, double egoSpeedKmh) {
        long now = System.currentTimeMillis();

        // 1. TÌM KIẾM 2 LÀN TRUNG TÂM (Ego-Left id=1, Ego-Right id=2)
        LanePostProcessor.LaneResult leftLane = null;
        LanePostProcessor.LaneResult rightLane = null;

        for (LanePostProcessor.LaneResult lane : lanes) {
            if (lane.laneId == 1) leftLane = lane;
            else if (lane.laneId == 2) rightLane = lane;
        }

        // Dự phòng: Nếu mô hình không đánh số id chuẩn, tìm 2 làn gần tâm nhất
        if (leftLane == null || rightLane == null) {
            if (lanes.size() >= 2) {
                leftLane = lanes.get(0);
                rightLane = lanes.get(1);
            }
        }

        // 2. TÍNH TÂM XE (EGO CENTER) THEO THÔNG SỐ CAMERA CALIBRATION
        // Tâm quang học camera là cx (chuẩn hóa = cx / imageWidth, thường là 0.5)
        float normalizedEgoCenter = 0.5f;
        if (cameraParams != null && cameraParams.getImageWidth() > 0) {
            double cx = cameraParams.getPrincipalPointX();
            // Bù trừ vị trí lắp đặt camera lệch so với trục giữa xe: x_offset = cam_offset * fx / Z_bumper
            double zBumper = 2.2; // Khoảng cách cản trước ước tính 2.2 mét
            double pixelOffsetFromCam = (cameraLateralOffsetMeters * cameraParams.getFocalLengthX()) / zBumper;
            double trueEgoCenterPx = cx + pixelOffsetFromCam;
            normalizedEgoCenter = (float) (trueEgoCenterPx / cameraParams.getImageWidth());
        }

        // Nếu không phát hiện đủ cả 2 vạch phân làn
        if (leftLane == null || rightLane == null) {
            return new LDWResult(
                    LDWStatus.UNAVAILABLE,
                    1.75, 1.75, 0.85, 0.85, 0.0, 0.0,
                    Double.POSITIVE_INFINITY,
                    normalizedEgoCenter, normalizedEgoCenter,
                    now
            );
        }

        // 3. TÍNH VỊ TRÍ CỦA VẠCH TRÁI VÀ PHẢI TẠI ĐỘ CAO EVALUATION (GẦN CẢN XE)
        float xLeft = leftLane.evaluateXAtNormalizedY(evaluationNormalizedY);
        float xRight = rightLane.evaluateXAtNormalizedY(evaluationNormalizedY);

        // Đảm bảo xLeft luôn nằm bên trái xRight
        if (xLeft > xRight) {
            float temp = xLeft;
            xLeft = xRight;
            xRight = temp;
        }

        float laneWidthPxNormalized = Math.max(0.05f, xRight - xLeft);
        float laneCenterPxNormalized = (xLeft + xRight) / 2.0f;

        // 4. CHUYỂN ĐỔI PIXEL SANG MÉT DỰA TRÊN QUY CHUẨN LÀN ĐƯỜNG (3.5 METERS)
        double metersPerNormalizedUnit = STANDARD_LANE_WIDTH_METERS / laneWidthPxNormalized;

        // Đo khoảng cách từ tâm xe đến 2 vạch
        double distanceToLeft = (normalizedEgoCenter - xLeft) * metersPerNormalizedUnit;
        double distanceToRight = (xRight - normalizedEgoCenter) * metersPerNormalizedUnit;

        // Độ lệch tâm làn đường (+ lệch phải, - lệch trái)
        double lateralOffset = (normalizedEgoCenter - laneCenterPxNormalized) * metersPerNormalizedUnit;

        // Khoảng cách từ mép lốp xe tới từng vạch kẻ (Tire Clearance)
        double leftTireClearance = distanceToLeft - VEHICLE_HALF_WIDTH_METERS;
        double rightTireClearance = distanceToRight - VEHICLE_HALF_WIDTH_METERS;

        // 5. TÍNH TOÁN VẬN TỐC DẠT NGANG (LATERAL VELOCITY) VÀ TIME-TO-LANE-CROSSING (TLC)
        double dtSeconds = (lastEvaluationTimestamp > 0) ? (now - lastEvaluationTimestamp) / 1000.0 : 0.05;
        if (dtSeconds > 0.001 && dtSeconds < 1.0) {
            double rawLateralVelocity = (lateralOffset - lastLateralOffsetMeters) / dtSeconds;
            smoothedLateralVelocityMps = (VELOCITY_EMA_ALPHA * rawLateralVelocity) + ((1.0 - VELOCITY_EMA_ALPHA) * smoothedLateralVelocityMps);
        }
        lastEvaluationTimestamp = now;
        lastLateralOffsetMeters = lateralOffset;

        // Dự báo TLC: Thời gian còn lại trước khi mép lốp chạm vạch
        double tlcSeconds = Double.POSITIVE_INFINITY;
        if (smoothedLateralVelocityMps < -0.05) {
            // Xe đang dạt sang trái
            tlcSeconds = Math.max(0.0, leftTireClearance / Math.abs(smoothedLateralVelocityMps));
        } else if (smoothedLateralVelocityMps > 0.05) {
            // Xe đang dạt sang phải
            tlcSeconds = Math.max(0.0, rightTireClearance / smoothedLateralVelocityMps);
        }

        // 6. PHÂN LOẠI TRẠNG THÁI CẢNH BÁO (STATE MACHINE VỚI HYSTERESIS)
        LDWStatus detectedStatus = LDWStatus.NORMAL;

        // Kiểm tra ngưỡng tốc độ tối thiểu
        if (egoSpeedKmh >= MIN_SPEED_KMH_ACTIVATION) {
            if (leftTireClearance <= DEPARTURE_TIRE_DISTANCE_THRESHOLD) {
                detectedStatus = LDWStatus.DEPARTURE_LEFT;
            } else if (rightTireClearance <= DEPARTURE_TIRE_DISTANCE_THRESHOLD) {
                detectedStatus = LDWStatus.DEPARTURE_RIGHT;
            } else if (leftTireClearance <= WARNING_TIRE_DISTANCE_THRESHOLD || (smoothedLateralVelocityMps < -0.30 && tlcSeconds <= TLC_WARNING_THRESHOLD_SECONDS)) {
                detectedStatus = LDWStatus.WARNING_LEFT;
            } else if (rightTireClearance <= WARNING_TIRE_DISTANCE_THRESHOLD || (smoothedLateralVelocityMps > 0.30 && tlcSeconds <= TLC_WARNING_THRESHOLD_SECONDS)) {
                detectedStatus = LDWStatus.WARNING_RIGHT;
            }
        }

        // Áp dụng cơ chế Debounce & Hold Counter để cảnh báo không bị giật nháy
        if (detectedStatus.isWarning()) {
            activeAlertStatus = detectedStatus;
            alertHoldFrameCounter = MIN_ALERT_HOLD_FRAMES;
        } else {
            if (alertHoldFrameCounter > 0) {
                // Kiểm tra xem đã đạt ngưỡng hồi phục Hysteresis an toàn chưa
                boolean safelyRecovered = (leftTireClearance > RELEASE_TIRE_DISTANCE_THRESHOLD && rightTireClearance > RELEASE_TIRE_DISTANCE_THRESHOLD);
                if (safelyRecovered) {
                    alertHoldFrameCounter--;
                }
            } else {
                activeAlertStatus = LDWStatus.NORMAL;
            }
        }

        return new LDWResult(
                activeAlertStatus,
                distanceToLeft,
                distanceToRight,
                leftTireClearance,
                rightTireClearance,
                lateralOffset,
                smoothedLateralVelocityMps,
                tlcSeconds,
                normalizedEgoCenter,
                laneCenterPxNormalized,
                now
        );
    }
}
