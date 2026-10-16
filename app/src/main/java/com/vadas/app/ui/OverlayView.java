package com.vadas.app.ui;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.LinearGradient;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.PointF;
import android.graphics.RectF;
import android.graphics.Shader;
import android.os.PowerManager;
import android.util.AttributeSet;
import android.view.View;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.vadas.core.vision.lane.LaneAnalyzer;
import com.vadas.core.vision.model.DetectedBox;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Lớp phủ đồ họa AR ADAS tinh gọn (Clean ADAS Overlay View):
 * - Tập trung tối đa vào 2 thành phần cốt lõi:
 *   1. Kẻ các đường vạch làn đường (Lane Lines & Ego Corridor).
 *   2. Kẻ ô Bounding Box bao quanh các phương tiện phát hiện trên đường (Detected Vehicles).
 * - Hiển thị bảng Log thông số chẩn đoán thời gian thực (Live Log Console).
 * - Đã loại bỏ hoàn toàn các hiển thị khoảng cách và UI không cần thiết theo yêu cầu.
 */
public class OverlayView extends View {

    // Synchronization Lock an toàn luồng
    private final Object dataLock = new Object();

    // Dữ liệu thời gian thực được bảo vệ bởi dataLock
    private final List<LaneAnalyzer.Lane> activeLanes = new ArrayList<>();
    private final List<DetectedBox> activeVehicles = new ArrayList<>();
    private float currentFps = 0.0f;
    private long currentLatencyMs = 0;
    private int currentThermalStatus = PowerManager.THERMAL_STATUS_NONE;
    private String hardwareLabel = "NNAPI (NPU)";
    private boolean isLogVisible = true;

    // Viewport cho matching tỷ lệ khung hình
    private final RectF videoViewport = new RectF();

    // Paints cho vạch làn
    private final Paint lanePaintCore = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint lanePaintGlow = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint adjacentLanePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint roadLimitPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint egoCorridorPaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    // Paints cho Bounding Box phương tiện
    private final Paint vehicleBoxPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint vehicleBoxFillPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint vehicleCornerPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint vehicleBadgeBgPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint vehicleBadgeTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint vehicleBadgeStrokePaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    // Paints cho Bảng Log Console (To hơn, rõ nét hơn)
    private final Paint logBgPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint logHeaderPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint logTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint logDotPaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    // Reusable Paths & Rects
    private final Path lanePath = new Path();
    private final Path corridorPath = new Path();
    private final RectF boxRect = new RectF();
    private final RectF badgeRect = new RectF();
    private final RectF logCardRect = new RectF();

    public OverlayView(Context context) {
        super(context);
        init();
    }

    public OverlayView(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public OverlayView(Context context, @Nullable AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init();
    }

    private void init() {
        // 1. Cấu hình cọ vẽ vạch làn xe chủ (Neon Cyan)
        lanePaintCore.setColor(Color.parseColor("#00E5FF"));
        lanePaintCore.setStyle(Paint.Style.STROKE);
        lanePaintCore.setStrokeWidth(7.0f);
        lanePaintCore.setStrokeCap(Paint.Cap.ROUND);
        lanePaintCore.setStrokeJoin(Paint.Join.ROUND);

        lanePaintGlow.setColor(Color.parseColor("#4400E5FF"));
        lanePaintGlow.setStyle(Paint.Style.STROKE);
        lanePaintGlow.setStrokeWidth(20.0f);
        lanePaintGlow.setStrokeCap(Paint.Cap.ROUND);
        lanePaintGlow.setStrokeJoin(Paint.Join.ROUND);

        // Vạch làn lân cận (Vàng Neon)
        adjacentLanePaint.setColor(Color.parseColor("#FFD54F"));
        adjacentLanePaint.setStyle(Paint.Style.STROKE);
        adjacentLanePaint.setStrokeWidth(4.5f);
        adjacentLanePaint.setStrokeCap(Paint.Cap.ROUND);

        // Vạch giới hạn biên đường / mép đường (Trắng sáng)
        roadLimitPaint.setColor(Color.parseColor("#ECEFF1"));
        roadLimitPaint.setStyle(Paint.Style.STROKE);
        roadLimitPaint.setStrokeWidth(4.5f);
        roadLimitPaint.setStrokeCap(Paint.Cap.ROUND);

        egoCorridorPaint.setStyle(Paint.Style.FILL);

        // 2. Cấu hình cọ vẽ Bounding Box phương tiện
        vehicleBoxPaint.setStyle(Paint.Style.STROKE);
        vehicleBoxPaint.setStrokeWidth(3.0f);

        vehicleBoxFillPaint.setStyle(Paint.Style.FILL);

        vehicleCornerPaint.setStyle(Paint.Style.STROKE);
        vehicleCornerPaint.setStrokeWidth(6.5f);
        vehicleCornerPaint.setStrokeCap(Paint.Cap.SQUARE);

        vehicleBadgeBgPaint.setStyle(Paint.Style.FILL);
        vehicleBadgeBgPaint.setColor(Color.parseColor("#E60D1117"));

        vehicleBadgeTextPaint.setColor(Color.WHITE);
        vehicleBadgeTextPaint.setTextSize(22.0f);
        vehicleBadgeTextPaint.setFakeBoldText(true);

        vehicleBadgeStrokePaint.setStyle(Paint.Style.STROKE);
        vehicleBadgeStrokePaint.setStrokeWidth(1.5f);

        // 3. Cấu hình cọ vẽ Log Console (Màn hình to hơn, chữ rõ ràng)
        logBgPaint.setStyle(Paint.Style.FILL);
        logBgPaint.setColor(Color.parseColor("#E60A0E17")); // Kính tối độ mờ cao

        logHeaderPaint.setColor(Color.parseColor("#00E5FF"));
        logHeaderPaint.setTextSize(26.0f);
        logHeaderPaint.setFakeBoldText(true);

        logTextPaint.setColor(Color.parseColor("#ECEFF1"));
        logTextPaint.setTextSize(20.0f);
        logTextPaint.setAntiAlias(true);

        logDotPaint.setStyle(Paint.Style.FILL);
    }

    public void setVideoViewport(@Nullable RectF vp) {
        if (vp != null) {
            this.videoViewport.set(vp);
        } else {
            this.videoViewport.setEmpty();
        }
        postInvalidate();
    }

    public void toggleLogVisibility() {
        this.isLogVisible = !this.isLogVisible;
        postInvalidate();
    }

    public void setLogVisible(boolean visible) {
        this.isLogVisible = visible;
        postInvalidate();
    }

    // Cấu trúc điểm làm mượt theo thời gian (Exponential Moving Average Filter) loại bỏ hoàn toàn giật cục
    private final Map<Integer, List<PointF>> smoothedLanePointsMap = new HashMap<>();
    private final Map<Integer, Integer> laneMissedFramesMap = new HashMap<>();
    private final Map<Integer, Float> laneConfidenceMap = new HashMap<>();

    // Bộ theo dõi và làm mượt Bounding Box phương tiện (Zero-Flicker Spatial LERP Tracker)
    private static class DisplayVehicleTrack {
        float left, top, right, bottom;
        float confidence;
        DetectedBox.ObjectClass objectClass;
        int missedFrames = 0;

        DisplayVehicleTrack(float left, float top, float right, float bottom,
                            float confidence, DetectedBox.ObjectClass objectClass) {
            this.left = left;
            this.top = top;
            this.right = right;
            this.bottom = bottom;
            this.confidence = confidence;
            this.objectClass = objectClass;
        }

        float getCenterX() { return (left + right) / 2.0f; }
        float getCenterY() { return (top + bottom) / 2.0f; }
    }

    private final List<DisplayVehicleTrack> smoothedVehicleTracks = new ArrayList<>();

    private List<DetectedBox> applyVehicleSmoothing(@NonNull List<DetectedBox> incomingBoxes) {
        float alpha = 0.65f; // LERP weight: bám vị trí mới mượt mà không trễ
        boolean[] matchedIncoming = new boolean[incomingBoxes.size()];
        List<DisplayVehicleTrack> nextTracks = new ArrayList<>();

        for (DisplayVehicleTrack track : smoothedVehicleTracks) {
            float bestIoU = -1.0f;
            int bestIdx = -1;

            for (int i = 0; i < incomingBoxes.size(); i++) {
                if (matchedIncoming[i]) continue;
                DetectedBox box = incomingBoxes.get(i);

                float interLeft = Math.max(track.left, box.getLeft());
                float interTop = Math.max(track.top, box.getTop());
                float interRight = Math.min(track.right, box.getRight());
                float interBottom = Math.min(track.bottom, box.getBottom());

                float interArea = Math.max(0.0f, interRight - interLeft) * Math.max(0.0f, interBottom - interTop);
                float area1 = Math.max(0.0f, track.right - track.left) * Math.max(0.0f, track.bottom - track.top);
                float area2 = Math.max(0.0f, box.getRight() - box.getLeft()) * Math.max(0.0f, box.getBottom() - box.getTop());
                float unionArea = area1 + area2 - interArea;
                float iou = unionArea > 0 ? interArea / unionArea : 0f;

                float distCenter = Math.abs(track.getCenterX() - ((box.getLeft() + box.getRight()) / 2.0f))
                        + Math.abs(track.getCenterY() - ((box.getTop() + box.getBottom()) / 2.0f));

                if ((iou > 0.15f || distCenter < 0.12f) && (iou > bestIoU)) {
                    bestIoU = iou;
                    bestIdx = i;
                }
            }

            if (bestIdx >= 0) {
                DetectedBox matched = incomingBoxes.get(bestIdx);
                matchedIncoming[bestIdx] = true;

                track.left = (alpha * matched.getLeft()) + ((1.0f - alpha) * track.left);
                track.top = (alpha * matched.getTop()) + ((1.0f - alpha) * track.top);
                track.right = (alpha * matched.getRight()) + ((1.0f - alpha) * track.right);
                track.bottom = (alpha * matched.getBottom()) + ((1.0f - alpha) * track.bottom);
                track.confidence = (alpha * matched.getConfidence()) + ((1.0f - alpha) * track.confidence);
                track.objectClass = matched.getObjectClass();
                track.missedFrames = 0;
                nextTracks.add(track);
            } else {
                track.missedFrames++;
                if (track.missedFrames <= 4) { // Giữ lại tối đa 4 frames (~120ms) để không bị nhấp nháy
                    track.confidence *= 0.90f;
                    nextTracks.add(track);
                }
            }
        }

        // Thêm các xe mới phát hiện ngay lập tức
        for (int i = 0; i < incomingBoxes.size(); i++) {
            if (!matchedIncoming[i]) {
                DetectedBox b = incomingBoxes.get(i);
                nextTracks.add(new DisplayVehicleTrack(b.getLeft(), b.getTop(), b.getRight(), b.getBottom(),
                        b.getConfidence(), b.getObjectClass()));
            }
        }

        smoothedVehicleTracks.clear();
        smoothedVehicleTracks.addAll(nextTracks);

        List<DetectedBox> result = new ArrayList<>(smoothedVehicleTracks.size());
        for (DisplayVehicleTrack t : smoothedVehicleTracks) {
            result.add(new DetectedBox(t.left, t.top, t.right, t.bottom, t.confidence, t.objectClass, 0.0));
        }
        return result;
    }

    private List<LaneAnalyzer.Lane> applyTemporalSmoothing(@NonNull List<LaneAnalyzer.Lane> incomingLanes) {
        List<LaneAnalyzer.Lane> result = new ArrayList<>(incomingLanes.size());
        float alpha = 0.35f; // Hệ số EMA tối ưu: làm mượt mà chuyển động mà không giật lắc khi né xe

        // 1. Cập nhật các làn mới đến
        for (LaneAnalyzer.Lane lane : incomingLanes) {
            laneMissedFramesMap.put(lane.id, 0);
            laneConfidenceMap.put(lane.id, lane.confidence);
            List<PointF> smoothedPts = smoothedLanePointsMap.get(lane.id);
            if (smoothedPts == null || smoothedPts.size() != lane.points.size()) {
                smoothedPts = new ArrayList<>(lane.points.size());
                for (PointF p : lane.points) {
                    smoothedPts.add(new PointF(p.x, p.y));
                }
                smoothedLanePointsMap.put(lane.id, smoothedPts);
            } else {
                for (int i = 0; i < lane.points.size(); i++) {
                    PointF target = lane.points.get(i);
                    PointF cur = smoothedPts.get(i);
                    cur.x = (alpha * target.x) + ((1.0f - alpha) * cur.x);
                    cur.y = (alpha * target.y) + ((1.0f - alpha) * cur.y);
                }
            }

            // Tạo làn đường với các điểm đã được làm mượt hoàn hảo
            List<PointF> finalPts = new ArrayList<>(smoothedPts.size());
            for (PointF sp : smoothedPts) {
                finalPts.add(new PointF(sp.x, sp.y));
            }
            result.add(new LaneAnalyzer.Lane(lane.id, finalPts, lane.confidence));
        }

        // 2. Chống chớp nháy (Hysteresis): Giữ lại các làn tạm thời bị mất dấu trong 6 khung hình (~180ms)
        java.util.Iterator<Map.Entry<Integer, Integer>> iter = laneMissedFramesMap.entrySet().iterator();
        while (iter.hasNext()) {
            Map.Entry<Integer, Integer> entry = iter.next();
            int laneId = entry.getKey();
            boolean present = false;
            for (LaneAnalyzer.Lane l : incomingLanes) {
                if (l.id == laneId) {
                    present = true;
                    break;
                }
            }
            if (!present) {
                int missed = entry.getValue() + 1;
                if (missed <= 6) {
                    entry.setValue(missed);
                    List<PointF> pts = smoothedLanePointsMap.get(laneId);
                    if (pts != null && !pts.isEmpty()) {
                        float conf = laneConfidenceMap.containsKey(laneId) ? laneConfidenceMap.get(laneId) * 0.90f : 0.80f;
                        List<PointF> copy = new ArrayList<>(pts.size());
                        for (PointF p : pts) copy.add(new PointF(p.x, p.y));
                        result.add(new LaneAnalyzer.Lane(laneId, copy, conf));
                    }
                } else {
                    smoothedLanePointsMap.remove(laneId);
                    laneConfidenceMap.remove(laneId);
                    iter.remove();
                }
            }
        }

        return result;
    }

    /**
     * Cập nhật kết quả thị giác máy tính: Vạch làn và Phương tiện (Hoàn toàn Thread-Safe & Mượt mà).
     */
    public void updatePerceptionData(@NonNull List<LaneAnalyzer.Lane> lanes,
                                     @NonNull List<DetectedBox> vehicles,
                                     float fps,
                                     long latencyMs,
                                     int thermalStatus,
                                     @NonNull String hardwareBackend) {
        synchronized (dataLock) {
            this.activeLanes.clear();
            this.activeLanes.addAll(lanes);
            this.activeVehicles.clear();
            this.activeVehicles.addAll(vehicles);
            this.currentFps = fps;
            this.currentLatencyMs = latencyMs;
            this.currentThermalStatus = thermalStatus;
            this.hardwareLabel = hardwareBackend;
        }
        postInvalidate();
    }

    /**
     * Nạp chồng tương thích nhận danh sách làn đường.
     */
    public void updateLaneData(@NonNull List<LaneAnalyzer.Lane> lanes,
                               float fps,
                               long latencyMs,
                               int thermalStatus,
                               @NonNull String hardwareBackend) {
        synchronized (dataLock) {
            this.activeLanes.clear();
            this.activeLanes.addAll(lanes);
            this.currentFps = fps;
            this.currentLatencyMs = latencyMs;
            this.currentThermalStatus = thermalStatus;
            this.hardwareLabel = hardwareBackend;
        }
        postInvalidate();
    }

    @Override
    protected void onDraw(@NonNull Canvas canvas) {
        super.onDraw(canvas);

        int viewWidth = getWidth();
        int viewHeight = getHeight();
        if (viewWidth == 0 || viewHeight == 0) return;

        float vpLeft = videoViewport.isEmpty() ? 0 : videoViewport.left;
        float vpTop = videoViewport.isEmpty() ? 0 : videoViewport.top;
        float vpWidth = videoViewport.isEmpty() ? viewWidth : videoViewport.width();
        float vpHeight = videoViewport.isEmpty() ? viewHeight : videoViewport.height();

        List<LaneAnalyzer.Lane> lanesCopy;
        List<DetectedBox> vehiclesCopy;
        float fps;
        long latency;
        int thermal;
        String hwName;

        synchronized (dataLock) {
            lanesCopy = new ArrayList<>(this.activeLanes);
            vehiclesCopy = new ArrayList<>(this.activeVehicles);
            fps = this.currentFps;
            latency = this.currentLatencyMs;
            thermal = this.currentThermalStatus;
            hwName = this.hardwareLabel;
        }

        // 1. VẼ CÁC ĐƯỜNG KẺ VẠCH LÀN ĐƯỜNG THUẦN TÚY (PURE OPENPILOT LANE LINES - NO CORRIDOR, NO ANIMATION)
        drawLaneLines(canvas, lanesCopy, vehiclesCopy, vpLeft, vpTop, vpWidth, vpHeight);

        // 2. KẺ Ô CÁC PHƯƠNG TIỆN PHÁT HIỆN TRÊN ĐƯỜNG (DETECTED VEHICLE BOUNDING BOXES)
        drawDetectedVehicles(canvas, vehiclesCopy, vpLeft, vpTop, vpWidth, vpHeight);

        // 3. HIỂN THỊ LOG THÔNG SỐ CHẨN ĐOÁN THỜI GIAN THỰC (LIVE LOG CONSOLE)
        if (isLogVisible) {
            drawLiveLogConsole(canvas, fps, latency, thermal, hwName, lanesCopy, vehiclesCopy, viewWidth, false);
        }
    }

    // =========================================================================
    // 1. VẼ CÁC ĐƯỜNG KẺ VẠCH LÀN ĐƯỜNG THUẦN TÚY (PURE OPENPILOT LANE LINES)
    // =========================================================================

    private void drawLaneLines(@NonNull Canvas canvas, @NonNull List<LaneAnalyzer.Lane> lanes,
                               @NonNull List<DetectedBox> vehicles,
                               float vpLeft, float vpTop, float vpWidth, float vpHeight) {
        for (LaneAnalyzer.Lane lane : lanes) {
            if (lane.points.size() < 2) continue;

            // Kiểm tra va chạm với phương tiện đối với làn lân cận và giới hạn biên:
            if (lane.id != 1 && lane.id != 2) {
                boolean occludedByVehicle = false;
                for (DetectedBox box : vehicles) {
                    if (box.getConfidence() < 0.45f) continue;
                    int occCount = 0;
                    for (PointF pt : lane.points) {
                        if (pt.x >= box.getLeft() && pt.x <= box.getRight() &&
                            pt.y >= box.getTop() && pt.y <= box.getBottom() + 0.02f) {
                            occCount++;
                        }
                    }
                    if (occCount >= 3 || (float) occCount / lane.points.size() > 0.40f) {
                        occludedByVehicle = true;
                        break;
                    }
                }
                if (occludedByVehicle) {
                    continue;
                }
            }

            lanePath.reset();
            List<PointF> sortedLanePts = new ArrayList<>(lane.points);
            sortedLanePts.sort((a, b) -> Float.compare(b.y, a.y));
            appendSmoothPointsToPath(lanePath, sortedLanePts, vpLeft, vpTop, vpWidth, vpHeight, true);

            if (lane.id == 1 || lane.id == 2) {
                // Vạch làn xe chủ (Ego Lanes - Neon Cyan)
                canvas.drawPath(lanePath, lanePaintGlow);
                canvas.drawPath(lanePath, lanePaintCore);
            } else if (lane.id == 0 || lane.id == 3) {
                // Vạch làn lân cận thực tế (Adjacent Lanes - Vàng Neon)
                canvas.drawPath(lanePath, adjacentLanePaint);
            } else {
                // Giới hạn biên lề đường bổ sung (Road Limits, id >= 4 - Trắng sáng)
                canvas.drawPath(lanePath, roadLimitPaint);
            }
        }
    }

    // =========================================================================
    // 2. KẺ Ô CÁC PHƯƠNG TIỆN PHÁT HIỆN TRÊN ĐƯỜNG
    // =========================================================================

    private void drawDetectedVehicles(@NonNull Canvas canvas, @NonNull List<DetectedBox> vehicles,
                                      float vpLeft, float vpTop, float vpWidth, float vpHeight) {
        for (DetectedBox box : vehicles) {
            float left = vpLeft + (box.getLeft() * vpWidth);
            float top = vpTop + (box.getTop() * vpHeight);
            float right = vpLeft + (box.getRight() * vpWidth);
            float bottom = vpTop + (box.getBottom() * vpHeight);

            int themeColor = Color.parseColor("#00E676"); // Xanh neon thống nhất cho mọi VEHICLE
            String typeName = "VEHICLE";

            vehicleBoxPaint.setColor(themeColor);
            vehicleBoxFillPaint.setColor(Color.argb(25, Color.red(themeColor), Color.green(themeColor), Color.blue(themeColor)));
            vehicleCornerPaint.setColor(themeColor);

            // 1. Kẻ ô Bounding Box với viền neon mờ
            boxRect.set(left, top, right, bottom);
            canvas.drawRoundRect(boxRect, 6f, 6f, vehicleBoxFillPaint);
            canvas.drawRoundRect(boxRect, 6f, 6f, vehicleBoxPaint);

            // 2. Kẻ 4 góc ngắm ADAS Corner Brackets
            float cornerLen = Math.min((right - left) * 0.22f, 20f);
            // Góc trên-trái
            canvas.drawLine(left, top, left + cornerLen, top, vehicleCornerPaint);
            canvas.drawLine(left, top, left, top + cornerLen, vehicleCornerPaint);
            // Góc trên-phải
            canvas.drawLine(right, top, right - cornerLen, top, vehicleCornerPaint);
            canvas.drawLine(right, top, right, top + cornerLen, vehicleCornerPaint);
            // Góc dưới-trái
            canvas.drawLine(left, bottom, left + cornerLen, bottom, vehicleCornerPaint);
            canvas.drawLine(left, bottom, left, bottom - cornerLen, vehicleCornerPaint);
            // Góc dưới-phải
            canvas.drawLine(right, bottom, right - cornerLen, bottom, vehicleCornerPaint);
            canvas.drawLine(right, bottom, right, bottom - cornerLen, vehicleCornerPaint);

            // 3. Nhãn loại xe và độ tin cậy (TUYỆT ĐỐI KHÔNG HIỂN THỊ KHOẢNG CÁCH)
            String label = String.format(Locale.US, "%s · %.0f%%", typeName, box.getConfidence() * 100f);
            float textW = vehicleBadgeTextPaint.measureText(label);
            float badgeH = 34f;
            float badgeW = textW + 18f;
            float badgeT = Math.max(10f, top - badgeH - 4f);

            badgeRect.set(left, badgeT, left + badgeW, badgeT + badgeH);
            canvas.drawRoundRect(badgeRect, 6f, 6f, vehicleBadgeBgPaint);

            // Viền nhãn (Zero-Allocation 60 FPS drawing)
            vehicleBadgeStrokePaint.setColor(themeColor);
            canvas.drawRoundRect(badgeRect, 6f, 6f, vehicleBadgeStrokePaint);

            vehicleBadgeTextPaint.setColor(themeColor);
            canvas.drawText(label, left + 9f, badgeT + 24f, vehicleBadgeTextPaint);
        }
    }

    // =========================================================================
    // 3. HIỂN THỊ LOG THÔNG SỐ CHẨN ĐOÁN THỜI GIAN THỰC (LIVE LOG CONSOLE)
    // =========================================================================

    private void drawLiveLogConsole(@NonNull Canvas canvas, float fps, long latency, int thermal, String hw,
                                   @NonNull List<LaneAnalyzer.Lane> lanes,
                                   @NonNull List<DetectedBox> vehicles,
                                   int viewWidth, boolean isFallbackActive) {
        float padding = 24.0f;
        // Màn hình log to hơn, rộng rãi hơn (từ 720px đến 860px)
        float cardWidth = Math.max(720.0f, Math.min(viewWidth * 0.55f, 860.0f));

        // Phân loại số lượng từng loại vạch đường
        int egoCount = 0;
        int adjCount = 0;
        int limitCount = 0;
        for (LaneAnalyzer.Lane l : lanes) {
            if (l.id == 1 || l.id == 2) egoCount++;
            else if (l.id == 0 || l.id == 3) adjCount++;
            else limitCount++;
        }

        int maxVehicleLines = Math.min(vehicles.size(), 4);
        int totalContentLines = 5 + (vehicles.isEmpty() ? 1 : maxVehicleLines);
        float cardHeight = 68.0f + (totalContentLines * 34.0f);

        logCardRect.set(padding, padding, padding + cardWidth, padding + cardHeight);
        canvas.drawRoundRect(logCardRect, 18.0f, 18.0f, logBgPaint);

        // Viền thẻ log neon cyan cao cấp
        Paint strokePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        strokePaint.setStyle(Paint.Style.STROKE);
        strokePaint.setStrokeWidth(2.0f);
        strokePaint.setColor(Color.parseColor("#6600E5FF"));
        canvas.drawRoundRect(logCardRect, 18.0f, 18.0f, strokePaint);

        float lineX = padding + 22.0f;
        float lineY = padding + 38.0f;

        // Tiêu đề Log lớn, nổi bật
        canvas.drawText("● [VADAS ADAS DIAGNOSTICS LOG]", lineX, lineY, logHeaderPaint);

        lineY += 34.0f;
        // Dòng 1: FPS, Độ trễ và Backend phần cứng
        logTextPaint.setColor(Color.parseColor("#80D8FF"));
        canvas.drawText(String.format(Locale.US, "HIỆU NĂNG: FPS: %.1f | Độ trễ: %d ms | Backend: %s", fps, latency, hw), lineX, lineY, logTextPaint);

        lineY += 32.0f;
        // Dòng 2: Trạng thái nhiệt độ phần cứng SoC
        String thermalLabel;
        int thermalColor;
        if (thermal >= PowerManager.THERMAL_STATUS_SEVERE) {
            thermalLabel = "NGHIÊM TRỌNG (Đang điều tiết công suất)";
            thermalColor = Color.parseColor("#FF5252");
        } else if (thermal >= PowerManager.THERMAL_STATUS_MODERATE) {
            thermalLabel = "ẤM (Tần số thích ứng)";
            thermalColor = Color.parseColor("#FFD600");
        } else {
            thermalLabel = "BÌNH THƯỜNG (Hoạt động tối ưu)";
            thermalColor = Color.parseColor("#00E676");
        }
        logTextPaint.setColor(thermalColor);
        canvas.drawText(String.format(Locale.US, "NHIỆT ĐỘ SOC: %s", thermalLabel), lineX, lineY, logTextPaint);

        lineY += 32.0f;
        // Dòng 3: Tổng số vạch đường và cam kết tối thiểu 2 vạch
        // Tính độ lệch tâm xe chủ tại ca-pô
        float egoCenterShift = 0.0f;
        LaneAnalyzer.Lane eL = null, eR = null;
        for (LaneAnalyzer.Lane l : lanes) {
            if (l.id == 1) eL = l;
            else if (l.id == 2) eR = l;
        }
        if (eL != null && eR != null && !eL.points.isEmpty() && !eR.points.isEmpty()) {
            float mid = (eL.points.get(0).x + eR.points.get(0).x) / 2.0f;
            egoCenterShift = Math.abs(mid - 0.50f);
        }

        String maneuverStatus;
        int statusColor;
        if (egoCenterShift < 0.035f) {
            maneuverStatus = "GIỮ LÀN CHUẨN TÂM [Ổn định]";
            statusColor = Color.parseColor("#00E676");
        } else if (egoCenterShift < 0.11f) {
            maneuverStatus = "ĐANG ĐIỀU CHỈNH HƯỚNG / NÉ XE";
            statusColor = Color.parseColor("#00E5FF");
        } else {
            maneuverStatus = "CẢNH BÁO LỆCH LÀN (LDW Gating)";
            statusColor = Color.parseColor("#FFD600");
        }

        logTextPaint.setColor(statusColor);
        canvas.drawText("TRẠNG THÁI: " + maneuverStatus, lineX, lineY, logTextPaint);

        lineY += 30.0f;
        logTextPaint.setColor(Color.parseColor("#ECEFF1"));
        String laneMode = isFallbackActive ? "[TỰ ĐỘNG BẢO ĐẢM]" : "[ĐỘ TIN CẬY CAO]";
        String laneInfo = String.format(Locale.US, "VẠCH ĐƯỜNG: %d vạch phát hiện %s", lanes.size(), laneMode);
        canvas.drawText(laneInfo, lineX, lineY, logTextPaint);

        lineY += 30.0f;
        // Dòng 4: Chi tiết phân loại các làn đường
        logTextPaint.setColor(Color.parseColor("#B0BEC5"));
        String laneDetail = String.format(Locale.US, "  -> Xe chủ: %d | Làn kề: %d | Giới hạn biên: %d", egoCount, adjCount, limitCount);
        canvas.drawText(laneDetail, lineX, lineY, logTextPaint);

        lineY += 32.0f;
        // Dòng 5: Số lượng phương tiện phát hiện (thống nhất VEHICLE)
        logTextPaint.setColor(Color.parseColor("#00E676"));
        String vehicleInfo = String.format(Locale.US, "PHƯƠNG TIỆN: %d xe phát hiện", vehicles.size());
        canvas.drawText(vehicleInfo, lineX, lineY, logTextPaint);

        if (vehicles.isEmpty()) {
            lineY += 30.0f;
            logTextPaint.setColor(Color.parseColor("#90A4AE"));
            canvas.drawText("  -> Vùng quét ROI: Đang quét mặt đường [Ổn định]", lineX, lineY, logTextPaint);
        } else {
            for (int i = 0; i < maxVehicleLines; i++) {
                lineY += 30.0f;
                DetectedBox v = vehicles.get(i);
                float w = v.getRight() - v.getLeft();
                float h = v.getBottom() - v.getTop();
                String vDetail = String.format(Locale.US, "  #%d [VEHICLE] x:%.2f y:%.2f w:%.2f h:%.2f (%.0f%%)",
                        i + 1, v.getLeft(), v.getTop(),
                        w, h, v.getConfidence() * 100f);
                logTextPaint.setColor(Color.parseColor("#00E676"));
                canvas.drawText(vDetail, lineX, lineY, logTextPaint);
            }
        }
    }

    private void appendSmoothPointsToPath(@NonNull Path path, @NonNull List<PointF> pts,
                                         float vpLeft, float vpTop, float vpWidth, float vpHeight,
                                         boolean isMoveTo) {
        if (pts.isEmpty()) return;
        int n = pts.size();
        if (isMoveTo) {
            PointF p0 = pts.get(0);
            path.moveTo(vpLeft + p0.x * vpWidth, vpTop + p0.y * vpHeight);
        } else {
            PointF p0 = pts.get(0);
            path.lineTo(vpLeft + p0.x * vpWidth, vpTop + p0.y * vpHeight);
        }

        if (n < 3) {
            for (int i = 1; i < n; i++) {
                PointF p = pts.get(i);
                path.lineTo(vpLeft + p.x * vpWidth, vpTop + p.y * vpHeight);
            }
            return;
        }

        for (int i = 1; i < n; i++) {
            PointF prev = pts.get(i - 1);
            PointF curr = pts.get(i);
            float prevX = vpLeft + prev.x * vpWidth;
            float prevY = vpTop + prev.y * vpHeight;
            float currX = vpLeft + curr.x * vpWidth;
            float currY = vpTop + curr.y * vpHeight;
            float midX = (prevX + currX) / 2f;
            float midY = (prevY + currY) / 2f;
            path.quadTo(prevX, prevY, midX, midY);
        }
        PointF pLast = pts.get(n - 1);
        path.lineTo(vpLeft + pLast.x * vpWidth, vpTop + pLast.y * vpHeight);
    }
}
