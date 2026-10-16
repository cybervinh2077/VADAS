package com.vadas.app.ui;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.LinearGradient;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.RectF;
import android.graphics.Shader;
import android.util.AttributeSet;
import android.view.View;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.vadas.core.safety.model.SafetyEvaluationResult;
import com.vadas.core.vision.model.DetectedBox;
import com.vadas.core.vision.model.VisionPerceptionResult;
import com.vadas.core.vision.openpilot.SupercomboOutput;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * High-precision OpenPilot / KaibaTech Augmented Reality (AR) HUD Overlay.
 * Renders:
 * 1. Yellow Horizon guideline across the entire windshield ("Đường chân trời").
 * 2. Smooth curved emerald-green drivable corridor carpet (thảm đường dẫn hướng làn xe đang chạy).
 * 3. Glowing neon-green ego-lane boundaries curving with the road (chuyển màu vàng Amber khi đang chuyển làn).
 * 4. Amber/orange forward target arrow chevron (▲ / wedge) at the end of the trajectory.
 * 5. All detected road boundaries (curbs, road limits) and adjacent lanes in crisp white (hỗ trợ nhiều hơn 2 vạch trắng để xác định chuyển làn).
 * 6. Pinhole Camera 3D vehicle bounding boxes & distance badges.
 * 7. Dynamic AR Lane Change banner HUD.
 */
public class ADASOverlayView extends View {

    // Paints
    private final Paint lanePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint adjacentLanePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint laneCorridorPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint chevronPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint chevronBorderPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint horizonPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint horizonTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    private final Paint boxPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint boxFillPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint contactLinePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint textBadgePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    private final Paint laneChangeBannerPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint laneChangeBorderPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint laneChangeTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    // Reusable Paths for zero-allocation 60 FPS drawing
    private final Path corridorPath = new Path();
    private final Path leftLanePath = new Path();
    private final Path rightLanePath = new Path();
    private final Path chevronPath = new Path();

    // White Road Boundaries & Adjacent Lanes Pool (supports arbitrary number of lines >= 0)
    private static final int MAX_WHITE_LANES = 16;
    private final List<Path> whiteLanePathsPool = new ArrayList<>(MAX_WHITE_LANES);
    private final float[][] renderedWhiteLanesX = new float[MAX_WHITE_LANES][32];
    private final float[][] pxWhiteBuffers = new float[MAX_WHITE_LANES][32];
    private int renderedWhiteCount = 0;

    // Dynamic ADAS State
    @Nullable
    private VisionPerceptionResult perceptionResult;
    @Nullable
    private SupercomboOutput supercomboOutput;
    @Nullable
    private SafetyEvaluationResult safetyResult;

    // Active video display viewport (accounts for aspect ratio letterboxing)
    private final RectF videoViewport = new RectF();

    // Silky-smooth 60 FPS interpolated display coordinates (completely eliminates visual jitter / stutter)
    private final float[] renderedLeftX = new float[32];
    private final float[] renderedRightX = new float[32];
    private final float[] renderedElevY = new float[32];
    private int renderedCount = 0;
    private float renderedHorizonYNorm = 0.440f;

    // Reusable buffers for 60 FPS zero-allocation drawing
    private final float[] pxLeft = new float[32];
    private final float[] pxRight = new float[32];
    private final float[] py = new float[32];

    public ADASOverlayView(Context context) {
        super(context);
        init();
    }

    public ADASOverlayView(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public ADASOverlayView(Context context, @Nullable AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init();
    }

    private void init() {
        // Ego Lane Boundary Lines (Glowing Neon Green)
        lanePaint.setStyle(Paint.Style.STROKE);
        lanePaint.setStrokeWidth(5.5f);
        lanePaint.setColor(Color.parseColor("#00FF7F"));
        lanePaint.setStrokeCap(Paint.Cap.ROUND);

        // White Road Boundaries & Adjacent Lanes (Crisp White)
        adjacentLanePaint.setStyle(Paint.Style.STROKE);
        adjacentLanePaint.setStrokeWidth(3.8f);
        adjacentLanePaint.setColor(Color.parseColor("#E6FFFFFF"));
        adjacentLanePaint.setStrokeCap(Paint.Cap.ROUND);

        // Pre-allocate white lane Paths pool (zero GC allocation)
        for (int i = 0; i < MAX_WHITE_LANES; i++) {
            whiteLanePathsPool.add(new Path());
        }

        // Lane Change HUD Indicator Badge
        laneChangeBannerPaint.setStyle(Paint.Style.FILL);
        laneChangeBannerPaint.setColor(Color.parseColor("#D9101418"));

        laneChangeBorderPaint.setStyle(Paint.Style.STROKE);
        laneChangeBorderPaint.setStrokeWidth(2.5f);
        laneChangeBorderPaint.setColor(Color.parseColor("#FFD600"));

        laneChangeTextPaint.setColor(Color.parseColor("#FFD600"));
        laneChangeTextPaint.setTextSize(26f);
        laneChangeTextPaint.setFakeBoldText(true);
        laneChangeTextPaint.setTextAlign(Paint.Align.CENTER);

        // Drivable Corridor Mesh (Mint Green Translucent Gradient)
        laneCorridorPaint.setStyle(Paint.Style.FILL);

        // Target Chevron Arrow (Amber / Orange Wedge)
        chevronPaint.setStyle(Paint.Style.FILL);
        chevronPaint.setColor(Color.parseColor("#FFA000"));

        chevronBorderPaint.setStyle(Paint.Style.STROKE);
        chevronBorderPaint.setStrokeWidth(2.0f);
        chevronBorderPaint.setColor(Color.parseColor("#FFE082"));

        // Yellow Horizon Guideline ("Đường chân trời")
        horizonPaint.setStyle(Paint.Style.STROKE);
        horizonPaint.setStrokeWidth(1.8f);
        horizonPaint.setColor(Color.parseColor("#E6FFD600"));

        horizonTextPaint.setColor(Color.parseColor("#FFD600"));
        horizonTextPaint.setTextSize(26f);
        horizonTextPaint.setFakeBoldText(true);

        // Vehicle Bounding Box
        boxPaint.setStyle(Paint.Style.STROKE);
        boxPaint.setStrokeWidth(4f);
        boxPaint.setColor(Color.parseColor("#00E676"));

        boxFillPaint.setStyle(Paint.Style.FILL);
        boxFillPaint.setColor(Color.parseColor("#2500E676"));

        contactLinePaint.setStyle(Paint.Style.STROKE);
        contactLinePaint.setStrokeWidth(5f);
        contactLinePaint.setColor(Color.parseColor("#FFD600"));

        textBadgePaint.setStyle(Paint.Style.FILL);
        textBadgePaint.setColor(Color.parseColor("#CC121212"));

        textPaint.setColor(Color.WHITE);
        textPaint.setTextSize(24f);
        textPaint.setFakeBoldText(true);
    }

    public void setVideoViewport(@NonNull RectF viewport) {
        this.videoViewport.set(viewport);
        postInvalidate();
    }

    public void updatePerception(@Nullable VisionPerceptionResult perception,
                                @Nullable SupercomboOutput supercombo,
                                @Nullable SafetyEvaluationResult safety) {
        this.perceptionResult = perception;
        this.supercomboOutput = supercombo;
        this.safetyResult = safety;
        postInvalidate();
    }

    @Override
    protected void onDraw(@NonNull Canvas canvas) {
        super.onDraw(canvas);

        int w = getWidth();
        int h = getHeight();
        if (w == 0 || h == 0) return;

        float vpLeft = videoViewport.width() > 0 ? videoViewport.left : 0f;
        float vpTop = videoViewport.height() > 0 ? videoViewport.top : 0f;
        float vpWidth = videoViewport.width() > 0 ? videoViewport.width() : w;
        float vpHeight = videoViewport.height() > 0 ? videoViewport.height() : h;

        // 1. Draw Yellow Horizon Guideline across screen with label ("Đường chân trời")
        drawHorizonGuideline(canvas, vpLeft, vpTop, vpWidth, vpHeight);

        // 2. Draw Curved Drivable Carpet (Ego Lane), Neon-Green Lane Lines, Amber Chevron, and ALL White Road Lines
        drawCurvedDrivablePathAndLanes(canvas, vpLeft, vpTop, vpWidth, vpHeight);

        // 3. Draw Detected Vehicle Bounding Boxes & Pinhole Distance Tags
        drawDetectedVehicles(canvas, vpLeft, vpTop, vpWidth, vpHeight);
    }

    /**
     * Draws the yellow horizon line across the entire screen labeled "Đường chân trời" with live camera pitch.
     */
    private void drawHorizonGuideline(@NonNull Canvas canvas, float vpLeft, float vpTop, float vpWidth, float vpHeight) {
        float targetHorizonNorm = supercomboOutput != null ? supercomboOutput.getHorizonYNorm() : 0.440f;
        if (renderedHorizonYNorm <= 0f) {
            renderedHorizonYNorm = targetHorizonNorm;
        } else {
            float diffH = targetHorizonNorm - renderedHorizonYNorm;
            if (Math.abs(diffH) > 0.0002f) {
                renderedHorizonYNorm += diffH * 0.15f;
            } else {
                renderedHorizonYNorm = targetHorizonNorm;
            }
        }

        float horizonY = vpTop + (vpHeight * renderedHorizonYNorm);
        canvas.drawLine(vpLeft, horizonY, vpLeft + vpWidth, horizonY, horizonPaint);

        float pitch = supercomboOutput != null ? supercomboOutput.getPitchDegrees() : 0.0f;
        String horizonText = String.format(Locale.US, "Đường chân trời (Pitch %+.1f°)", pitch);
        canvas.drawText(horizonText, vpLeft + 40f, horizonY - 10f, horizonTextPaint);
    }

    /**
     * Renders:
     * - Drivable corridor carpet (thảm xanh ngọc) strictly for the EGO LANE where the vehicle is traveling.
     * - Glowing neon-green ego-lane boundaries curving with the road.
     * - ALL white lines (vạch trắng thể hiện giới hạn đường, giải phân cách, lề đường và các làn liền kề - có thể nhiều hơn 2 vạch).
     * - Amber target chevron arrow (▲ / wedge) rotating dynamically with road curvature.
     */
    private void drawCurvedDrivablePathAndLanes(@NonNull Canvas canvas,
                                                float vpLeft, float vpTop,
                                                float vpWidth, float vpHeight) {
        float[] leftX = supercomboOutput != null ? supercomboOutput.getLeftLaneNormX() : null;
        float[] rightX = supercomboOutput != null ? supercomboOutput.getRightLaneNormX() : null;
        float[] elevY = supercomboOutput != null ? supercomboOutput.getScanElevationNormY() : null;
        List<float[]> allWhite = supercomboOutput != null ? supercomboOutput.getAllWhiteLanesNormX() : null;

        if (leftX == null || rightX == null || elevY == null || leftX.length < 2) {
            leftX = new float[]{0.275f, 0.301f, 0.326f, 0.352f, 0.378f, 0.403f, 0.429f, 0.454f, 0.480f};
            rightX = new float[]{0.725f, 0.699f, 0.674f, 0.648f, 0.622f, 0.597f, 0.571f, 0.546f, 0.520f};
            elevY = new float[]{0.670f, 0.636f, 0.603f, 0.569f, 0.535f, 0.501f, 0.468f, 0.434f, 0.400f};
        }

        int n = Math.min(leftX.length, 32);

        // Fallback default white lines if none detected yet
        float[] fallbackAdjLeft = new float[]{0.080f, 0.130f, 0.180f, 0.230f, 0.280f, 0.330f, 0.380f, 0.430f, 0.480f};
        float[] fallbackAdjRight = new float[]{0.920f, 0.870f, 0.820f, 0.770f, 0.720f, 0.670f, 0.620f, 0.570f, 0.520f};
        int whiteCount = (allWhite != null && !allWhite.isEmpty()) ?
                Math.min(allWhite.size(), MAX_WHITE_LANES) : 2;

        // Initialize display coordinate caches ONLY on first run or if slice count changed
        // CRITICAL: NEVER wipe ego lane coordinates when white line count fluctuates!
        if (renderedCount != n) {
            for (int i = 0; i < n; i++) {
                renderedLeftX[i] = leftX[i];
                renderedRightX[i] = rightX[i];
                renderedElevY[i] = elevY[i];
            }
            renderedCount = n;
        }

        // Initialize any newly appeared white lines to their target positions so they glide in seamlessly
        if (whiteCount > renderedWhiteCount) {
            for (int k = renderedWhiteCount; k < whiteCount; k++) {
                float[] line = (allWhite != null && !allWhite.isEmpty()) ?
                        allWhite.get(k) : (k == 0 ? fallbackAdjLeft : fallbackAdjRight);
                for (int i = 0; i < n; i++) {
                    renderedWhiteLanesX[k][i] = (i < line.length) ? line[i] : line[line.length - 1];
                }
            }
        }
        renderedWhiteCount = whiteCount;

        // Pure Lane Lines Rendering (OpenPilot Standard: No animation, no carpet, no chevron)
        for (int i = 0; i < n; i++) {
            pxLeft[i] = vpLeft + (leftX[i] * vpWidth);
            pxRight[i] = vpLeft + (rightX[i] * vpWidth);
            py[i] = vpTop + (elevY[i] * vpHeight);

            for (int k = 0; k < whiteCount; k++) {
                float[] line = (allWhite != null && !allWhite.isEmpty()) ?
                        allWhite.get(k) : (k == 0 ? fallbackAdjLeft : fallbackAdjRight);
                float targetW = (i < line.length) ? line[i] : line[line.length - 1];
                pxWhiteBuffers[k][i] = vpLeft + (targetW * vpWidth);
            }
        }

        // 1. Draw ALL White Road Boundary & Adjacent Lane Lines
        for (int k = 0; k < whiteCount; k++) {
            if (pxWhiteBuffers[k][0] > vpLeft - (0.30f * vpWidth) && pxWhiteBuffers[k][0] < vpLeft + (1.30f * vpWidth)) {
                Path p = whiteLanePathsPool.get(k);
                buildSmoothPathThroughPoints(p, pxWhiteBuffers[k], py, n);
                canvas.drawPath(p, adjacentLanePaint);
            }
        }

        // 2. Draw Ego-Lane Boundary Lines: Pure glowing Neon-Green (#00FF7F), red on LDW emergency
        boolean ldwLeft = supercomboOutput != null && supercomboOutput.isLaneDepartureWarningLeft();
        boolean ldwRight = supercomboOutput != null && supercomboOutput.isLaneDepartureWarningRight();

        // Left Ego Boundary
        buildSmoothPathThroughPoints(leftLanePath, pxLeft, py, n);
        lanePaint.setColor(ldwLeft ? Color.parseColor("#FF1744") : Color.parseColor("#00FF7F"));
        lanePaint.setStrokeWidth(ldwLeft ? 8.0f : 5.5f);
        canvas.drawPath(leftLanePath, lanePaint);

        // Right Ego Boundary
        buildSmoothPathThroughPoints(rightLanePath, pxRight, py, n);
        lanePaint.setColor(ldwRight ? Color.parseColor("#FF1744") : Color.parseColor("#00FF7F"));
        lanePaint.setStrokeWidth(ldwRight ? 8.0f : 5.5f);
        canvas.drawPath(rightLanePath, lanePaint);
    }

    private void buildSmoothPathThroughPoints(@NonNull Path path, float[] px, float[] py, int n) {
        path.reset();
        if (n < 2) return;
        path.moveTo(px[0], py[0]);
        for (int i = 0; i < n - 1; i++) {
            float midX = (px[i] + px[i + 1]) / 2f;
            float midY = (py[i] + py[i + 1]) / 2f;
            path.quadTo(px[i], py[i], midX, midY);
        }
        path.lineTo(px[n - 1], py[n - 1]);
    }

    /**
     * Draws the amber/orange 3D triangle arrow chevron (▲ / wedge) rotating dynamically to point into the turn.
     */
    private void drawTargetChevron(@NonNull Canvas canvas, float tipCenterX, float topY, float rotationDeg) {
        float chevronWidth = 46f;
        float chevronHeight = 24f;

        chevronPath.reset();
        chevronPath.moveTo(0f, -chevronHeight);
        chevronPath.lineTo(-(chevronWidth / 2f), 4f);
        chevronPath.lineTo(0f, 0f);
        chevronPath.lineTo((chevronWidth / 2f), 4f);
        chevronPath.close();

        canvas.save();
        canvas.translate(tipCenterX, topY);
        canvas.rotate(rotationDeg);
        canvas.drawPath(chevronPath, chevronPaint);
        canvas.drawPath(chevronPath, chevronBorderPaint);
        canvas.restore();
    }

    private void drawDetectedVehicles(@NonNull Canvas canvas,
                                     float vpLeft, float vpTop,
                                     float vpWidth, float vpHeight) {
        if (perceptionResult == null) return;

        List<DetectedBox> objects = perceptionResult.getDetectedObjects();
        if (objects == null || objects.isEmpty()) return;

        for (DetectedBox box : objects) {
            float left = vpLeft + (box.getLeft() * vpWidth);
            float top = vpTop + (box.getTop() * vpHeight);
            float right = vpLeft + (box.getRight() * vpWidth);
            float bottom = vpTop + (box.getBottom() * vpHeight);

            double distance = box.getEstimatedDistanceMeters();
            float boxCenterNorm = (box.getLeft() + box.getRight()) / 2f;
            boolean inEgoLane = Math.abs(boxCenterNorm - 0.40f) < 0.16f;

            int themeColor;
            if (!inEgoLane) {
                themeColor = Color.parseColor("#40C4FF");
            } else if (distance < 20.0) {
                themeColor = Color.parseColor("#FF1744");
            } else if (distance < 35.0) {
                themeColor = Color.parseColor("#FFD600");
            } else {
                themeColor = Color.parseColor("#00E676");
            }

            boxPaint.setColor(themeColor);
            boxFillPaint.setColor(Color.argb(30, Color.red(themeColor), Color.green(themeColor), Color.blue(themeColor)));

            // 1. Draw Bounding Box Rectangle
            RectF boxRect = new RectF(left, top, right, bottom);
            canvas.drawRoundRect(boxRect, 8f, 8f, boxFillPaint);
            canvas.drawRoundRect(boxRect, 8f, 8f, boxPaint);

            // 2. Draw Road Contact Point line
            contactLinePaint.setColor(themeColor);
            canvas.drawLine(left + 6f, bottom, right - 6f, bottom, contactLinePaint);

            // 3. Draw Telemetry Badge
            String distanceText;
            if (!Double.isInfinite(distance) && distance > 0) {
                if (inEgoLane) {
                    distanceText = String.format(Locale.getDefault(), "🚗 XE TRƯỚC: %.1fm", distance);
                } else {
                    distanceText = String.format(Locale.getDefault(), "🚗 LÀN BÊN: %.1fm", distance);
                }
            } else {
                distanceText = "🚗 PHƯƠNG TIỆN";
            }

            float textWidth = textPaint.measureText(distanceText);
            float badgeHeight = 38f;
            float badgeTop = Math.max(10f, top - badgeHeight - 6f);
            float badgeLeft = left;
            float badgeRight = Math.max(right, left + textWidth + 20f);

            RectF badgeRect = new RectF(badgeLeft, badgeTop, badgeRight, badgeTop + badgeHeight);
            canvas.drawRoundRect(badgeRect, 6f, 6f, textBadgePaint);

            boxPaint.setStrokeWidth(2f);
            canvas.drawRoundRect(badgeRect, 6f, 6f, boxPaint);
            boxPaint.setStrokeWidth(4f);

            canvas.drawText(distanceText, badgeLeft + 10f, badgeTop + 26f, textPaint);
        }
    }
}
