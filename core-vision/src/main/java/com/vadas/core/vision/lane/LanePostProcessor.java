package com.vadas.core.vision.lane;

import android.graphics.PointF;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Xử lý hậu kỳ (Post-processing) và Nội suy toán học đường cong làn đường:
 * - Giải mã (Decode) tensor đầu ra từ mô hình phát hiện làn đường (Ultra-Fast-Lane-Detection / OpenPilot / YOLOP).
 * - Sử dụng thuật toán Softmax Gridding Expectation để trích xuất tọa độ điểm mốc (row anchors).
 * - Hồi quy đa thức bình phương tối thiểu (Ordinary Least Squares Polynomial Fitting bậc 2 hoặc 3)
 *   theo phương $x = f(y) = a_0 + a_1 y + a_2 y^2 + a_3 y^3$, giải hệ phương trình chuẩn qua Gaussian Elimination.
 * - Nội suy các điểm cong liên tục (Smooth Continuous Curves) để phục vụ hiển thị AR HUD và đánh giá LDW.
 */
public class LanePostProcessor {

    private static final String TAG = "LanePostProcessor";

    // Row anchors chuẩn của CULane dataset (18 hàng pixel trên ảnh 800x288)
    public static final int[] DEFAULT_CULANE_ROW_ANCHORS = new int[]{
            121, 131, 141, 150, 160, 170, 180, 189, 199, 209, 219, 228, 238, 248, 258, 267, 277, 287
    };

    private final int numGridCells;   // Số lượng cột gridding (e.g. 201: 200 grid + 1 no-lane)
    private final int numRowAnchors;  // Số lượng hàng neo (e.g. 18 hoặc 56)
    private final int numLanes;       // Số làn tối đa mô hình xuất ra (thường là 4: 0, 1, 2, 3)
    private final int[] rowAnchors;   // Tọa độ y pixel trên ảnh đầu vào mô hình (e.g. 288px)
    private final int inputWidth;
    private final int inputHeight;
    private final int polynomialDegree; // Bậc đa thức hồi quy (2 hoặc 3)

    /**
     * Đại diện cho kết quả của một vạch làn đường đã được giải mã và làm mượt bằng hồi quy đa thức.
     */
    public static class LaneResult {
        public final int laneId;                 // 0: Outer Left, 1: Ego Left, 2: Ego Right, 3: Outer Right
        public final List<PointF> rawPoints;     // Điểm mốc thô từ tensor decode (tọa độ chuẩn hóa [0.0 - 1.0])
        public final List<PointF> smoothedCurve; // Các điểm cong nội suy liên tục (tọa độ chuẩn hóa [0.0 - 1.0])
        public final double[] polyCoefficients;  // Hệ số đa thức [a0, a1, a2, ...] với x = sum(a_k * y^k)
        public final float confidence;           // Độ tin cậy của làn (tỷ lệ row anchors phát hiện được)
        public final double rSquared;            // Hệ số xác định R^2 đo độ khớp của đường cong hồi quy

        public LaneResult(int laneId,
                          @NonNull List<PointF> rawPoints,
                          @NonNull List<PointF> smoothedCurve,
                          @Nullable double[] polyCoefficients,
                          float confidence,
                          double rSquared) {
            this.laneId = laneId;
            this.rawPoints = rawPoints;
            this.smoothedCurve = smoothedCurve;
            this.polyCoefficients = polyCoefficients;
            this.confidence = confidence;
            this.rSquared = rSquared;
        }

        /**
         * Tính tọa độ X (chuẩn hóa 0.0 - 1.0) tại một độ cao Y bất kỳ bằng phương trình đa thức đã fit.
         */
        public float evaluateXAtNormalizedY(float y) {
            if (polyCoefficients == null || polyCoefficients.length == 0) {
                if (!smoothedCurve.isEmpty()) {
                    return smoothedCurve.get(smoothedCurve.size() - 1).x;
                }
                return 0.5f;
            }
            double x = 0.0;
            double yPower = 1.0;
            for (double coef : polyCoefficients) {
                x += coef * yPower;
                yPower *= y;
            }
            return (float) Math.max(0.0, Math.min(1.0, x));
        }
    }

    public LanePostProcessor() {
        this(201, 18, 4, DEFAULT_CULANE_ROW_ANCHORS, 800, 288, 2);
    }

    public LanePostProcessor(int numGridCells,
                             int numRowAnchors,
                             int numLanes,
                             int[] rowAnchors,
                             int inputWidth,
                             int inputHeight,
                             int polynomialDegree) {
        this.numGridCells = numGridCells;
        this.numRowAnchors = numRowAnchors;
        this.numLanes = numLanes;
        this.rowAnchors = rowAnchors != null ? rowAnchors : DEFAULT_CULANE_ROW_ANCHORS;
        this.inputWidth = inputWidth;
        this.inputHeight = inputHeight;
        this.polynomialDegree = Math.max(1, Math.min(3, polynomialDegree)); // Bậc 2 hoặc 3
    }

    /**
     * Giải mã tensor đầu ra 4D [1, Grids, Rows, Lanes] từ mô hình Ultra-Fast-Lane-Detection
     * và tự động thực hiện hồi quy đa thức làm mượt các đường cong vạch kẻ.
     */
    @NonNull
    public List<LaneResult> processOutputTensor(@NonNull float[][][][] outputTensor) {
        List<LaneResult> results = new ArrayList<>(numLanes);
        int numValidGrids = numGridCells - 1; // Loại bỏ cột cuối cùng (no-lane background class)

        for (int laneIdx = 0; laneIdx < numLanes; laneIdx++) {
            List<PointF> rawPoints = new ArrayList<>();

            for (int rowIdx = 0; rowIdx < numRowAnchors; rowIdx++) {
                // 1. Tìm giá trị cực đại (ArgMax) trên toàn bộ grid cell (bao gồm no-lane)
                float maxVal = -Float.MAX_VALUE;
                int maxIdx = -1;

                for (int gridIdx = 0; gridIdx < numGridCells; gridIdx++) {
                    float val = outputTensor[0][gridIdx][rowIdx][laneIdx];
                    if (val > maxVal) {
                        maxVal = val;
                        maxIdx = gridIdx;
                    }
                }

                // Nếu argmax rơi vào cell cuối cùng -> Không có vạch làn tại row anchor này
                if (maxIdx >= numValidGrids) {
                    continue;
                }

                // 2. Tính Softmax và kỳ vọng toán học vị trí cột (Grid Expectation)
                // E[X] = sum(p_i * i) với p_i = exp(z_i - max) / sum(exp(z_k - max))
                float sumExp = 0.0f;
                for (int gridIdx = 0; gridIdx < numValidGrids; gridIdx++) {
                    sumExp += (float) Math.exp(outputTensor[0][gridIdx][rowIdx][laneIdx] - maxVal);
                }
                if (sumExp <= 0.0f) continue;

                float expectedCol = 0.0f;
                for (int gridIdx = 0; gridIdx < numValidGrids; gridIdx++) {
                    float prob = (float) Math.exp(outputTensor[0][gridIdx][rowIdx][laneIdx] - maxVal) / sumExp;
                    expectedCol += prob * gridIdx;
                }

                // Chuẩn hóa tọa độ X về khoảng [0.0, 1.0]
                float normalizedX = expectedCol / (float) (numValidGrids - 1);
                normalizedX = Math.max(0.0f, Math.min(1.0f, normalizedX));

                // Chuẩn hóa tọa độ Y theo Row Anchor thực tế của mô hình
                float normalizedY = (float) rowAnchors[rowIdx] / (float) inputHeight;

                rawPoints.add(new PointF(normalizedX, normalizedY));
            }

            // Chỉ chấp nhận làn nếu phát hiện ít nhất 3 điểm mốc hợp lệ
            if (rawPoints.size() >= 3) {
                float confidence = (float) rawPoints.size() / (float) numRowAnchors;

                // Sắp xếp các điểm theo chiều Y tăng dần (từ chân trời xuống đáy kính lái)
                Collections.sort(rawPoints, (p1, p2) -> Float.compare(p1.y, p2.y));

                // Lọc nhiễu hình học phối cảnh thích ứng (Adaptive Perspective Outlier Rejection)
                // Hỗ trợ trọn vẹn trạng thái xe đánh lái né xe hoặc điều chỉnh hướng chạy
                List<PointF> filteredPoints = new ArrayList<>();
                if (laneIdx <= 1) { // Left lanes: Nằm phía bên trái trục di chuyển, x <= 0.62f
                    PointF prevValid = null;
                    for (PointF pt : rawPoints) {
                        if (pt.x > 0.62f) continue; // Loại bỏ điểm thuộc làn đối diện hẳn bên phải
                        if (prevValid == null) {
                            filteredPoints.add(pt);
                            prevValid = pt;
                        } else {
                            float dy = pt.y - prevValid.y;
                            float dx = pt.x - prevValid.x;
                            // Theo phối cảnh, vạch trái khi Y tăng thì X giảm dần hoặc hơi tăng nhẹ (dx/dy <= 0.25)
                            if (dy <= 1e-4f || (dx / dy) <= 0.25f) {
                                filteredPoints.add(pt);
                                prevValid = pt;
                            }
                        }
                    }
                } else { // Right lanes: Nằm phía bên phải trục di chuyển, x >= 0.38f
                    PointF prevValid = null;
                    for (PointF pt : rawPoints) {
                        if (pt.x < 0.38f) continue; // Loại bỏ điểm thuộc làn đối diện hẳn bên trái
                        if (prevValid == null) {
                            filteredPoints.add(pt);
                            prevValid = pt;
                        } else {
                            float dy = pt.y - prevValid.y;
                            float dx = pt.x - prevValid.x;
                            // Theo phối cảnh, vạch phải khi Y tăng thì X tăng dần hoặc hơi giảm nhẹ (dx/dy >= -0.25)
                            if (dy <= 1e-4f || (dx / dy) >= -0.25f) {
                                filteredPoints.add(pt);
                                prevValid = pt;
                            }
                        }
                    }
                }

                List<PointF> ptsToFit = filteredPoints.size() >= 3 ? filteredPoints : rawPoints;

                // 3. Thực hiện hồi quy đa thức (Polynomial Regression x = f(y))
                PolynomialFitResult fitResult = fitPolynomial(ptsToFit, polynomialDegree);

                // Kiểm tra đạo hàm dx/dy trên khoảng quan sát [0.45, 0.85] để triệt tiêu võng chữ S
                boolean hasInflection = false;
                if (fitResult.coefficients != null && fitResult.coefficients.length >= 3) {
                    double a1 = fitResult.coefficients[1];
                    double a2 = fitResult.coefficients[2];
                    if (laneIdx <= 1) {
                        if (a1 + 2 * a2 * 0.50 > 0.05 || a1 + 2 * a2 * 0.75 > 0.05) {
                            hasInflection = true;
                        }
                    } else {
                        if (a1 + 2 * a2 * 0.50 < -0.05 || a1 + 2 * a2 * 0.75 < -0.05) {
                            hasInflection = true;
                        }
                    }
                }

                // Nếu số lượng điểm ít (< 5) hoặc khoảng trải Y hẹp (< 0.18) hoặc có võng bất thường, dùng hồi quy bậc 1 (tuyến tính)
                float ySpan = ptsToFit.get(ptsToFit.size() - 1).y - ptsToFit.get(0).y;
                if ((ptsToFit.size() < 5 || ySpan < 0.18f || hasInflection || fitResult.rSquared < 0.85) && polynomialDegree > 1) {
                    fitResult = fitPolynomial(ptsToFit, 1);
                }

                // 4. Sinh các điểm nội suy cong liên tục (Smooth Continuous Curve)
                List<PointF> smoothedCurve = generateSmoothedCurve(ptsToFit, fitResult.coefficients, 35);

                results.add(new LaneResult(
                        laneIdx,
                        ptsToFit,
                        smoothedCurve,
                        fitResult.coefficients,
                        confidence,
                        fitResult.rSquared
                ));
            }
        }

        return results;
    }

    /**
     * Giải mã từ mảng 1D flattened tensor (dùng cho TFLite flat buffer hoặc OpenPilot tensor slices).
     */
    @NonNull
    public List<LaneResult> processFlattenedTensor(@NonNull float[] flattenedTensor) {
        int expectedSize = numGridCells * numRowAnchors * numLanes;
        if (flattenedTensor.length < expectedSize) {
            Log.w(TAG, "Mảng tensor đầu vào không đủ kích thước (" + flattenedTensor.length + " < " + expectedSize + ")");
            return Collections.emptyList();
        }

        float[][][][] reconstructed = new float[1][numGridCells][numRowAnchors][numLanes];
        int index = 0;
        for (int g = 0; g < numGridCells; g++) {
            for (int r = 0; r < numRowAnchors; r++) {
                for (int l = 0; l < numLanes; l++) {
                    reconstructed[0][g][r][l] = flattenedTensor[index++];
                }
            }
        }
        return processOutputTensor(reconstructed);
    }

    // =========================================================================
    // THUẬT TOÁN HỒI QUY ĐA THỨC BÌNH PHƯƠNG TỐI THIỂU (ORDINARY LEAST SQUARES)
    // =========================================================================

    public static class PolynomialFitResult {
        public final double[] coefficients; // [a0, a1, a2, ...]
        public final double rSquared;

        public PolynomialFitResult(double[] coefficients, double rSquared) {
            this.coefficients = coefficients;
            this.rSquared = rSquared;
        }
    }

    /**
     * Hồi quy đa thức bình phương tối thiểu (OLS):
     * Trong phối cảnh camera xe hơi, vạch làn có xu hướng thẳng đứng (dọc theo trục Y từ chân trời xuống đáy ảnh).
     * Do đó, ta mô hình hóa X như một hàm của Y:
     *   x = a0 + a1 * y + a2 * y^2 (+ a3 * y^3)
     * Điều này loại bỏ hoàn toàn hiện tượng suy biến đạo hàm thẳng đứng (dy/dx -> vô cực) khi xe chạy thẳng!
     */
    /**
     * Hồi quy đa thức bình phương tối thiểu (OLS) cho danh sách PointF.
     */
    @NonNull
    public static PolynomialFitResult fitPolynomial(@NonNull List<PointF> points, int degree) {
        int n = points.size();
        double[] xVals = new double[n];
        double[] yVals = new double[n];
        for (int i = 0; i < n; i++) {
            PointF pt = points.get(i);
            xVals[i] = pt.x;
            yVals[i] = pt.y;
        }
        return fitPolynomial(xVals, yVals, degree);
    }

    /**
     * Hồi quy đa thức bình phương tối thiểu (OLS) trên các mảng tọa độ số thực thuần túy:
     *   x = a0 + a1 * y + a2 * y^2 (+ a3 * y^3)
     * Hoàn toàn độc lập với Android runtime framework, hỗ trợ 100% JVM Unit Test.
     */
    @NonNull
    public static PolynomialFitResult fitPolynomial(@NonNull double[] xVals, @NonNull double[] yVals, int degree) {
        int n = Math.min(xVals.length, yVals.length);
        if (n <= degree) {
            degree = Math.max(1, n - 1);
        }

        int m = degree + 1; // Số lượng hệ số [a0 ... a_degree]

        // Xây dựng ma trận Vandermonde thu gọn cho phương trình chuẩn: (Y^T * Y) * A = Y^T * X
        double[][] normalMatrix = new double[m][m];
        double[] rhsVector = new double[m];

        // Tính các tổng lũy thừa: sum(y^k) với k từ 0 đến 2 * degree
        double[] sumYPowers = new double[2 * degree + 1];
        for (int i = 0; i < n; i++) {
            double yVal = yVals[i];
            double currentPower = 1.0;
            for (int p = 0; p <= 2 * degree; p++) {
                sumYPowers[p] += currentPower;
                currentPower *= yVal;
            }
        }

        // Điền vào ma trận phương trình chuẩn C[j][k] = sum(y^(j+k))
        for (int j = 0; j < m; j++) {
            for (int k = 0; k < m; k++) {
                normalMatrix[j][k] = sumYPowers[j + k];
            }
        }

        // Điền vào vế phải d[j] = sum(x * y^j)
        for (int i = 0; i < n; i++) {
            double yVal = yVals[i];
            double xVal = xVals[i];
            double currentPower = 1.0;
            for (int j = 0; j < m; j++) {
                rhsVector[j] += xVal * currentPower;
                currentPower *= yVal;
            }
        }

        // Giải hệ phương trình tuyến tính C * A = d bằng phép khử Gauss với Partial Pivoting
        double[] coefficients = solveLinearSystemGauss(normalMatrix, rhsVector);
        if (coefficients == null) {
            // Trường hợp suy biến: Fallback tuyến tính bậc 1 đơn giản
            double x0 = xVals[0], y0 = yVals[0];
            double x1 = xVals[n - 1], y1 = yVals[n - 1];
            double slope = (x1 - x0) / Math.max(1e-5, (y1 - y0));
            double intercept = x0 - slope * y0;
            coefficients = new double[]{intercept, slope, 0.0};
        }

        // Tính hệ số xác định R^2 (Coefficient of Determination)
        double meanX = 0.0;
        for (int i = 0; i < n; i++) meanX += xVals[i];
        meanX /= n;

        double ssTot = 0.0;
        double ssRes = 0.0;
        for (int i = 0; i < n; i++) {
            double yVal = yVals[i];
            double xVal = xVals[i];
            double xPred = 0.0;
            double yPow = 1.0;
            for (double coef : coefficients) {
                xPred += coef * yPow;
                yPow *= yVal;
            }
            ssTot += (xVal - meanX) * (xVal - meanX);
            ssRes += (xVal - xPred) * (xVal - xPred);
        }

        double rSquared = ssTot > 1e-9 ? Math.max(0.0, 1.0 - (ssRes / ssTot)) : 1.0;
        return new PolynomialFitResult(coefficients, rSquared);
    }

    /**
     * Giải hệ phương trình đại số tuyến tính A * x = b bằng thuật toán Khử Gauss có chọn phần tử trội
     * (Gaussian Elimination with Partial Pivoting) hoàn toàn bằng Java thuần, tối ưu zero-dependency.
     */
    @Nullable
    private static double[] solveLinearSystemGauss(double[][] A, double[] b) {
        int n = b.length;
        double[][] augmented = new double[n][n + 1];

        for (int i = 0; i < n; i++) {
            System.arraycopy(A[i], 0, augmented[i], 0, n);
            augmented[i][n] = b[i];
        }

        // Khử xuôi (Forward Elimination)
        for (int p = 0; p < n; p++) {
            // Tìm phần tử trội theo cột p
            int maxRow = p;
            for (int i = p + 1; i < n; i++) {
                if (Math.abs(augmented[i][p]) > Math.abs(augmented[maxRow][p])) {
                    maxRow = i;
                }
            }

            // Đổi hàng
            double[] temp = augmented[p];
            augmented[p] = augmented[maxRow];
            augmented[maxRow] = temp;

            // Kiểm tra ma trận suy biến (singular)
            if (Math.abs(augmented[p][p]) < 1e-12) {
                return null;
            }

            // Khử các hàng bên dưới
            for (int i = p + 1; i < n; i++) {
                double alpha = augmented[i][p] / augmented[p][p];
                for (int j = p; j <= n; j++) {
                    augmented[i][j] -= alpha * augmented[p][j];
                }
            }
        }

        // Thế ngược (Back Substitution)
        double[] x = new double[n];
        for (int i = n - 1; i >= 0; i--) {
            double sum = 0.0;
            for (int j = i + 1; j < n; j++) {
                sum += augmented[i][j] * x[j];
            }
            x[i] = (augmented[i][n] - sum) / augmented[i][i];
        }

        return x;
    }

    /**
     * Sinh danh sách các điểm cong mịn từ phương trình đa thức nội suy:
     * Tạo 'numSamples' điểm chia đều từ y_min (chân trời) đến y_max (đáy kính lái / capô xe).
     */
    @NonNull
    public static List<PointF> generateSmoothedCurve(@NonNull List<PointF> rawPoints,
                                                    @NonNull double[] coefficients,
                                                    int numSamples) {
        if (rawPoints.isEmpty()) return Collections.emptyList();

        float minY = rawPoints.get(0).y;
        float maxY = rawPoints.get(rawPoints.size() - 1).y;
        maxY = Math.min(0.95f, maxY);

        return generateSmoothedCurve(minY, maxY, coefficients, numSamples);
    }

    /**
     * Sinh các điểm nội suy cong liên tục dựa trên khoảng Y [minY, maxY] và hệ số đa thức.
     */
    @NonNull
    public static List<PointF> generateSmoothedCurve(float minY,
                                                    float maxY,
                                                    @NonNull double[] coefficients,
                                                    int numSamples) {
        List<PointF> curve = new ArrayList<>(numSamples);
        float step = (maxY - minY) / (float) Math.max(1, numSamples - 1);

        for (int i = 0; i < numSamples; i++) {
            float y = minY + (i * step);
            double xVal = 0.0;
            double yPower = 1.0;
            for (double coef : coefficients) {
                xVal += coef * yPower;
                yPower *= y;
            }
            float clampedX = (float) Math.max(0.0, Math.min(1.0, xVal));
            curve.add(new PointF(clampedX, y));
        }

        return curve;
    }
}
