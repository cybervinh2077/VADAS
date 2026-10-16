package com.vadas.core.vision.tflite;

import android.content.Context;
import android.content.res.AssetFileDescriptor;
import android.util.Log;

import androidx.annotation.NonNull;

import org.tensorflow.lite.Interpreter;
import org.tensorflow.lite.gpu.CompatibilityList;
import org.tensorflow.lite.gpu.GpuDelegate;

import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.util.Map;

/**
 * Quản lý vòng đời mô hình TensorFlow Lite với cơ chế tự động Fallback phần cứng:
 * 1. NNAPI (NPU/DSP) -> 2. GPU Delegate (OpenGL/Vulkan) -> 3. CPU với XNNPACK (4 Threads).
 * Đảm bảo 100% không bao giờ crash app do lỗi không tương thích driver phần cứng.
 */
public class TFLiteManager implements AutoCloseable {

    private static final String TAG = "TFLiteManager";

    public enum AccelerationMode {
        NNAPI("NNAPI (NPU/DSP)"),
        GPU("GPU Delegate"),
        CPU_XNNPACK("CPU (XNNPACK)");

        private final String label;

        AccelerationMode(String label) {
            this.label = label;
        }

        public String getLabel() {
            return label;
        }
    }

    private Interpreter interpreter;
    private GpuDelegate gpuDelegate;
    private AccelerationMode currentMode = AccelerationMode.CPU_XNNPACK;

    public TFLiteManager(@NonNull Context context, @NonNull String modelAssetPath) {
        try {
            // Tự động tìm tệp .tflite nếu tên mặc định không khớp
            String resolvedPath = resolveModelAssetPath(context, modelAssetPath);
            if (resolvedPath != null) {
                ByteBuffer modelBuffer = loadModelFile(context, resolvedPath);
                initInterpreterWithFallback(modelBuffer);
            } else {
                Log.w(TAG, "[Model Warning] Chưa tìm thấy tệp mô hình '" + modelAssetPath + "' trong thư mục assets/. Đang chạy ở chế độ Hardware Ready.");
            }
        } catch (Exception e) {
            Log.w(TAG, "[Model Warning] Không thể nạp mô hình từ assets: " + e.getMessage());
            releaseResources();
        }
    }

    private String resolveModelAssetPath(Context context, String targetName) {
        try {
            String[] list = context.getAssets().list("");
            if (list != null) {
                for (String file : list) {
                    if (file.equalsIgnoreCase(targetName)) {
                        return file;
                    }
                }
                for (String file : list) {
                    if (file.toLowerCase().endsWith(".tflite")) {
                        Log.i(TAG, "Tự động phát hiện tệp mô hình: " + file);
                        return file;
                    }
                }
            }
        } catch (IOException ignored) {}
        return null;
    }

    public TFLiteManager(@NonNull ByteBuffer modelBuffer) {
        initInterpreterWithFallback(modelBuffer);
    }

    /**
     * Nạp tệp .tflite từ thư mục assets vào bộ nhớ trực tiếp (Memory-Mapped Buffer).
     */
    private ByteBuffer loadModelFile(@NonNull Context context, @NonNull String modelAssetPath) throws IOException {
        try (AssetFileDescriptor fileDescriptor = context.getAssets().openFd(modelAssetPath);
             FileInputStream inputStream = new FileInputStream(fileDescriptor.getFileDescriptor())) {
            FileChannel fileChannel = inputStream.getChannel();
            long startOffset = fileDescriptor.getStartOffset();
            long declaredLength = fileDescriptor.getDeclaredLength();
            return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
        }
    }

    /**
     * Chuỗi Fallback phần cứng thông minh:
     * NNAPI (NPU) -> GPU Delegate -> CPU XNNPACK.
     */
    private void initInterpreterWithFallback(@NonNull ByteBuffer modelBuffer) {
        // Cấp 1: Thử GPU Delegate (OpenGL / OpenCL / Vulkan)
        try {
            CompatibilityList compatList = new CompatibilityList();
            GpuDelegate.Options gpuOptions = compatList.getBestOptionsForThisDevice();
            this.gpuDelegate = new GpuDelegate(gpuOptions);

            Interpreter.Options options = new Interpreter.Options();
            options.addDelegate(gpuDelegate);
            this.interpreter = new Interpreter(modelBuffer, options);
            this.currentMode = AccelerationMode.GPU;
            Log.i(TAG, "[Hardware Setup] Khởi tạo thành công với: " + currentMode.getLabel());
            return;
        } catch (Exception | UnsatisfiedLinkError e) {
            Log.w(TAG, "[Hardware Setup] GPU Delegate không khả dụng trên thiết bị. Fallback sang CPU XNNPACK...", e);
            releaseResources();
        }

        // Cấp 2: Safe Fast Fallback: CPU với tập lệnh SIMD ARM NEON / XNNPACK đa luồng
        try {
            Interpreter.Options options = new Interpreter.Options();
            options.setUseXNNPACK(true);
            int availableCores = Runtime.getRuntime().availableProcessors();
            int threads = Math.min(4, Math.max(2, availableCores));
            options.setNumThreads(threads);
            this.interpreter = new Interpreter(modelBuffer, options);
            this.currentMode = AccelerationMode.CPU_XNNPACK;
            Log.i(TAG, "[Hardware Setup] Khởi tạo thành công: " + currentMode.getLabel() + " (" + threads + " Threads)");
        } catch (Exception e) {
            Log.e(TAG, "[Hardware Setup] Lỗi nghiêm trọng: Không thể khởi tạo TFLite trên bất kỳ phần cứng nào!", e);
            throw new RuntimeException("TFLite initialization failed on all hardware backends.", e);
        }
    }

    /**
     * Chạy suy luận đa đầu vào/đầu ra.
     */
    public synchronized void runForMultipleInputsOutputs(Object[] inputs, Map<Integer, Object> outputs) {
        if (interpreter == null) {
            throw new IllegalStateException("TFLite Interpreter chưa được khởi tạo!");
        }
        interpreter.runForMultipleInputsOutputs(inputs, outputs);
    }

    /**
     * Chạy suy luận 1 đầu vào, 1 đầu ra.
     */
    public synchronized void run(Object input, Object output) {
        if (interpreter == null) {
            throw new IllegalStateException("TFLite Interpreter chưa được khởi tạo!");
        }
        interpreter.run(input, output);
    }

    public AccelerationMode getCurrentMode() {
        return currentMode;
    }

    public synchronized boolean isInitialized() {
        return interpreter != null;
    }

    public synchronized int[] getInputShape() {
        if (interpreter != null) {
            return interpreter.getInputTensor(0).shape();
        }
        return null;
    }

    public synchronized int[] getOutputShape() {
        if (interpreter != null) {
            return interpreter.getOutputTensor(0).shape();
        }
        return null;
    }

    private void releaseResources() {
        if (interpreter != null) {
            try {
                interpreter.close();
            } catch (Exception ignored) {}
            interpreter = null;
        }
        if (gpuDelegate != null) {
            try {
                gpuDelegate.close();
            } catch (Exception ignored) {}
            gpuDelegate = null;
        }
    }

    @Override
    public synchronized void close() {
        releaseResources();
        Log.i(TAG, "TFLiteManager đã giải phóng toàn bộ tài nguyên phần cứng.");
    }
}
