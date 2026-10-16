package com.vadas.app.ui;

import android.Manifest;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.pm.PackageManager;
import android.content.res.AssetFileDescriptor;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.PointF;
import android.graphics.RectF;
import android.graphics.SurfaceTexture;
import android.media.MediaPlayer;
import android.os.Bundle;
import android.os.IBinder;
import android.os.PowerManager;
import android.util.Log;
import android.util.Size;
import android.view.Surface;
import android.view.TextureView;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;
import androidx.camera.core.CameraSelector;
import androidx.camera.core.ImageAnalysis;
import androidx.camera.core.Preview;
import androidx.camera.lifecycle.ProcessCameraProvider;
import androidx.camera.view.PreviewView;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.google.common.util.concurrent.ListenableFuture;
import com.vadas.app.R;
import com.vadas.app.service.VADASService;
import com.vadas.app.util.VideoAssetHelper;
import com.vadas.core.camera.model.CameraFrame;
import com.vadas.core.vision.lane.LaneAnalyzer;
import com.vadas.core.vision.model.DetectedBox;
import com.vadas.core.vision.openpilot.SupercomboOutput;
import com.vadas.core.vision.tflite.TFLiteManager;
import com.vadas.core.vision.thermal.ThermalManager;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * Main Cockpit Activity (ADAS Clean Focus):
 * - Chỉ tập trung vào 2 nhiệm vụ thị giác cốt lõi:
 *   1. Kẻ các đường vạch làn đường (Lane Lines & Corridor).
 *   2. Kẻ ô Bounding Box bao quanh các phương tiện phát hiện trên đường (Detected Vehicles).
 * - Hiển thị bảng Log thông số chẩn đoán (Live Log Console).
 * - Đã loại bỏ toàn bộ các UI không cần thiết (thẻ khoảng cách, tốc độ, nút thời tiết, watermark).
 */
public class MainActivity extends AppCompatActivity {

    private static final String TAG = "MainActivity";
    private static final int CAMERA_PERMISSION_REQUEST_CODE = 2002;
    private static final String DEFAULT_LANE_MODEL = "ultra_fast_lane.tflite";

    // View Components
    private PreviewView previewView;
    private TextureView textureVideoView;
    private ImageView ivVideoPreview;
    private OverlayView overlayView;

    // Minimal Controls
    private Button btnCameraXMode;
    private Button btnToggleLog;

    // CameraX & TFLite Adaptive Perception Pipeline
    private ExecutorService cameraExecutor;
    private TFLiteManager tfLiteManager;
    private ThermalManager thermalManager;
    private LaneAnalyzer laneAnalyzer;
    private ProcessCameraProvider cameraProvider;
    private boolean isCameraXActive = false;

    // Hardware-accelerated Video Playback (Dashcam Mode)
    private MediaPlayer mediaPlayer;
    private Surface videoSurface;
    private boolean isPlayingVideo = false;
    private int videoWidth = 1920;
    private int videoHeight = 1080;

    // Background Frame Sampler for ADAS Vision
    private ScheduledExecutorService frameSamplerExecutor;

    private VADASService vadasService;
    private boolean isBound = false;
    private SupercomboOutput currentSupercombo;

    private final ServiceConnection serviceConnection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName name, IBinder service) {
            VADASService.VADASBinder binder = (VADASService.VADASBinder) service;
            vadasService = binder.getService();
            isBound = true;
            if (laneAnalyzer != null) {
                vadasService.setLaneAnalyzer(laneAnalyzer);
            }
            vadasService.setPipelineRunning(true);
            observeServiceState();
            if (isPlayingVideo && frameSamplerExecutor == null) {
                startPerceptionFrameSampling();
            }
        }

        @Override
        public void onServiceDisconnected(ComponentName name) {
            vadasService = null;
            isBound = false;
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        previewView = findViewById(R.id.previewView);
        textureVideoView = findViewById(R.id.textureVideoView);
        ivVideoPreview = findViewById(R.id.ivVideoPreview);
        overlayView = findViewById(R.id.overlayView);

        btnCameraXMode = findViewById(R.id.btnCameraXMode);
        btnToggleLog = findViewById(R.id.btnToggleLog);

        setupTextureView();
        initLaneVisionPipeline();

        btnCameraXMode.setOnClickListener(v -> toggleCameraXMode());
        btnToggleLog.setOnClickListener(v -> overlayView.toggleLogVisibility());
    }

    private void setupTextureView() {
        textureVideoView.setSurfaceTextureListener(new TextureView.SurfaceTextureListener() {
            @Override
            public void onSurfaceTextureAvailable(@NonNull SurfaceTexture surface, int width, int height) {
                videoSurface = new Surface(surface);
                adjustAspectRatio(videoWidth, videoHeight);
                startHardwareVideoPlayback();
            }

            @Override
            public void onSurfaceTextureSizeChanged(@NonNull SurfaceTexture surface, int width, int height) {
                adjustAspectRatio(videoWidth, videoHeight);
            }

            @Override
            public boolean onSurfaceTextureDestroyed(@NonNull SurfaceTexture surface) {
                if (videoSurface != null) {
                    videoSurface.release();
                    videoSurface = null;
                }
                return true;
            }

            @Override
            public void onSurfaceTextureUpdated(@NonNull SurfaceTexture surface) {
            }
        });
    }

    @Override
    protected void onStart() {
        super.onStart();
        Intent intent = new Intent(this, VADASService.class);
        bindService(intent, serviceConnection, Context.BIND_AUTO_CREATE);
    }

    @Override
    protected void onStop() {
        super.onStop();
        stopHardwareVideoPlayback();
        if (isBound) {
            unbindService(serviceConnection);
            isBound = false;
        }
    }

    private void observeServiceState() {
        if (vadasService == null) return;

        vadasService.getSupercomboLiveData().observe(this, supercombo -> {
            this.currentSupercombo = supercombo;
        });

        // Nhận kết quả thị giác từ Dashcam Video Engine (Làn đường & Phương tiện)
        vadasService.getUnifiedPerceptionLiveData().observe(this, packet -> {
            if (packet != null && !isCameraXActive) {
                this.currentSupercombo = packet.supercombo;
                List<DetectedBox> vehicles = packet.perception.getDetectedObjects();
                List<LaneAnalyzer.Lane> lanes = mapSupercomboLanes(packet.supercombo, vehicles);
                int thermal = thermalManager != null ? thermalManager.getCurrentThermalStatus() : PowerManager.THERMAL_STATUS_NONE;

                overlayView.updatePerceptionData(
                        lanes,
                        vehicles,
                        packet.smoothedFps,
                        packet.smoothedLatencyMs,
                        thermal,
                        "Dashcam Video Engine"
                );
            }
        });
    }

    private boolean isLaneOccludedByVehicle(List<PointF> pts, @Nullable List<DetectedBox> vehicles) {
        if (vehicles == null || vehicles.isEmpty() || pts == null || pts.size() < 2) return false;
        for (DetectedBox box : vehicles) {
            if (box.getConfidence() < 0.45f) continue;
            int count = 0;
            for (PointF pt : pts) {
                if (pt.x >= box.getLeft() && pt.x <= box.getRight() &&
                    pt.y >= box.getTop() && pt.y <= box.getBottom() + 0.02f) {
                    count++;
                }
            }
            if (count >= 3 || (float) count / pts.size() > 0.40f) {
                return true;
            }
        }
        return false;
    }

    private List<LaneAnalyzer.Lane> mapSupercomboLanes(@Nullable SupercomboOutput sc) {
        return mapSupercomboLanes(sc, null);
    }

    private List<LaneAnalyzer.Lane> mapSupercomboLanes(@Nullable SupercomboOutput sc, @Nullable List<DetectedBox> vehicles) {
        List<LaneAnalyzer.Lane> list = new ArrayList<>();

        if (sc != null) {
            // 1. Ưu tiên cao nhất: Dùng trực tiếp output thuần túy từ mô hình nhận diện làn AI
            List<LaneAnalyzer.Lane> rawDetected = sc.getDetectedLanes();
            if (rawDetected != null && !rawDetected.isEmpty()) {
                for (LaneAnalyzer.Lane lane : rawDetected) {
                    if (lane.points != null && lane.points.size() >= 2) {
                        if (lane.id != 1 && lane.id != 2 && isLaneOccludedByVehicle(lane.points, vehicles)) {
                            continue;
                        }
                        list.add(lane);
                    }
                }
                if (!list.isEmpty()) {
                    return list;
                }
            }

            // 2. Dự phòng: Đọc trực tiếp các vạch kẻ đường từ Supercombo tensor
            float[] yElev = sc.getScanElevationNormY();
            if (yElev != null && yElev.length > 0) {
                // Làn bên trái ngoài (Adjacent Left Lane - ID 0)
                float[] adjLeftX = sc.getAdjLeftLaneNormX();
                float[] leftXForBase = sc.getLeftLaneNormX();
                float leftBase = (leftXForBase != null && leftXForBase.length > 0) ? leftXForBase[0] : 0.200f;
                if (adjLeftX != null && adjLeftX.length == yElev.length && adjLeftX[0] < leftBase - 0.05f && adjLeftX[0] >= 0.01f) {
                    List<PointF> pts = new ArrayList<>();
                    for (int i = 0; i < adjLeftX.length; i++) {
                        pts.add(new PointF(adjLeftX[i], yElev[i]));
                    }
                    if (!isLaneOccludedByVehicle(pts, vehicles)) {
                        list.add(new LaneAnalyzer.Lane(0, pts, 0.90f));
                    }
                }

                // Làn trái xe chủ (Ego Left Lane - ID 1)
                float[] leftX = sc.getLeftLaneNormX();
                if (leftX != null && leftX.length == yElev.length) {
                    List<PointF> pts = new ArrayList<>();
                    for (int i = 0; i < leftX.length; i++) {
                        pts.add(new PointF(leftX[i], yElev[i]));
                    }
                    list.add(new LaneAnalyzer.Lane(1, pts, 0.98f));
                }

                // Làn phải xe chủ (Ego Right Lane - ID 2)
                float[] rightX = sc.getRightLaneNormX();
                if (rightX != null && rightX.length == yElev.length) {
                    List<PointF> pts = new ArrayList<>();
                    for (int i = 0; i < rightX.length; i++) {
                        pts.add(new PointF(rightX[i], yElev[i]));
                    }
                    list.add(new LaneAnalyzer.Lane(2, pts, 0.98f));
                }

                // Làn bên phải ngoài (Adjacent Right Lane - ID 3)
                float[] adjRightX = sc.getAdjRightLaneNormX();
                if (adjRightX != null && adjRightX.length == yElev.length) {
                    float rightBase = rightX != null ? rightX[0] : 0.80f;
                    if (adjRightX[0] > rightBase + 0.05f && adjRightX[0] <= 0.99f) {
                        List<PointF> pts = new ArrayList<>();
                        for (int i = 0; i < adjRightX.length; i++) {
                            pts.add(new PointF(adjRightX[i], yElev[i]));
                        }
                        if (!isLaneOccludedByVehicle(pts, vehicles)) {
                            list.add(new LaneAnalyzer.Lane(3, pts, 0.90f));
                        }
                    }
                }

                // Toàn bộ vạch trắng giới hạn lề đường bổ sung
                java.util.List<float[]> allWhite = sc.getAllWhiteLanesNormX();
                if (allWhite != null) {
                    int boundaryId = 4;
                    for (float[] boundaryX : allWhite) {
                        if (boundaryX != null && boundaryX.length == yElev.length) {
                            boolean isDuplicate = false;
                            for (LaneAnalyzer.Lane existing : list) {
                                if (existing.points != null && !existing.points.isEmpty()) {
                                    if (Math.abs(existing.points.get(0).x - boundaryX[0]) < 0.06f) {
                                        isDuplicate = true;
                                        break;
                                    }
                                }
                            }
                            if (!isDuplicate && boundaryX[0] >= 0.02f && boundaryX[0] <= 0.99f) {
                                List<PointF> pts = new ArrayList<>();
                                for (int i = 0; i < boundaryX.length; i++) {
                                    pts.add(new PointF(boundaryX[i], yElev[i]));
                                }
                                if (!isLaneOccludedByVehicle(pts, vehicles)) {
                                    list.add(new LaneAnalyzer.Lane(boundaryId++, pts, 0.88f));
                                }
                            }
                        }
                    }
                }
            }
        }

        return list;
    }


    private void startHardwareVideoPlayback() {
        stopHardwareVideoPlayback();

        try {
            ivVideoPreview.setVisibility(android.view.View.GONE);
            textureVideoView.setVisibility(android.view.View.VISIBLE);

            VideoAssetHelper.prepareVideoAsync(this, videoFile -> {
                if (isFinishing() || isDestroyed()) {
                    return;
                }
                try {
                    mediaPlayer = new MediaPlayer();
                    if (videoFile != null && videoFile.exists()) {
                        mediaPlayer.setDataSource(videoFile.getAbsolutePath());
                    } else {
                        AssetFileDescriptor afd = getAssets().openFd("dashcam_sample.mp4");
                        mediaPlayer.setDataSource(afd.getFileDescriptor(), afd.getStartOffset(), afd.getLength());
                        afd.close();
                    }

                    if (videoSurface != null && videoSurface.isValid()) {
                        mediaPlayer.setSurface(videoSurface);
                    }

                    mediaPlayer.setLooping(true);
                    mediaPlayer.setOnVideoSizeChangedListener((mp, width, height) -> {
                        if (width > 0 && height > 0) {
                            videoWidth = width;
                            videoHeight = height;
                            adjustAspectRatio(videoWidth, videoHeight);
                        }
                    });

                    mediaPlayer.prepare();
                    mediaPlayer.start();
                    isPlayingVideo = true;

                    if (vadasService != null) {
                        vadasService.setPipelineRunning(true);
                    }
                    startPerceptionFrameSampling();

                    Log.i(TAG, "Dashcam hardware playback started");
                } catch (Exception e) {
                    Log.e(TAG, "Lỗi phát video dashcam", e);
                }
            });
        } catch (Exception e) {
            Log.e(TAG, "Lỗi khởi tạo video dashcam", e);
        }
    }

    private void stopHardwareVideoPlayback() {
        isPlayingVideo = false;
        stopPerceptionFrameSampling();

        if (mediaPlayer != null) {
            try {
                if (mediaPlayer.isPlaying()) {
                    mediaPlayer.stop();
                }
                mediaPlayer.release();
            } catch (Exception ignored) {}
            mediaPlayer = null;
        }
    }

    private final Bitmap[] sampleBitmaps = new Bitmap[2];
    private int sampleBufferIdx = 0;

    private void startPerceptionFrameSampling() {
        stopPerceptionFrameSampling();

        for (int i = 0; i < 2; i++) {
            if (sampleBitmaps[i] == null || sampleBitmaps[i].isRecycled()) {
                sampleBitmaps[i] = Bitmap.createBitmap(480, 270, Bitmap.Config.ARGB_8888);
            }
        }

        frameSamplerExecutor = Executors.newSingleThreadScheduledExecutor(r -> {
            Thread thread = new Thread(r, "VADAS-FrameSamplerThread");
            thread.setPriority(Thread.NORM_PRIORITY);
            return thread;
        });

        frameSamplerExecutor.scheduleWithFixedDelay(() -> {
            if (!isPlayingVideo || textureVideoView == null || !textureVideoView.isAvailable() || vadasService == null) {
                return;
            }

            // Nếu vision pipeline đang bận xử lý, bỏ qua lượt lấy mẫu để không làm đứng GPU render thread
            if (vadasService.isProcessingFrame()) {
                return;
            }

            try {
                // Double-buffered Bitmaps cố định 480x270: Triệt tiêu 100% rác GC, chống xung đột ghi/đọc giữa 2 luồng
                final Bitmap targetBuf = sampleBitmaps[sampleBufferIdx];
                sampleBufferIdx = (sampleBufferIdx + 1) % 2;

                final Bitmap sampled = textureVideoView.getBitmap(targetBuf);
                if (sampled != null) {
                    CameraFrame frame = new CameraFrame(
                            sampled,
                            sampled.getWidth(),
                            sampled.getHeight(),
                            0,
                            System.nanoTime(),
                            CameraFrame.PixelFormat.RGBA_8888,
                            f -> {} // Reusable bitmap, không recycle
                    );
                    vadasService.onFrameAvailable(frame);
                }
            } catch (Exception e) {
                Log.w(TAG, "Frame sampling glitch: " + e.getMessage());
            }
        }, 20, 10, TimeUnit.MILLISECONDS);
    }

    private void stopPerceptionFrameSampling() {
        if (frameSamplerExecutor != null) {
            frameSamplerExecutor.shutdownNow();
            frameSamplerExecutor = null;
        }
    }

    private void adjustAspectRatio(int videoW, int videoH) {
        if (videoW <= 0 || videoH <= 0 || textureVideoView == null) return;
        int viewWidth = textureVideoView.getWidth();
        int viewHeight = textureVideoView.getHeight();
        if (viewWidth == 0 || viewHeight == 0) return;

        double aspectRatio = (double) videoH / videoW;
        int newWidth, newHeight;
        if (viewHeight > (int) (viewWidth * aspectRatio)) {
            newWidth = viewWidth;
            newHeight = (int) (viewWidth * aspectRatio);
        } else {
            newWidth = (int) (viewHeight / aspectRatio);
            newHeight = viewHeight;
        }

        int xoff = (viewWidth - newWidth) / 2;
        int yoff = (viewHeight - newHeight) / 2;

        if (overlayView != null) {
            overlayView.setVideoViewport(new RectF(xoff, yoff, xoff + newWidth, yoff + newHeight));
        }

        Matrix txform = new Matrix();
        txform.setScale((float) newWidth / viewWidth, (float) newHeight / viewHeight);
        txform.postTranslate(xoff, yoff);
        textureVideoView.setTransform(txform);
    }

    // =========================================================================
    // CAMERAX & TFLITE ADAPTIVE PERCEPTION PIPELINE
    // =========================================================================

    private void initLaneVisionPipeline() {
        cameraExecutor = Executors.newSingleThreadExecutor(r -> {
            Thread t = new Thread(r, "ADAS-CameraX-Worker");
            t.setPriority(Thread.MAX_PRIORITY);
            return t;
        });

        tfLiteManager = new TFLiteManager(this, DEFAULT_LANE_MODEL);
        thermalManager = new ThermalManager(this);
        thermalManager.startMonitoring(cameraExecutor);

        // Khởi tạo LaneAnalyzer tích hợp nhận diện làn đường + phương tiện
        laneAnalyzer = new LaneAnalyzer(tfLiteManager, new LaneAnalyzer.OnLaneDetectionListener() {
            @Override
            public void onLaneResult(List<LaneAnalyzer.Lane> lanes, float calculatedFps, long latencyMs, int thermalStatus) {
                if (isCameraXActive) {
                    overlayView.updateLaneData(lanes, calculatedFps, latencyMs, thermalStatus, tfLiteManager.getCurrentMode().getLabel());
                }
            }

            @Override
            public void onVisionResult(List<LaneAnalyzer.Lane> lanes,
                                       List<DetectedBox> vehicles,
                                       float calculatedFps,
                                       long latencyMs,
                                       int thermalStatus) {
                if (isCameraXActive) {
                    overlayView.updatePerceptionData(
                            lanes,
                            vehicles,
                            calculatedFps,
                            latencyMs,
                            thermalStatus,
                            tfLiteManager.getCurrentMode().getLabel()
                    );
                }
            }
        });

        thermalManager.addListener((status, statusName) -> {
            laneAnalyzer.updateThermalStatus(status);
        });

        cameraExecutor.execute(() -> {
            laneAnalyzer.initialize(MainActivity.this);
            if (vadasService != null) {
                vadasService.setLaneAnalyzer(laneAnalyzer);
            }
        });
    }

    private void toggleCameraXMode() {
        if (!isCameraXActive) {
            if (checkCameraPermission()) {
                switchToCameraXMode();
            } else {
                ActivityCompat.requestPermissions(this,
                        new String[]{Manifest.permission.CAMERA},
                        CAMERA_PERMISSION_REQUEST_CODE);
            }
        } else {
            switchToDashcamMode();
        }
    }

    private boolean checkCameraPermission() {
        return ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED;
    }

    private void switchToCameraXMode() {
        stopHardwareVideoPlayback();

        textureVideoView.setVisibility(android.view.View.GONE);
        ivVideoPreview.setVisibility(android.view.View.GONE);
        previewView.setVisibility(android.view.View.VISIBLE);
        overlayView.setVisibility(android.view.View.VISIBLE);

        startCameraXPipeline();
        isCameraXActive = true;
        btnCameraXMode.setText("🎥 Dashcam MP4");
        btnCameraXMode.setTextColor(Color.parseColor("#FFD600"));
        Toast.makeText(this, "Đã bật CameraX (" + tfLiteManager.getCurrentMode().getLabel() + ")", Toast.LENGTH_SHORT).show();
    }

    private void switchToDashcamMode() {
        stopCameraXPipeline();

        previewView.setVisibility(android.view.View.GONE);
        textureVideoView.setVisibility(android.view.View.VISIBLE);
        overlayView.setVisibility(android.view.View.VISIBLE);

        startHardwareVideoPlayback();
        isCameraXActive = false;
        btnCameraXMode.setText("📷 CameraX ADAS");
        btnCameraXMode.setTextColor(Color.parseColor("#00E5FF"));
    }

    private void startCameraXPipeline() {
        ListenableFuture<ProcessCameraProvider> cameraProviderFuture = ProcessCameraProvider.getInstance(this);
        cameraProviderFuture.addListener(() -> {
            try {
                cameraProvider = cameraProviderFuture.get();

                Preview preview = new Preview.Builder()
                        .setTargetResolution(new Size(1280, 720))
                        .build();
                preview.setSurfaceProvider(previewView.getSurfaceProvider());

                ImageAnalysis imageAnalysis = new ImageAnalysis.Builder()
                        .setTargetResolution(new Size(1280, 720))
                        .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                        .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_YUV_420_888)
                        .build();

                imageAnalysis.setAnalyzer(cameraExecutor, laneAnalyzer);

                CameraSelector cameraSelector = CameraSelector.DEFAULT_BACK_CAMERA;
                cameraProvider.unbindAll();
                cameraProvider.bindToLifecycle(this, cameraSelector, preview, imageAnalysis);

                Log.i(TAG, "CameraX Lane & Vehicle Perception pipeline started successfully");
            } catch (Exception e) {
                Log.e(TAG, "Lỗi khi kích hoạt CameraX", e);
                Toast.makeText(this, "Lỗi CameraX: " + e.getMessage(), Toast.LENGTH_SHORT).show();
            }
        }, ContextCompat.getMainExecutor(this));
    }

    private void stopCameraXPipeline() {
        if (cameraProvider != null) {
            cameraProvider.unbindAll();
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == CAMERA_PERMISSION_REQUEST_CODE) {
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                switchToCameraXMode();
            } else {
                Toast.makeText(this, "Cần cấp quyền Camera để nhận diện làn đường & xe!", Toast.LENGTH_SHORT).show();
            }
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        stopCameraXPipeline();
        if (thermalManager != null) {
            thermalManager.stopMonitoring();
        }
        if (cameraExecutor != null) {
            cameraExecutor.shutdown();
        }
        if (tfLiteManager != null) {
            tfLiteManager.close();
        }
    }
}
