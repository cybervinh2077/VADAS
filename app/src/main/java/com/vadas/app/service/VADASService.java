package com.vadas.app.service;

import android.app.Notification;
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;
import android.content.pm.ServiceInfo;
import android.net.wifi.WifiManager;
import android.os.Binder;
import android.os.Build;
import android.os.IBinder;
import android.os.PowerManager;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.core.app.NotificationCompat;
import androidx.lifecycle.LifecycleService;
import androidx.lifecycle.LiveData;
import androidx.lifecycle.MutableLiveData;

import com.vadas.app.R;
import com.vadas.app.ui.MainActivity;
import com.vadas.core.camera.model.CameraFrame;
import com.vadas.core.camera.pipeline.FrameConsumer;
import com.vadas.core.safety.SafeDistanceEvaluator;
import com.vadas.core.safety.model.LeadObstacle;
import com.vadas.core.safety.model.SafetyEvaluationResult;
import com.vadas.core.safety.model.ThreatLevel;
import com.vadas.core.safety.model.WeatherCondition;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Core Driving Assistant Foreground Service.
 * Manages the real-time perception pipeline, camera ingestion, threat evaluation,
 * and background execution across long driving sessions.
 */
public class VADASService extends LifecycleService implements FrameConsumer {

    private static final String TAG = "VADASService";
    private static final String CHANNEL_ID = "vadas_driving_safety_channel";
    private static final int NOTIFICATION_ID = 1001;

    // Actions
    public static final String ACTION_START_DETECTION = "com.vadas.action.START_DETECTION";
    public static final String ACTION_STOP_DETECTION = "com.vadas.action.STOP_DETECTION";

    // Concurrency & Zero-Latency Frame Dropping Pipeline
    private ExecutorService visionPipelineExecutor;
    private final AtomicBoolean isProcessingFrame = new AtomicBoolean(false);

    public boolean isProcessingFrame() {
        return isProcessingFrame.get();
    }

    // Hardware Locks to prevent CPU sleep and WiFi power save during RTSP streaming
    private PowerManager.WakeLock wakeLock;
    private WifiManager.WifiLock wifiLock;

    // Safety Engine & Neural Vision
    private final SafeDistanceEvaluator safeDistanceEvaluator = new SafeDistanceEvaluator();
    private final com.vadas.core.vision.openpilot.SupercomboVisionEngine visionEngine =
            new com.vadas.core.vision.openpilot.SupercomboVisionEngine();
    private com.vadas.core.camera.video.VideoFileSource videoFileSource;

    private WeatherCondition currentWeather = WeatherCondition.DRY_NORMAL;
    private double currentSpeedKmh = 60.0; // Default simulated cruising speed for video test

    /**
     * Gói dữ liệu thị giác đồng bộ nguyên tử (Atomic Perception Packet):
     * Khóa chặt kết quả làn đường và xe cùng frame, triệt tiêu 100% hiện tượng cập nhật lệch pha gây giật refresh.
     */
    public static class UnifiedPerceptionPacket {
        @NonNull public final com.vadas.core.vision.model.VisionPerceptionResult perception;
        @Nullable public final com.vadas.core.vision.openpilot.SupercomboOutput supercombo;
        public final float smoothedFps;
        public final long smoothedLatencyMs;

        public UnifiedPerceptionPacket(@NonNull com.vadas.core.vision.model.VisionPerceptionResult perception,
                                      @Nullable com.vadas.core.vision.openpilot.SupercomboOutput supercombo,
                                      float smoothedFps,
                                      long smoothedLatencyMs) {
            this.perception = perception;
            this.supercombo = supercombo;
            this.smoothedFps = smoothedFps;
            this.smoothedLatencyMs = smoothedLatencyMs;
        }
    }

    // Observable Safety & Vision State for UI
    private final MutableLiveData<SafetyEvaluationResult> safetyResultLiveData = new MutableLiveData<>();
    private final MutableLiveData<UnifiedPerceptionPacket> unifiedPerceptionLiveData = new MutableLiveData<>();
    private final MutableLiveData<com.vadas.core.vision.openpilot.SupercomboOutput> supercomboLiveData = new MutableLiveData<>();
    private final MutableLiveData<com.vadas.core.vision.model.VisionPerceptionResult> perceptionResultLiveData = new MutableLiveData<>();
    private final MutableLiveData<android.graphics.Bitmap> videoFrameBitmapLiveData = new MutableLiveData<>();
    private final MutableLiveData<Boolean> isPipelineRunningLiveData = new MutableLiveData<>(true);

    public void setPipelineRunning(boolean running) {
        isPipelineRunningLiveData.postValue(running);
    }

    public void setLaneAnalyzer(@Nullable com.vadas.core.vision.lane.LaneAnalyzer laneAnalyzer) {
        visionEngine.setLaneAnalyzer(laneAnalyzer);
    }

    // Binder for client Activities
    private final IBinder binder = new VADASBinder();

    public class VADASBinder extends Binder {
        @NonNull
        public VADASService getService() {
            return VADASService.this;
        }
    }

    @Override
    public void onCreate() {
        super.onCreate();
        Log.i(TAG, "Initializing VADAS Driving Safety Service");
        createNotificationChannel();
        acquireHardwareLocks();

        // Khởi tạo Deep Learning Vision Engine & Vehicle Detector
        visionEngine.initialize(this);

        // Bounded single-thread executor ensures sequential inference without thread queue bloat
        visionPipelineExecutor = Executors.newSingleThreadExecutor(r -> {
            Thread thread = new Thread(r, "VADAS-VisionPipelineThread");
            thread.setPriority(Thread.MAX_PRIORITY);
            return thread;
        });
    }

    @Override
    public int onStartCommand(@Nullable Intent intent, int flags, int startId) {
        super.onStartCommand(intent, flags, startId);

        if (intent != null && ACTION_STOP_DETECTION.equals(intent.getAction())) {
            stopDetectionPipeline();
            stopSelf();
            return START_NOT_STICKY;
        }

        startForegroundWithNotification();
        startDetectionPipeline();
        return START_STICKY;
    }

    @Nullable
    @Override
    public IBinder onBind(@NonNull Intent intent) {
        super.onBind(intent);
        return binder;
    }

    /**
     * Frame ingestion interface implementation.
     * Implements zero-latency frame dropping: if inference is still busy with a previous frame,
     * the incoming frame is immediately released without allocation to maintain 0ms lag.
     */
    @Override
    public void onFrameAvailable(@NonNull CameraFrame frame) {
        if (!Boolean.TRUE.equals(isPipelineRunningLiveData.getValue())) {
            frame.release();
            return;
        }

        // Drop frame if AI inference pipeline is currently busy
        if (!isProcessingFrame.compareAndSet(false, true)) {
            frame.release();
            return;
        }

        visionPipelineExecutor.execute(() -> {
            try {
                processVisionPipeline(frame);
            } catch (Exception e) {
                Log.e(TAG, "Error processing camera frame", e);
            } finally {
                // Guaranteed buffer release back to memory pool
                frame.release();
                isProcessingFrame.set(false);
            }
        });
    }

    private long lastFrameTimeNs = 0;
    private float smoothedFps = 30.0f;
    private long smoothedLatencyMs = 22;

    public float getSmoothedFps() {
        return smoothedFps;
    }

    public long getSmoothedLatencyMs() {
        return smoothedLatencyMs;
    }

    /**
     * Internal vision processing stage: Run OpenPilot Supercombo + Pinhole Camera Distance + Circular 31 Threat.
     */
    private void processVisionPipeline(@NonNull CameraFrame frame) {
        long startNs = System.nanoTime();

        // 1. Process OpenPilot Supercombo (lanes, road edges) + Pinhole monocular distance estimation
        com.vadas.core.vision.model.VisionPerceptionResult perception = visionEngine.processFrame(frame);

        long elapsedMs = (System.nanoTime() - startNs) / 1_000_000L;
        smoothedLatencyMs = (long) ((0.2f * elapsedMs) + (0.8f * smoothedLatencyMs));

        long nowNs = System.nanoTime();
        if (lastFrameTimeNs > 0) {
            long deltaNs = nowNs - lastFrameTimeNs;
            if (deltaNs > 0) {
                float instantFps = 1_000_000_000.0f / deltaNs;
                smoothedFps = (0.15f * instantFps) + (0.85f * smoothedFps);
            }
        }
        lastFrameTimeNs = nowNs;

        com.vadas.core.vision.openpilot.SupercomboOutput supercombo = visionEngine.getLatestSupercomboOutput();
        UnifiedPerceptionPacket packet = new UnifiedPerceptionPacket(
                perception,
                supercombo,
                smoothedFps,
                smoothedLatencyMs
        );
        unifiedPerceptionLiveData.postValue(packet);
        perceptionResultLiveData.postValue(perception);
        supercomboLiveData.postValue(supercombo);

        // 2. Identify lead vehicle from Pinhole Camera Model
        LeadObstacle leadObstacle = null;
        com.vadas.core.vision.model.DetectedBox leadBox = perception.getPrimaryLeadVehicle();

        if (leadBox != null && !Double.isInfinite(leadBox.getEstimatedDistanceMeters())) {
            double distance = leadBox.getEstimatedDistanceMeters();
            leadObstacle = new LeadObstacle(
                    distance,
                    0.0, // Relative speed (m/s)
                    LeadObstacle.ObstacleType.CAR,
                    leadBox.getConfidence()
            );
        }

        // 3. Safety evaluation under Circular 31/2019/TT-BGTVT
        SafetyEvaluationResult evaluation = safeDistanceEvaluator.evaluate(
                currentSpeedKmh,
                leadObstacle,
                currentWeather
        );

        // Check if lane departure warning triggered by OpenPilot
        if (supercombo != null && supercombo.hasLaneDeparture()) {
            String ldwSide = supercombo.isLaneDepartureWarningLeft() ? "TRÁI" : "PHẢI";
            Log.w(TAG, "OpenPilot Lane Departure Warning: Lệch làn " + ldwSide);
        }

        safetyResultLiveData.postValue(evaluation);

        // Update persistent notification if threat level escalated
        if (evaluation.getThreatLevel() == ThreatLevel.DANGER) {
            updateNotificationAlert(evaluation.getAlertMessage(), true);
        }
    }

    public LiveData<UnifiedPerceptionPacket> getUnifiedPerceptionLiveData() {
        return unifiedPerceptionLiveData;
    }

    public LiveData<com.vadas.core.vision.openpilot.SupercomboOutput> getSupercomboLiveData() {
        return supercomboLiveData;
    }

    public LiveData<com.vadas.core.vision.model.VisionPerceptionResult> getPerceptionResultLiveData() {
        return perceptionResultLiveData;
    }

    public LiveData<android.graphics.Bitmap> getVideoFrameBitmapLiveData() {
        return videoFrameBitmapLiveData;
    }

    public String getActiveVideoSourceDescription() {
        return videoFileSource != null ? videoFileSource.getActiveVideoSourceDescription() : "None";
    }

    public void startVideoTestFeed(@Nullable String videoFilePath) {
        if (videoFileSource != null) {
            videoFileSource.stop();
        }
        videoFileSource = new com.vadas.core.camera.video.VideoFileSource(this, videoFilePath);
        videoFileSource.setFrameListener(bitmap -> videoFrameBitmapLiveData.postValue(bitmap));
        videoFileSource.start(this);
        isPipelineRunningLiveData.setValue(true);
    }

    public void stopVideoTestFeed() {
        if (videoFileSource != null) {
            videoFileSource.stop();
            videoFileSource = null;
        }
        isPipelineRunningLiveData.setValue(false);
    }

    public void updateTelemetry(double speedKmh, @NonNull WeatherCondition weather) {
        this.currentSpeedKmh = speedKmh;
        this.currentWeather = weather;
    }

    public LiveData<SafetyEvaluationResult> getSafetyResultLiveData() {
        return safetyResultLiveData;
    }

    public LiveData<Boolean> getIsPipelineRunningLiveData() {
        return isPipelineRunningLiveData;
    }

    private void startDetectionPipeline() {
        Log.i(TAG, "Starting VADAS ADAS Detection Pipeline");
        isPipelineRunningLiveData.setValue(true);
    }

    private void stopDetectionPipeline() {
        Log.i(TAG, "Stopping VADAS ADAS Detection Pipeline");
        isPipelineRunningLiveData.setValue(false);
    }

    private void startForegroundWithNotification() {
        Notification notification = buildServiceNotification(
                getString(R.string.service_running_title),
                getString(R.string.service_running_desc),
                false
        );

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            int serviceTypes = ServiceInfo.FOREGROUND_SERVICE_TYPE_CAMERA;
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
                serviceTypes |= ServiceInfo.FOREGROUND_SERVICE_TYPE_LOCATION;
            }
            startForeground(NOTIFICATION_ID, notification, serviceTypes);
        } else {
            startForeground(NOTIFICATION_ID, notification);
        }
    }

    private void updateNotificationAlert(String alertText, boolean isDanger) {
        NotificationManager manager = (NotificationManager) getSystemService(Context.NOTIFICATION_SERVICE);
        if (manager != null) {
            Notification notification = buildServiceNotification(
                    isDanger ? "⚠️ CẢNH BÁO NGUY HIỂM VADAS" : getString(R.string.service_running_title),
                    alertText,
                    isDanger
            );
            manager.notify(NOTIFICATION_ID, notification);
        }
    }

    @NonNull
    private Notification buildServiceNotification(String title, String content, boolean isDanger) {
        Intent notificationIntent = new Intent(this, MainActivity.class);
        PendingIntent pendingIntent = PendingIntent.getActivity(
                this, 0, notificationIntent,
                PendingIntent.FLAG_IMMUTABLE | PendingIntent.FLAG_UPDATE_CURRENT
        );

        return new NotificationCompat.Builder(this, CHANNEL_ID)
                .setContentTitle(title)
                .setContentText(content)
                .setSmallIcon(android.R.drawable.ic_menu_camera)
                .setContentIntent(pendingIntent)
                .setOngoing(true)
                .setPriority(isDanger ? NotificationCompat.PRIORITY_MAX : NotificationCompat.PRIORITY_LOW)
                .setCategory(NotificationCompat.CATEGORY_SERVICE)
                .build();
    }

    private void createNotificationChannel() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            NotificationChannel channel = new NotificationChannel(
                    CHANNEL_ID,
                    getString(R.string.notification_channel_name),
                    NotificationManager.IMPORTANCE_LOW
            );
            channel.setDescription(getString(R.string.notification_channel_desc));
            channel.setShowBadge(false);

            NotificationManager manager = getSystemService(NotificationManager.class);
            if (manager != null) {
                manager.createNotificationChannel(channel);
            }
        }
    }

    private void acquireHardwareLocks() {
        PowerManager pm = (PowerManager) getSystemService(Context.POWER_SERVICE);
        if (pm != null) {
            wakeLock = pm.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "VADAS:WakeLock");
            wakeLock.acquire(12 * 60 * 60 * 1000L); // 12 hours max driving session
        }

        WifiManager wm = (WifiManager) getApplicationContext().getSystemService(Context.WIFI_SERVICE);
        if (wm != null) {
            wifiLock = wm.createWifiLock(WifiManager.WIFI_MODE_FULL_HIGH_PERF, "VADAS:WifiLock");
            wifiLock.acquire();
        }
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        Log.i(TAG, "Destroying VADAS Driving Safety Service");
        stopDetectionPipeline();

        if (visionPipelineExecutor != null) {
            visionPipelineExecutor.shutdown();
            try {
                if (!visionPipelineExecutor.awaitTermination(800, TimeUnit.MILLISECONDS)) {
                    visionPipelineExecutor.shutdownNow();
                }
            } catch (InterruptedException ignored) {
                visionPipelineExecutor.shutdownNow();
            }
        }

        if (wakeLock != null && wakeLock.isHeld()) {
            wakeLock.release();
        }
        if (wifiLock != null && wifiLock.isHeld()) {
            wifiLock.release();
        }
    }
}
