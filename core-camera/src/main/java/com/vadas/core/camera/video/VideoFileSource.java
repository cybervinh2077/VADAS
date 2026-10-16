package com.vadas.core.camera.video;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.media.MediaMetadataRetriever;
import android.net.Uri;
import android.os.Build;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.vadas.core.camera.CameraSource;
import com.vadas.core.camera.model.CameraFrame;
import com.vadas.core.camera.pipeline.FrameConsumer;

import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Feeds video frames directly from a recorded MP4/dashcam file located in assets or local storage.
 * Automatically scans the app's assets directory for any video files (*.mp4) without requiring hardcoded names.
 * Delivers both raw ByteBuffer for the AI perception pipeline and Bitmap for real-time UI rendering.
 */
public class VideoFileSource implements CameraSource {

    private static final String TAG = "VideoFileSource";

    public interface VideoFrameListener {
        void onFrameDecoded(@NonNull Bitmap bitmap);
    }

    private final Context context;
    @Nullable
    private final String videoFilePath;
    @Nullable
    private final Uri videoUri;
    @Nullable
    private VideoFrameListener frameListener;

    private int targetFps = 15;
    private boolean isLooping = true;

    private final AtomicBoolean isStreaming = new AtomicBoolean(false);
    private ScheduledExecutorService playbackExecutor;
    private MediaMetadataRetriever retriever;

    private long videoDurationUs = 0;
    private long currentPositionUs = 0;
    private int videoWidth = 1280;
    private int videoHeight = 720;
    private boolean hasRealVideoSource = false;
    private String activeVideoSourceDescription = "None";

    // Direct buffer pool for frame conversion
    private ByteBuffer frameBuffer;
    private Bitmap syntheticBitmap;
    private Canvas syntheticCanvas;
    private Paint roadPaint;
    private Paint dashPaint;
    private Paint carPaint;

    public VideoFileSource(@NonNull Context context, @Nullable String videoFilePath) {
        this.context = context.getApplicationContext();
        this.videoFilePath = videoFilePath;
        this.videoUri = null;
    }

    public VideoFileSource(@NonNull Context context, @NonNull Uri videoUri) {
        this.context = context.getApplicationContext();
        this.videoFilePath = null;
        this.videoUri = videoUri;
    }

    public void setFrameListener(@Nullable VideoFrameListener listener) {
        this.frameListener = listener;
    }

    @NonNull
    @Override
    public SourceType getType() {
        return SourceType.VIDEO_FILE;
    }

    public void setTargetFps(int targetFps) {
        this.targetFps = Math.max(1, Math.min(60, targetFps));
    }

    public void setLooping(boolean looping) {
        isLooping = looping;
    }

    public String getActiveVideoSourceDescription() {
        return activeVideoSourceDescription;
    }

    @Override
    public synchronized void start(@NonNull FrameConsumer consumer) {
        if (isStreaming.get()) {
            Log.w(TAG, "VideoFileSource is already running");
            return;
        }

        try {
            initializeRetriever();
        } catch (Exception e) {
            Log.e(TAG, "Failed to initialize video retriever, falling back to simulated frames", e);
            hasRealVideoSource = false;
        }

        isStreaming.set(true);
        currentPositionUs = 0;

        playbackExecutor = Executors.newSingleThreadScheduledExecutor(r -> {
            Thread t = new Thread(r, "VADAS-VideoPlaybackThread");
            t.setPriority(Thread.MAX_PRIORITY);
            return t;
        });

        long frameIntervalMs = 1000L / targetFps;
        long frameStepUs = (1_000_000L / targetFps);

        playbackExecutor.scheduleAtFixedRate(() -> {
            if (!isStreaming.get()) return;

            try {
                CameraFrame frame = extractNextFrame(currentPositionUs);
                currentPositionUs += frameStepUs;

                if (hasRealVideoSource && videoDurationUs > 0 && currentPositionUs >= videoDurationUs) {
                    if (isLooping) {
                        currentPositionUs = 0;
                    } else {
                        stop();
                        return;
                    }
                }

                if (frame != null) {
                    consumer.onFrameAvailable(frame);
                }
            } catch (Exception e) {
                Log.e(TAG, "Error decoding test video frame", e);
            }
        }, 0, frameIntervalMs, TimeUnit.MILLISECONDS);

        Log.i(TAG, "VideoFileSource started at " + targetFps + " FPS | Source: " + activeVideoSourceDescription);
    }

    @Override
    public synchronized void stop() {
        if (!isStreaming.get()) return;
        isStreaming.set(false);

        if (playbackExecutor != null) {
            playbackExecutor.shutdownNow();
            playbackExecutor = null;
        }

        if (retriever != null) {
            try {
                retriever.release();
            } catch (Exception ignored) {
            }
            retriever = null;
        }

        Log.i(TAG, "VideoFileSource stopped");
    }

    @Override
    public boolean isStreaming() {
        return isStreaming.get();
    }

    private void initializeRetriever() throws Exception {
        retriever = new MediaMetadataRetriever();
        hasRealVideoSource = false;

        File targetVideoFile = null;

        // 1. Explicit file path provided
        if (videoFilePath != null && new File(videoFilePath).exists()) {
            targetVideoFile = new File(videoFilePath);
            activeVideoSourceDescription = "Local File: " + targetVideoFile.getName();
        } else {
            // 2. Scan assets directory for ANY video file (*.mp4, *.mkv, *.mov, *.webm, *.ts)
            String[] assetList = context.getAssets().list("");
            String matchedAsset = null;

            if (assetList != null) {
                // First look for dashcam_test.mp4, or else pick the first video asset found
                for (String asset : assetList) {
                    if ("dashcam_test.mp4".equalsIgnoreCase(asset)) {
                        matchedAsset = asset;
                        break;
                    }
                }
                if (matchedAsset == null) {
                    for (String asset : assetList) {
                        String lower = asset.toLowerCase();
                        if (lower.endsWith(".mp4") || lower.endsWith(".mkv") || lower.endsWith(".mov") || lower.endsWith(".webm")) {
                            matchedAsset = asset;
                            break;
                        }
                    }
                }
            }

            if (matchedAsset != null) {
                Log.i(TAG, "Found target video asset: " + matchedAsset);
                File cacheDir = context.getCacheDir();
                File cachedFile = new File(cacheDir, "vadas_dashcam_active.mp4");

                // Stream asset to cache file for 100% reliable random-access seeking by MediaMetadataRetriever
                try (InputStream in = context.getAssets().open(matchedAsset)) {
                    long assetLength = in.available();
                    if (!cachedFile.exists() || cachedFile.length() != assetLength) {
                        Log.i(TAG, "Copying asset video (" + (assetLength / (1024 * 1024)) + " MB) to " + cachedFile.getAbsolutePath());
                        try (OutputStream out = new FileOutputStream(cachedFile)) {
                            byte[] buffer = new byte[128 * 1024]; // 128KB chunks
                            int read;
                            while ((read = in.read(buffer)) != -1) {
                                out.write(buffer, 0, read);
                            }
                            out.flush();
                        }
                    }
                    targetVideoFile = cachedFile;
                    activeVideoSourceDescription = "Asset Video: " + matchedAsset;
                } catch (Exception e) {
                    Log.e(TAG, "Failed to cache asset video: " + e.getMessage(), e);
                }
            }
        }

        if (targetVideoFile != null && targetVideoFile.exists() && targetVideoFile.length() > 0) {
            retriever.setDataSource(targetVideoFile.getAbsolutePath());
            hasRealVideoSource = true;
            Log.i(TAG, "MediaMetadataRetriever attached to " + targetVideoFile.getAbsolutePath()
                    + " (" + (targetVideoFile.length() / (1024 * 1024)) + " MB)");

            try {
                String durationStr = retriever.extractMetadata(MediaMetadataRetriever.METADATA_KEY_DURATION);
                if (durationStr != null) {
                    videoDurationUs = Long.parseLong(durationStr) * 1000L;
                }

                String widthStr = retriever.extractMetadata(MediaMetadataRetriever.METADATA_KEY_VIDEO_WIDTH);
                String heightStr = retriever.extractMetadata(MediaMetadataRetriever.METADATA_KEY_VIDEO_HEIGHT);
                if (widthStr != null && heightStr != null) {
                    videoWidth = Integer.parseInt(widthStr);
                    videoHeight = Integer.parseInt(heightStr);
                }
                Log.i(TAG, String.format("Video telemetry: %dx%d, Duration: %.1fs",
                        videoWidth, videoHeight, videoDurationUs / 1_000_000.0));
            } catch (Exception e) {
                Log.w(TAG, "Could not extract full video metadata: " + e.getMessage());
            }
        } else if (videoUri != null) {
            retriever.setDataSource(context, videoUri);
            hasRealVideoSource = true;
            activeVideoSourceDescription = "URI: " + videoUri;
        } else {
            Log.w(TAG, "No valid video found in assets or storage; activating fallback synthetic road");
            hasRealVideoSource = false;
            activeVideoSourceDescription = "Synthetic Road Simulator";
            initSyntheticRenderer();
        }

        int bufferSize = videoWidth * videoHeight * 4;
        frameBuffer = ByteBuffer.allocateDirect(bufferSize);
    }

    private void initSyntheticRenderer() {
        videoWidth = 1280;
        videoHeight = 720;
        syntheticBitmap = Bitmap.createBitmap(videoWidth, videoHeight, Bitmap.Config.ARGB_8888);
        syntheticCanvas = new Canvas(syntheticBitmap);

        roadPaint = new Paint();
        roadPaint.setColor(Color.parseColor("#1C1E24")); // Dark asphalt

        dashPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        dashPaint.setColor(Color.WHITE);
        dashPaint.setStrokeWidth(12f);

        carPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        carPaint.setColor(Color.parseColor("#D32F2F")); // Red forward vehicle
    }

    @Nullable
    private CameraFrame extractNextFrame(long positionUs) {
        long timestampNs = System.nanoTime();
        Bitmap frameBitmap = null;

        if (hasRealVideoSource && retriever != null) {
            try {
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O_MR1) {
                    // Scale down to 640x360 for fast processing
                    int targetW = Math.min(640, videoWidth);
                    int targetH = (int) (targetW * ((float) videoHeight / videoWidth));
                    frameBitmap = retriever.getScaledFrameAtTime(positionUs,
                            MediaMetadataRetriever.OPTION_CLOSEST, targetW, targetH);
                } else {
                    frameBitmap = retriever.getFrameAtTime(positionUs, MediaMetadataRetriever.OPTION_CLOSEST);
                }
            } catch (Exception e) {
                Log.w(TAG, "Frame extraction error at " + positionUs + "us: " + e.getMessage());
            }
        }

        // Fallback to animated road if frame is null
        if (frameBitmap == null) {
            frameBitmap = renderSyntheticRoadFrame(positionUs);
        }

        if (frameBitmap != null) {
            if (frameListener != null) {
                frameListener.onFrameDecoded(frameBitmap);
            }

            if (frameBuffer == null || frameBuffer.capacity() < frameBitmap.getByteCount()) {
                frameBuffer = ByteBuffer.allocateDirect(frameBitmap.getByteCount());
            }
            frameBuffer.clear();
            frameBitmap.copyPixelsToBuffer(frameBuffer);
            frameBuffer.flip();

            return new CameraFrame(
                    frameBuffer,
                    frameBitmap,
                    frameBitmap.getWidth(),
                    frameBitmap.getHeight(),
                    0,
                    timestampNs,
                    CameraFrame.PixelFormat.RGBA_8888,
                    frame -> {}
            );
        }

        return null;
    }

    private Bitmap renderSyntheticRoadFrame(long positionUs) {
        if (syntheticBitmap == null) {
            initSyntheticRenderer();
        }

        // Sky & environment
        syntheticCanvas.drawColor(Color.parseColor("#0B0D13"));

        float horizonY = videoHeight * 0.50f;
        float centerX = videoWidth / 2f;

        android.graphics.Path roadPath = new android.graphics.Path();
        roadPath.moveTo(centerX - 100f, horizonY);
        roadPath.lineTo(centerX + 100f, horizonY);
        roadPath.lineTo(videoWidth * 0.92f, videoHeight);
        roadPath.lineTo(videoWidth * 0.08f, videoHeight);
        roadPath.close();
        syntheticCanvas.drawPath(roadPath, roadPaint);

        // Animated dashed center line
        float offset = (positionUs / 4000f) % 120f;
        for (float y = horizonY + 20f + (offset / 2f); y < videoHeight; y += 90f) {
            float progress = (y - horizonY) / (videoHeight - horizonY);
            float currentDashLength = 25f + (progress * 60f);
            dashPaint.setStrokeWidth(4f + (progress * 10f));
            syntheticCanvas.drawLine(centerX, y, centerX, Math.min(videoHeight, y + currentDashLength), dashPaint);
        }

        // Lead vehicle on road ahead
        float carWidth = 150f;
        float carHeight = 70f;
        float carX = centerX - (carWidth / 2f) + (float) (Math.sin(positionUs / 2000000.0) * 20f);
        float carY = horizonY + 40f;

        syntheticCanvas.drawRoundRect(carX, carY, carX + carWidth, carY + carHeight, 12f, 12f, carPaint);
        Paint taillightPaint = new Paint();
        taillightPaint.setColor(Color.parseColor("#FF1744"));
        syntheticCanvas.drawCircle(carX + 15f, carY + carHeight - 15f, 8f, taillightPaint);
        syntheticCanvas.drawCircle(carX + carWidth - 15f, carY + carHeight - 15f, 8f, taillightPaint);

        return syntheticBitmap;
    }
}
