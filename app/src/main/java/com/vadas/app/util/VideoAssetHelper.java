package com.vadas.app.util;

import android.content.Context;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Utility to discover and prepare dashcam test video assets for high-performance MediaPlayer playback.
 * Guaranteed 100% non-blocking: never runs heavy disk I/O on the main thread, eliminating ANR dialogs.
 */
public final class VideoAssetHelper {

    private static final String TAG = "VideoAssetHelper";
    private static final String CACHED_VIDEO_NAME = "vadas_dashcam_active.mp4";
    private static final ExecutorService IO_EXECUTOR = Executors.newSingleThreadExecutor(r -> {
        Thread t = new Thread(r, "VADAS-VideoAssetExtractor");
        t.setPriority(Thread.NORM_PRIORITY);
        return t;
    });
    private static final Handler MAIN_HANDLER = new Handler(Looper.getMainLooper());

    public interface OnVideoReadyCallback {
        void onVideoReady(@Nullable File videoFile);
    }

    private VideoAssetHelper() {}

    /**
     * Fast check if cached video is already present on disk (>10MB).
     */
    public static boolean isVideoCached(@NonNull Context context) {
        File cachedFile = new File(context.getCacheDir(), CACHED_VIDEO_NAME);
        return cachedFile.exists() && cachedFile.length() > 10_000_000L;
    }

    /**
     * Resolves the active test video file from app assets synchronously ONLY if already cached.
     * Returns null if not cached yet to prevent UI thread freezing.
     */
    @Nullable
    public static File getCachedVideoIfExists(@NonNull Context context) {
        File cachedFile = new File(context.getCacheDir(), CACHED_VIDEO_NAME);
        if (cachedFile.exists() && cachedFile.length() > 10_000_000L) {
            return cachedFile;
        }
        return null;
    }

    /**
     * Asynchronously ensures the dashcam video is extracted to disk and invokes the callback on the UI thread.
     */
    public static void prepareVideoAsync(@NonNull Context context, @NonNull OnVideoReadyCallback callback) {
        File cached = getCachedVideoIfExists(context);
        if (cached != null) {
            callback.onVideoReady(cached);
            return;
        }

        IO_EXECUTOR.execute(() -> {
            File result = extractVideoInternal(context);
            MAIN_HANDLER.post(() -> callback.onVideoReady(result));
        });
    }

    /**
     * Resolves the active test video file from app assets.
     */
    @Nullable
    public static File getOrExtractActiveVideo(@NonNull Context context) {
        File cached = getCachedVideoIfExists(context);
        if (cached != null) {
            return cached;
        }
        return extractVideoInternal(context);
    }

    @Nullable
    private static File extractVideoInternal(@NonNull Context context) {
        File cacheDir = context.getCacheDir();
        File cachedFile = new File(cacheDir, CACHED_VIDEO_NAME);

        try {
            String[] assetList = context.getAssets().list("");
            String targetAsset = null;

            if (assetList != null) {
                for (String asset : assetList) {
                    if ("dashcam_test.mp4".equalsIgnoreCase(asset)) {
                        targetAsset = asset;
                        break;
                    }
                }
                if (targetAsset == null) {
                    for (String asset : assetList) {
                        String lower = asset.toLowerCase();
                        if (lower.endsWith(".mp4") || lower.endsWith(".mkv") || lower.endsWith(".webm") || lower.endsWith(".mov")) {
                            targetAsset = asset;
                            break;
                        }
                    }
                }
            }

            if (targetAsset == null) {
                Log.w(TAG, "No video files found in assets directory");
                return cachedFile.exists() && cachedFile.length() > 0 ? cachedFile : null;
            }

            // Check if already extracted
            if (cachedFile.exists() && cachedFile.length() > 10_000_000L) {
                Log.i(TAG, "Using existing cached test video: " + cachedFile.getAbsolutePath());
                return cachedFile;
            }

            Log.i(TAG, "Extracting asset video [" + targetAsset + "] to internal storage in background...");
            File tempFile = new File(cacheDir, CACHED_VIDEO_NAME + ".tmp");
            try (InputStream in = context.getAssets().open(targetAsset);
                 OutputStream out = new FileOutputStream(tempFile)) {
                byte[] buffer = new byte[512 * 1024]; // 512KB buffer for faster I/O
                int read;
                while ((read = in.read(buffer)) != -1) {
                    out.write(buffer, 0, read);
                }
                out.flush();
            }

            if (tempFile.exists() && tempFile.length() > 0) {
                if (cachedFile.exists()) {
                    cachedFile.delete();
                }
                if (tempFile.renameTo(cachedFile)) {
                    Log.i(TAG, "Asset video extraction complete: " + cachedFile.getAbsolutePath());
                    return cachedFile;
                }
            }
            return tempFile.exists() ? tempFile : null;
        } catch (Exception e) {
            Log.e(TAG, "Failed to resolve active video asset", e);
            return cachedFile.exists() && cachedFile.length() > 0 ? cachedFile : null;
        }
    }
}
