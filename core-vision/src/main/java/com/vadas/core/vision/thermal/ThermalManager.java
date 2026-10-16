package com.vadas.core.vision.thermal;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.BatteryManager;
import android.os.Build;
import android.os.PowerManager;
import android.util.Log;

import androidx.annotation.NonNull;

import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.Executor;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Quản lý nhiệt độ động cho hệ thống ADAS:
 * Giám sát nhiệt độ thiết bị theo thời gian thực (Android Q+ qua OnThermalStatusChangedListener,
 * và Fallback qua Battery Thermal Monitoring cho thiết bị Android cũ).
 */
public class ThermalManager {

    private static final String TAG = "ThermalManager";

    public interface OnThermalStatusListener {
        /**
         * Được gọi khi mức nhiệt thiết bị thay đổi.
         *
         * @param status Mức nhiệt theo hằng số PowerManager.THERMAL_STATUS_*
         * @param statusName Tên mô tả (NORMAL, MODERATE, SEVERE, CRITICAL, ...)
         */
        void onThermalStatusChanged(int status, String statusName);
    }

    private final Context context;
    private final PowerManager powerManager;
    private final AtomicInteger currentThermalStatus = new AtomicInteger(PowerManager.THERMAL_STATUS_NONE);
    private final CopyOnWriteArrayList<OnThermalStatusListener> listeners = new CopyOnWriteArrayList<>();

    private PowerManager.OnThermalStatusChangedListener androidThermalListener;
    private BroadcastReceiver legacyBatteryReceiver;

    public ThermalManager(@NonNull Context context) {
        this.context = context.getApplicationContext();
        this.powerManager = (PowerManager) this.context.getSystemService(Context.POWER_SERVICE);
    }

    /**
     * Kích hoạt giám sát nhiệt độ thiết bị trên background executor.
     */
    public void startMonitoring(@NonNull Executor executor) {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q && powerManager != null) {
            // Android 10+ (API 29+) chuẩn công nghiệp
            currentThermalStatus.set(powerManager.getCurrentThermalStatus());
            androidThermalListener = status -> {
                currentThermalStatus.set(status);
                dispatchThermalStatus(status);
            };
            powerManager.addThermalStatusListener(executor, androidThermalListener);
            Log.i(TAG, "Đã kích hoạt OnThermalStatusChangedListener (Android Q+). Mức ban đầu: " + getStatusName(currentThermalStatus.get()));
        } else {
            // Fallback cho Android < 29: Sử dụng cảm biến nhiệt pin
            registerLegacyBatteryThermalMonitoring();
        }
    }

    private void registerLegacyBatteryThermalMonitoring() {
        legacyBatteryReceiver = new BroadcastReceiver() {
            @Override
            public void onReceive(Context ctx, Intent intent) {
                if (Intent.ACTION_BATTERY_CHANGED.equals(intent.getAction())) {
                    int tempTenthCelsius = intent.getIntExtra(BatteryManager.EXTRA_TEMPERATURE, 0);
                    float tempCelsius = tempTenthCelsius / 10.0f;
                    int calculatedStatus;

                    if (tempCelsius >= 52.0f) {
                        calculatedStatus = PowerManager.THERMAL_STATUS_CRITICAL;
                    } else if (tempCelsius >= 46.0f) {
                        calculatedStatus = PowerManager.THERMAL_STATUS_SEVERE;
                    } else if (tempCelsius >= 41.0f) {
                        calculatedStatus = PowerManager.THERMAL_STATUS_MODERATE;
                    } else if (tempCelsius >= 37.0f) {
                        calculatedStatus = PowerManager.THERMAL_STATUS_LIGHT;
                    } else {
                        calculatedStatus = PowerManager.THERMAL_STATUS_NONE;
                    }

                    if (calculatedStatus != currentThermalStatus.get()) {
                        currentThermalStatus.set(calculatedStatus);
                        dispatchThermalStatus(calculatedStatus);
                    }
                }
            }
        };
        context.registerReceiver(legacyBatteryReceiver, new IntentFilter(Intent.ACTION_BATTERY_CHANGED));
        Log.i(TAG, "Đã kích hoạt Fallback Battery Temperature Monitoring (Legacy Android).");
    }

    private void dispatchThermalStatus(int status) {
        String name = getStatusName(status);
        Log.w(TAG, "[Thermal Alert] Trạng thái nhiệt độ thay đổi -> " + name + " (Mức: " + status + ")");
        for (OnThermalStatusListener listener : listeners) {
            try {
                listener.onThermalStatusChanged(status, name);
            } catch (Exception e) {
                Log.e(TAG, "Lỗi khi thông báo thermal listener", e);
            }
        }
    }

    public void addListener(@NonNull OnThermalStatusListener listener) {
        listeners.add(listener);
        // Bắn ngay trạng thái hiện tại cho listener mới đăng ký
        listener.onThermalStatusChanged(currentThermalStatus.get(), getStatusName(currentThermalStatus.get()));
    }

    public void removeListener(@NonNull OnThermalStatusListener listener) {
        listeners.remove(listener);
    }

    public int getCurrentThermalStatus() {
        return currentThermalStatus.get();
    }

    public static String getStatusName(int status) {
        switch (status) {
            case PowerManager.THERMAL_STATUS_NONE:
                return "NORMAL";
            case PowerManager.THERMAL_STATUS_LIGHT:
                return "LIGHT";
            case PowerManager.THERMAL_STATUS_MODERATE:
                return "MODERATE";
            case PowerManager.THERMAL_STATUS_SEVERE:
                return "SEVERE";
            case PowerManager.THERMAL_STATUS_CRITICAL:
                return "CRITICAL";
            case PowerManager.THERMAL_STATUS_EMERGENCY:
                return "EMERGENCY";
            case PowerManager.THERMAL_STATUS_SHUTDOWN:
                return "SHUTDOWN";
            default:
                return "UNKNOWN (" + status + ")";
        }
    }

    public void stopMonitoring() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q && powerManager != null && androidThermalListener != null) {
            try {
                powerManager.removeThermalStatusListener(androidThermalListener);
            } catch (Exception ignored) {}
            androidThermalListener = null;
        }
        if (legacyBatteryReceiver != null) {
            try {
                context.unregisterReceiver(legacyBatteryReceiver);
            } catch (Exception ignored) {}
            legacyBatteryReceiver = null;
        }
        listeners.clear();
        Log.i(TAG, "Đã dừng ThermalManager.");
    }
}
