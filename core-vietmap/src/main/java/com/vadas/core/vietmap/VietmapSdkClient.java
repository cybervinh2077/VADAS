package com.vadas.core.vietmap;

import androidx.annotation.NonNull;

import com.vadas.core.vietmap.model.VietmapRoadSign;
import com.vadas.core.vietmap.model.VietmapSpeedInfo;

/**
 * Abstraction layer for Vietmap SDK navigation, speed limits, and road signs.
 */
public interface VietmapSdkClient {

    interface SpeedListener {
        void onSpeedUpdated(@NonNull VietmapSpeedInfo speedInfo);
    }

    interface RoadSignListener {
        void onRoadSignApproaching(@NonNull VietmapRoadSign sign);
    }

    void initialize(@NonNull String apiKey);

    void registerSpeedListener(@NonNull SpeedListener listener);

    void unregisterSpeedListener(@NonNull SpeedListener listener);

    void registerRoadSignListener(@NonNull RoadSignListener listener);

    void unregisterRoadSignListener(@NonNull RoadSignListener listener);

    void destroy();
}
