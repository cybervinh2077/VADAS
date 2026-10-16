package com.vadas.core.safety;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.vadas.core.safety.model.LeadObstacle;
import com.vadas.core.safety.model.SafetyEvaluationResult;
import com.vadas.core.safety.model.ThreatLevel;
import com.vadas.core.safety.model.WeatherCondition;

import java.util.Locale;

/**
 * Vietnamese Safe Distance & Threat Level Evaluator.
 * Fully complies with Circular 31/2019/TT-BGTVT (Điều 11: Khoảng cách an toàn giữa hai xe).
 *
 * Statutory Matrix (Dry Road):
 * - Speed &lt; 60 km/h: TTC &lt;= 2.0s rule (dynamic urban headway).
 * - Speed = 60 km/h: Minimum 35 meters.
 * - 60 &lt; Speed &lt;= 80 km/h: Minimum 55 meters.
 * - 80 &lt; Speed &lt;= 100 km/h: Minimum 70 meters.
 * - 100 &lt; Speed &lt;= 120 km/h: Minimum 100 meters.
 * - Speed &gt; 120 km/h: Scaled statutory safe distance (&gt;= 100m + dynamic excess).
 *
 * Environmental Multiplier:
 * - Rain, fog, or slippery wet road adds +30% to required distance (multiplier = 1.30).
 */
public final class SafeDistanceEvaluator {

    /**
     * Baseline time headway for urban / low-speed driving (< 60 km/h) under standard conditions.
     */
    public static final double URBAN_BASE_TTC_THRESHOLD_SECONDS = 2.0;

    /**
     * Critical TTC threshold triggering immediate emergency braking alerts across all speeds.
     */
    public static final double CRITICAL_TTC_DANGER_SECONDS = 1.5;

    /**
     * Fraction of statutory distance below which a DANGER threat is declared.
     */
    public static final double DANGER_DISTANCE_RATIO_THRESHOLD = 0.50;

    /**
     * Evaluates driving safety based on ego vehicle speed, lead obstacle, and weather.
     *
     * @param egoSpeedKmh      current ego vehicle ground speed from GPS/OBD (km/h).
     * @param leadObstacle     the leading obstacle/vehicle detected by vision, or null if clear road.
     * @param weatherCondition current weather condition (rain, fog, dry).
     * @return {@link SafetyEvaluationResult} with threat level, required gap, and alerts.
     */
    @NonNull
    public SafetyEvaluationResult evaluate(double egoSpeedKmh,
                                          @Nullable LeadObstacle leadObstacle,
                                          @NonNull WeatherCondition weatherCondition) {
        long now = System.currentTimeMillis();

        // 1. Clear road / No obstacle detected
        if (leadObstacle == null) {
            double required = calculateRequiredDistance(egoSpeedKmh, weatherCondition);
            return new SafetyEvaluationResult(
                    ThreatLevel.SAFE,
                    egoSpeedKmh,
                    Double.POSITIVE_INFINITY,
                    required,
                    Double.POSITIVE_INFINITY,
                    weatherCondition,
                    "Phía trước thông thoáng",
                    now
            );
        }

        double distance = leadObstacle.getDistanceMeters();
        double relativeSpeedKmh = leadObstacle.getRelativeSpeedKmh();

        // 2. Compute statutory required safe distance under Circular 31/2019/TT-BGTVT
        double requiredDistance = calculateRequiredDistance(egoSpeedKmh, weatherCondition);

        // 3. Compute Time-To-Collision (TTC) in seconds
        double ttc = calculateTimeToCollision(distance, egoSpeedKmh, relativeSpeedKmh);

        // 4. Threat Level Classification
        ThreatLevel threatLevel;
        String alertMessage;

        // Vehicle stopped or idling (< 2 km/h)
        if (egoSpeedKmh < 2.0) {
            threatLevel = ThreatLevel.SAFE;
            alertMessage = String.format(Locale.getDefault(), "Xe đang dừng. Cự ly: %.1fm", distance);
        } else if (ttc <= CRITICAL_TTC_DANGER_SECONDS || distance <= (requiredDistance * DANGER_DISTANCE_RATIO_THRESHOLD)) {
            // Imminent collision: TTC under critical threshold or gap breached by > 50%
            threatLevel = ThreatLevel.DANGER;
            alertMessage = String.format(Locale.getDefault(),
                    "NGUY HIỂM: Phanh khẩn cấp! Cự ly: %.1fm (Tối thiểu TT31: %.1fm, TTC: %.1fs)",
                    distance, requiredDistance, ttc);
        } else if (distance < requiredDistance || (egoSpeedKmh < 60.0 && ttc <= (URBAN_BASE_TTC_THRESHOLD_SECONDS * weatherCondition.getMultiplier()))) {
            // Safe distance violated according to Circular 31 or headway gap < 2.0s (+weather)
            threatLevel = ThreatLevel.WARNING;
            alertMessage = String.format(Locale.getDefault(),
                    "CẢNH BÁO: Giữ khoảng cách an toàn! Cự ly: %.1fm (Tối thiểu TT31: %.1fm)",
                    distance, requiredDistance);
        } else {
            // Compliant with Circular 31/2019/TT-BGTVT
            threatLevel = ThreatLevel.SAFE;
            alertMessage = String.format(Locale.getDefault(),
                    "Khoảng cách an toàn (TT31). Cự ly: %.1fm / Yêu cầu: %.1fm",
                    distance, requiredDistance);
        }

        return new SafetyEvaluationResult(
                threatLevel,
                egoSpeedKmh,
                distance,
                requiredDistance,
                ttc,
                weatherCondition,
                alertMessage,
                now
        );
    }

    /**
     * Calculates the minimum statutory safe distance (in meters) under Circular 31/2019/TT-BGTVT.
     *
     * @param speedKmh current speed in km/h.
     * @param weather  weather condition multiplier.
     * @return required safe distance in meters.
     */
    public double calculateRequiredDistance(double speedKmh, @NonNull WeatherCondition weather) {
        if (speedKmh <= 0.0) {
            return 2.0; // Minimal buffer at dead stop
        }

        double baseDistanceMeters;

        if (speedKmh < 60.0) {
            // TTC <= 2.0s rule: distance traveled in 2.0 seconds: d = (v / 3.6) * 2.0
            double speedMps = speedKmh / 3.6;
            baseDistanceMeters = speedMps * URBAN_BASE_TTC_THRESHOLD_SECONDS;
            // Floor at 5.0m for creeping traffic
            baseDistanceMeters = Math.max(5.0, baseDistanceMeters);
        } else if (Math.abs(speedKmh - 60.0) < 0.001) {
            // Speed = 60 km/h -> 35m
            baseDistanceMeters = 35.0;
        } else if (speedKmh <= 80.0) {
            // 60 < Speed <= 80 km/h -> 55m
            baseDistanceMeters = 55.0;
        } else if (speedKmh <= 100.0) {
            // 80 < Speed <= 100 km/h -> 70m
            baseDistanceMeters = 70.0;
        } else if (speedKmh <= 120.0) {
            // 100 < Speed <= 120 km/h -> 100m
            baseDistanceMeters = 100.0;
        } else {
            // Speed > 120 km/h: 100m statutory base + 1.25m per km/h over 120
            baseDistanceMeters = 100.0 + (speedKmh - 120.0) * 1.25;
        }

        // Apply weather condition multiplier (e.g. +30% for rain/fog)
        return baseDistanceMeters * weather.getMultiplier();
    }

    /**
     * Calculates Time-To-Collision (TTC) in seconds.
     * TTC = distance / closing_speed.
     *
     * @param distanceMeters   gap to lead obstacle.
     * @param egoSpeedKmh      ego vehicle speed in km/h.
     * @param relativeSpeedKmh relative speed (positive = closing in; negative = lead pulling away).
     *                         If 0 or unknown, ego speed is used as stationary closing speed.
     * @return TTC in seconds, or {@link Double#POSITIVE_INFINITY} if gap is increasing.
     */
    public double calculateTimeToCollision(double distanceMeters, double egoSpeedKmh, double relativeSpeedKmh) {
        if (distanceMeters <= 0.0) {
            return 0.0;
        }

        // Determine effective closing speed
        double closingSpeedKmh;
        if (relativeSpeedKmh > 0.0) {
            closingSpeedKmh = relativeSpeedKmh;
        } else if (relativeSpeedKmh < 0.0) {
            // Lead vehicle is moving away faster than ego vehicle
            return Double.POSITIVE_INFINITY;
        } else {
            // Lead is stationary or relative speed is unmeasured: assume obstacle is stationary
            closingSpeedKmh = egoSpeedKmh;
        }

        if (closingSpeedKmh <= 0.1) {
            return Double.POSITIVE_INFINITY;
        }

        double closingSpeedMps = closingSpeedKmh / 3.6;
        return distanceMeters / closingSpeedMps;
    }
}
