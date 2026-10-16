package com.vadas.core.safety.model;

import androidx.annotation.NonNull;

/**
 * Immutable evaluation output containing threat classification, statutory compliance,
 * and actionable safety telemetry according to Circular 31/2019/TT-BGTVT.
 */
public final class SafetyEvaluationResult {

    private final ThreatLevel threatLevel;
    private final double egoSpeedKmh;
    private final double currentDistanceMeters;
    private final double requiredSafeDistanceMeters;
    private final double timeToCollisionSeconds;
    private final WeatherCondition weatherCondition;
    private final String alertMessage;
    private final long evaluationTimestampMs;

    public SafetyEvaluationResult(@NonNull ThreatLevel threatLevel,
                                  double egoSpeedKmh,
                                  double currentDistanceMeters,
                                  double requiredSafeDistanceMeters,
                                  double timeToCollisionSeconds,
                                  @NonNull WeatherCondition weatherCondition,
                                  @NonNull String alertMessage,
                                  long evaluationTimestampMs) {
        this.threatLevel = threatLevel;
        this.egoSpeedKmh = egoSpeedKmh;
        this.currentDistanceMeters = currentDistanceMeters;
        this.requiredSafeDistanceMeters = requiredSafeDistanceMeters;
        this.timeToCollisionSeconds = timeToCollisionSeconds;
        this.weatherCondition = weatherCondition;
        this.alertMessage = alertMessage;
        this.evaluationTimestampMs = evaluationTimestampMs;
    }

    @NonNull
    public ThreatLevel getThreatLevel() {
        return threatLevel;
    }

    public double getEgoSpeedKmh() {
        return egoSpeedKmh;
    }

    public double getCurrentDistanceMeters() {
        return currentDistanceMeters;
    }

    public double getRequiredSafeDistanceMeters() {
        return requiredSafeDistanceMeters;
    }

    public double getTimeToCollisionSeconds() {
        return timeToCollisionSeconds;
    }

    @NonNull
    public WeatherCondition getWeatherCondition() {
        return weatherCondition;
    }

    @NonNull
    public String getAlertMessage() {
        return alertMessage;
    }

    public long getEvaluationTimestampMs() {
        return evaluationTimestampMs;
    }

    public boolean isSafeDistanceViolated() {
        return currentDistanceMeters < requiredSafeDistanceMeters;
    }

    @Override
    public String toString() {
        return "SafetyEvaluationResult{" +
                "threat=" + threatLevel +
                ", speed=" + String.format("%.1f", egoSpeedKmh) + "km/h" +
                ", distance=" + String.format("%.1f", currentDistanceMeters) + "m" +
                ", required=" + String.format("%.1f", requiredSafeDistanceMeters) + "m" +
                ", TTC=" + (Double.isInfinite(timeToCollisionSeconds) ? "INF" : String.format("%.2f", timeToCollisionSeconds) + "s") +
                ", weather=" + weatherCondition.name() +
                '}';
    }
}
