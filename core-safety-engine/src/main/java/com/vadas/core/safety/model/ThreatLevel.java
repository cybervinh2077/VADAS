package com.vadas.core.safety.model;

/**
 * Visual and acoustic alert level for VADAS driver warnings.
 */
public enum ThreatLevel {
    /**
     * Gap is safe, exceeds Circular 31/2019/TT-BGTVT statutory distance.
     */
    SAFE(0, "An toàn"),

    /**
     * Gap is below statutory distance or closing fast (TTC <= 2.0s). Driver must lift accelerator.
     */
    WARNING(1, "Cảnh báo cự ly gần"),

    /**
     * Critical collision hazard (TTC <= 1.5s or distance <= 50% statutory safe gap). Emergency brake required.
     */
    DANGER(2, "Nguy hiểm - Phanh khẩn cấp");

    private final int severity;
    private final String description;

    ThreatLevel(int severity, String description) {
        this.severity = severity;
        this.description = description;
    }

    public int getSeverity() {
        return severity;
    }

    public String getDescription() {
        return description;
    }

    public boolean isAlert() {
        return this != SAFE;
    }
}
