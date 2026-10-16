package com.vadas.core.safety.model;

/**
 * Environmental and weather conditions affecting road grip and braking distances
 * according to Circular 31/2019/TT-BGTVT Article 11.
 */
public enum WeatherCondition {
    DRY_NORMAL(1.00, "Mặt đường khô ráo"),
    RAIN(1.30, "Trời mưa (+30% cự ly)"),
    FOG(1.30, "Sương mù tầm nhìn hạn chế (+30% cự ly)"),
    WET_ROAD(1.30, "Mặt đường trơn trượt (+30% cự ly)"),
    SEVERE_STORM(1.50, "Mưa bão lớn (+50% cự ly)");

    private final double multiplier;
    private final String description;

    WeatherCondition(double multiplier, String description) {
        this.multiplier = multiplier;
        this.description = description;
    }

    public double getMultiplier() {
        return multiplier;
    }

    public String getDescription() {
        return description;
    }
}
