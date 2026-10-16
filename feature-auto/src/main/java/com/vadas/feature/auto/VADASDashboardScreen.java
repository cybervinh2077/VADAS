package com.vadas.feature.auto;

import androidx.annotation.NonNull;
import androidx.car.app.CarContext;
import androidx.car.app.Screen;
import androidx.car.app.model.Action;
import androidx.car.app.model.ActionStrip;
import androidx.car.app.model.CarColor;
import androidx.car.app.model.CarIcon;
import androidx.car.app.model.Pane;
import androidx.car.app.model.PaneTemplate;
import androidx.car.app.model.Row;
import androidx.car.app.model.Template;

import com.vadas.core.safety.SafeDistanceEvaluator;
import com.vadas.core.safety.model.LeadObstacle;
import com.vadas.core.safety.model.SafetyEvaluationResult;
import com.vadas.core.safety.model.ThreatLevel;
import com.vadas.core.safety.model.WeatherCondition;

import java.util.Locale;

/**
 * Android Auto Dashboard Screen presenting Driver Assistance & Circular 31 telemetry.
 * Conforms to Google Driving Safety policies (strictly glanceable text, high contrast,
 * zero video streaming to avoid driver distraction).
 */
public class VADASDashboardScreen extends Screen {

    private final SafeDistanceEvaluator evaluator = new SafeDistanceEvaluator();
    private double currentSpeedKmh = 65.0;
    private double currentLeadDistanceMeters = 50.0;
    private WeatherCondition weatherCondition = WeatherCondition.DRY_NORMAL;

    public VADASDashboardScreen(@NonNull CarContext carContext) {
        super(carContext);
    }

    @NonNull
    @Override
    public Template onGetTemplate() {
        // Run safety evaluation for the dashboard presentation
        LeadObstacle simulatedLead = new LeadObstacle(
                currentLeadDistanceMeters,
                0.0,
                LeadObstacle.ObstacleType.CAR,
                0.95f
        );

        SafetyEvaluationResult evaluation = evaluator.evaluate(
                currentSpeedKmh,
                simulatedLead,
                weatherCondition
        );

        // Map ThreatLevel to Automotive CarColor
        CarColor statusColor;
        String statusText;
        if (evaluation.getThreatLevel() == ThreatLevel.DANGER) {
            statusColor = CarColor.RED;
            statusText = "NGUY HIỂM: Phanh khẩn cấp!";
        } else if (evaluation.getThreatLevel() == ThreatLevel.WARNING) {
            statusColor = CarColor.YELLOW;
            statusText = "CẢNH BÁO: Vi phạm cự ly an toàn TT31!";
        } else {
            statusColor = CarColor.GREEN;
            statusText = "AN TOÀN: Đạt chuẩn Thông tư 31";
        }

        // Build Rows conforming to 4-row glanceable limit
        Row statusRow = new Row.Builder()
                .setTitle(statusText)
                .addText(evaluation.getAlertMessage())
                .build();

        Row distanceRow = new Row.Builder()
                .setTitle(String.format(Locale.getDefault(), "Cự ly trước: %.1fm", evaluation.getCurrentDistanceMeters()))
                .addText(String.format(Locale.getDefault(),
                        "Yêu cầu tối thiểu TT 31/2019: %.1fm (%s)",
                        evaluation.getRequiredSafeDistanceMeters(),
                        weatherCondition.getDescription()))
                .build();

        Row speedRow = new Row.Builder()
                .setTitle(String.format(Locale.getDefault(), "Tốc độ hiện tại: %.0f km/h", currentSpeedKmh))
                .addText("Giới hạn tốc độ tuyến đường (Vietmap): 80 km/h")
                .build();

        Pane pane = new Pane.Builder()
                .addRow(statusRow)
                .addRow(distanceRow)
                .addRow(speedRow)
                .addAction(new Action.Builder()
                        .setTitle("Cập nhật thời tiết")
                        .setOnClickListener(this::toggleWeather)
                        .build())
                .build();

        ActionStrip actionStrip = new ActionStrip.Builder()
                .addAction(new Action.Builder()
                        .setTitle("VADAS Active")
                        .setOnClickListener(this::invalidate)
                        .build())
                .build();

        return new PaneTemplate.Builder(pane)
                .setTitle("VADAS - Trợ Lý An Toàn Giao Thông")
                .setHeaderAction(Action.APP_ICON)
                .setActionStrip(actionStrip)
                .build();
    }

    private void toggleWeather() {
        if (weatherCondition == WeatherCondition.DRY_NORMAL) {
            weatherCondition = WeatherCondition.RAIN;
        } else {
            weatherCondition = WeatherCondition.DRY_NORMAL;
        }
        invalidate();
    }
}
