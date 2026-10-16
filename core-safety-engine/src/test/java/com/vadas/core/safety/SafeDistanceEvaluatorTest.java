package com.vadas.core.safety;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.vadas.core.safety.model.LeadObstacle;
import com.vadas.core.safety.model.SafetyEvaluationResult;
import com.vadas.core.safety.model.ThreatLevel;
import com.vadas.core.safety.model.WeatherCondition;

import org.junit.Before;
import org.junit.Test;

/**
 * Unit tests verifying compliance of {@link SafeDistanceEvaluator}
 * with Circular 31/2019/TT-BGTVT Article 11.
 */
public class SafeDistanceEvaluatorTest {

    private SafeDistanceEvaluator evaluator;

    @Before
    public void setUp() {
        evaluator = new SafeDistanceEvaluator();
    }

    @Test
    public void testDryRoadStatutoryDistanceBrackets() {
        WeatherCondition dry = WeatherCondition.DRY_NORMAL;

        // Speed = 60 km/h -> statutory min 35m
        assertEquals(35.0, evaluator.calculateRequiredDistance(60.0, dry), 0.01);

        // 60 < Speed <= 80 km/h -> statutory min 55m
        assertEquals(55.0, evaluator.calculateRequiredDistance(70.0, dry), 0.01);
        assertEquals(55.0, evaluator.calculateRequiredDistance(80.0, dry), 0.01);

        // 80 < Speed <= 100 km/h -> statutory min 70m
        assertEquals(70.0, evaluator.calculateRequiredDistance(90.0, dry), 0.01);
        assertEquals(70.0, evaluator.calculateRequiredDistance(100.0, dry), 0.01);

        // 100 < Speed <= 120 km/h -> statutory min 100m
        assertEquals(100.0, evaluator.calculateRequiredDistance(110.0, dry), 0.01);
        assertEquals(100.0, evaluator.calculateRequiredDistance(120.0, dry), 0.01);
    }

    @Test
    public void testSpeedUnder60Kmh_TwoSecondRule() {
        WeatherCondition dry = WeatherCondition.DRY_NORMAL;

        // Speed = 36 km/h = 10 m/s -> 2.0s rule = 20m
        assertEquals(20.0, evaluator.calculateRequiredDistance(36.0, dry), 0.01);

        // Speed = 54 km/h = 15 m/s -> 2.0s rule = 30m
        assertEquals(30.0, evaluator.calculateRequiredDistance(54.0, dry), 0.01);
    }

    @Test
    public void testWeatherConditionMultiplierRainFog() {
        WeatherCondition rain = WeatherCondition.RAIN; // +30%
        WeatherCondition fog = WeatherCondition.FOG;   // +30%

        // Base 60 km/h is 35m -> in rain/fog: 35 * 1.30 = 45.5m
        assertEquals(35.0 * 1.30, evaluator.calculateRequiredDistance(60.0, rain), 0.01);
        assertEquals(35.0 * 1.30, evaluator.calculateRequiredDistance(60.0, fog), 0.01);

        // Base 100 km/h is 70m -> in rain: 70 * 1.30 = 91.0m
        assertEquals(91.0, evaluator.calculateRequiredDistance(100.0, rain), 0.01);

        // Base 120 km/h is 100m -> in rain: 100 * 1.30 = 130.0m
        assertEquals(130.0, evaluator.calculateRequiredDistance(120.0, rain), 0.01);
    }

    @Test
    public void testThreatLevelSafe() {
        // Driving at 80 km/h (required 55m), obstacle at 65m, equal speed
        LeadObstacle obstacle = new LeadObstacle(65.0, 0.0, LeadObstacle.ObstacleType.CAR, 0.95f);
        SafetyEvaluationResult result = evaluator.evaluate(80.0, obstacle, WeatherCondition.DRY_NORMAL);

        assertEquals(ThreatLevel.SAFE, result.getThreatLevel());
        assertEquals(55.0, result.getRequiredSafeDistanceMeters(), 0.01);
        assertEquals(65.0, result.getCurrentDistanceMeters(), 0.01);
    }

    @Test
    public void testThreatLevelWarningWhenDistanceUnderStatutory() {
        // Driving at 80 km/h (required 55m), obstacle at 45m (under 55m, but above 50% = 27.5m)
        LeadObstacle obstacle = new LeadObstacle(45.0, 0.0, LeadObstacle.ObstacleType.CAR, 0.95f);
        SafetyEvaluationResult result = evaluator.evaluate(80.0, obstacle, WeatherCondition.DRY_NORMAL);

        assertEquals(ThreatLevel.WARNING, result.getThreatLevel());
        assertTrue(result.isSafeDistanceViolated());
    }

    @Test
    public void testThreatLevelDangerWhenDistanceCriticallyShort() {
        // Driving at 80 km/h (required 55m), obstacle at 20m (less than 50% of 55m)
        LeadObstacle obstacle = new LeadObstacle(20.0, 0.0, LeadObstacle.ObstacleType.CAR, 0.95f);
        SafetyEvaluationResult result = evaluator.evaluate(80.0, obstacle, WeatherCondition.DRY_NORMAL);

        assertEquals(ThreatLevel.DANGER, result.getThreatLevel());
    }

    @Test
    public void testThreatLevelDangerWhenTtcUnderCriticalThreshold() {
        // Driving at 72 km/h (20 m/s), closing at 20 m/s relative speed to stationary vehicle at 25m
        // TTC = 25 / 20 = 1.25s (under 1.5s critical threshold)
        LeadObstacle obstacle = new LeadObstacle(25.0, 72.0, LeadObstacle.ObstacleType.CAR, 0.95f);
        SafetyEvaluationResult result = evaluator.evaluate(72.0, obstacle, WeatherCondition.DRY_NORMAL);

        assertEquals(ThreatLevel.DANGER, result.getThreatLevel());
        assertEquals(1.25, result.getTimeToCollisionSeconds(), 0.05);
    }

    @Test
    public void testClearRoadEvaluation() {
        SafetyEvaluationResult result = evaluator.evaluate(90.0, null, WeatherCondition.DRY_NORMAL);
        assertEquals(ThreatLevel.SAFE, result.getThreatLevel());
        assertEquals(70.0, result.getRequiredSafeDistanceMeters(), 0.01);
        assertTrue(Double.isInfinite(result.getCurrentDistanceMeters()));
    }
}
