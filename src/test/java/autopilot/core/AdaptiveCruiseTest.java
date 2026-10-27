package autopilot.core;

import autopilot.core.AdaptiveCruise.Decision;
import autopilot.core.AdaptiveCruise.Mode;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.ValueSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.params.provider.CsvFileSource;

class AdaptiveCruiseTest {

    private static final double DEFAULT_TARGET_SPEED = 100.0;
    private static final double SPEED_BELOW_TARGET = 80.0;
    private static final double SPEED_ABOVE_TARGET = 110.0;

    private static final double SAFE_DISTANCE_AT_80 = 40.0;
    private static final double SAFE_DISTANCE_AT_100 = 50.0;
    private static final double TRIGGER_BRAKE_DISTANCE = 40.0;
    private static final double HYSTERESIS_DISTANCE = 55.0;
    private static final double RELEASE_DISTANCE_AT_100 = 60.0;
    private static final double CLEAR_ROAD_DISTANCE = 70.0;

    private static final double EMERGENCY_PRESSURE = 100.0;
    private static final double HYSTERESIS_PRESSURE = 10.0;
    private static final double HOLD_AMOUNT = 0.0;
    private static final double DELTA = 1e-6;

    private AdaptiveCruise cruise;

    @BeforeEach
    void setUp() {
        cruise = new AdaptiveCruise(DEFAULT_TARGET_SPEED);
    }

    @ParameterizedTest
    @CsvSource({
            "0.0, 0.0",
            "60.0, 30.0",
            "100.0, 50.0"
    })
    void safeDistanceFor_ValidSpeed_ReturnsExpectedSafeDistance(double speed, double expectedDistance) {
        double safeDistanceResult = cruise.safeDistanceFor(speed);
        assertEquals(expectedDistance, safeDistanceResult, DELTA);
    }

    @ParameterizedTest
    @ValueSource(doubles = {-0.1, -50.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void safeDistanceFor_InvalidSpeed_ThrowsIllegalArgumentException(double invalidSpeed) {
        assertThrows(IllegalArgumentException.class, () -> cruise.safeDistanceFor(invalidSpeed));
    }

    @ParameterizedTest
    @MethodSource("provideInvalidSpeedAndDistance")
    void update_InvalidInputs_ThrowsIllegalArgumentException(double speed, double distance) {
        assertThrows(IllegalArgumentException.class, () -> cruise.update(speed, distance));
    }

    private static Stream<Arguments> provideInvalidSpeedAndDistance() {
        return Stream.of(
                Arguments.of(-1.0, SAFE_DISTANCE_AT_100),
                Arguments.of(Double.NaN, SAFE_DISTANCE_AT_100),
                Arguments.of(Double.POSITIVE_INFINITY, SAFE_DISTANCE_AT_100),
                Arguments.of(SPEED_BELOW_TARGET, -5.0),
                Arguments.of(SPEED_BELOW_TARGET, Double.NaN),
                Arguments.of(SPEED_BELOW_TARGET, Double.POSITIVE_INFINITY)
        );
    }

    @ParameterizedTest
    @CsvSource({
            "100.0, 25.0",
            "100.0, 10.0",
            "0.0, 0.0"
    })
    void update_DistanceAtOrBelowEmergencyThreshold_ReturnsEmergencyBrake(double speed, double distance) {
        Decision decision = cruise.update(speed, distance);
        assertEquals(new Decision(Mode.EMERGENCY_BRAKE, EMERGENCY_PRESSURE), decision);
    }

    @ParameterizedTest
    @CsvFileSource(resources = "/ProportionalBrakeData.csv", numLinesToSkip = 1)
    void update_DistanceBelowSafeAndAboveEmergency_ReturnsProportionalBrake(double speed, double distance, double expectedPressure) {
        Decision decision = cruise.update(speed, distance);
        assertEquals(new Decision(Mode.BRAKE, expectedPressure), decision);
    }

    @ParameterizedTest
    @ValueSource(doubles = {SAFE_DISTANCE_AT_100, HYSTERESIS_DISTANCE})
    void update_WhileBrakingAndDistanceBetweenSafeAndRelease_ReturnsHysteresisBrake(double currentDistance) {
        cruise.update(DEFAULT_TARGET_SPEED, TRIGGER_BRAKE_DISTANCE);
        Decision decision = cruise.update(DEFAULT_TARGET_SPEED, currentDistance);
        assertEquals(new Decision(Mode.BRAKE, HYSTERESIS_PRESSURE), decision);
    }

    @Test
    void update_WhileBrakingAndDistanceReachesReleaseThreshold_ReleasesBrakeAndHolds() {
        cruise.update(DEFAULT_TARGET_SPEED, TRIGGER_BRAKE_DISTANCE);
        Decision decision = cruise.update(DEFAULT_TARGET_SPEED, RELEASE_DISTANCE_AT_100);
        assertEquals(new Decision(Mode.HOLD, HOLD_AMOUNT), decision);
    }

    @ParameterizedTest
    @CsvSource({
            "80.0, 60.0, 5.0",
            "97.0, 60.0, 3.0"
    })
    void update_SafeDistanceAndSpeedBelowTarget_ReturnsAccelerate(
            double currentSpeed, double distance, double expectedAcceleration) {
        Decision decision = cruise.update(currentSpeed, distance);
        assertEquals(new Decision(Mode.ACCELERATE, expectedAcceleration), decision);
    }

    @ParameterizedTest
    @MethodSource("provideHoldScenarios")
    void update_WhenConditionsDoNotRequireSpeedChange_ReturnsHold(double speed, double distance) {
        Decision decision = cruise.update(speed, distance);
        assertEquals(new Decision(Mode.HOLD, HOLD_AMOUNT), decision);
    }

    private static Stream<Arguments> provideHoldScenarios() {
        return Stream.of(
                Arguments.of(SPEED_BELOW_TARGET, SAFE_DISTANCE_AT_80),
                Arguments.of(DEFAULT_TARGET_SPEED, CLEAR_ROAD_DISTANCE),
                Arguments.of(SPEED_ABOVE_TARGET, CLEAR_ROAD_DISTANCE)
        );
    }
}