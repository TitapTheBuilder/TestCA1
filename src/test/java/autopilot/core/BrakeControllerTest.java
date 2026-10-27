package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.ValueSource;

import java.lang.reflect.Field;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

class BrakeControllerTest {

    // Domain Threshold & Pulse Constants
    private static final double MIN_VALID_VALUE = 0.0;
    private static final double MAX_PRESSURE = 100.0;
    private static final double ABS_PRESSURE_THRESHOLD = 80.0;
    private static final double ABS_SPEED_THRESHOLD_KMH = 30.0;
    private static final double ABS_HIGH_PULSE = 90.0;
    private static final double ABS_LOW_PULSE = 60.0;

    // Boundary Test Constants
    private static final double BOUNDARY_DELTA = 0.01;
    private static final double FLOATING_POINT_DELTA = 1e-6;

    private static final double MIN_INVALID_NEGATIVE_VALUE =
            MIN_VALID_VALUE - BOUNDARY_DELTA;               // -0.01
    private static final double MIN_ABS_TRIGGER_PRESSURE =
            ABS_PRESSURE_THRESHOLD + BOUNDARY_DELTA;        // 80.01
    private static final double MIN_ABS_TRIGGER_SPEED_KMH =
            ABS_SPEED_THRESHOLD_KMH + BOUNDARY_DELTA;       // 30.01
    private static final double ABOVE_MAX_PRESSURE =
            MAX_PRESSURE + BOUNDARY_DELTA;                  // 100.01

    private static final double NORMAL_PRESSURE = 50.0;
    private static final double NORMAL_SPEED_KMH = 20.0;

    private BrakeController brakeController;

    @BeforeEach
    void setUp() {
        brakeController = new BrakeController();
    }

    @ParameterizedTest
    @ValueSource(doubles = {
            MIN_INVALID_NEGATIVE_VALUE,
            Double.NaN,
            Double.POSITIVE_INFINITY,
            Double.NEGATIVE_INFINITY
    })
    void apply_InvalidRequestedPressure_ThrowsIllegalArgumentException(double invalidPressure) {
        assertThrows(IllegalArgumentException.class,
                () -> brakeController.apply(invalidPressure, NORMAL_SPEED_KMH, GearBox.Gear.D));
    }

    @ParameterizedTest
    @ValueSource(doubles = {
            MIN_INVALID_NEGATIVE_VALUE,
            Double.NaN,
            Double.POSITIVE_INFINITY,
            Double.NEGATIVE_INFINITY
    })
    void apply_InvalidSpeed_ThrowsIllegalArgumentException(double invalidSpeed) {
        assertThrows(IllegalArgumentException.class,
                () -> brakeController.apply(NORMAL_PRESSURE, invalidSpeed, GearBox.Gear.D));
    }

    @Test
    void apply_NullGear_ThrowsNullPointerException() {
        assertThrows(NullPointerException.class,
                () -> brakeController.apply(NORMAL_PRESSURE, NORMAL_SPEED_KMH, null));
    }

    @Test
    void apply_GearInPark_ThrowsIllegalStateException() {
        assertThrows(IllegalStateException.class,
                () -> brakeController.apply(NORMAL_PRESSURE, NORMAL_SPEED_KMH, GearBox.Gear.P));
    }

    @ParameterizedTest
    @MethodSource("nonAbsWithinMaxPressureProvider")
    void apply_NonAbsWithinMaxPressure_AppliesRequestedPressure(
            double requestedPressure, double speedKmh, GearBox.Gear gear) {

        double appliedPressure = brakeController.apply(requestedPressure, speedKmh, gear);

        assertEquals(requestedPressure, appliedPressure, FLOATING_POINT_DELTA);
        assertEquals(requestedPressure, brakeController.getPressure(), FLOATING_POINT_DELTA);
    }

    private static Stream<Arguments> nonAbsWithinMaxPressureProvider() {
        return Stream.of(
                Arguments.of(MIN_VALID_VALUE, MIN_VALID_VALUE, GearBox.Gear.D),
                Arguments.of(ABS_PRESSURE_THRESHOLD, MIN_ABS_TRIGGER_SPEED_KMH, GearBox.Gear.R),
                Arguments.of(MIN_ABS_TRIGGER_PRESSURE, ABS_SPEED_THRESHOLD_KMH, GearBox.Gear.N),
                Arguments.of(MAX_PRESSURE, ABS_SPEED_THRESHOLD_KMH, GearBox.Gear.D)
        );
    }

    @Test
    void apply_NonAbsExceedingMaxPressure_CapsPressureAtMax() {
        double appliedPressure = brakeController.apply(
                ABOVE_MAX_PRESSURE, ABS_SPEED_THRESHOLD_KMH, GearBox.Gear.D);

        assertEquals(MAX_PRESSURE, appliedPressure, FLOATING_POINT_DELTA);
        assertEquals(MAX_PRESSURE, brakeController.getPressure(), FLOATING_POINT_DELTA);
    }

    @Test
    void apply_NonAbsWhenNextPulseIsLow_ResetsNextAbsPulseToHigh() {
        setNextAbsPulseIsHigh(false);

        brakeController.apply(NORMAL_PRESSURE, NORMAL_SPEED_KMH, GearBox.Gear.D);

        assertTrue(getNextAbsPulseIsHigh());
    }

    @Test
    void apply_AbsConditionsWhenNextPulseIsHigh_AppliesHighPulseAndFlipsFlag() {
        setNextAbsPulseIsHigh(true);

        double appliedPressure = brakeController.apply(
                MIN_ABS_TRIGGER_PRESSURE, MIN_ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);

        assertEquals(ABS_HIGH_PULSE, appliedPressure, FLOATING_POINT_DELTA);
        assertFalse(getNextAbsPulseIsHigh());
    }

    @Test
    void apply_AbsConditionsWhenNextPulseIsLow_AppliesLowPulseAndFlipsFlag() {
        setNextAbsPulseIsHigh(false);

        double appliedPressure = brakeController.apply(
                MIN_ABS_TRIGGER_PRESSURE, MIN_ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);

        assertEquals(ABS_LOW_PULSE, appliedPressure, FLOATING_POINT_DELTA);
        assertTrue(getNextAbsPulseIsHigh());
    }

    @Test
    void emergencyBrake_NullGear_ThrowsNullPointerException() {
        assertThrows(NullPointerException.class,
                () -> brakeController.emergencyBrake(null));
    }

    @Test
    void emergencyBrake_GearInPark_ThrowsIllegalStateException() {
        assertThrows(IllegalStateException.class,
                () -> brakeController.emergencyBrake(GearBox.Gear.P));
    }

    @ParameterizedTest
    @EnumSource(value = GearBox.Gear.class, names = {"P"}, mode = EnumSource.Mode.EXCLUDE)
    void emergencyBrake_ValidDrivingGear_SetsMaxPressureWarningAndResetsAbsPulse(GearBox.Gear validGear) {
        setNextAbsPulseIsHigh(false);

        brakeController.emergencyBrake(validGear);

        assertEquals(MAX_PRESSURE, brakeController.getPressure(), FLOATING_POINT_DELTA);
        assertTrue(brakeController.isWarningActive());
        assertTrue(getNextAbsPulseIsHigh());
    }

    private void setNextAbsPulseIsHigh(boolean value) {
        try {
            Field field = BrakeController.class.getDeclaredField("nextAbsPulseIsHigh");
            field.setAccessible(true);
            field.setBoolean(brakeController, value);
        } catch (ReflectiveOperationException e) {
            throw new RuntimeException("Failed to access nextAbsPulseIsHigh via reflection", e);
        }
    }

    private boolean getNextAbsPulseIsHigh() {
        try {
            Field field = BrakeController.class.getDeclaredField("nextAbsPulseIsHigh");
            field.setAccessible(true);
            return field.getBoolean(brakeController);
        } catch (ReflectiveOperationException e) {
            throw new RuntimeException("Failed to access nextAbsPulseIsHigh via reflection", e);
        }
    }
}