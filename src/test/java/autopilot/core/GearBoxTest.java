package autopilot.core;

import autopilot.exception.IllegalGearChangeException;
import autopilot.sensor.SpeedSensor;
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

class GearBoxTest {

    // Speed Domain & Boundary Constants
    private static final double STATIONARY_SPEED_KMH = 0.0;
    private static final double REVERSE_ENTRY_SPEED_LIMIT_KMH = 5.0;
    private static final double SPEED_BOUNDARY_DELTA = 0.01;

    private static final double MAX_VALID_REVERSE_ENTRY_SPEED_KMH =
            REVERSE_ENTRY_SPEED_LIMIT_KMH - SPEED_BOUNDARY_DELTA; // 4.99 km/h
    private static final double MIN_MOVING_SPEED_KMH =
            STATIONARY_SPEED_KMH + SPEED_BOUNDARY_DELTA;          // 0.01 km/h
    private static final double MIN_INVALID_NEGATIVE_SPEED_KMH =
            STATIONARY_SPEED_KMH - SPEED_BOUNDARY_DELTA;          // -0.01 km/h
    private static final double NORMAL_CRUISING_SPEED_KMH = 20.0;

    private ConfigurableSpeedSensor speedSensor;
    private GearBox gearBox;

    @BeforeEach
    void setUp() {
        speedSensor = new ConfigurableSpeedSensor();
        gearBox = new GearBox(speedSensor);
    }

    @Test
    void changeTo_NullTargetGear_ThrowsNullPointerException() {
        assertThrows(NullPointerException.class,
                () -> gearBox.changeTo(null));
    }

    @ParameterizedTest
    @EnumSource(GearBox.Gear.class)
    void changeTo_SameGearAsCurrent_NoOperation(GearBox.Gear gear) {
        setCurrentGear(gear);

        assertDoesNotThrow(() -> gearBox.changeTo(gear));
        assertEquals(gear, gearBox.getCurrentGear());
    }

    @ParameterizedTest
    @ValueSource(doubles = {
            MIN_INVALID_NEGATIVE_SPEED_KMH,
            Double.NaN,
            Double.POSITIVE_INFINITY,
            Double.NEGATIVE_INFINITY
    })
    void changeTo_SensorReturnsInvalidSpeed_ThrowsIllegalArgumentException(double invalidSpeed) {
        speedSensor.setSpeedKmh(invalidSpeed);

        assertThrows(IllegalArgumentException.class,
                () -> gearBox.changeTo(GearBox.Gear.R));
        assertEquals(GearBox.Gear.P, gearBox.getCurrentGear());
    }

    @ParameterizedTest
    @MethodSource("validGearTransitionsProvider")
    void changeTo_AllowedTransitionWithValidSpeed_UpdatesCurrentGear(
            GearBox.Gear startGear, GearBox.Gear targetGear, double validSpeedKmh) {

        setCurrentGear(startGear);
        speedSensor.setSpeedKmh(validSpeedKmh);

        gearBox.changeTo(targetGear);

        assertEquals(targetGear, gearBox.getCurrentGear());
    }

    private static Stream<Arguments> validGearTransitionsProvider() {
        return Stream.of(
                Arguments.of(GearBox.Gear.P, GearBox.Gear.R, STATIONARY_SPEED_KMH),
                Arguments.of(GearBox.Gear.P, GearBox.Gear.R, MAX_VALID_REVERSE_ENTRY_SPEED_KMH),
                Arguments.of(GearBox.Gear.R, GearBox.Gear.D, STATIONARY_SPEED_KMH),
                Arguments.of(GearBox.Gear.D, GearBox.Gear.R, STATIONARY_SPEED_KMH),
                Arguments.of(GearBox.Gear.R, GearBox.Gear.N, NORMAL_CRUISING_SPEED_KMH),
                Arguments.of(GearBox.Gear.N, GearBox.Gear.D, NORMAL_CRUISING_SPEED_KMH)
        );
    }

    @ParameterizedTest
    @MethodSource("exceedingSpeedTransitionsProvider")
    void changeTo_AllowedTransitionWithExceedingSpeed_ThrowsIllegalGearChangeException(
            GearBox.Gear startGear, GearBox.Gear targetGear, double exceedingSpeedKmh) {

        setCurrentGear(startGear);
        speedSensor.setSpeedKmh(exceedingSpeedKmh);

        assertThrows(IllegalGearChangeException.class,
                () -> gearBox.changeTo(targetGear));
        assertEquals(startGear, gearBox.getCurrentGear());
    }

    private static Stream<Arguments> exceedingSpeedTransitionsProvider() {
        return Stream.of(
                Arguments.of(GearBox.Gear.P, GearBox.Gear.R, REVERSE_ENTRY_SPEED_LIMIT_KMH),
                Arguments.of(GearBox.Gear.R, GearBox.Gear.D, MIN_MOVING_SPEED_KMH),
                Arguments.of(GearBox.Gear.D, GearBox.Gear.R, MIN_MOVING_SPEED_KMH)
        );
    }

    @ParameterizedTest
    @MethodSource("unsupportedGearPairsProvider")
    void changeTo_UnsupportedGearTransition_ThrowsIllegalGearChangeException(
            GearBox.Gear startGear, GearBox.Gear targetGear) {

        setCurrentGear(startGear);
        speedSensor.setSpeedKmh(STATIONARY_SPEED_KMH);

        assertThrows(IllegalGearChangeException.class,
                () -> gearBox.changeTo(targetGear));
        assertEquals(startGear, gearBox.getCurrentGear());
    }

    private static Stream<Arguments> unsupportedGearPairsProvider() {
        return Stream.of(
                Arguments.of(GearBox.Gear.P, GearBox.Gear.N),
                Arguments.of(GearBox.Gear.P, GearBox.Gear.D),
                Arguments.of(GearBox.Gear.R, GearBox.Gear.P),
                Arguments.of(GearBox.Gear.N, GearBox.Gear.P),
                Arguments.of(GearBox.Gear.N, GearBox.Gear.R),
                Arguments.of(GearBox.Gear.D, GearBox.Gear.P),
                Arguments.of(GearBox.Gear.D, GearBox.Gear.N)
        );
    }

    private void setCurrentGear(GearBox.Gear desiredGear) {
        try {
            Field currentGearField = GearBox.class.getDeclaredField("currentGear");
            currentGearField.setAccessible(true);
            currentGearField.set(gearBox, desiredGear);
        } catch (ReflectiveOperationException e) {
            throw new RuntimeException("Failed to set currentGear via reflection", e);
        }
    }

    private static class ConfigurableSpeedSensor implements SpeedSensor {
        private double speedKmh = STATIONARY_SPEED_KMH;

        void setSpeedKmh(double speedKmh) {
            this.speedKmh = speedKmh;
        }

        @Override
        public double getSpeedKmh() {
            return speedKmh;
        }
    }
}