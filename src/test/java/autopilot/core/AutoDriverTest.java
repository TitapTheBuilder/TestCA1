package autopilot.core;

import autopilot.exception.SensorFailureException;
import autopilot.sensor.LaneSensor;
import autopilot.sensor.ObstacleSensor;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

class AutoDriverTest {

    private static final double DEFAULT_SPEED_KMH = 80.0;
    private static final double TARGET_SPEED_KMH = 80.0;
    private static final double SAFE_OBSTACLE_DISTANCE_METERS = 100.0;
    private static final double CENTERED_LANE_OFFSET_METERS = 0.0;

    private static final double ZERO_BRAKE_PRESSURE = 0.0;
    private static final double ZERO_STEERING_ANGLE = 0.0;
    private static final double SAFE_MODE_BRAKE_PRESSURE = 30.0;

    private static final String SPEED_SENSOR_FAILURE_MESSAGE = "Speed sensor failed";
    private static final String OBSTACLE_SENSOR_FAILURE_MESSAGE = "Obstacle sensor failed";

    private static final AutoDriver.DriveResult EXPECTED_SAFE_MODE_RESULT =
            new AutoDriver.DriveResult(
                    ZERO_STEERING_ANGLE,
                    SAFE_MODE_BRAKE_PRESSURE,
                    AdaptiveCruise.Mode.BRAKE,
                    false,
                    true
            );

    private SpeedSensor speedSensor;
    private ObstacleSensor obstacleSensor;
    private LaneSensor laneSensor;
    private AutoDriver autoDriver;

    @BeforeEach
    void setUp() {
        speedSensor = new SpeedSensor() {
            @Override
            public double getSpeedKmh() {
                return DEFAULT_SPEED_KMH;
            }
        };

        obstacleSensor = new ObstacleSensor() {
            @Override
            public double getDistanceMeters() {
                return SAFE_OBSTACLE_DISTANCE_METERS;
            }
        };

        laneSensor = new LaneSensor() {
            @Override
            public double getLaneOffsetMeters() {
                return CENTERED_LANE_OFFSET_METERS;
            }
        };

        autoDriver = new AutoDriver(
                speedSensor,
                obstacleSensor,
                laneSensor,
                TARGET_SPEED_KMH
        );
    }

    @Test
    void tick_whenSensorsAreNormal_drivesWithZeroBrakePressureAndSafeModeOff() {
        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertFalse(result.safeMode());
        assertEquals(ZERO_BRAKE_PRESSURE, result.brakePressure());
    }

    @ParameterizedTest
    @CsvSource({
            "1.0,  -1.0",
            "-1.0,  1.0"
    })
    void tick_whenLaneOffsetExceedsThreshold_steersInOppositeDirection(
            double laneOffsetMeters,
            double expectedSteeringSign) {
        laneSensor = new LaneSensor() {
            @Override
            public double getLaneOffsetMeters() {
                return laneOffsetMeters;
            }
        };

        autoDriver = new AutoDriver(
                speedSensor,
                obstacleSensor,
                laneSensor,
                TARGET_SPEED_KMH
        );

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertEquals(expectedSteeringSign, Math.signum(result.steeringAngleDegrees()));
    }

    @ParameterizedTest
    @MethodSource("provideAutoDriversWithFailingSensors")
    void tick_whenAnySensorFails_returnsSafeModeDriveResult(AutoDriver driverWithFailingSensor) {
        AutoDriver.DriveResult result = driverWithFailingSensor.tick(GearBox.Gear.D);

        assertEquals(EXPECTED_SAFE_MODE_RESULT, result);
    }

    private static Stream<Arguments> provideAutoDriversWithFailingSensors() {
        SpeedSensor normalSpeedSensor = new SpeedSensor() {
            @Override
            public double getSpeedKmh() {
                return DEFAULT_SPEED_KMH;
            }
        };

        ObstacleSensor normalObstacleSensor = new ObstacleSensor() {
            @Override
            public double getDistanceMeters() {
                return SAFE_OBSTACLE_DISTANCE_METERS;
            }
        };

        LaneSensor normalLaneSensor = new LaneSensor() {
            @Override
            public double getLaneOffsetMeters() {
                return CENTERED_LANE_OFFSET_METERS;
            }
        };

        SpeedSensor failingSpeedSensor = new SpeedSensor() {
            @Override
            public double getSpeedKmh() {
                throw new SensorFailureException(SPEED_SENSOR_FAILURE_MESSAGE);
            }
        };

        ObstacleSensor failingObstacleSensor = new ObstacleSensor() {
            @Override
            public double getDistanceMeters() {
                throw new SensorFailureException(OBSTACLE_SENSOR_FAILURE_MESSAGE);
            }
        };

        AutoDriver driverWithFailingSpeedSensor = new AutoDriver(
                failingSpeedSensor,
                normalObstacleSensor,
                normalLaneSensor,
                TARGET_SPEED_KMH
        );

        AutoDriver driverWithFailingObstacleSensor = new AutoDriver(
                normalSpeedSensor,
                failingObstacleSensor,
                normalLaneSensor,
                TARGET_SPEED_KMH
        );

        return Stream.of(
                Arguments.of(driverWithFailingSpeedSensor),
                Arguments.of(driverWithFailingObstacleSensor)
        );
    }
}