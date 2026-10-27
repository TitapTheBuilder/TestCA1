package autopilot.core;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvFileSource;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.ValueSource;

class SteeringControllerTest {

    private static final double LOW_SPEED_KMH = 50.0;
    private static final double HIGH_SPEED_KMH = 130.0;
    private static final double ANY_VALID_ANGLE = 10.0;
    private static final double FAR_ANGLE = 180.0;
    private static final double ONE_BACK_FROM_FAR_ANGLE = 120.0;
    private static final double CENTER_ANGLE = 0.0;
    private static final double REQUEST_BEYOND_ONE_STEP = 100.0;
    private static final double REGULAR_ANGLE_TARGET = 30.0;

    private SteeringController steering;

    @BeforeEach
    void setUp() {
        steering = new SteeringController();
    }

    @ParameterizedTest
    @ValueSource(doubles = {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void update_invalidAngle_throwsIllegalArgumentException(double invalidAngle) {
        assertThrows(IllegalArgumentException.class, () -> steering.update(invalidAngle, LOW_SPEED_KMH));
    }

    @ParameterizedTest
    @MethodSource("invalidSpeeds")
    void update_invalidSpeeds_throwsIllegalArgumentException(double invalidSpeed) {
        assertThrows(IllegalArgumentException.class,
                () -> steering.update(ANY_VALID_ANGLE, invalidSpeed));
    }

    static Stream<Double> invalidSpeeds() {
        return Stream.of(-0.1, -1.0, Double.NaN,
                Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY);
    }

    @Test
    void update_noChangeCausedByInvalidInput_leavesSteeringStateUnchanged() {
        steering.update(REQUEST_BEYOND_ONE_STEP, LOW_SPEED_KMH);

        assertThrows(IllegalArgumentException.class,
                () -> steering.update(Double.NaN, LOW_SPEED_KMH));

        assertAll("state after rejected input",
                () -> assertEquals(60.0, steering.getCurrentAngleDegrees()),
                () -> assertEquals(100.0, steering.getTargetAngleDegrees()));
    }

    @ParameterizedTest(name = "request {0} -> angle {1}")
    @CsvFileSource(resources = "/SmallerThanTickFromCenterSteerMoves.csv", numLinesToSkip = 1)
    void update_validSmallerThanTickLimitMovesFromCenter_movesAtTheRequestedAngle(double requestedAngle, double expectedAngle) {
        double angle = steering.update(requestedAngle, LOW_SPEED_KMH);
        assertEquals(expectedAngle, angle);
    }

    @ParameterizedTest(name = "request {0} -> angle {1}")
    @CsvSource({
            "61.0,   60.0",
            "65.0,   60.0",
            "100.0,  60.0",
            "-61.0, -60.0",
            "-65.0, -60.0"
    })
    void update_validBiggerThanTickLimitMovesFromCenter_movesAtMostOneStepTowardRequest(double requestedAngle, double expectedAngle) {
        double angle = steering.update(requestedAngle, LOW_SPEED_KMH);
        assertEquals(expectedAngle, angle);
    }

    @Test
    void update_reachFinalAngle_reachesTargetStepByStepAcrossTicks() {
        double firstTick = steering.update(FAR_ANGLE, LOW_SPEED_KMH);
        double secondTick = steering.update(FAR_ANGLE, LOW_SPEED_KMH);
        double thirdTick = steering.update(FAR_ANGLE, LOW_SPEED_KMH);

        assertEquals(List.of(60.0, 120.0, 180.0), List.of(firstTick, secondTick, thirdTick));
    }

    @Test
    void update_reachCurrentTarget_staysAtTarget() {
        steering.update(REGULAR_ANGLE_TARGET, LOW_SPEED_KMH);
        double angle = steering.update(REGULAR_ANGLE_TARGET, LOW_SPEED_KMH);
        assertEquals(REGULAR_ANGLE_TARGET, angle);
    }

    @ParameterizedTest(name = "request {0} -> target {1}")
    @CsvSource({
            "449.0,   449.0",
            "450.0,   450.0",
            "451.0,   450.0",
            "1000.0,  450.0",
            "-450.0, -450.0",
            "-451.0, -450.0"
    })
    void update_lowSpeedAngleLimit_clampsTargetToPhysicalLimit(
            double requestedAngle, double expectedTarget) {
        steering.update(requestedAngle, LOW_SPEED_KMH);
        assertEquals(expectedTarget, steering.getTargetAngleDegrees());
    }

    @ParameterizedTest(name = "speed {0}, request {1} -> target {2}")
    @CsvFileSource(resources = "/SpeedEffectOnTargetThreshold.csv", numLinesToSkip = 1)
    void update_speedAroundHighSpeedThreshold_appliesMatchingAngleLimit(
            double speedKmh, double requestedAngle, double expectedTarget) {
        steering.update(requestedAngle, speedKmh);
        assertEquals(expectedTarget, steering.getTargetAngleDegrees());
    }

    @Test
    void update_speedRisesWhileWheelIsTurnedFar_turnsBackOneStepTowardHighSpeedLimit() {
        turnWheelToFarAngle();
        double angle = steering.update(FAR_ANGLE, HIGH_SPEED_KMH);
        assertEquals(120.0, angle);
    }

    @ParameterizedTest
    @ValueSource(doubles = {-60.0, -30.0, 30.0, 60.0})
    void selfCenter_startFromOneStepValidAngles_returnsToZero(double startAngle) {
        steering.update(startAngle, LOW_SPEED_KMH);
        double angle = steering.selfCenter();
        assertEquals(CENTER_ANGLE, angle);
    }

    @Test
    void selfCenter_wheelFarFromCenterAfterOneTick_movesOneStepTowardZero() {
        turnWheelToFarAngle();
        double angle = steering.selfCenter();
        assertEquals(ONE_BACK_FROM_FAR_ANGLE, angle);
    }

    @Test
    void selfCenter_wheelAlreadyCentered_staysAtZero() {
        double angle = steering.selfCenter();
        assertEquals(CENTER_ANGLE, angle);
    }

    @Test
    void selfCenter_wheelFarFromCenterAfterManyTicksToTheCenter_reachesZeroStepByStep() {
        turnWheelToFarAngle();

        double firstTick = steering.selfCenter();
        double secondTick = steering.selfCenter();
        double thirdTick = steering.selfCenter();

        assertEquals(List.of(120.0, 60.0, 0.0), List.of(firstTick, secondTick, thirdTick));
    }

    @Test
    void selfCenter_afterPreviousRequestCheckingTheTargetAngle_replacesTargetWithZero() {
        steering.update(REQUEST_BEYOND_ONE_STEP, LOW_SPEED_KMH);
        steering.selfCenter();
        assertEquals(CENTER_ANGLE, steering.getTargetAngleDegrees());
    }

    private void turnWheelToFarAngle() {
        steering.update(FAR_ANGLE, LOW_SPEED_KMH);
        steering.update(FAR_ANGLE, LOW_SPEED_KMH);
        steering.update(FAR_ANGLE, LOW_SPEED_KMH);
    }
}