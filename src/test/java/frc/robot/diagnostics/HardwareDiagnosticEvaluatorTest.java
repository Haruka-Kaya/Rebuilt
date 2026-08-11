package frc.robot.diagnostics;

import static frc.robot.diagnostics.HardwareDiagnosticEvaluator.MAX_APPLIED_OUTPUT_ERROR;
import static frc.robot.diagnostics.HardwareDiagnosticEvaluator.MIN_APPLIED_OUTPUT_FRACTION;
import static frc.robot.diagnostics.HardwareDiagnosticEvaluator.MIN_OBSERVED_VELOCITY_RPM;
import static frc.robot.diagnostics.HardwareDiagnosticEvaluator.MIN_VALID_BUS_VOLTAGE;
import static frc.robot.diagnostics.HardwareDiagnosticEvaluator.STALL_CURRENT_FRACTION;
import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.diagnostics.HardwareDiagnosticEvaluator.MotionResult;
import frc.robot.diagnostics.HardwareDiagnosticEvaluator.Snapshot;
import org.junit.jupiter.api.Test;

class HardwareDiagnosticEvaluatorTest {
  private static final double REQUESTED_DUTY = 0.03;
  private static final double CURRENT_LIMIT_AMPS = 10.0;

  @Test
  void zeroDutyIsAnIntentionalMotionSkip() {
    assertEquals(
        MotionResult.SKIPPED,
        HardwareDiagnosticEvaluator.evaluateOpenLoop(null, 0.0, CURRENT_LIMIT_AMPS));
    assertEquals(
        MotionResult.SKIPPED,
        HardwareDiagnosticEvaluator.evaluateOpenLoop(null, -0.0, CURRENT_LIMIT_AMPS));
    assertEquals(
        MotionResult.STALL_SUSPECTED,
        HardwareDiagnosticEvaluator.evaluateOpenLoop(
            sample(true, 0.0, CURRENT_LIMIT_AMPS, 0.0, 12.0, true),
            0.0,
            CURRENT_LIMIT_AMPS));
  }

  @Test
  void rejectsMissingUnreadyNonFiniteAndLowVoltageSamples() {
    assertEquals(
        MotionResult.FAIL_NOT_READY,
        HardwareDiagnosticEvaluator.evaluateOpenLoop(null, REQUESTED_DUTY, CURRENT_LIMIT_AMPS));
    assertResultForSample(MotionResult.FAIL_NOT_READY, sample(false, 0.03, 1.0, 20.0, 12.0, true));
    assertResultForSample(
        MotionResult.FAIL_NOT_READY, sample(true, Double.NaN, 1.0, 20.0, 12.0, true));
    assertResultForSample(
        MotionResult.FAIL_NOT_READY, sample(true, 0.03, Double.NaN, 20.0, 12.0, true));
    assertResultForSample(
        MotionResult.FAIL_NOT_READY, sample(true, 0.03, 1.0, Double.NaN, 12.0, true));
    assertResultForSample(
        MotionResult.FAIL_NOT_READY, sample(true, 0.03, 1.0, 20.0, Double.NaN, true));
    assertResultForSample(
        MotionResult.FAIL_NOT_READY,
        sample(true, 0.03, 1.0, 20.0, Math.nextDown(MIN_VALID_BUS_VOLTAGE), true));
    assertResultForSample(
        MotionResult.PASS_OBSERVED,
        sample(true, 0.03, 1.0, 20.0, MIN_VALID_BUS_VOLTAGE, true));
  }

  @Test
  void rejectsInvalidConfigurationAndRejectedCommands() {
    Snapshot valid = sample(true, 0.03, 1.0, 20.0, 12.0, true);
    assertEquals(
        MotionResult.FAIL_COMMAND_REJECTED,
        HardwareDiagnosticEvaluator.evaluateOpenLoop(valid, Double.NaN, CURRENT_LIMIT_AMPS));
    assertEquals(
        MotionResult.FAIL_COMMAND_REJECTED,
        HardwareDiagnosticEvaluator.evaluateOpenLoop(valid, 1.01, CURRENT_LIMIT_AMPS));
    assertEquals(
        MotionResult.FAIL_COMMAND_REJECTED,
        HardwareDiagnosticEvaluator.evaluateOpenLoop(valid, REQUESTED_DUTY, Double.NaN));
    assertEquals(
        MotionResult.FAIL_COMMAND_REJECTED,
        HardwareDiagnosticEvaluator.evaluateOpenLoop(valid, REQUESTED_DUTY, 0.0));
    assertResultForSample(
        MotionResult.FAIL_COMMAND_REJECTED, sample(true, 0.03, 1.0, 20.0, 12.0, false));
  }

  @Test
  void requiresAppliedOutputToMatchRequestedDirectionAndRange() {
    double minimumApplied = REQUESTED_DUTY * MIN_APPLIED_OUTPUT_FRACTION;
    double maximumApplied = REQUESTED_DUTY + MAX_APPLIED_OUTPUT_ERROR;

    assertResultForSample(
        MotionResult.FAIL_COMMAND_REJECTED,
        sample(true, -REQUESTED_DUTY, 1.0, 20.0, 12.0, true));
    assertResultForSample(
        MotionResult.FAIL_COMMAND_REJECTED,
        sample(true, Math.nextDown(minimumApplied), 1.0, 20.0, 12.0, true));
    assertResultForSample(
        MotionResult.PASS_OBSERVED,
        sample(true, minimumApplied, 1.0, 20.0, 12.0, true));
    assertResultForSample(
        MotionResult.PASS_OBSERVED,
        sample(true, maximumApplied, 1.0, 20.0, 12.0, true));
    assertResultForSample(
        MotionResult.FAIL_COMMAND_REJECTED,
        sample(true, Math.nextUp(maximumApplied), 1.0, 20.0, 12.0, true));

    assertEquals(
        MotionResult.PASS_OBSERVED,
        HardwareDiagnosticEvaluator.evaluateOpenLoop(
            sample(true, -REQUESTED_DUTY, 1.0, -20.0, 12.0, true),
            -REQUESTED_DUTY,
            CURRENT_LIMIT_AMPS));
  }

  @Test
  void observesMotionAtTheVelocityBoundary() {
    assertResultForSample(
        MotionResult.INCONCLUSIVE_NO_MOTION,
        sample(
            true,
            REQUESTED_DUTY,
            1.0,
            Math.nextDown(MIN_OBSERVED_VELOCITY_RPM),
            12.0,
            true));
    assertResultForSample(
        MotionResult.PASS_OBSERVED,
        sample(true, REQUESTED_DUTY, 1.0, MIN_OBSERVED_VELOCITY_RPM, 12.0, true));
    assertResultForSample(
        MotionResult.FAIL_DIRECTION_MISMATCH,
        sample(true, REQUESTED_DUTY, 1.0, -MIN_OBSERVED_VELOCITY_RPM, 12.0, true));
  }

  @Test
  void separatesStaticFrictionFromAHighCurrentSuspectedStall() {
    double stallCurrent = CURRENT_LIMIT_AMPS * STALL_CURRENT_FRACTION;
    assertResultForSample(
        MotionResult.INCONCLUSIVE_NO_MOTION,
        sample(true, REQUESTED_DUTY, Math.nextDown(stallCurrent), 0.0, 12.0, true));
    assertResultForSample(
        MotionResult.STALL_SUSPECTED,
        sample(true, REQUESTED_DUTY, stallCurrent, 0.0, 12.0, true));
    assertResultForSample(
        MotionResult.STALL_SUSPECTED,
        sample(true, REQUESTED_DUTY, -stallCurrent, 0.0, 12.0, true));
    assertResultForSample(
        MotionResult.STALL_SUSPECTED,
        sample(true, 0.0, stallCurrent, 0.0, 5.0, false));
  }

  private static Snapshot sample(
      boolean ready,
      double appliedOutput,
      double currentAmps,
      double velocityRpm,
      double busVoltage,
      boolean commandAccepted) {
    return new Snapshot(
        32, ready, appliedOutput, currentAmps, velocityRpm, busVoltage, commandAccepted);
  }

  private static void assertResultForSample(MotionResult expected, Snapshot snapshot) {
    assertEquals(
        expected,
        HardwareDiagnosticEvaluator.evaluateOpenLoop(
            snapshot, REQUESTED_DUTY, CURRENT_LIMIT_AMPS));
  }
}
