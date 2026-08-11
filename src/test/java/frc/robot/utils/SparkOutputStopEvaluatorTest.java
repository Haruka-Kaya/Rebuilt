package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.utils.SparkOutputStopEvaluator.Evaluation;
import frc.robot.utils.SparkOutputStopEvaluator.Limits;
import frc.robot.utils.SparkOutputStopEvaluator.Observation;
import frc.robot.utils.SparkOutputStopEvaluator.Status;

class SparkOutputStopEvaluatorTest {
  private static final Limits LIMITS = new Limits(0.50, 0.01, 2.0);

  @Test
  void confirmsMatchingZeroEpochWithFreshNeutralTelemetry() {
    Evaluation result = SparkOutputStopEvaluator.evaluate(healthyObservation(), LIMITS);

    assertTrue(result.confirmed());
    assertEquals(Status.CONFIRMED, result.status());
  }

  @Test
  void rejectsAStopTokenSupersededByANewerNonzeroOutput() {
    Observation observation = observation(
        8L, 9L, 9L,
        true, false, false, false,
        4.0, 4.1, 4.2,
        0.0, true, 0.0,
        false, true, false);

    assertStatus(Status.REQUEST_SUPERSEDED, observation);
  }

  @Test
  void requiresAZeroForTheRequestedOutputEpoch() {
    Observation observation = observation(
        8L, 8L, 7L,
        true, false, false, false,
        4.0, 4.1, 4.2,
        0.0, true, 0.0,
        false, true, false);

    assertStatus(Status.ZERO_NOT_CONFIRMED_FOR_EPOCH, observation);
  }

  @Test
  void keepsPendingWhileAZeroWriteOrOutputUncertaintyRemains() {
    assertStatus(Status.ZERO_PENDING, observation(
        8L, 8L, 8L,
        true, true, false, false,
        4.0, 4.1, 4.2,
        0.0, true, 0.0,
        false, true, false));
    assertStatus(Status.ZERO_PENDING, observation(
        8L, 8L, 8L,
        true, false, true, false,
        4.0, 4.1, 4.2,
        0.0, true, 0.0,
        false, true, false));
    assertStatus(Status.ZERO_PENDING, observation(
        8L, 8L, 8L,
        true, false, false, true,
        4.0, 4.1, 4.2,
        0.0, true, 0.0,
        false, true, false));
  }

  @Test
  void requiresAFreshSampleCapturedAfterTheConfirmedZero() {
    assertStatus(Status.POST_ZERO_SAMPLE_PENDING, observation(
        8L, 8L, 8L,
        true, false, false, false,
        4.0, 3.99, 4.2,
        0.0, true, 0.0,
        false, true, false));
    assertStatus(Status.SAMPLE_STALE, observation(
        8L, 8L, 8L,
        true, false, false, false,
        4.0, 4.1, 4.61,
        0.0, true, 0.0,
        false, true, false));
  }

  @Test
  void rejectsNonzeroOrNonfiniteAppliedOutput() {
    assertStatus(Status.APPLIED_OUTPUT_NOT_ZERO, observation(
        8L, 8L, 8L,
        true, false, false, false,
        4.0, 4.1, 4.2,
        0.011, true, 0.0,
        false, true, false));
    assertStatus(Status.APPLIED_OUTPUT_NOT_ZERO, observation(
        8L, 8L, 8L,
        true, false, false, false,
        4.0, 4.1, 4.2,
        Double.NaN, true, 0.0,
        false, true, false));
  }

  @Test
  void requiresStoppedVelocityOnlyWhenVelocityTelemetryExists() {
    assertStatus(Status.VELOCITY_NOT_STOPPED, observation(
        8L, 8L, 8L,
        true, false, false, false,
        4.0, 4.1, 4.2,
        0.0, true, 2.01,
        false, true, false));
    Evaluation noVelocitySensor = SparkOutputStopEvaluator.evaluate(observation(
        8L, 8L, 8L,
        true, false, false, false,
        4.0, 4.1, 4.2,
        0.0, false, Double.NaN,
        false, true, false), LIMITS);

    assertTrue(noVelocitySensor.confirmed());
  }

  @Test
  void followerMustFinishItsTransitionAndReportFollowerMode() {
    assertStatus(Status.FOLLOWER_NOT_RESTORED, observation(
        8L, 8L, 8L,
        true, false, false, false,
        4.0, 4.1, 4.2,
        0.0, true, 0.0,
        true, false, false));
    assertStatus(Status.FOLLOWER_NOT_RESTORED, observation(
        8L, 8L, 8L,
        true, false, false, false,
        4.0, 4.1, 4.2,
        0.0, true, 0.0,
        true, true, false));

    Evaluation restored = SparkOutputStopEvaluator.evaluate(observation(
        8L, 8L, 8L,
        true, false, false, false,
        4.0, 4.1, 4.2,
        0.0, true, 0.0,
        true, true, true), LIMITS);
    assertTrue(restored.confirmed());
  }

  @Test
  void invalidLimitsAndFutureDatedSamplesFailClosed() {
    Evaluation invalidLimits = SparkOutputStopEvaluator.evaluate(
        healthyObservation(), new Limits(-0.1, 0.01, 2.0));
    assertFalse(invalidLimits.confirmed());
    assertEquals(Status.INVALID_OBSERVATION, invalidLimits.status());

    assertStatus(Status.SAMPLE_STALE, observation(
        8L, 8L, 8L,
        true, false, false, false,
        4.0, 4.3, 4.2,
        0.0, true, 0.0,
        false, true, false));
  }

  private static Observation healthyObservation() {
    return observation(
        8L, 8L, 8L,
        true, false, false, false,
        4.0, 4.1, 4.2,
        0.005, true, -1.5,
        false, false, false);
  }

  private static Observation observation(
      long requestedOutputEpoch,
      long outputEpoch,
      long lastZeroedOutputEpoch,
      boolean configurationReady,
      boolean zeroInFlight,
      boolean zeroRequired,
      boolean outputMayBeNonzero,
      double lastZeroConfirmedAtSeconds,
      double lastSampleAtSeconds,
      double nowSeconds,
      double appliedOutput,
      boolean velocityAvailable,
      double velocityRpm,
      boolean followerExpected,
      boolean followerTransitionComplete,
      boolean cachedFollower) {
    return new Observation(
        31,
        requestedOutputEpoch,
        outputEpoch,
        lastZeroedOutputEpoch,
        configurationReady,
        zeroInFlight,
        zeroRequired,
        outputMayBeNonzero,
        lastZeroConfirmedAtSeconds,
        lastSampleAtSeconds,
        nowSeconds,
        appliedOutput,
        velocityAvailable,
        velocityRpm,
        followerExpected,
        followerTransitionComplete,
        cachedFollower);
  }

  private static void assertStatus(Status expected, Observation observation) {
    Evaluation result = SparkOutputStopEvaluator.evaluate(observation, LIMITS);
    assertFalse(result.confirmed());
    assertEquals(expected, result.status());
  }
}
