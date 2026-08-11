package frc.robot.utils;

/**
 * Pure evaluator for evidence that a specific SPARK output epoch is safely stopped.
 *
 * <p>A configured follower's leader must be included in the same stop barrier (or independently
 * confirmed stopped); follower topology restoration alone cannot prove that leader output is zero.
 */
public final class SparkOutputStopEvaluator {
  private SparkOutputStopEvaluator() {}

  public enum Status {
    CONFIRMED,
    INVALID_OBSERVATION,
    REQUEST_SUPERSEDED,
    CONTROLLER_NOT_READY,
    ZERO_PENDING,
    ZERO_NOT_CONFIRMED_FOR_EPOCH,
    POST_ZERO_SAMPLE_PENDING,
    SAMPLE_STALE,
    APPLIED_OUTPUT_NOT_ZERO,
    VELOCITY_NOT_STOPPED,
    FOLLOWER_NOT_RESTORED
  }

  /** Thresholds supplied by the SPARK owner so this class remains hardware independent. */
  public record Limits(
      double maximumSampleAgeSeconds,
      double maximumAppliedOutput,
      double maximumStoppedVelocityRpm) {
    private boolean isValid() {
      return Double.isFinite(maximumSampleAgeSeconds)
          && maximumSampleAgeSeconds >= 0.0
          && Double.isFinite(maximumAppliedOutput)
          && maximumAppliedOutput >= 0.0
          && Double.isFinite(maximumStoppedVelocityRpm)
          && maximumStoppedVelocityRpm >= 0.0;
    }
  }

  /**
   * Immutable state captured under the controller's output/state locks.
   *
   * <p>{@code outputEpoch} must advance only after a nonzero setpoint succeeds. Safety retries may
   * advance their private worker generation, but must not advance this epoch. A successful zero
   * records the current output epoch in {@code lastZeroedOutputEpoch}.
   */
  public record Observation(
      int canId,
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
      boolean cachedFollower) {}

  public record Evaluation(Status status) {
    public boolean confirmed() {
      return status == Status.CONFIRMED;
    }
  }

  public static Evaluation evaluate(Observation observation, Limits limits) {
    if (observation == null
        || limits == null
        || !limits.isValid()
        || observation.canId() < 0
        || !Double.isFinite(observation.nowSeconds())) {
      return result(Status.INVALID_OBSERVATION);
    }
    if (observation.outputEpoch() != observation.requestedOutputEpoch()) {
      return result(Status.REQUEST_SUPERSEDED);
    }
    if (!observation.configurationReady()) {
      return result(Status.CONTROLLER_NOT_READY);
    }
    if (observation.zeroInFlight()
        || observation.zeroRequired()
        || observation.outputMayBeNonzero()) {
      return result(Status.ZERO_PENDING);
    }
    if (observation.lastZeroedOutputEpoch() != observation.requestedOutputEpoch()
        || !Double.isFinite(observation.lastZeroConfirmedAtSeconds())) {
      return result(Status.ZERO_NOT_CONFIRMED_FOR_EPOCH);
    }
    if (!Double.isFinite(observation.lastSampleAtSeconds())
        || observation.lastSampleAtSeconds() < observation.lastZeroConfirmedAtSeconds()) {
      return result(Status.POST_ZERO_SAMPLE_PENDING);
    }

    double sampleAgeSeconds = observation.nowSeconds() - observation.lastSampleAtSeconds();
    if (!Double.isFinite(sampleAgeSeconds)
        || sampleAgeSeconds < 0.0
        || sampleAgeSeconds > limits.maximumSampleAgeSeconds()) {
      return result(Status.SAMPLE_STALE);
    }
    if (!Double.isFinite(observation.appliedOutput())
        || Math.abs(observation.appliedOutput()) > limits.maximumAppliedOutput()) {
      return result(Status.APPLIED_OUTPUT_NOT_ZERO);
    }
    if (observation.velocityAvailable()
        && (!Double.isFinite(observation.velocityRpm())
            || Math.abs(observation.velocityRpm()) > limits.maximumStoppedVelocityRpm())) {
      return result(Status.VELOCITY_NOT_STOPPED);
    }
    if (observation.followerExpected()
        && (!observation.followerTransitionComplete() || !observation.cachedFollower())) {
      return result(Status.FOLLOWER_NOT_RESTORED);
    }
    return result(Status.CONFIRMED);
  }

  private static Evaluation result(Status status) {
    return new Evaluation(status);
  }
}
