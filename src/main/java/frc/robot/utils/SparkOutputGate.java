package frc.robot.utils;

/** Pure output-state gate used to rate-limit fail-closed zero retries. */
final class SparkOutputGate {
  static final double INITIAL_ZERO_RETRY_SECONDS = 0.25;
  static final double MAX_ZERO_RETRY_SECONDS = 0.50;

  enum ZeroDecision {
    NOT_NEEDED,
    ATTEMPT,
    RETRY_LATER
  }

  // A roboRIO code restart does not prove that the controller's last setpoint was zero.
  private boolean outputMayBeNonzero = true;
  private boolean zeroRequired = true;
  private long generation;
  private int consecutiveZeroFailures;
  private double nextZeroAttemptAt = Double.NEGATIVE_INFINITY;

  ZeroDecision decideZero(double nowSeconds) {
    if (!zeroRequired) {
      return ZeroDecision.NOT_NEEDED;
    }
    if (nowSeconds < nextZeroAttemptAt) {
      return ZeroDecision.RETRY_LATER;
    }
    return ZeroDecision.ATTEMPT;
  }

  long requireZero(double nowSeconds, boolean retryImmediately) {
    if (!retryImmediately && !needsZeroCommand()) {
      return generation;
    }
    if (!zeroRequired || retryImmediately) {
      generation++;
    }
    zeroRequired = true;
    if (retryImmediately) {
      nextZeroAttemptAt = nowSeconds;
    }
    return generation;
  }

  void zeroSucceeded() {
    outputMayBeNonzero = false;
    zeroRequired = false;
    consecutiveZeroFailures = 0;
    nextZeroAttemptAt = Double.NEGATIVE_INFINITY;
  }

  void zeroFailed(double nowSeconds) {
    outputMayBeNonzero = true;
    zeroRequired = true;
    consecutiveZeroFailures++;
    nextZeroAttemptAt = nowSeconds + zeroRetryDelaySeconds(consecutiveZeroFailures);
  }

  void nonzeroSucceeded() {
    outputMayBeNonzero = true;
    zeroRequired = false;
    consecutiveZeroFailures = 0;
    nextZeroAttemptAt = Double.NEGATIVE_INFINITY;
    generation++;
  }

  void observeNonzero() {
    outputMayBeNonzero = true;
  }

  boolean outputMayBeNonzero() {
    return outputMayBeNonzero;
  }

  boolean isZeroRequired() {
    return zeroRequired;
  }

  boolean needsZeroCommand() {
    return zeroRequired || outputMayBeNonzero;
  }

  long generation() {
    return generation;
  }

  static double zeroRetryDelaySeconds(int failures) {
    double multiplier = Math.scalb(1.0, Math.max(0, Math.min(3, failures - 1)));
    return Math.min(MAX_ZERO_RETRY_SECONDS, INITIAL_ZERO_RETRY_SECONDS * multiplier);
  }
}
