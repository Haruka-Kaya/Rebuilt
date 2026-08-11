package frc.robot.utils;

/** Establishes a cleared sticky-reset baseline before a SPARK may accept nonzero output. */
final class SparkResetGuard {
  static final double BASELINE_TIMEOUT_SECONDS = 0.75;

  enum Observation {
    WAITING_FOR_BASELINE,
    CLEAN,
    RESET_DETECTED,
    BASELINE_TIMEOUT
  }

  private boolean armed;
  private double baselineDeadline = Double.NEGATIVE_INFINITY;

  void fullConfigurationCompleted(double nowSeconds) {
    armed = false;
    baselineDeadline = nowSeconds + BASELINE_TIMEOUT_SECONDS;
  }

  Observation observe(double nowSeconds, boolean activeReset, boolean stickyReset) {
    boolean resetPresent = activeReset || stickyReset;
    if (!armed) {
      if (resetPresent) {
        return nowSeconds > baselineDeadline
            ? Observation.BASELINE_TIMEOUT
            : Observation.WAITING_FOR_BASELINE;
      }
      armed = true;
      return Observation.CLEAN;
    }
    return resetPresent ? Observation.RESET_DETECTED : Observation.CLEAN;
  }

  boolean isArmed() {
    return armed;
  }
}
