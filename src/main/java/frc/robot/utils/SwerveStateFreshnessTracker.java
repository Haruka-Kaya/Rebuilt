package frc.robot.utils;

/** Tracks current progress of the Phoenix swerve state-publication loop. */
public final class SwerveStateFreshnessTracker {
  public static final double MAX_AGE_SECONDS = 0.100;
  public static final int HEALTHY_ADVANCES_TO_RECOVER = 3;

  private boolean hasTimestampBaseline;
  private double previousStateTimestampSeconds;
  private double lastAdvanceObservedAtSeconds = Double.NaN;
  private int consecutiveHealthyAdvances;
  private boolean fresh;

  /**
   * Observes one copied swerve state using a clock in the same epoch as its timestamp.
   * Re-reading one cached state never counts as recovery evidence.
   */
  public boolean observe(double nowSeconds, double stateTimestampSeconds) {
    boolean timestampValid = Double.isFinite(nowSeconds)
        && Double.isFinite(stateTimestampSeconds)
        && stateTimestampSeconds <= nowSeconds
        && nowSeconds - stateTimestampSeconds <= MAX_AGE_SECONDS;
    if (!timestampValid) {
      reset();
      return false;
    }

    if (!hasTimestampBaseline) {
      hasTimestampBaseline = true;
      previousStateTimestampSeconds = stateTimestampSeconds;
      invalidateProgress();
      return false;
    }

    if (stateTimestampSeconds < previousStateTimestampSeconds) {
      previousStateTimestampSeconds = stateTimestampSeconds;
      invalidateProgress();
      return false;
    }

    boolean advanced = stateTimestampSeconds > previousStateTimestampSeconds;
    previousStateTimestampSeconds = stateTimestampSeconds;
    if (advanced) {
      lastAdvanceObservedAtSeconds = nowSeconds;
      consecutiveHealthyAdvances++;
      fresh = consecutiveHealthyAdvances >= HEALTHY_ADVANCES_TO_RECOVER;
    }

    boolean advancedRecently = Double.isFinite(lastAdvanceObservedAtSeconds)
        && nowSeconds >= lastAdvanceObservedAtSeconds
        && nowSeconds - lastAdvanceObservedAtSeconds <= MAX_AGE_SECONDS;
    if (!advancedRecently) {
      invalidateProgress();
      return false;
    }
    return fresh;
  }

  public boolean isFresh() {
    return fresh;
  }

  public int getConsecutiveHealthyAdvances() {
    return consecutiveHealthyAdvances;
  }

  public void reset() {
    hasTimestampBaseline = false;
    previousStateTimestampSeconds = 0.0;
    invalidateProgress();
  }

  private void invalidateProgress() {
    lastAdvanceObservedAtSeconds = Double.NaN;
    consecutiveHealthyAdvances = 0;
    fresh = false;
  }
}
