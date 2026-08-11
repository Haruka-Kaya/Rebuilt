package frc.robot.utils;

/**
 * Tracks whether the CTRE swerve odometry acquisition loop is currently producing fresh data.
 *
 * <p>The Phoenix counters are cumulative, so historic successful acquisitions are not enough to
 * establish current health. This class starts fail-closed and requires current counter progress
 * before allowing recovery.
 */
public final class SwerveDaqFreshnessTracker {
  public static final double MAX_AGE_SECONDS = 0.100;
  public static final int HEALTHY_OBSERVATIONS_TO_RECOVER = 3;

  private boolean hasCounterBaseline;
  private int previousSuccessfulDaqs;
  private int previousFailedDaqs;
  private double lastSuccessfulDaqAtSeconds = Double.NaN;
  private int consecutiveHealthyObservations;
  private boolean fresh;

  /**
   * Observes one immutable snapshot of a {@code SwerveDriveState}.
   *
   * @param nowSeconds current time in the same timebase as {@code stateTimestampSeconds}
   * @param stateTimestampSeconds the state's capture timestamp
   * @param successfulDaqs cumulative successful acquisition count
   * @param failedDaqs cumulative failed acquisition count
   * @return whether the acquisition loop is currently considered fresh
   */
  public boolean observe(
      double nowSeconds, double stateTimestampSeconds, int successfulDaqs, int failedDaqs) {
    if (!hasCounterBaseline) {
      rebaseline(successfulDaqs, failedDaqs);
      invalidate();
      return false;
    }

    boolean countersRolledBack =
        successfulDaqs < previousSuccessfulDaqs || failedDaqs < previousFailedDaqs;
    boolean successfulDaqAdvanced = successfulDaqs > previousSuccessfulDaqs;
    boolean failedDaqAdvanced = failedDaqs > previousFailedDaqs;

    rebaseline(successfulDaqs, failedDaqs);

    boolean timestampValid =
        Double.isFinite(nowSeconds)
            && Double.isFinite(stateTimestampSeconds)
            && stateTimestampSeconds <= nowSeconds
            && nowSeconds - stateTimestampSeconds <= MAX_AGE_SECONDS;

    // A failure observed in the same sample as a success still wins.
    if (countersRolledBack || failedDaqAdvanced || !timestampValid) {
      invalidate();
      return false;
    }

    if (successfulDaqAdvanced) {
      lastSuccessfulDaqAtSeconds = nowSeconds;
    }

    boolean acquisitionRecentlyAdvanced =
        Double.isFinite(lastSuccessfulDaqAtSeconds)
            && nowSeconds >= lastSuccessfulDaqAtSeconds
            && nowSeconds - lastSuccessfulDaqAtSeconds <= MAX_AGE_SECONDS;
    if (!acquisitionRecentlyAdvanced) {
      invalidate();
      return false;
    }

    // Recovery evidence must consist of three distinct successful acquisitions. Re-reading the
    // same cached state three times must never turn a stopped DAQ loop healthy.
    if (successfulDaqAdvanced) {
      consecutiveHealthyObservations++;
      fresh = consecutiveHealthyObservations >= HEALTHY_OBSERVATIONS_TO_RECOVER;
    }
    return fresh;
  }

  public boolean isFresh() {
    return fresh;
  }

  public int getConsecutiveHealthyObservations() {
    return consecutiveHealthyObservations;
  }

  /** Clears all history and returns the tracker to its fail-closed startup state. */
  public void reset() {
    hasCounterBaseline = false;
    previousSuccessfulDaqs = 0;
    previousFailedDaqs = 0;
    invalidate();
  }

  private void rebaseline(int successfulDaqs, int failedDaqs) {
    hasCounterBaseline = true;
    previousSuccessfulDaqs = successfulDaqs;
    previousFailedDaqs = failedDaqs;
  }

  private void invalidate() {
    lastSuccessfulDaqAtSeconds = Double.NaN;
    consecutiveHealthyObservations = 0;
    fresh = false;
  }
}
