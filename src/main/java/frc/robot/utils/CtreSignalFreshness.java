package frc.robot.utils;

/** Pure fail-closed predicate for one cached CTRE status signal observation. */
public final class CtreSignalFreshness {
  private CtreSignalFreshness() {}

  public static boolean isFresh(
      boolean statusOk,
      boolean timestampValid,
      double latencySeconds,
      double value,
      double maximumAgeSeconds) {
    return statusOk
        && timestampValid
        && Double.isFinite(latencySeconds)
        && latencySeconds >= 0.0
        && Double.isFinite(maximumAgeSeconds)
        && maximumAgeSeconds >= 0.0
        && latencySeconds <= maximumAgeSeconds
        && Double.isFinite(value);
  }
}
