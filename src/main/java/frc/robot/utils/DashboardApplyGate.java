package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Requires an explicit, released-between-uses Dashboard apply request while disabled and off FMS.
 */
public final class DashboardApplyGate {
  public enum Decision {
    NONE,
    APPLY,
    REJECT_ENABLED,
    REJECT_FMS,
    RELEASE_REQUIRED
  }

  private boolean neutralObserved;
  private boolean releaseWarningReported;

  public Decision poll(String applyKey) {
    boolean requested = SmartDashboard.getBoolean(applyKey, false);
    return evaluate(requested, DriverStation.isDisabled(), DriverStation.isFMSAttached());
  }

  Decision evaluate(boolean requested, boolean disabled, boolean fmsAttached) {
    if (!requested) {
      neutralObserved = true;
      releaseWarningReported = false;
      return Decision.NONE;
    }
    if (!neutralObserved) {
      if (releaseWarningReported) {
        return Decision.NONE;
      }
      releaseWarningReported = true;
      return Decision.RELEASE_REQUIRED;
    }

    neutralObserved = false;
    releaseWarningReported = true;
    if (fmsAttached) {
      return Decision.REJECT_FMS;
    }
    if (!disabled) {
      return Decision.REJECT_ENABLED;
    }
    return Decision.APPLY;
  }

  public static boolean allFiniteInRange(
      double[] values, double[] minimums, double[] maximums) {
    if (values == null
        || minimums == null
        || maximums == null
        || values.length != minimums.length
        || values.length != maximums.length) {
      return false;
    }
    for (int index = 0; index < values.length; index++) {
      if (!Double.isFinite(values[index])
          || !Double.isFinite(minimums[index])
          || !Double.isFinite(maximums[index])
          || minimums[index] > maximums[index]
          || values[index] < minimums[index]
          || values[index] > maximums[index]) {
        return false;
      }
    }
    return true;
  }
}
