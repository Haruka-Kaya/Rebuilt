package frc.robot.subsystems;

import java.util.OptionalDouble;

/** Pure guard that prevents motor or direction changes during one enabled Test session. */
public final class ClimberDiagnosticLatch {
  public enum MotorSide {
    LEFT,
    RIGHT
  }

  private MotorSide latchedSide;
  private int latchedSign;

  /**
   * Returns the safely clamped diagnostic duty, or empty when any interlock is not satisfied.
   */
  public OptionalDouble accept(
      MotorSide side,
      double requestedDuty,
      double maximumDuty,
      boolean testEnabled,
      boolean fmsAttached,
      boolean armed,
      boolean bothControllersReady) {
    if (!testEnabled) {
      reset();
      return OptionalDouble.empty();
    }
    if (side == null
        || !Double.isFinite(requestedDuty)
        || !Double.isFinite(maximumDuty)
        || maximumDuty <= 0.0
        || Math.abs(requestedDuty) <= 1e-9
        || fmsAttached
        || !armed
        || !bothControllersReady) {
      return OptionalDouble.empty();
    }

    int requestedSign = requestedDuty > 0.0 ? 1 : -1;
    if (latchedSide == null) {
      latchedSide = side;
      latchedSign = requestedSign;
    } else if (latchedSide != side || latchedSign != requestedSign) {
      return OptionalDouble.empty();
    }

    return OptionalDouble.of(
        Math.copySign(Math.min(Math.abs(requestedDuty), Math.abs(maximumDuty)), requestedDuty));
  }

  public void reset() {
    latchedSide = null;
    latchedSign = 0;
  }

  public String getSelection() {
    return latchedSide == null
        ? "NONE"
        : latchedSide + (latchedSign > 0 ? "+" : "-");
  }
}
