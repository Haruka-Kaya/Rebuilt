package frc.robot.diagnostics;

/** Pure, fail-closed evaluation of a low-duty open-loop motor diagnostic sample. */
public final class HardwareDiagnosticEvaluator {
  static final double MIN_VALID_BUS_VOLTAGE = 6.0;
  static final double MIN_OBSERVED_VELOCITY_RPM = 5.0;
  static final double MIN_APPLIED_OUTPUT_FRACTION = 0.5;
  static final double MAX_APPLIED_OUTPUT_ERROR = 0.02;
  static final double STALL_CURRENT_FRACTION = 0.8;

  private static final double ZERO_EPSILON = 1e-9;

  private HardwareDiagnosticEvaluator() {}

  /** Immutable telemetry captured while a diagnostic command is active. */
  public record Snapshot(
      int id,
      boolean ready,
      double appliedOutput,
      double currentAmps,
      double velocityRpm,
      double busVoltage,
      boolean commandAccepted) {}

  /** Motion-specific result; communication health should be reported separately per CAN ID. */
  public enum MotionResult {
    PASS_OBSERVED,
    FAIL_NOT_READY,
    FAIL_COMMAND_REJECTED,
    FAIL_DIRECTION_MISMATCH,
    STALL_SUSPECTED,
    INCONCLUSIVE_NO_MOTION,
    SKIPPED,
    BLOCKED_KNOWN_FAULT,
    NOT_RUN
  }

  /**
   * Evaluates one open-loop sample without treating a low-duty static-friction result as success.
   *
   * <p>A zero request is an intentional motion skip. Invalid configuration or command evidence is
   * rejected before velocity/current evidence is considered. A motor only passes when encoder
   * motion is observed; high current without motion is reported as a suspected stall.
   */
  public static MotionResult evaluateOpenLoop(
      Snapshot snapshot, double requestedDuty, double maxCurrentAmps) {
    boolean validCurrentThreshold = Double.isFinite(maxCurrentAmps) && maxCurrentAmps > 0.0;
    if (snapshot != null
        && validCurrentThreshold
        && Double.isFinite(snapshot.currentAmps())
        && Double.isFinite(snapshot.velocityRpm())
        && Math.abs(snapshot.velocityRpm()) < MIN_OBSERVED_VELOCITY_RPM
        && Math.abs(snapshot.currentAmps()) >= maxCurrentAmps * STALL_CURRENT_FRACTION) {
      // Prioritize stopping a suspected stall even when voltage/current limiting makes the
      // applied-output or bus-voltage fields fail their normal command validation.
      return MotionResult.STALL_SUSPECTED;
    }

    if (Double.isFinite(requestedDuty) && Math.abs(requestedDuty) <= ZERO_EPSILON) {
      return MotionResult.SKIPPED;
    }

    if (snapshot == null
        || !snapshot.ready()
        || !allFinite(
            snapshot.appliedOutput(),
            snapshot.currentAmps(),
            snapshot.velocityRpm(),
            snapshot.busVoltage())
        || snapshot.busVoltage() < MIN_VALID_BUS_VOLTAGE) {
      return MotionResult.FAIL_NOT_READY;
    }

    if (!Double.isFinite(requestedDuty)
        || Math.abs(requestedDuty) > 1.0
        || !validCurrentThreshold
        || !snapshot.commandAccepted()) {
      return MotionResult.FAIL_COMMAND_REJECTED;
    }

    double requestedMagnitude = Math.abs(requestedDuty);
    double appliedMagnitude = Math.abs(snapshot.appliedOutput());
    boolean appliedSignMatches = Math.signum(snapshot.appliedOutput()) == Math.signum(requestedDuty);
    double minimumAppliedMagnitude = requestedMagnitude * MIN_APPLIED_OUTPUT_FRACTION;
    double maximumAppliedMagnitude = Math.min(1.0, requestedMagnitude + MAX_APPLIED_OUTPUT_ERROR);
    if (!appliedSignMatches
        || appliedMagnitude < minimumAppliedMagnitude
        || appliedMagnitude > maximumAppliedMagnitude) {
      return MotionResult.FAIL_COMMAND_REJECTED;
    }

    if (Math.abs(snapshot.velocityRpm()) >= MIN_OBSERVED_VELOCITY_RPM) {
      return Math.signum(snapshot.velocityRpm()) == Math.signum(requestedDuty)
          ? MotionResult.PASS_OBSERVED
          : MotionResult.FAIL_DIRECTION_MISMATCH;
    }
    return MotionResult.INCONCLUSIVE_NO_MOTION;
  }

  private static boolean allFinite(double... values) {
    for (double value : values) {
      if (!Double.isFinite(value)) {
        return false;
      }
    }
    return true;
  }
}
