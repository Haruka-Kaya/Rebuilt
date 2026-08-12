package frc.robot.utils;

import java.util.Objects;
import java.util.function.Supplier;

/**
 * Process-wide, fail-closed authorization boundary for every nonzero motor request.
 *
 * <p>Only {@link RobotOutputSafetySupervisor} may grant authorization. Motor wrappers read the
 * immutable snapshot at their final ordered-output boundary immediately before calling a vendor
 * API. Zero-output requests never depend on this authorization.
 */
public final class ProcessOutputSafety {
  private static final Object LOCK = new Object();

  private static long generation;
  private static boolean outputAuthorized;
  private static String reason = "STARTUP_NOT_AUTHORIZED";

  private ProcessOutputSafety() {}

  /** Returns the atomic process-wide decision used by SPARK and CTRE output boundaries. */
  public static boolean isOutputAuthorized() {
    synchronized (LOCK) {
      return outputAuthorized;
    }
  }

  /**
   * Runs a vendor call while holding the same lock used by revoke/authorize.
   *
   * <p>This closes the check-then-call window: once revoke returns, no older nonzero call can
   * start later. Callers must already hold their vendor-specific output-order lock, and this
   * callback must never call back into the robot safety supervisor.
   */
  public static <T> AuthorizedCall<T> callIfAuthorized(Supplier<T> vendorCall) {
    Objects.requireNonNull(vendorCall, "vendorCall");
    synchronized (LOCK) {
      if (!outputAuthorized) {
        return new AuthorizedCall<>(false, null);
      }
      return new AuthorizedCall<>(true, vendorCall.get());
    }
  }

  /** Returns the current immutable authorization evidence. */
  public static Snapshot snapshot() {
    synchronized (LOCK) {
      return new Snapshot(generation, outputAuthorized, reason);
    }
  }

  /** Revokes first and advances the generation so an older grant cannot be restored. */
  static long revoke(String requestedReason) {
    synchronized (LOCK) {
      generation++;
      outputAuthorized = false;
      reason = normalize(requestedReason);
      return generation;
    }
  }

  /** Grants only if no newer revoke occurred after the supervisor captured the generation. */
  static boolean authorize(long expectedGeneration) {
    synchronized (LOCK) {
      if (generation != expectedGeneration) {
        return false;
      }
      outputAuthorized = true;
      reason = "AUTHORIZED_FRESH_ROBOT_LOOP_HEARTBEAT";
      return true;
    }
  }

  static void resetForTesting() {
    synchronized (LOCK) {
      generation = 0;
      outputAuthorized = false;
      reason = "STARTUP_NOT_AUTHORIZED";
    }
  }

  private static String normalize(String value) {
    if (value == null || value.isBlank()) {
      return "OUTPUT_AUTHORIZATION_REVOKED";
    }
    return value.trim().replaceAll("\\s+", " ");
  }

  /** Atomic evidence exposed for diagnostics and deterministic tests. */
  public record Snapshot(long generation, boolean outputAuthorized, String reason) {
    public Snapshot {
      Objects.requireNonNull(reason, "reason");
    }
  }

  /** Result of an atomically authorized vendor call. Value is null when authorization was absent. */
  public record AuthorizedCall<T>(boolean authorized, T value) {}
}
