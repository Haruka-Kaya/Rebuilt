package frc.robot.utils;

import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

/**
 * Process-wide, fail-closed authorization boundary for every nonzero motor request.
 *
 * <p>Only {@link RobotOutputSafetySupervisor} may grant authorization. Motor wrappers read the
 * immutable snapshot at their final ordered-output boundary immediately before calling a vendor
 * API. Zero-output requests never depend on this authorization.
 */
public final class ProcessOutputSafety {
  private static final AtomicReference<Snapshot> STATE = new AtomicReference<>(
      new Snapshot(0L, false, "STARTUP_NOT_AUTHORIZED"));

  private ProcessOutputSafety() {}

  /** Returns the atomic process-wide decision used by SPARK and CTRE output boundaries. */
  public static boolean isOutputAuthorized() {
    return STATE.get().outputAuthorized();
  }

  /**
   * Admits a vendor call from the current immutable process grant.
   *
   * <p>Revoke is intentionally nonblocking even if a vendor API hangs. Callers must already hold
   * their vendor-specific output-order lock. A call admitted just before revoke is therefore
   * ordered before that vendor's neutral request; any later admission is rejected. This callback
   * must never call back into the robot safety supervisor.
   */
  public static <T> AuthorizedCall<T> callIfAuthorized(Supplier<T> vendorCall) {
    Objects.requireNonNull(vendorCall, "vendorCall");
    Snapshot admitted = STATE.get();
    if (!admitted.outputAuthorized()) {
      return new AuthorizedCall<>(false, null);
    }
    return new AuthorizedCall<>(true, vendorCall.get());
  }

  /** Returns the current immutable authorization evidence. */
  public static Snapshot snapshot() {
    return STATE.get();
  }

  /** Revokes first and advances the generation so an older grant cannot be restored. */
  static long revoke(String requestedReason) {
    String normalized = normalize(requestedReason);
    while (true) {
      Snapshot current = STATE.get();
      Snapshot revoked = new Snapshot(current.generation() + 1L, false, normalized);
      if (STATE.compareAndSet(current, revoked)) {
        return revoked.generation();
      }
    }
  }

  /** Grants only if no newer revoke occurred after the supervisor captured the generation. */
  static boolean authorize(long expectedGeneration) {
    while (true) {
      Snapshot current = STATE.get();
      if (current.generation() != expectedGeneration) {
        return false;
      }
      Snapshot authorized = new Snapshot(
          current.generation(), true, "AUTHORIZED_FRESH_ROBOT_LOOP_HEARTBEAT");
      if (STATE.compareAndSet(current, authorized)) {
        return true;
      }
    }
  }

  static void resetForTesting() {
    STATE.set(new Snapshot(0L, false, "STARTUP_NOT_AUTHORIZED"));
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
