package frc.robot.utils;

import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

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
   * Captures the current process grant for a prospective nonzero output transaction.
   *
   * <p>The returned permit has no authority until {@link #claimIfCurrent(NonzeroPermit)} succeeds
   * inside the vendor-specific ordered-output lane. No vendor callback is accepted here, so a
   * process-wide revoke never waits for a vendor API or a device lock.
   */
  public static Optional<NonzeroPermit> acquireNonzeroPermit() {
    Snapshot admitted = STATE.get();
    return admitted.outputAuthorized()
        ? Optional.of(new NonzeroPermit(admitted))
        : Optional.empty();
  }

  /**
   * Atomically consumes a permit if and only if its exact immutable grant remains current.
   *
   * <p>Callers must claim under their own ordered-output lane immediately before reserving the
   * nonzero transaction. After a successful claim, a concurrent revoke may return before the
   * already-reserved vendor call completes; that lane must retain a pending zero barrier behind
   * the admitted transaction. A permit is one-shot even when the claim fails.
   */
  public static boolean claimIfCurrent(NonzeroPermit permit) {
    if (permit == null || !permit.claimed.compareAndSet(false, true)) {
      return false;
    }
    Snapshot current = STATE.get();
    return current == permit.admitted && current.outputAuthorized();
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

  /** Opaque, one-shot capability tied by identity to one immutable process authorization grant. */
  public static final class NonzeroPermit {
    private final Snapshot admitted;
    private final AtomicBoolean claimed = new AtomicBoolean();

    private NonzeroPermit(Snapshot admitted) {
      this.admitted = Objects.requireNonNull(admitted, "admitted");
    }
  }
}
