package frc.robot.utils;

import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;

/** One-way, process-lifetime latch for an unrecoverable robot runtime fault. */
public final class RuntimeSafetyLatch {
  private static final Snapshot HEALTHY = new Snapshot(true, "HEALTHY");

  private final AtomicReference<Snapshot> state = new AtomicReference<>(HEALTHY);

  /**
   * Records the first runtime exception. Later faults cannot replace the original diagnostic.
   *
   * @return the latched state after this call
   */
  public Snapshot latch(RuntimeException exception) {
    Snapshot current = state.get();
    if (!current.healthy()) {
      return current;
    }

    Snapshot faulted = new Snapshot(false, normalizeReason(exception));
    state.compareAndSet(current, faulted);
    return state.get();
  }

  /** Returns an atomic snapshot of the current process-lifetime safety state. */
  public Snapshot snapshot() {
    return state.get();
  }

  public boolean healthy() {
    return state.get().healthy();
  }

  public String reason() {
    return state.get().reason();
  }

  private static String normalizeReason(RuntimeException exception) {
    if (exception == null) {
      return "RuntimeException";
    }

    String type = exception.getClass().getSimpleName();
    if (type == null || type.isBlank()) {
      type = "RuntimeException";
    }
    String message = exception.getMessage();
    if (message == null || message.isBlank()) {
      return type;
    }
    return type + ": " + message.trim().replaceAll("\\s+", " ");
  }

  /** Immutable state returned to the robot loop and diagnostics. */
  public record Snapshot(boolean healthy, String reason) {
    public Snapshot {
      Objects.requireNonNull(reason, "reason");
      if (reason.isBlank()) {
        throw new IllegalArgumentException("reason must not be blank");
      }
    }
  }
}
