package frc.robot.subsystems;

import java.util.HashSet;
import java.util.Set;

/**
 * Lock-owned ticket/barrier state for ordered nonzero and certifying-neutral CTRE work.
 *
 * <p>This class performs no I/O and owns no locks. Its caller serializes every method. Keeping the
 * ordering policy separate makes late-completion and close-drain behavior deterministic to test
 * without loading Phoenix JNI.
 */
final class OutputLaneBarrierTracker {
  private final Set<Long> inFlightNonzeroTickets = new HashSet<>();
  private long nextTicket;
  private long barrierGeneration;
  private long barrierOutputEpoch = -1;
  private long barrierCutoffTicket = -1;
  private long completedBarrierGeneration = -1;

  long beginNonzero() {
    if (nextTicket == Long.MAX_VALUE) {
      throw new IllegalStateException("CTRE output lane ticket exhausted");
    }
    long ticket = ++nextTicket;
    inFlightNonzeroTickets.add(ticket);
    return ticket;
  }

  /**
   * Completes one admitted transaction. Returns true when its barrier can now run strictly after
   * every transaction at or before the barrier cutoff.
   */
  boolean completeNonzero(long ticket) {
    boolean removed = inFlightNonzeroTickets.remove(ticket);
    if (!removed) {
      return false;
    }
    if (hasBarrier() && ticket <= barrierCutoffTicket) {
      // Even if an early best-effort neutral completed, a late pre-barrier output invalidates it.
      completedBarrierGeneration = -1;
      return !hasPreBarrierInFlight();
    }
    return false;
  }

  Barrier reserveBarrier(long outputEpoch) {
    if (barrierOutputEpoch != outputEpoch) {
      // Saturation is fail-closed: outputEpoch still distinguishes each later barrier, while
      // throwing here could prevent a stop caller from publishing its neutral requirement.
      if (barrierGeneration != Long.MAX_VALUE) {
        barrierGeneration++;
      }
      barrierOutputEpoch = outputEpoch;
      barrierCutoffTicket = nextTicket;
      completedBarrierGeneration = -1;
    }
    return new Barrier(barrierGeneration, barrierOutputEpoch, barrierCutoffTicket);
  }

  void clearBarrierForNewOutputEpoch() {
    barrierOutputEpoch = -1;
    barrierCutoffTicket = -1;
    completedBarrierGeneration = -1;
  }

  boolean markNeutralCompleted(long generation, long outputEpoch) {
    if (!matches(generation, outputEpoch) || hasPreBarrierInFlight()) {
      return false;
    }
    completedBarrierGeneration = generation;
    return true;
  }

  boolean requireNeutralRetry(long generation, long outputEpoch) {
    if (!matches(generation, outputEpoch)) {
      return false;
    }
    completedBarrierGeneration = -1;
    return true;
  }

  boolean matches(long generation, long outputEpoch) {
    return hasBarrier()
        && generation == barrierGeneration
        && outputEpoch == barrierOutputEpoch;
  }

  boolean hasBarrier() {
    return barrierOutputEpoch >= 0;
  }

  boolean hasInFlightNonzero() {
    return !inFlightNonzeroTickets.isEmpty();
  }

  boolean hasPreBarrierInFlight() {
    if (!hasBarrier()) {
      return false;
    }
    return inFlightNonzeroTickets.stream().anyMatch(ticket -> ticket <= barrierCutoffTicket);
  }

  boolean neutralRequired() {
    return hasBarrier() && completedBarrierGeneration != barrierGeneration;
  }

  /** A new output epoch may supersede only an already-completed barrier. */
  boolean canStartNewOutput() {
    return !neutralRequired() && !hasPreBarrierInFlight();
  }

  long barrierGeneration() {
    return barrierGeneration;
  }

  long barrierOutputEpoch() {
    return barrierOutputEpoch;
  }

  long barrierCutoffTicket() {
    return barrierCutoffTicket;
  }

  record Barrier(long generation, long outputEpoch, long cutoffTicket) {}
}
