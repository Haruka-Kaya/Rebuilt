package frc.robot.utils;

/**
 * Pure per-controller ordering lane for blocking nonzero vendor calls and their zero barrier.
 *
 * <p>A nonzero call is reserved under the SPARK output/state locks, executed after those locks are
 * released, and completed under the same locks. A stop requested while the call is in flight is
 * therefore retained behind it and prevents another nonzero reservation until zero completes.
 */
final class SparkOutputLane {
  private boolean nonzeroInFlight;
  private boolean zeroPendingBehindNonzero;
  private long generation;
  private long stopSequence;

  long reserveNonzero() {
    if (nonzeroInFlight || zeroPendingBehindNonzero) {
      return -1L;
    }
    nonzeroInFlight = true;
    return ++generation;
  }

  void requestZero() {
    if (stopSequence != Long.MAX_VALUE) {
      stopSequence++;
    }
    if (nonzeroInFlight) {
      zeroPendingBehindNonzero = true;
    }
  }

  /** Changes for every stop request, including one that completes while authorization is evaluated. */
  long stopSequence() {
    return stopSequence;
  }

  /** Captures the latest nonzero reservation that this stop must be ordered behind. */
  long stopBarrierGeneration() {
    return generation;
  }

  boolean completeNonzero(long reservationGeneration) {
    if (!nonzeroInFlight || generation != reservationGeneration) {
      return true;
    }
    nonzeroInFlight = false;
    return zeroPendingBehindNonzero;
  }

  boolean canReserveZero() {
    return !nonzeroInFlight;
  }

  void zeroCompleted() {
    // A concurrent/stale zero completion can never certify a stop ahead of an admitted nonzero.
    if (!nonzeroInFlight) {
      zeroPendingBehindNonzero = false;
    }
  }

  boolean nonzeroInFlight() {
    return nonzeroInFlight;
  }

  boolean zeroPendingBehindNonzero() {
    return zeroPendingBehindNonzero;
  }

  /** Pure stop-barrier check used by unit tests and cached stop-evidence composition. */
  boolean zeroCompletedFor(long requestedGeneration, long lastZeroedGeneration) {
    return !nonzeroInFlight
        && !zeroPendingBehindNonzero
        && lastZeroedGeneration >= requestedGeneration;
  }
}
