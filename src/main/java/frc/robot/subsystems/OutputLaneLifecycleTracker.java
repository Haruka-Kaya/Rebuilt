package frc.robot.subsystems;

/** Lock-owned, I/O-free lifecycle latch for the CTRE output lane. */
final class OutputLaneLifecycleTracker {
  enum Phase {
    OPEN,
    CLOSING,
    CLOSED
  }

  private Phase phase = Phase.OPEN;
  private int nativeCloseStarts;

  boolean requestClose() {
    if (phase != Phase.OPEN) {
      return false;
    }
    phase = Phase.CLOSING;
    return true;
  }

  boolean tryBeginNativeClose(boolean laneDrained, boolean neutralCompleted) {
    if (phase != Phase.CLOSING
        || nativeCloseStarts != 0
        || !laneDrained
        || !neutralCompleted) {
      return false;
    }
    nativeCloseStarts++;
    phase = Phase.CLOSED;
    return true;
  }

  boolean isOpen() {
    return phase == Phase.OPEN;
  }

  boolean isClosing() {
    return phase == Phase.CLOSING;
  }

  boolean isClosed() {
    return phase == Phase.CLOSED;
  }

  Phase phase() {
    return phase;
  }

  int nativeCloseStarts() {
    return nativeCloseStarts;
  }
}
