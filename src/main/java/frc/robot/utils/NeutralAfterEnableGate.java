package frc.robot.utils;

/** Requires one fully neutral input sample after enable or input-source changes. */
public final class NeutralAfterEnableGate {
  private boolean previouslyEnabled;
  private boolean neutralObserved;
  private int sourceId = Integer.MIN_VALUE;

  public synchronized boolean allow(boolean enabled, int currentSourceId, boolean anyPressed) {
    if (!enabled) {
      previouslyEnabled = false;
      neutralObserved = false;
      sourceId = currentSourceId;
      return false;
    }

    if (!previouslyEnabled || sourceId != currentSourceId) {
      previouslyEnabled = true;
      neutralObserved = false;
      sourceId = currentSourceId;
    }

    if (!neutralObserved) {
      if (!anyPressed) {
        neutralObserved = true;
      }
      return false;
    }
    return true;
  }

  public boolean allow(boolean enabled, boolean anyPressed) {
    return allow(enabled, 0, anyPressed);
  }

  /** Forces a complete release after an invalid multi-button gesture. */
  public synchronized void blockUntilNeutral() {
    neutralObserved = false;
  }
}
