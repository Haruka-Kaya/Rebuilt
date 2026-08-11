package frc.robot.containers;

/** Pure lifecycle state for one managed autonomous command. */
final class AutonomousRunState {
  enum Phase {
    IDLE,
    RUNNING,
    COMPLETED,
    INTERRUPTED,
    ABORTED
  }

  private Phase phase = Phase.IDLE;

  void started() {
    phase = Phase.RUNNING;
  }

  boolean abort() {
    if (phase != Phase.RUNNING) {
      return false;
    }
    phase = Phase.ABORTED;
    return true;
  }

  Phase finished(boolean interrupted) {
    if (phase != Phase.ABORTED) {
      phase = interrupted ? Phase.INTERRUPTED : Phase.COMPLETED;
    }
    return phase;
  }

  boolean running() {
    return phase == Phase.RUNNING;
  }

  Phase phase() {
    return phase;
  }
}
