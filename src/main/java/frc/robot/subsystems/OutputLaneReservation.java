package frc.robot.subsystems;

/** I/O-free lock-owned reservation helper shared by registration and odometry apply paths. */
final class OutputLaneReservation {
  private OutputLaneReservation() {}

  static long reserveNonzero(
      OutputLaneBarrierTracker lane,
      boolean laneOpenSnapshot,
      boolean externalAuthorizationSnapshot,
      boolean processAuthorizationClaimed,
      long stopSequenceSnapshot,
      long currentStopSequence) {
    if (lane == null
        || !laneOpenSnapshot
        || !externalAuthorizationSnapshot
        || !processAuthorizationClaimed
        || stopSequenceSnapshot != currentStopSequence
        || !lane.canStartNewOutput()) {
      return -1L;
    }
    return lane.beginNonzero();
  }
}
