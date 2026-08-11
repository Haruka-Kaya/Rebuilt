package frc.robot.commands;

/** Pure release gate shared by teleoperated and autonomous shooting commands. */
final class ShotReleaseInterlock {
  private ShotReleaseInterlock() {}

  static boolean mayRelease(
      boolean shooterReady,
      boolean conveyorReady,
      boolean feederReady,
      boolean hubActive,
      boolean aimReady) {
    return shooterReady
        && conveyorReady
        && feederReady
        && hubActive
        && aimReady;
  }
}
