package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;

/** Pure signed-speed contract for the inverted shooter follower pair. */
final class FlywheelPairReadiness {
  private FlywheelPairReadiness() {}

  static boolean atRequestedSpeed(
      double requestedLeaderRpm,
      double measuredLeaderRpm,
      double measuredFollowerRpm,
      double toleranceRpm) {
    return Double.isFinite(requestedLeaderRpm)
        && Double.isFinite(measuredLeaderRpm)
        && Double.isFinite(measuredFollowerRpm)
        && Double.isFinite(toleranceRpm)
        && requestedLeaderRpm > toleranceRpm
        && toleranceRpm >= 0.0
        && MathUtil.isNear(requestedLeaderRpm, measuredLeaderRpm, toleranceRpm)
        && MathUtil.isNear(-requestedLeaderRpm, measuredFollowerRpm, toleranceRpm);
  }
}
