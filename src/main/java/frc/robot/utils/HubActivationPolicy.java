package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Pure implementation of the 2026 HUB shift timing published by WPILib. */
public final class HubActivationPolicy {
  private HubActivationPolicy() {}

  public static boolean isActive(
      Alliance alliance,
      boolean autonomousEnabled,
      boolean teleopEnabled,
      double matchTimeSeconds,
      String gameData) {
    if (alliance == null) {
      return false;
    }
    if (autonomousEnabled) {
      return true;
    }
    if (!teleopEnabled) {
      return false;
    }

    String data = gameData == null ? "" : gameData;
    if (data.isEmpty()) {
      return true;
    }

    boolean redInactiveFirst;
    switch (data.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        return true;
      }
    }

    boolean shift1Active = alliance == Alliance.Red
        ? !redInactiveFirst
        : redInactiveFirst;
    if (matchTimeSeconds > 130.0) {
      return true;
    } else if (matchTimeSeconds > 105.0) {
      return shift1Active;
    } else if (matchTimeSeconds > 80.0) {
      return !shift1Active;
    } else if (matchTimeSeconds > 55.0) {
      return shift1Active;
    } else if (matchTimeSeconds > 30.0) {
      return !shift1Active;
    }
    return true;
  }
}
