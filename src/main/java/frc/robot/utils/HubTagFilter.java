package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.Constants.AprilTagConstants;

/** Alliance-aware filter for the HUB AprilTags listed in the 2026 game manual. */
public final class HubTagFilter {
  private HubTagFilter() {}

  public static boolean isHubTagForAlliance(Alliance alliance, int tagId) {
    if (alliance == null || tagId < 0) {
      return false;
    }
    int[] validIds = alliance == Alliance.Red
        ? AprilTagConstants.VALID_RED_HUB_TAG_IDS
        : AprilTagConstants.VALID_BLUE_HUB_TAG_IDS;
    for (int validId : validIds) {
      if (tagId == validId) {
        return true;
      }
    }
    return false;
  }
}
