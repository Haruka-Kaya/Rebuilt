package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ShotReleaseInterlockTest {
  @Test
  void releasesOnlyWhenEveryRequiredConditionIsReady() {
    assertTrue(ShotReleaseInterlock.mayRelease(true, true, true, true, true));

    assertFalse(ShotReleaseInterlock.mayRelease(false, true, true, true, true));
    assertFalse(ShotReleaseInterlock.mayRelease(true, false, true, true, true));
    assertFalse(ShotReleaseInterlock.mayRelease(true, true, false, true, true));
    assertFalse(ShotReleaseInterlock.mayRelease(true, true, true, false, true));
    assertFalse(ShotReleaseInterlock.mayRelease(true, true, true, true, false));
  }

  @Test
  void teleopCanExplicitlyTreatManualAimAsSatisfiedButStillRequiresActiveHub() {
    boolean manualAimSatisfied = true;

    assertTrue(ShotReleaseInterlock.mayRelease(
        true, true, true, true, manualAimSatisfied));
    assertFalse(ShotReleaseInterlock.mayRelease(
        true, true, true, false, manualAimSatisfied));
  }
}
