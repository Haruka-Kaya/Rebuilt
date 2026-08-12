package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
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

  @Test
  void returnsEveryBlockerInStableSafetyPriorityOrder() {
    ShotReleaseInterlock.Decision decision = ShotReleaseInterlock.evaluate(
        false, false, false, false, false);

    assertFalse(decision.allowed());
    assertEquals(List.of(
        ShotReleaseInterlock.Reason.SHOOTER_NOT_READY,
        ShotReleaseInterlock.Reason.CONVEYOR_NOT_READY,
        ShotReleaseInterlock.Reason.FEEDER_NOT_READY,
        ShotReleaseInterlock.Reason.HUB_INACTIVE,
        ShotReleaseInterlock.Reason.AIM_NOT_READY), decision.blockers());
    assertEquals(ShotReleaseInterlock.Reason.SHOOTER_NOT_READY, decision.reason());
    assertEquals(
        "SHOOTER_NOT_READY+CONVEYOR_NOT_READY+FEEDER_NOT_READY+HUB_INACTIVE+AIM_NOT_READY",
        decision.summary());
    assertThrows(
        UnsupportedOperationException.class,
        () -> decision.blockers().add(ShotReleaseInterlock.Reason.READY));
  }

  @Test
  void readyDecisionHasNoBlockersButRetainsReadyCompatibilityReason() {
    ShotReleaseInterlock.Decision decision = ShotReleaseInterlock.evaluate(
        true, true, true, true, true);

    assertTrue(decision.allowed());
    assertTrue(decision.blockers().isEmpty());
    assertEquals(ShotReleaseInterlock.Reason.READY, decision.reason());
    assertEquals("READY", decision.summary());
  }

  @Test
  void fireEvidenceNamesKnownFeederStallWithoutHidingOtherBlockers() {
    ShotReleaseInterlock.Decision decision = ShotReleaseInterlock.evaluate(
        false, true, false, false, true);

    assertEquals(
        "SHOOTER_NOT_READY+FEEDER_KNOWN_STALL+HUB_INACTIVE",
        FireCommand.releaseBlockReason(decision, true));
    assertEquals(
        "SHOOTER_NOT_READY+FEEDER_NOT_READY+HUB_INACTIVE",
        FireCommand.releaseBlockReason(decision, false));
  }
}
