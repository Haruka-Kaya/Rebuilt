package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.utils.SparkResetGuard.Observation;

class SparkResetGuardTest {
  @Test
  void requiresACleanFrameAfterFullConfiguration() {
    SparkResetGuard guard = new SparkResetGuard();
    guard.fullConfigurationCompleted(10.0);

    assertFalse(guard.isArmed());
    assertEquals(Observation.WAITING_FOR_BASELINE, guard.observe(10.5, true, true));
    assertEquals(Observation.CLEAN, guard.observe(10.6, false, false));
    assertTrue(guard.isArmed());
  }

  @Test
  void stickyResetAfterBaselineIsImmediatelyDetected() {
    SparkResetGuard guard = new SparkResetGuard();
    guard.fullConfigurationCompleted(0.0);
    assertEquals(Observation.CLEAN, guard.observe(0.1, false, false));

    assertEquals(Observation.RESET_DETECTED, guard.observe(0.2, false, true));
  }

  @Test
  void unclearedBaselineTimesOutFailClosed() {
    SparkResetGuard guard = new SparkResetGuard();
    guard.fullConfigurationCompleted(5.0);

    assertEquals(
        Observation.BASELINE_TIMEOUT,
        guard.observe(5.0 + SparkResetGuard.BASELINE_TIMEOUT_SECONDS + 0.01, true, true));
  }
}
