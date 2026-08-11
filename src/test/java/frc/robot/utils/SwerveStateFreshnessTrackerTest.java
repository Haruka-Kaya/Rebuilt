package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class SwerveStateFreshnessTrackerTest {
  @Test
  void requiresThreeDistinctFreshTimestampAdvances() {
    SwerveStateFreshnessTracker tracker = new SwerveStateFreshnessTracker();

    assertFalse(tracker.observe(1.00, 1.00));
    assertFalse(tracker.observe(1.02, 1.01));
    assertFalse(tracker.observe(1.04, 1.03));
    assertTrue(tracker.observe(1.06, 1.05));
    assertEquals(3, tracker.getConsecutiveHealthyAdvances());
  }

  @Test
  void repeatedCachedTimestampCannotRecover() {
    SwerveStateFreshnessTracker tracker = new SwerveStateFreshnessTracker();

    assertFalse(tracker.observe(2.00, 1.99));
    assertFalse(tracker.observe(2.02, 2.01));
    assertFalse(tracker.observe(2.04, 2.01));
    assertFalse(tracker.observe(2.06, 2.01));
    assertEquals(1, tracker.getConsecutiveHealthyAdvances());
  }

  @Test
  void stoppedStateLoopExpiresAfterOneHundredMilliseconds() {
    SwerveStateFreshnessTracker tracker = healthyTracker();

    assertTrue(tracker.observe(3.10, 3.06));
    assertFalse(tracker.observe(3.161, 3.06));
  }

  @Test
  void rollbackStaleFutureAndNonfiniteTimestampsFailClosed() {
    SwerveStateFreshnessTracker tracker = healthyTracker();

    assertFalse(tracker.observe(3.08, 2.99));
    assertFalse(tracker.observe(4.20, 4.099));
    assertFalse(tracker.observe(5.00, 5.001));
    assertFalse(tracker.observe(Double.NaN, 5.00));
    assertFalse(tracker.observe(5.00, Double.NaN));
  }

  @Test
  void resetDiscardsHistoricProgress() {
    SwerveStateFreshnessTracker tracker = healthyTracker();

    tracker.reset();

    assertFalse(tracker.isFresh());
    assertFalse(tracker.observe(6.00, 6.00));
    assertEquals(0, tracker.getConsecutiveHealthyAdvances());
  }

  private static SwerveStateFreshnessTracker healthyTracker() {
    SwerveStateFreshnessTracker tracker = new SwerveStateFreshnessTracker();
    assertFalse(tracker.observe(3.00, 3.00));
    assertFalse(tracker.observe(3.02, 3.01));
    assertFalse(tracker.observe(3.04, 3.03));
    assertTrue(tracker.observe(3.06, 3.05));
    return tracker;
  }
}
