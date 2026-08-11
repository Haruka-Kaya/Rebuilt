package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class SwerveDaqFreshnessTrackerTest {
  @Test
  void startsFailClosedAndRequiresThreeCurrentHealthyObservations() {
    SwerveDaqFreshnessTracker tracker = new SwerveDaqFreshnessTracker();

    assertFalse(tracker.observe(10.00, 9.99, 1_000, 12));
    assertFalse(tracker.observe(10.02, 10.01, 1_002, 12));
    assertFalse(tracker.observe(10.04, 10.03, 1_004, 12));
    assertTrue(tracker.observe(10.06, 10.05, 1_006, 12));

    assertTrue(tracker.isFresh());
    assertEquals(3, tracker.getConsecutiveHealthyObservations());
  }

  @Test
  void successfulCounterMustHaveAdvancedWithinOneHundredMilliseconds() {
    SwerveDaqFreshnessTracker tracker = healthyTracker();

    assertTrue(tracker.observe(1.10, 1.10, 4, 0));
    assertFalse(tracker.observe(1.201, 1.201, 4, 0));

    assertFalse(tracker.isFresh());
    assertEquals(0, tracker.getConsecutiveHealthyObservations());
  }

  @Test
  void repeatedReadsOfOneSuccessfulAcquisitionCannotRecover() {
    SwerveDaqFreshnessTracker tracker = new SwerveDaqFreshnessTracker();

    assertFalse(tracker.observe(3.00, 3.00, 10, 0));
    assertFalse(tracker.observe(3.02, 3.02, 11, 0));
    assertFalse(tracker.observe(3.04, 3.02, 11, 0));
    assertFalse(tracker.observe(3.06, 3.02, 11, 0));
    assertEquals(1, tracker.getConsecutiveHealthyObservations());
  }

  @Test
  void failedDaqWinsWhenBothCountersAdvance() {
    SwerveDaqFreshnessTracker tracker = healthyTracker();

    assertFalse(tracker.observe(1.08, 1.08, 5, 1));
    assertFalse(tracker.isFresh());
    assertEquals(0, tracker.getConsecutiveHealthyObservations());
  }

  @Test
  void counterRollbackFailsAndRequiresAFullRecoveryStreak() {
    SwerveDaqFreshnessTracker tracker = healthyTracker();

    assertFalse(tracker.observe(1.08, 1.08, 1, 0));
    assertFalse(tracker.observe(1.10, 1.10, 2, 0));
    assertFalse(tracker.observe(1.12, 1.12, 3, 0));
    assertTrue(tracker.observe(1.14, 1.14, 4, 0));
  }

  @Test
  void rejectsNonFiniteStaleAndFutureTimestamps() {
    SwerveDaqFreshnessTracker tracker = healthyTracker();

    assertFalse(tracker.observe(1.08, Double.NaN, 5, 0));
    assertFalse(tracker.observe(Double.NaN, 1.09, 6, 0));
    assertFalse(tracker.observe(1.20, 1.099, 7, 0));
    assertFalse(tracker.observe(1.22, 1.221, 8, 0));
  }

  @Test
  void acceptsAStateYoungerThanOneHundredMilliseconds() {
    SwerveDaqFreshnessTracker tracker = new SwerveDaqFreshnessTracker();

    assertFalse(tracker.observe(2.00, 1.901, 0, 0));
    assertFalse(tracker.observe(2.02, 1.921, 1, 0));
    assertFalse(tracker.observe(2.04, 1.941, 2, 0));
    assertTrue(tracker.observe(2.06, 1.961, 3, 0));
  }

  @Test
  void resetDiscardsHistoricSuccess() {
    SwerveDaqFreshnessTracker tracker = healthyTracker();

    tracker.reset();

    assertFalse(tracker.isFresh());
    assertFalse(tracker.observe(2.0, 2.0, 500, 20));
    assertEquals(0, tracker.getConsecutiveHealthyObservations());
  }

  private static SwerveDaqFreshnessTracker healthyTracker() {
    SwerveDaqFreshnessTracker tracker = new SwerveDaqFreshnessTracker();
    assertFalse(tracker.observe(1.00, 1.00, 0, 0));
    assertFalse(tracker.observe(1.02, 1.02, 1, 0));
    assertFalse(tracker.observe(1.04, 1.04, 2, 0));
    assertTrue(tracker.observe(1.06, 1.06, 3, 0));
    return tracker;
  }
}
