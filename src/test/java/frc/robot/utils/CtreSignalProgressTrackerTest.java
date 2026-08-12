package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.utils.CtreDeviceEvidence.Metric;
import frc.robot.utils.CtreDeviceEvidence.SignalObservation;
import java.util.List;
import org.junit.jupiter.api.Test;

class CtreSignalProgressTrackerTest {
  @Test
  void sameCachedFrameCannotRearmButThreeDistinctAdvancesCan() {
    var tracker = new CtreSignalProgressTracker(3);
    assertFalse(tracker.observe(sample(1.0)));
    assertFalse(tracker.observe(sample(1.0)));
    assertEquals(0, tracker.distinctAdvances());
    assertFalse(tracker.observe(sample(2.0)));
    assertFalse(tracker.observe(sample(3.0)));
    assertTrue(tracker.observe(sample(4.0)));
  }

  @Test
  void failureResetsProgressAndRequiresACompleteNewSequence() {
    var tracker = new CtreSignalProgressTracker(2);
    tracker.observe(sample(1.0));
    tracker.observe(sample(2.0));
    assertFalse(tracker.observe(List.of(new SignalObservation(
        "signal", Metric.DUTY_CYCLE, false, "STALE", 0.0, 2.0, 2.0, 0.2))));
    assertEquals(0, tracker.distinctAdvances());
    assertFalse(tracker.observe(sample(3.0)));
    assertFalse(tracker.observe(sample(4.0)));
    assertTrue(tracker.observe(sample(5.0)));
  }

  @Test
  void timestampRollbackDropsAReadyDeviceImmediately() {
    var tracker = new CtreSignalProgressTracker(1);
    assertFalse(tracker.observe(sample(2.0)));
    assertTrue(tracker.observe(sample(3.0)));
    assertFalse(tracker.observe(sample(1.0)));
    assertEquals(0, tracker.distinctAdvances());
  }

  private static List<SignalObservation> sample(double timestamp) {
    return List.of(new SignalObservation(
        "signal", Metric.DUTY_CYCLE, true, "OK", 0.0, timestamp, timestamp, 0.01));
  }
}
