package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class CtreSignalFreshnessTest {
  @Test
  void acceptsFiniteOkSignalAtAgeBoundary() {
    assertTrue(CtreSignalFreshness.isFresh(true, true, 0.100, -2.5, 0.100));
  }

  @Test
  void rejectsErrorInvalidTimestampStaleFutureAndNonfiniteValue() {
    assertFalse(CtreSignalFreshness.isFresh(false, true, 0.01, 1.0, 0.100));
    assertFalse(CtreSignalFreshness.isFresh(true, false, 0.01, 1.0, 0.100));
    assertFalse(CtreSignalFreshness.isFresh(true, true, 0.101, 1.0, 0.100));
    assertFalse(CtreSignalFreshness.isFresh(true, true, -0.001, 1.0, 0.100));
    assertFalse(CtreSignalFreshness.isFresh(true, true, 0.01, Double.NaN, 0.100));
    assertFalse(CtreSignalFreshness.isFresh(true, true, 0.01, 1.0, Double.NaN));
  }
}
