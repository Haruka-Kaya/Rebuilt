package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;

class SparkTelemetryValidatorTest {
  @Test
  void acceptsFiniteStatusWithOrWithoutAnEncoder() {
    assertNull(SparkTelemetryValidator.failureReason(
        0.2, 12.4, 5.0, 30.0, true, 1.5, -200.0));
    assertNull(SparkTelemetryValidator.failureReason(
        0.0, 12.4, 0.0, 30.0, false, Double.NaN, Double.NaN));
  }

  @Test
  void rejectsEveryNonfiniteStatusZeroField() {
    assertNotNull(SparkTelemetryValidator.failureReason(
        Double.NaN, 12.0, 1.0, 30.0, true, 0.0, 0.0));
    assertNotNull(SparkTelemetryValidator.failureReason(
        0.0, Double.POSITIVE_INFINITY, 1.0, 30.0, true, 0.0, 0.0));
    assertNotNull(SparkTelemetryValidator.failureReason(
        0.0, 12.0, Double.NaN, 30.0, true, 0.0, 0.0));
    assertNotNull(SparkTelemetryValidator.failureReason(
        0.0, 12.0, 1.0, Double.NaN, true, 0.0, 0.0));
  }

  @Test
  void rejectsImpossibleRangesAndEncoderTelemetry() {
    assertNotNull(SparkTelemetryValidator.failureReason(
        1.1, 12.0, 1.0, 30.0, true, 0.0, 0.0));
    assertNotNull(SparkTelemetryValidator.failureReason(
        0.0, 0.0, 1.0, 30.0, true, 0.0, 0.0));
    assertNotNull(SparkTelemetryValidator.failureReason(
        0.0, 12.0, -1.0, 30.0, true, 0.0, 0.0));
    assertNotNull(SparkTelemetryValidator.failureReason(
        0.0, 12.0, 1.0, 30.0, true, Double.NaN, 0.0));
    assertNotNull(SparkTelemetryValidator.failureReason(
        0.0, 12.0, 1.0, 30.0, true, 0.0, Double.NEGATIVE_INFINITY));
  }
}
