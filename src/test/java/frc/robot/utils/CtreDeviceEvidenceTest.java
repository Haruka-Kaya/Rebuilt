package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.utils.CtreDeviceEvidence.Metric;
import frc.robot.constants.ConfiguredCanHardware;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class CtreDeviceEvidenceTest {
  @Test
  void signalEvaluationIsFailClosedAndReportsTheFirstExactReason() {
    var healthy = CtreDeviceEvidence.evaluateSignal(
        "position",
        Metric.CONTROL_POSITION_ROTATIONS,
        true,
        "OK",
        true,
        4.9,
        4.9,
        0.1,
        2.0,
        0.1);
    var stale = CtreDeviceEvidence.evaluateSignal(
        "current", Metric.STATOR_CURRENT_AMPS, true, "OK", true, 4.7, 4.7, 0.3, 4.0, 0.1);

    assertTrue(healthy.fresh());
    assertFalse(stale.fresh());
    assertEquals("STALE", stale.reason());
    assertFalse(CtreDeviceEvidence.allFresh(List.of(healthy, stale)));
    assertEquals("current/STALE", CtreDeviceEvidence.firstFailureReason(List.of(healthy, stale)));
  }

  @Test
  void rejectsStatusTimestampAgeAndValueFailuresWithoutInventingReadiness() {
    assertEquals(
        "STATUS_RxTimeout",
        signal(false, "RxTimeout", true, 1.0, 0.01, 1.0).reason());
    assertEquals(
        "TIMESTAMP_INVALID",
        signal(true, "OK", false, 1.0, 0.01, 1.0).reason());
    assertEquals(
        "SAMPLE_AGE_INVALID",
        signal(true, "OK", true, 1.0, -0.01, 1.0).reason());
    assertEquals(
        "VALUE_NONFINITE",
        signal(true, "OK", true, 1.0, 0.01, Double.NaN).reason());
    assertFalse(CtreDeviceEvidence.allFresh(List.of()));
    assertEquals("NO_SIGNALS", CtreDeviceEvidence.firstFailureReason(List.of()));
  }

  @Test
  void publisherProjectsEveryManifestIdAndInvalidatesMissingDuplicateAndNullEvidence() {
    Map<String, Object> missing = CtreDeviceEvidence.valuesFor(List.of(), false);
    for (int canId : ConfiguredCanHardware.ctreDeviceIds()) {
      String prefix = CtreDeviceEvidence.dashboardPrefixForId(canId);
      assertEquals(false, missing.get(prefix + "Ready"));
      assertEquals("DEVICE_EVIDENCE_NOT_CAPTURED", missing.get(prefix + "Reason"));
      assertTrue(Double.isNaN((double) missing.get(prefix + "Applied Output")));
    }

    var snapshot = new CtreDeviceEvidence.Snapshot(
        20, "swerve pigeon", "SWERVE_GYRO", true, "OK", true, "OK", 1.0,
        List.of(signal(true, "OK", true, 1.0, 0.01, 0.0)));
    Map<String, Object> duplicate = CtreDeviceEvidence.valuesFor(
        List.of(snapshot, snapshot), false);
    String prefix = CtreDeviceEvidence.dashboardPrefixForId(20);
    assertEquals(false, duplicate.get(prefix + "Ready"));
    assertEquals("DUPLICATE_DEVICE_EVIDENCE", duplicate.get(prefix + "Reason"));

    Map<String, Object> nullOnly = CtreDeviceEvidence.valuesFor(
        Collections.singletonList(null), false);
    assertEquals("DEVICE_EVIDENCE_NOT_CAPTURED", nullOnly.get(prefix + "Reason"));
  }

  @Test
  void fmsSuppressionClearsEveryDynamicCtreMetric() {
    Map<String, Object> suppressed = CtreDeviceEvidence.valuesFor(List.of(), true);
    for (int canId : ConfiguredCanHardware.ctreDeviceIds()) {
      String prefix = CtreDeviceEvidence.dashboardPrefixForId(canId);
      assertEquals(false, suppressed.get(prefix + "Ready"));
      assertEquals("SUPPRESSED_FMS", suppressed.get(prefix + "Reason"));
      assertTrue(Double.isNaN((double) suppressed.get(prefix + "Current Amps")));
      assertTrue(Double.isNaN((double) suppressed.get(prefix + "Motor Voltage Volts")));
      assertTrue(Double.isNaN((double) suppressed.get(prefix + "Rotor Position Rotations")));
      assertTrue(Double.isNaN((double) suppressed.get(prefix + "Absolute Position Rotations")));
      assertTrue(Double.isNaN((double) suppressed.get(prefix + "Yaw Degrees")));
      assertEquals("UNAVAILABLE_SUPPRESSED_FMS", suppressed.get(prefix + "Stop State"));
    }
  }

  private static CtreDeviceEvidence.SignalObservation signal(
      boolean statusOk,
      String status,
      boolean timestampValid,
      double sampleTimestamp,
      double sampleAge,
      double value) {
    return CtreDeviceEvidence.evaluateSignal(
        "signal",
        Metric.CONTROL_VELOCITY_ROTATIONS_PER_SECOND,
        statusOk,
        status,
        timestampValid,
        sampleTimestamp,
        sampleTimestamp,
        sampleAge,
        value,
        0.1);
  }
}
