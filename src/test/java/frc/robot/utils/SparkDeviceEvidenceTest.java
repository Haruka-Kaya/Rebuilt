package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.constants.ConfiguredCanHardware;
import frc.robot.utils.SparkMAXContainer.DeviceEvidenceSnapshot;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class SparkDeviceEvidenceTest {
  @Test
  void configuredRevDevicesComeFromTheTypedCanManifest() {
    assertEquals(
        ConfiguredCanHardware.sparkDeviceIds(),
        SparkDeviceEvidence.configuredSparkDevices().stream()
            .map(ConfiguredCanHardware.Device::canId)
            .toList());
    assertEquals(
        List.of(30, 31, 32, 33, 34, 35, 36, 37, 38, 39),
        SparkDeviceEvidence.configuredSparkDevices().stream()
            .map(ConfiguredCanHardware.Device::canId)
            .toList());
  }

  @Test
  void liveProjectionPublishesFactsAndManifestMetadataUnderTheExactDevicePath() {
    Map<String, Object> values = SparkDeviceEvidence.valuesFor(List.of(snapshot(30)), false);
    String prefix = "Hardware/Device/ID30/";

    assertEquals("/SmartDashboard/Hardware/Device/ID30/",
        SparkDeviceEvidence.networkTablesPrefixForId(30));
    assertEquals("intake actuator", values.get(prefix + "Label"));
    assertEquals("REV", values.get(prefix + "Vendor"));
    assertEquals("MOTOR_CONTROLLER", values.get(prefix + "Type"));
    assertEquals("MECHANISM_MOTOR", values.get(prefix + "Role"));
    assertEquals("[]", values.get(prefix + "Dependency CAN IDs"));
    assertEquals("LIVE_CACHED_SPARK_TELEMETRY", values.get(prefix + "Source"));
    assertEquals(true, values.get(prefix + "Ready"));
    assertEquals("READY", values.get(prefix + "Reason"));
    assertEquals(0.25, values.get(prefix + "Applied Output"));
    assertEquals(7.5, values.get(prefix + "Current Amps"));
    assertEquals(1234.0, values.get(prefix + "Velocity RPM"));
    assertEquals(0.125, values.get(prefix + "Sample Age Seconds"));
    assertEquals(5L, values.get(prefix + "Sample Output Epoch"));
    assertEquals(true, values.get(prefix + "Sample Matches Current Output Epoch"));
    assertEquals(true, values.get(prefix + "Last Nonzero Setpoint API Returned OK"));
    assertEquals(8L, values.get(prefix + "Last Nonzero Setpoint Epoch"));
    assertEquals(
        "REV_API_RETURNED_K_OK_NOT_MOTION_PROOF",
        values.get(prefix + "Last Nonzero Setpoint Reason"));
    assertEquals(5L, values.get(prefix + "Output Epoch"));
    assertEquals("OUTPUT_MAY_BE_NONZERO", values.get(prefix + "Stop State"));
    assertTrue(values.get(prefix + "Snapshot").toString().contains("NOT_MOTION_PROOF"));

    for (int canId : ConfiguredCanHardware.sparkDeviceIds()) {
      assertTrue(values.containsKey(SparkDeviceEvidence.dashboardPrefixForId(canId) + "Snapshot"));
    }
  }

  @Test
  void missingAndDuplicateRegistrationsNeverLookReadyOrStopped() {
    Map<String, Object> missing = SparkDeviceEvidence.valuesFor(List.of(), false);
    String missingPrefix = "Hardware/Device/ID30/";
    assertEquals(false, missing.get(missingPrefix + "Ready"));
    assertEquals("DEVICE_NOT_REGISTERED", missing.get(missingPrefix + "Reason"));
    assertTrue(Double.isNaN((double) missing.get(missingPrefix + "Applied Output")));
    assertEquals(
        "UNAVAILABLE_DEVICE_NOT_REGISTERED", missing.get(missingPrefix + "Stop State"));

    Map<String, Object> duplicate = SparkDeviceEvidence.valuesFor(
        List.of(snapshot(30), snapshot(30)), false);
    assertEquals(false, duplicate.get(missingPrefix + "Ready"));
    assertEquals("DUPLICATE_REGISTERED_CAN_ID", duplicate.get(missingPrefix + "Reason"));
    assertFalse(duplicate.get(missingPrefix + "Stop State").toString().contains("CONFIRMED"));
  }

  @Test
  void fmsSuppressionInvalidatesEveryDynamicRevValue() {
    Map<String, Object> values = SparkDeviceEvidence.valuesFor(List.of(snapshot(30)), true);

    for (int canId : ConfiguredCanHardware.sparkDeviceIds()) {
      String prefix = SparkDeviceEvidence.dashboardPrefixForId(canId);
      assertEquals(false, values.get(prefix + "Ready"));
      assertEquals("SUPPRESSED_FMS", values.get(prefix + "Reason"));
      assertEquals("SUPPRESSED_FMS", values.get(prefix + "Source"));
      assertTrue(Double.isNaN((double) values.get(prefix + "Applied Output")));
      assertTrue(Double.isNaN((double) values.get(prefix + "Current Amps")));
      assertTrue(Double.isNaN((double) values.get(prefix + "Velocity RPM")));
      assertTrue(Double.isNaN((double) values.get(prefix + "Sample Age Seconds")));
      assertEquals(-1L, values.get(prefix + "Sample Output Epoch"));
      assertEquals(false, values.get(prefix + "Sample Matches Current Output Epoch"));
      assertEquals("UNAVAILABLE_SUPPRESSED_FMS", values.get(prefix + "Stop State"));
    }
  }

  @Test
  void stopStateDoesNotCallAnActiveOutputAPendingZero() {
    assertEquals(
        "OUTPUT_MAY_BE_NONZERO",
        SparkDeviceEvidence.stopState(
            SparkOutputStopEvaluator.Status.ZERO_PENDING,
            false, false, true, false, false));
    assertEquals(
        "ZERO_WRITE_IN_FLIGHT",
        SparkDeviceEvidence.stopState(
            SparkOutputStopEvaluator.Status.ZERO_PENDING,
            true, true, true, false, false));
    assertEquals(
        "ZERO_REQUIRED_OR_RETRY_PENDING",
        SparkDeviceEvidence.stopState(
            SparkOutputStopEvaluator.Status.ZERO_PENDING,
            false, true, true, false, false));
    assertEquals(
        "CONFIRMED",
        SparkDeviceEvidence.stopState(
            SparkOutputStopEvaluator.Status.CONFIRMED,
            false, false, false, false, false));
    assertEquals(
        "LEADER_STOP_NOT_CONFIRMED",
        SparkDeviceEvidence.stopState(
            SparkOutputStopEvaluator.Status.CONFIRMED,
            false, false, false, true, false));
  }

  private static DeviceEvidenceSnapshot snapshot(int canId) {
    return new DeviceEvidenceSnapshot(
        canId,
        true,
        "READY",
        0.25,
        7.5,
        1234.0,
        0.125,
        5L,
        true,
        true,
        8L,
        "REV_API_RETURNED_K_OK_NOT_MOTION_PROOF",
        5L,
        "OUTPUT_MAY_BE_NONZERO");
  }
}
