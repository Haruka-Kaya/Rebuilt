package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.constants.ConfiguredCanHardware;
import frc.robot.utils.SparkMAXContainer.DeviceEvidenceSnapshot;
import java.util.List;
import org.junit.jupiter.api.Test;

class CanDeviceEvidenceSummaryTest {
  @Test
  void summaryContainsEveryConfiguredIdExactlyOnceAndDoesNotInventMissingEvidence() {
    String summary = CanDeviceEvidenceSummary.format(
        List.of(new DeviceEvidenceSnapshot(
            30, true, "READY", 0.1, 2.0, 30.0, 0.02, 3, true,
            true, 4, "REV_API_RETURNED_K_OK_NOT_MOTION_PROOF", 3, "OUTPUT_MAY_BE_NONZERO")),
        List.of());
    List<String> lines = summary.lines().toList();

    assertEquals(ConfiguredCanHardware.devices().size(), lines.size());
    for (var device : ConfiguredCanHardware.devices()) {
      assertEquals(1, lines.stream()
          .filter(line -> line.startsWith("ID" + device.canId() + " "))
          .count());
    }
    assertTrue(summary.contains("ID30 REV intake actuator"));
    assertTrue(CanDeviceEvidenceSummary.SCOPE.contains("MOTION"));
    assertTrue(CanDeviceEvidenceSummary.SCOPE.lines().allMatch(line -> line.length() <= 80));
    assertTrue(summary.contains("ID20 CTRE swerve pigeon ready=false"));
    assertTrue(lines.stream().allMatch(line -> line.length() <= 160), summary);
    assertFalse(summary.contains("NaN"));
  }

  @Test
  void duplicateControllerEvidenceIsFailClosedInsteadOfLastWriterWins() {
    DeviceEvidenceSnapshot snapshot = new DeviceEvidenceSnapshot(
        30, true, "READY", 0.1, 2.0, 30.0, 0.02, 3, true,
        true, 4, "REV_API_RETURNED_K_OK_NOT_MOTION_PROOF", 3,
        "OUTPUT_MAY_BE_NONZERO");

    String id30 = CanDeviceEvidenceSummary.format(List.of(snapshot, snapshot), List.of())
        .lines()
        .filter(line -> line.startsWith("ID30 "))
        .findFirst()
        .orElseThrow();

    assertTrue(id30.contains("ready=false reason=DUPLICATE_REGISTERED_CAN_ID"));
    assertFalse(id30.contains("ready=true"));
  }

  @Test
  void unboundedVendorFailureTextCannotShrinkTheWholeOperatorSummary() {
    String unboundedFailure = "vendor failure\n" + "X".repeat(1000);
    DeviceEvidenceSnapshot snapshot = new DeviceEvidenceSnapshot(
        30, false, unboundedFailure, 0.0, 0.0, 0.0, 0.02, 3, false,
        false, 4, unboundedFailure, 3, unboundedFailure);

    String summary = CanDeviceEvidenceSummary.format(List.of(snapshot), List.of());
    String id30 = summary.lines()
        .filter(line -> line.startsWith("ID30 "))
        .findFirst()
        .orElseThrow();

    assertTrue(id30.length() <= 160, id30);
    assertTrue(id30.contains("..."));
    assertEquals(ConfiguredCanHardware.devices().size(), summary.lines().count());
  }
}
