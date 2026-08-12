package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.constants.ConfiguredCanHardware;
import frc.robot.constants.ConfiguredCanHardware.Vendor;
import frc.robot.utils.CtreDeviceEvidence;
import frc.robot.utils.SparkMAXContainer.DeviceEvidenceSnapshot;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class HardwareSelfTestCanEvidenceTest {
  @Test
  void configuredManifestIsEvaluatedExactlyOnceAcrossBothVendors() {
    var results = HardwareSelfTestCommand.evaluateCanEvidence(
        readySparkSnapshots(), readyCtreSnapshots());

    assertEquals(ConfiguredCanHardware.devices().size(), results.size());
    assertEquals(ConfiguredCanHardware.allDeviceIds(), results.keySet().stream().toList());
    assertTrue(results.values().stream().allMatch(
        HardwareSelfTestCommand.CanEvidenceResult::ready));
  }

  @Test
  void missingAndDuplicateEvidenceFailClosedPerCanId() {
    List<DeviceEvidenceSnapshot> sparks = new ArrayList<>(readySparkSnapshots());
    sparks.removeIf(snapshot -> snapshot.canId() == 30);
    DeviceEvidenceSnapshot duplicateSpark = readySpark(31);
    sparks.add(duplicateSpark);

    List<CtreDeviceEvidence.Snapshot> ctres = new ArrayList<>(readyCtreSnapshots());
    ctres.removeIf(snapshot -> snapshot.canId() == 20);
    CtreDeviceEvidence.Snapshot duplicateCtre = readyCtre(40);
    ctres.add(duplicateCtre);

    var results = HardwareSelfTestCommand.evaluateCanEvidence(sparks, ctres);

    assertEquals("DEVICE_NOT_REGISTERED", results.get(30).reason());
    assertEquals("DUPLICATE_REGISTERED_CAN_ID", results.get(31).reason());
    assertEquals("DEVICE_EVIDENCE_NOT_CAPTURED", results.get(20).reason());
    assertEquals("DUPLICATE_DEVICE_EVIDENCE", results.get(40).reason());
    assertFalse(results.get(30).ready());
    assertFalse(results.get(31).ready());
    assertFalse(results.get(20).ready());
    assertFalse(results.get(40).ready());
  }

  @Test
  void evidenceSourceExplicitlyDistinguishesHardwareFromRawDesktopEcho() {
    assertEquals("LIVE_HARDWARE", HardwareSelfTestCommand.evidenceSource(false));
    assertEquals(
        "DESKTOP_SIMULATION_RAW_COMMAND_ECHO_NO_MECHANISM_PHYSICS",
        HardwareSelfTestCommand.evidenceSource(true));
  }

  @Test
  void architectureReadsBothPublicSnapshotApisAndPreservesSparkTopics() throws IOException {
    String source = Files.readString(Path.of(
        "src", "main", "java", "frc", "robot", "commands",
        "HardwareSelfTestCommand.java"));

    assertTrue(source.contains("SparkMAXContainer.getDeviceEvidenceSnapshots()"));
    assertTrue(source.contains("drivetrain.getDeviceEvidenceSnapshots()"));
    assertTrue(source.contains("\"Hardware Self-Test/CAN Results\""));
    assertTrue(source.contains("\"Hardware Self-Test/All CAN Results\""));
    assertTrue(source.contains("vendorPrefix + device.canId() + \"/\""));
    assertTrue(source.contains("publishOverall(results, allCanResults, runState)"));
    assertTrue(source.contains("allCanResults.values().stream()"));
  }

  private static List<DeviceEvidenceSnapshot> readySparkSnapshots() {
    return ConfiguredCanHardware.devices().stream()
        .filter(device -> device.vendor() == Vendor.REV)
        .map(device -> readySpark(device.canId()))
        .toList();
  }

  private static List<CtreDeviceEvidence.Snapshot> readyCtreSnapshots() {
    return ConfiguredCanHardware.devices().stream()
        .filter(device -> device.vendor() == Vendor.CTRE)
        .map(device -> readyCtre(device.canId()))
        .toList();
  }

  private static DeviceEvidenceSnapshot readySpark(int canId) {
    return new DeviceEvidenceSnapshot(
        canId,
        true,
        "READY",
        0.0,
        0.0,
        0.0,
        0.01,
        1,
        true,
        true,
        1,
        "REV_API_RETURNED_K_OK_NOT_MOTION_PROOF",
        1,
        "CONFIRMED_STOPPED");
  }

  private static CtreDeviceEvidence.Snapshot readyCtre(int canId) {
    var device = ConfiguredCanHardware.byCanId(canId).orElseThrow();
    return new CtreDeviceEvidence.Snapshot(
        canId,
        device.label(),
        device.role().name(),
        true,
        "READY",
        true,
        "READY",
        1.0,
        List.of());
  }
}
