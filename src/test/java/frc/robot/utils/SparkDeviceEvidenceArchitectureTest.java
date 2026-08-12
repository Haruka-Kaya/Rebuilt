package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

class SparkDeviceEvidenceArchitectureTest {
  private static final Path CONTAINER_SOURCE = Path.of(
      "src", "main", "java", "frc", "robot", "utils", "SparkMAXContainer.java");
  private static final Path PUBLISHER_SOURCE = Path.of(
      "src", "main", "java", "frc", "robot", "utils", "SparkDeviceEvidence.java");

  @Test
  void dashboardCoverageIsManifestDrivenRatherThanAHardcodedIdList() throws IOException {
    String source = Files.readString(PUBLISHER_SOURCE);

    assertTrue(source.contains("ConfiguredCanHardware.devices()"));
    assertTrue(source.contains("device.vendor() == Vendor.REV"));
    assertFalse(source.contains("ID30/"));
    assertFalse(source.contains("List.of(30"));
  }

  @Test
  void evidenceSnapshotUsesOnlyAtomicCachedStateAndNeverPerformsVendorReads() throws IOException {
    String source = Files.readString(CONTAINER_SOURCE);
    String snapshotMethod = between(
        source,
        "private DeviceEvidenceSnapshot getDeviceEvidenceSnapshot()",
        "private String readinessReasonLocked");

    assertFalse(snapshotMethod.contains("getPeriodicStatus"));
    assertFalse(snapshotMethod.contains("getFirmwareVersion"));
    assertFalse(snapshotMethod.contains("motor."));
    assertFalse(snapshotMethod.contains("encoder."));
    assertTrue(snapshotMethod.contains("SparkOutputStopEvaluator.evaluate"));
    assertTrue(snapshotMethod.contains("sampleValid ? cachedAppliedOutput : Double.NaN"));
    assertTrue(snapshotMethod.contains("sampleValid ? cachedCurrent : Double.NaN"));
    assertTrue(snapshotMethod.contains("lastSampleOutputEpoch == outputEpoch"));
    assertTrue(snapshotMethod.contains("sampleMatchesCurrentOutputEpoch"));
    assertTrue(snapshotMethod.contains("lastSampleAt >= leaderStoppedAt"));
  }

  @Test
  void requestAcceptanceAndOutputEpochAdvanceOnlyInTheSuccessfulVendorBranch()
      throws IOException {
    String source = Files.readString(CONTAINER_SOURCE);
    String setpointMethod = between(
        source, "private boolean trySetpoint(", "public boolean beginFollowerDiagnostic(");
    int successfulBranch = setpointMethod.indexOf("if (result.succeeded())");
    int outputEpochAdvance = setpointMethod.indexOf("outputEpoch++", successfulBranch);
    int acceptedEvidence = setpointMethod.indexOf("lastRequestAccepted = true", successfulBranch);
    int failureBranch = setpointMethod.indexOf("} else {", successfulBranch);

    assertTrue(successfulBranch >= 0);
    assertTrue(outputEpochAdvance > successfulBranch && outputEpochAdvance < failureBranch);
    assertTrue(acceptedEvidence > successfulBranch && acceptedEvidence < failureBranch);
    assertTrue(setpointMethod.contains("REV_API_RETURNED_K_OK_NOT_MOTION_PROOF"));
  }

  private static String between(String source, String start, String end) {
    int startIndex = source.indexOf(start);
    int endIndex = source.indexOf(end, startIndex + start.length());
    assertTrue(startIndex >= 0, () -> "Missing source marker: " + start);
    assertTrue(endIndex > startIndex, () -> "Missing source marker: " + end);
    return source.substring(startIndex, endIndex);
  }
}
