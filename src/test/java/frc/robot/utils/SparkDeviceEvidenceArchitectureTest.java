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
  private static final Path PROCESS_SAFETY_SOURCE = Path.of(
      "src", "main", "java", "frc", "robot", "utils", "ProcessOutputSafety.java");
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
    int outputLock = setpointMethod.indexOf("synchronized (OUTPUT_ORDER_LOCK)");
    int authorizationCheck = setpointMethod.indexOf(
        "authorizationStillValid.getAsBoolean()", outputLock);
    int processAuthorizationCheck = setpointMethod.indexOf(
        "ProcessOutputSafety.callIfAuthorized", authorizationCheck);
    int vendorSetpoint = setpointMethod.indexOf(
        "sendSetpointTracked(value, controlType)", processAuthorizationCheck);

    assertTrue(successfulBranch >= 0);
    assertTrue(outputLock >= 0);
    assertTrue(authorizationCheck > outputLock);
    assertTrue(processAuthorizationCheck > authorizationCheck);
    assertTrue(vendorSetpoint > processAuthorizationCheck);
    assertTrue(outputEpochAdvance > successfulBranch && outputEpochAdvance < failureBranch);
    assertTrue(acceptedEvidence > successfulBranch && acceptedEvidence < failureBranch);
    assertTrue(setpointMethod.contains("REV_API_RETURNED_K_OK_NOT_MOTION_PROOF"));
    assertTrue(setpointMethod.contains("PROCESS_OUTPUT_HEARTBEAT_EXPIRED"));
  }

  @Test
  void processHeartbeatIsCheckedInsideBothOutputLocksImmediatelyBeforeNonzeroVendorApi()
      throws IOException {
    String source = Files.readString(CONTAINER_SOURCE);
    String setpointMethod = between(
        source, "private boolean trySetpoint(", "public boolean beginFollowerDiagnostic(");

    int outputLock = setpointMethod.indexOf("synchronized (OUTPUT_ORDER_LOCK)");
    int stateLock = setpointMethod.indexOf("synchronized (stateLock)", outputLock);
    int processAuthorizationCheck = setpointMethod.indexOf(
        "ProcessOutputSafety.callIfAuthorized", stateLock);
    int vendorSetpoint = setpointMethod.indexOf(
        "sendSetpointTracked(value, controlType)", processAuthorizationCheck);
    int outputLockEnd = matchingBrace(setpointMethod, setpointMethod.indexOf('{', outputLock));
    int stateLockEnd = matchingBrace(setpointMethod, setpointMethod.indexOf('{', stateLock));

    assertTrue(outputLock >= 0);
    assertTrue(stateLock > outputLock);
    assertTrue(processAuthorizationCheck > stateLock);
    assertTrue(vendorSetpoint > processAuthorizationCheck);
    assertTrue(vendorSetpoint < stateLockEnd);
    assertTrue(vendorSetpoint < outputLockEnd);
    assertTrue(setpointMethod.substring(processAuthorizationCheck, vendorSetpoint)
        .contains("SparkVendorCall.execute"));
    String processSafety = Files.readString(PROCESS_SAFETY_SOURCE);
    String atomicCall = between(
        processSafety, "public static <T> AuthorizedCall<T> callIfAuthorized(",
        "/** Returns the current immutable authorization evidence.");
    assertTrue(atomicCall.indexOf("synchronized (LOCK)")
        < atomicCall.indexOf("vendorCall.get()"));
    assertTrue(setpointMethod.contains(
        "lastRequestReason = \"PROCESS_OUTPUT_HEARTBEAT_EXPIRED\""));
    assertTrue(setpointMethod.contains("outputGate.requireZero(now, false)"));
  }

  @Test
  void zeroRequestAndWorkerRemainAuthorizedAfterTheProcessHeartbeatExpires()
      throws IOException {
    String source = Files.readString(CONTAINER_SOURCE);
    String zeroRequestMethod = between(
        source,
        "private void requestZeroOutput()",
        "private void rejectSetpointRequestAndRequestZero");
    String zeroWorkerMethod = between(
        source, "private static void runZeroWork(", "private void completeZeroLocked(");

    assertFalse(zeroRequestMethod.contains("ProcessOutputSafety"));
    assertFalse(zeroWorkerMethod.contains("ProcessOutputSafety"));
    assertTrue(zeroWorkerMethod.contains(
        "sendSetpointTracked(0.0, ControlType.kDutyCycle)"));
  }

  private static String between(String source, String start, String end) {
    int startIndex = source.indexOf(start);
    int endIndex = source.indexOf(end, startIndex + start.length());
    assertTrue(startIndex >= 0, () -> "Missing source marker: " + start);
    assertTrue(endIndex > startIndex, () -> "Missing source marker: " + end);
    return source.substring(startIndex, endIndex);
  }

  private static int matchingBrace(String source, int openingBrace) {
    assertTrue(openingBrace >= 0, "Missing opening brace");
    int depth = 0;
    for (int index = openingBrace; index < source.length(); index++) {
      if (source.charAt(index) == '{') {
        depth++;
      } else if (source.charAt(index) == '}' && --depth == 0) {
        return index;
      }
    }
    throw new AssertionError("Missing closing brace");
  }
}
