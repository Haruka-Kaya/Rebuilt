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
        source,
        "private boolean trySetpoint(",
        "public boolean beginFollowerDiagnosticIfAuthorized(");
    int successfulBranch = setpointMethod.indexOf("if (vendorResult.succeeded())");
    int outputEpochAdvance = setpointMethod.indexOf("outputEpoch++", successfulBranch);
    int acceptedEvidence = setpointMethod.indexOf("lastRequestAccepted = true", successfulBranch);
    int failureBranch = setpointMethod.indexOf("} else {", successfulBranch);
    int firstOutputLock = setpointMethod.indexOf("synchronized (OUTPUT_ORDER_LOCK)");
    int firstOutputLockEnd = matchingBrace(
        setpointMethod, setpointMethod.indexOf('{', firstOutputLock));
    int authorizationCheck = setpointMethod.indexOf("authorizationAllows(authorizationStillValid)");
    int outputLock = setpointMethod.indexOf("synchronized (OUTPUT_ORDER_LOCK)", authorizationCheck);
    int stopSequenceRecheck = setpointMethod.indexOf(
        "outputLane.stopSequence() == authorizationStopSequence", outputLock);
    int processAuthorizationCheck = setpointMethod.indexOf(
        "ProcessOutputSafety.claimIfCurrent", stopSequenceRecheck);
    int vendorSetpoint = setpointMethod.indexOf(
        "sendSetpointTracked(value, controlType)", processAuthorizationCheck);

    assertTrue(successfulBranch >= 0);
    assertTrue(firstOutputLock >= 0);
    assertTrue(authorizationCheck >= 0);
    assertTrue(outputLock >= 0);
    assertTrue(authorizationCheck > firstOutputLockEnd);
    assertTrue(authorizationCheck < outputLock);
    assertTrue(stopSequenceRecheck > outputLock);
    assertTrue(processAuthorizationCheck > stopSequenceRecheck);
    assertTrue(vendorSetpoint > processAuthorizationCheck);
    assertTrue(outputEpochAdvance > successfulBranch && outputEpochAdvance < failureBranch);
    assertTrue(acceptedEvidence > successfulBranch && acceptedEvidence < failureBranch);
    assertTrue(setpointMethod.contains("REV_API_RETURNED_K_OK_NOT_MOTION_PROOF"));
    assertTrue(setpointMethod.contains("PROCESS_OUTPUT_HEARTBEAT_EXPIRED"));
  }

  @Test
  void nonzeroJniRunsOutsideApplicationLocksBehindAnAtomicLaneReservation()
      throws IOException {
    String source = Files.readString(CONTAINER_SOURCE);
    String setpointMethod = between(
        source,
        "private boolean trySetpoint(",
        "public boolean beginFollowerDiagnosticIfAuthorized(");

    int processAuthorizationCheck = setpointMethod.indexOf(
        "ProcessOutputSafety.claimIfCurrent");
    int outputLock = setpointMethod.lastIndexOf(
        "synchronized (OUTPUT_ORDER_LOCK)", processAuthorizationCheck);
    int stateLock = setpointMethod.indexOf("synchronized (stateLock)", outputLock);
    int laneReservation = setpointMethod.indexOf("outputLane.reserveNonzero()", processAuthorizationCheck);
    int vendorSetpoint = setpointMethod.indexOf(
        "sendSetpointTracked(value, controlType)", laneReservation);
    int outputLockEnd = matchingBrace(setpointMethod, setpointMethod.indexOf('{', outputLock));
    int stateLockEnd = matchingBrace(setpointMethod, setpointMethod.indexOf('{', stateLock));

    assertTrue(outputLock >= 0);
    assertTrue(stateLock > outputLock);
    assertTrue(processAuthorizationCheck > stateLock);
    assertTrue(laneReservation > processAuthorizationCheck);
    assertTrue(vendorSetpoint > stateLockEnd);
    assertTrue(vendorSetpoint > outputLockEnd);
    assertTrue(setpointMethod.substring(outputLockEnd, vendorSetpoint)
        .contains("SparkVendorCall.execute"));
    String processSafety = Files.readString(PROCESS_SAFETY_SOURCE);
    String atomicClaim = between(
        processSafety, "public static boolean claimIfCurrent(",
        "/** Returns the current immutable authorization evidence.");
    assertTrue(atomicClaim.contains("STATE.get()"));
    assertFalse(atomicClaim.contains("vendorCall"));
    assertFalse(atomicClaim.contains("synchronized"));
    assertTrue(setpointMethod.contains("reserveRequiredFollowerLanesLocked(now)"));
    assertTrue(setpointMethod.contains("completeRequiredFollowerLanesLocked("));
    assertTrue(setpointMethod.contains("outputLane.completeNonzero(laneReservation)"));
    assertTrue(setpointMethod.contains("outputGate.requireZero(now, true)"));
    assertTrue(setpointMethod.contains(
        "lastRequestReason = \"PROCESS_OUTPUT_HEARTBEAT_EXPIRED\""));
    assertTrue(setpointMethod.contains("outputGate.requireZero(now, false)"));
    assertFalse(setpointMethod.substring(outputLock, outputLockEnd)
        .contains("authorizationStillValid.getAsBoolean()"));
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

  @Test
  void desktopFollowerDiagnosticNeverCallsUnsupportedNativePauseOrResume() throws IOException {
    String source = Files.readString(CONTAINER_SOURCE);
    String beginDiagnostic = between(
        source,
        "public boolean beginFollowerDiagnosticIfAuthorized(",
        "/** Called only while OUTPUT_ORDER_LOCK is held. */");
    String zeroWorker = between(
        source, "private static void runZeroWork(", "private void completeZeroLocked(");

    int simulationPauseBranch = beginDiagnostic.indexOf("if (simulationHandle != null)");
    int nativePause = beginDiagnostic.indexOf("motor::pauseFollowerModeAsync");
    int activeOutputBranch = beginDiagnostic.indexOf("if (applyOutput)");
    int activeTrySetpoint = beginDiagnostic.indexOf("return trySetpoint(", activeOutputBranch);
    int transitionAuthorization = beginDiagnostic.indexOf(
        "if (!authorizationAllows(authorizationStillValid))", activeTrySetpoint);
    int simulationResumeBranch = zeroWorker.indexOf("if (device.simulationHandle != null)");
    int nativeResume = zeroWorker.indexOf("device.motor::resumeFollowerMode");

    assertTrue(simulationPauseBranch >= 0 && simulationPauseBranch < nativePause);
    assertTrue(beginDiagnostic.substring(simulationPauseBranch, nativePause)
        .contains("FollowerDiagnosticMode.PAUSE_PENDING"));
    assertTrue(activeOutputBranch >= 0 && activeTrySetpoint > activeOutputBranch);
    assertTrue(transitionAuthorization > activeTrySetpoint,
        "ACTIVE output must use trySetpoint's sole stop-sequence-bound authorization sample");
    assertTrue(simulationResumeBranch >= 0 && simulationResumeBranch < nativeResume);
    assertTrue(zeroWorker.substring(simulationResumeBranch, nativeResume)
        .contains("resumeResult = REVLibError.kOk"));
  }

  @Test
  void followerPauseJniAndSetpointJniNeverRunUnderSparkApplicationLocks() throws IOException {
    String source = Files.readString(CONTAINER_SOURCE);
    String beginDiagnostic = between(
        source,
        "public boolean beginFollowerDiagnosticIfAuthorized(",
        "/** Called only while OUTPUT_ORDER_LOCK is held. */");
    int outputLock = beginDiagnostic.indexOf("synchronized (OUTPUT_ORDER_LOCK)");
    int outputLockEnd = matchingBrace(beginDiagnostic, beginDiagnostic.indexOf('{', outputLock));
    int nativePause = beginDiagnostic.indexOf("motor::pauseFollowerModeAsync");
    assertTrue(nativePause > outputLockEnd);
    assertTrue(beginDiagnostic.substring(0, nativePause).contains("outputLane.reserveNonzero()"));

    String endDiagnostic = between(
        source,
        "public void endFollowerDiagnostic()",
        "public boolean isFollowerDiagnosticActive()");
    assertTrue(endDiagnostic.contains("FollowerDiagnosticMode.RESUME_ZERO_PENDING"));
    assertTrue(endDiagnostic.contains("outputLane.requestZero()"));
    assertTrue(endDiagnostic.contains("outputGate.requireZero(now, false)"));

    String cleanup = between(
        source,
        "static boolean cleanupSimulationDevicesForTesting()",
        "private Snapshot getControllerTelemetrySnapshot()");
    assertTrue(cleanup.contains("device.outputLane.nonzeroInFlight()"));
    int cleanupOutputLock = cleanup.indexOf("synchronized (OUTPUT_ORDER_LOCK)");
    int cleanupOutputLockEnd = matchingBrace(
        cleanup, cleanup.indexOf('{', cleanupOutputLock));
    assertTrue(cleanup.indexOf("device.closedForTesting = true") < cleanupOutputLockEnd);
    assertTrue(cleanup.indexOf("device.motor.close()") > cleanupOutputLockEnd);
    assertTrue(cleanup.contains("catch (RuntimeException exception)"));

    String serviceAll = between(
        source, "public static void serviceAll()", "private static SampleWork selectSampleWork");
    assertTrue(serviceAll.contains("synchronized (OUTPUT_ORDER_LOCK)"));
    assertTrue(serviceAll.contains("serviceAllLocked()"));
    String sampleReservation = between(
        source, "private SampleWork beginSampleWork(double now)",
        "private static void runConfigurationWork(");
    assertTrue(sampleReservation.contains("synchronized (OUTPUT_ORDER_LOCK)"));
    assertTrue(sampleReservation.contains("closedForTesting"));

    String configurationAdmission = between(
        source,
        "private boolean hasConfigurationWork(double now)",
        "private SampleWork beginSampleWork(double now)");
    assertTrue(configurationAdmission.contains("outputLane.canReserveZero()"));
    assertTrue(configurationAdmission.contains("!outputGate.needsZeroCommand()"));

    String stopSnapshot = between(
        source,
        "public static final class OutputStopBatch",
        "public record OutputStopSnapshot(");
    assertTrue(stopSnapshot.contains("outputLane.zeroCompletedFor("));
    assertTrue(stopSnapshot.contains("lastZeroedLaneGeneration"));
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
