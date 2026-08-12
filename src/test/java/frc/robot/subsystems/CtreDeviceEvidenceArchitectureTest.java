package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

class CtreDeviceEvidenceArchitectureTest {
  private static final Path SOURCE = Path.of(
      "src", "main", "java", "frc", "robot", "subsystems",
      "CommandSwerveDrivetrain.java");

  @Test
  void deviceEvidenceUsesNonblockingValidatedSignalsAndNotAggregateCopies() throws IOException {
    String source = Files.readString(SOURCE);
    String healthSummary = between(
        source, "public String getDeviceHealthSummary()", "public boolean isGyroConnected()");
    String motionSummary = between(
        source, "public String getMotionDiagnosticSummary()", "private static double diagnosticValue");
    String stopEvidence = between(
        source, "public SwerveStopEvidence getStopEvidence(", "public Command safeIdleCommand()");

    assertTrue(source.contains(".refresh(false)"));
    assertFalse(source.contains("refreshAll("));
    assertFalse(source.contains("waitForAll("));
    assertFalse(healthSummary.contains("areAllModulesConnected()"));
    assertFalse(healthSummary.contains("areAllDevicesConnected()"));
    assertFalse(motionSummary.contains("getStatorCurrent("));
    assertTrue(source.contains("drive.getStatorCurrent(false)"));
    assertTrue(source.contains("steer.getStatorCurrent(false)"));
    assertTrue(source.contains("getAllTimestamps().getSystemTimestamp()"));
    assertTrue(source.contains("CtreSignalProgressTracker"));
    assertTrue(source.contains("synchronized boolean observeIfDue(double now)"));
    assertTrue(source.contains("synchronized Snapshot captureDiagnosticNow(double now)"));
    assertTrue(source.contains("synchronized Snapshot captureOutputNow(double now)"));
    assertTrue(source.contains("synchronized Snapshot snapshot()"));
    assertTrue(source.contains("public boolean areAllRequiredDeviceSignalsFresh()"));
    assertTrue(stopEvidence.contains("captureOutputNow"));
    assertTrue(stopEvidence.contains("postNeutralOutputReason"));
  }

  @Test
  void hardwareSelfTestUsesPostCommandTokensForSparkAndSwerve() throws IOException {
    String source = Files.readString(Path.of(
        "src", "main", "java", "frc", "robot", "commands",
        "HardwareSelfTestCommand.java"));
    String sparkStage = between(source, "private static Command openLoopStage(",
        "private static Command swerveStage(");
    String swerveStage = between(source, "private static Command swerveStage(",
        "private static Command guardedStage(");

    assertTrue(sparkStage.contains("if (!attempted[0])"));
    assertTrue(sparkStage.contains("captureTimedTargetStates"));
    assertTrue(sparkStage.contains("evaluateFreshTimedTargets"));
    assertTrue(sparkStage.contains("evaluateCurrentTimedTargets"));
    assertTrue(sparkStage.contains("timedTargetsStillReady"));
    assertFalse(sparkStage.contains("evaluateTargets(targets, true)"));
    assertTrue(swerveStage.contains("captureSwerveDiagnosticBaseline"));
    assertTrue(swerveStage.contains("completeSwerveDiagnosticRequest"));
    assertTrue(swerveStage.contains("postCommandEvidenceReady"));
    assertTrue(swerveStage.contains("areAllRequiredDeviceSignalsFresh"));
    assertTrue(swerveStage.contains("WAITING_FOR_POST_COMMAND_SIGNALS"));
    assertTrue(swerveStage.contains("action.apply(pulsePermit[0])"));
    assertTrue(source.contains("outputSession.trip(\"HST_STOP_UNCONFIRMED_\" + name)"));
    assertTrue(source.contains("boolean sessionInvalid = !outputSession.isValid()"));
    assertTrue(source.contains("? \"DIAGNOSTIC_OUTPUT_SESSION_INVALID\""));
  }

  @Test
  void swerveOutputLaneReservesUnderLockButRunsPhoenixOutsideLocks() throws IOException {
    String source = Files.readString(SOURCE);
    String reserveNeutral = between(
        source, "private SwerveStopToken reserveNeutralBarrier()", "private void scheduleNeutralWorker()");
    String scheduleNeutral = between(
        source, "private void scheduleNeutralWorker()", "private void runNeutralWorker()");
    String beginNeutral = between(
        source, "private NeutralLaneAttempt beginNeutralLaneAttempt(",
        "private NeutralCommandOutcome issueNeutralForBarrier(");
    String neutralVendorCall = between(
        source, "private NeutralCommandOutcome issueNeutralForBarrier(",
        "private void completeNeutralLaneAttempt(");
    String applyNonNeutral = between(
        source, "private ControlResult applyNonNeutralRequest(", "public enum ControlResult");
    String orderedApply = between(
        source, "private final class OrderedNonzeroRequest", "private ControlResult applyNonNeutralRequest(");
    String beginNonzeroApply = between(
        source, "private NonzeroApplyAttempt beginNonzeroApply(",
        "private static boolean authorizationAllows(");
    String close = between(
        source, "public void close()", "private void scheduleNativeCloseCheck()");

    assertTrue(source.contains("private final Object m_outputApplicationLock = new Object()"));
    assertOrdered(
        reserveNeutral,
        "synchronized (m_outputApplicationLock)",
        "synchronized (m_outputEvidenceLock)",
        "m_outputLaneBarrier.reserveBarrier(outputEpoch)");
    assertFalse(reserveNeutral.contains("this.setControl("));
    assertFalse(reserveNeutral.contains("issueDirectNeutralNoThrow()"));
    assertTrue(reserveNeutral.contains("outputLaneMonotonicSeconds()"));
    assertFalse(reserveNeutral.contains("safePhoenixTimeSeconds()"));
    assertTrue(scheduleNeutral.contains("outputLaneMonotonicSeconds()"));
    assertFalse(scheduleNeutral.contains("safePhoenixTimeSeconds()"));
    assertTrue(beginNeutral.contains("outputLaneMonotonicSeconds()"));
    assertFalse(beginNeutral.contains("safePhoenixTimeSeconds()"));
    assertOrdered(
        neutralVendorCall,
        "this.setControl(m_safeNeutralRequest)",
        "issueDirectNeutralNoThrow()");
    int permitAcquisition = applyNonNeutral.indexOf("ProcessOutputSafety.acquireNonzeroPermit()");
    int sequenceCaptureLock = applyNonNeutral.indexOf("synchronized (m_outputApplicationLock)");
    int sequenceCapture = applyNonNeutral.indexOf(
        "stopSequenceSnapshot = m_stopRequestSequence", sequenceCaptureLock);
    int liveRegistrationAuthorization = applyNonNeutral.indexOf(
        "authorizationAllows(additionalAuthorization)", sequenceCapture);
    int finalRegistrationLock = applyNonNeutral.indexOf(
        "synchronized (m_outputApplicationLock)", liveRegistrationAuthorization);
    int finalRegistrationClaim = applyNonNeutral.indexOf(
        "ProcessOutputSafety.claimIfCurrent(processPermit)", finalRegistrationLock);
    int evidenceLock = applyNonNeutral.indexOf(
        "synchronized (m_outputEvidenceLock)", finalRegistrationClaim);
    int outputEpochAdvance = applyNonNeutral.indexOf(
        "outputEpoch = ++m_outputEpoch", evidenceLock);
    int registrationReservation = applyNonNeutral.indexOf(
        "OutputLaneReservation.reserveNonzero(", outputEpochAdvance);
    int registrationVendorCall = applyNonNeutral.indexOf(
        "this.setControl(new OrderedNonzeroRequest(request, registration))", registrationReservation);
    assertTrue(permitAcquisition >= 0);
    assertTrue(sequenceCaptureLock > permitAcquisition);
    assertTrue(sequenceCapture > sequenceCaptureLock);
    assertTrue(liveRegistrationAuthorization > sequenceCapture);
    assertTrue(finalRegistrationLock > liveRegistrationAuthorization);
    assertTrue(finalRegistrationClaim > finalRegistrationLock);
    assertTrue(evidenceLock > finalRegistrationClaim);
    assertTrue(outputEpochAdvance > evidenceLock);
    assertTrue(registrationReservation > outputEpochAdvance);
    assertTrue(registrationVendorCall > registrationReservation);
    assertFalse(source.contains("ProcessOutputSafety.callIfAuthorized"));
    assertTrue(applyNonNeutral.contains("ControlResult.OUTPUT_AUTHORIZATION_REVOKED"));
    assertTrue(applyNonNeutral.contains("authorizationAllows(additionalAuthorization)"));
    assertTrue(source.contains("public ControlResult driveDiagnostic("));
    assertFalse(source.contains("DiagnosticLeaseAwareRequest"));
    assertOrdered(
        orderedApply,
        "OutputLaneApplyExecutor.apply(",
        "beginNonzeroApply(registration)",
        "delegate.apply(parameters, modulesToApply)",
        "CommandSwerveDrivetrain.this::completeNonzeroLaneAction",
        "CommandSwerveDrivetrain.this::reserveNeutralBarrier");
    assertOrdered(
        beginNonzeroApply,
        "ProcessOutputSafety.acquireNonzeroPermit()",
        "stopSequenceSnapshot = m_stopRequestSequence",
        "authorizationAllows(registration.additionalAuthorization())");
    assertTrue(beginNonzeroApply.contains("OutputLaneReservation.reserveNonzero("));
    int liveAuthorization = beginNonzeroApply.indexOf(
        "authorizationAllows(registration.additionalAuthorization())");
    int finalApplicationLock = beginNonzeroApply.indexOf(
        "synchronized (m_outputApplicationLock)", liveAuthorization);
    int finalProcessClaim = beginNonzeroApply.indexOf(
        "ProcessOutputSafety.claimIfCurrent(applyPermit)", finalApplicationLock);
    assertTrue(finalApplicationLock > liveAuthorization,
        "live pulse authorization must be evaluated outside the application lock");
    assertTrue(finalProcessClaim > finalApplicationLock,
        "the one-shot process permit must be claimed at the final lane boundary");
    assertOrdered(
        close,
        "m_outputLaneLifecycle.requestClose()",
        "reserveNeutralBarrier()",
        "simNotifier.close()");
    assertTrue(source.contains("&& m_simNotifierDrained"));
    assertOrdered(
        source,
        "m_outputLaneLifecycle.tryBeginNativeClose(",
        "super.close()");
    int applicationLock = applyNonNeutral.indexOf("synchronized (m_outputApplicationLock)");
    int applicationOpenBrace = applyNonNeutral.indexOf('{', applicationLock);
    int applicationCloseBrace = matchingBrace(applyNonNeutral, applicationOpenBrace);
    String applicationBody =
        applyNonNeutral.substring(applicationOpenBrace, applicationCloseBrace);
    assertFalse(applicationBody.contains("this.setControl("));
    assertTrue(applyNonNeutral.indexOf("this.setControl(", applicationCloseBrace)
        > applicationCloseBrace);

    assertEvidenceLocksNeverAcquireOutputApplication(source);
    assertApplicationLocksNeverRunPhoenix(source);
  }

  private static String between(String source, String start, String end) {
    int startIndex = source.indexOf(start);
    int endIndex = source.indexOf(end, startIndex + start.length());
    assertTrue(startIndex >= 0 && endIndex > startIndex);
    return source.substring(startIndex, endIndex);
  }

  private static void assertOrdered(String source, String... snippets) {
    int previousIndex = -1;
    for (String snippet : snippets) {
      int index = source.indexOf(snippet);
      assertTrue(index > previousIndex, () -> "Expected ordered snippet: " + snippet);
      previousIndex = index;
    }
  }

  private static void assertEvidenceLocksNeverAcquireOutputApplication(String source) {
    String evidenceLock = "synchronized (m_outputEvidenceLock)";
    int searchFrom = 0;
    while (true) {
      int lockIndex = source.indexOf(evidenceLock, searchFrom);
      if (lockIndex < 0) {
        return;
      }
      int openBrace = source.indexOf('{', lockIndex + evidenceLock.length());
      assertTrue(openBrace >= 0);
      int closeBrace = matchingBrace(source, openBrace);
      String lockBody = source.substring(openBrace + 1, closeBrace);
      assertFalse(lockBody.contains("m_outputApplicationLock"));
      assertFalse(lockBody.contains("requestIdle("));
      assertFalse(lockBody.contains("requestIdleChecked("));
      assertFalse(lockBody.contains("emergencyNeutralNoThrow("));
      assertFalse(lockBody.contains("applyNonNeutralRequest("));
      assertFalse(lockBody.contains("issueDirectNeutralNoThrow("));
      searchFrom = closeBrace + 1;
    }
  }

  private static void assertApplicationLocksNeverRunPhoenix(String source) {
    String applicationLock = "synchronized (m_outputApplicationLock)";
    int searchFrom = 0;
    while (true) {
      int lockIndex = source.indexOf(applicationLock, searchFrom);
      if (lockIndex < 0) {
        return;
      }
      int openBrace = source.indexOf('{', lockIndex + applicationLock.length());
      assertTrue(openBrace >= 0);
      int closeBrace = matchingBrace(source, openBrace);
      String lockBody = source.substring(openBrace + 1, closeBrace);
      assertFalse(lockBody.contains("setControl("));
      assertFalse(lockBody.contains("module.apply("));
      assertFalse(lockBody.contains("delegate.apply("));
      assertFalse(lockBody.contains("issueDirectNeutralNoThrow("));
      assertFalse(lockBody.contains("captureMotorOutputProgressBaselinesNoThrow("));
      assertFalse(lockBody.contains("getStateCopy("));
      assertFalse(lockBody.contains("safePhoenixTimeSeconds("));
      assertFalse(lockBody.contains("getAsBoolean("));
      assertFalse(lockBody.contains("authorizationAllows("));
      assertFalse(lockBody.contains("super.close("));
      searchFrom = closeBrace + 1;
    }
  }

  private static int matchingBrace(String source, int openBrace) {
    int depth = 0;
    for (int index = openBrace; index < source.length(); index++) {
      char character = source.charAt(index);
      if (character == '{') {
        depth++;
      } else if (character == '}' && --depth == 0) {
        return index;
      }
    }
    throw new AssertionError("Unmatched source brace");
  }
}
