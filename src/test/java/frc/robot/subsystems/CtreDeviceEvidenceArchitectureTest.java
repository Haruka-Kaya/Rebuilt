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
  void swerveOutputApplicationSerializesAuthorizationAndNeutralRequests() throws IOException {
    String source = Files.readString(SOURCE);
    String requestIdleChecked = between(
        source, "private void requestIdleChecked()", "private boolean issueDirectNeutralNoThrow()");
    String emergencyNeutral = between(
        source, "private void emergencyNeutralNoThrow()", "private static double safePhoenixTimeSeconds()");
    String applyNonNeutral = between(
        source, "private ControlResult applyNonNeutralRequest(", "public enum ControlResult");

    assertTrue(source.contains("private final Object m_outputApplicationLock = new Object()"));
    assertOrdered(
        requestIdleChecked,
        "synchronized (m_outputApplicationLock)",
        "synchronized (m_outputEvidenceLock)",
        "this.setControl(m_safeNeutralRequest)",
        "issueDirectNeutralNoThrow()");
    assertOrdered(
        emergencyNeutral,
        "synchronized (m_outputApplicationLock)",
        "synchronized (m_outputEvidenceLock)",
        "this.setControl(m_safeNeutralRequest)",
        "issueDirectNeutralNoThrow()");
    assertOrdered(
        applyNonNeutral,
        "synchronized (m_outputApplicationLock)",
        "ProcessOutputSafety.callIfAuthorized",
        "synchronized (m_outputEvidenceLock)",
        "m_outputEpoch++",
        "this.setControl(request)");
    assertTrue(applyNonNeutral.contains("ControlResult.OUTPUT_AUTHORIZATION_REVOKED"));
    assertTrue(applyNonNeutral.contains("additionalAuthorization.getAsBoolean()"));
    assertTrue(source.contains("public ControlResult driveDiagnostic("));
    assertTrue(source.contains("new DiagnosticLeaseAwareRequest("));
    assertTrue(source.contains("authorization.getAsBoolean()"));
    int applicationLock = applyNonNeutral.indexOf("synchronized (m_outputApplicationLock)");
    int applicationOpenBrace = applyNonNeutral.indexOf('{', applicationLock);
    int applicationCloseBrace = matchingBrace(applyNonNeutral, applicationOpenBrace);
    assertFalse(applyNonNeutral.substring(applicationOpenBrace, applicationCloseBrace)
        .contains("requestIdle()"));
    assertTrue(applyNonNeutral.indexOf("requestIdle()", applicationCloseBrace)
        > applicationCloseBrace);

    assertEvidenceLocksNeverAcquireOutputApplication(source);
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
