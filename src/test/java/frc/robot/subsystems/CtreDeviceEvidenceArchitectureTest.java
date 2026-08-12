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
  }

  private static String between(String source, String start, String end) {
    int startIndex = source.indexOf(start);
    int endIndex = source.indexOf(end, startIndex + start.length());
    assertTrue(startIndex >= 0 && endIndex > startIndex);
    return source.substring(startIndex, endIndex);
  }
}
