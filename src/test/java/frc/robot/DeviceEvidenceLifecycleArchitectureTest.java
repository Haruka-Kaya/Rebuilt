package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

class DeviceEvidenceLifecycleArchitectureTest {
  private static final Path ROBOT_SOURCE = Path.of(
      "src", "main", "java", "frc", "robot", "Robot.java");

  @Test
  void fmsSuppressionRunsBeforeTheIrreversibleRuntimeFaultEarlyReturn() throws IOException {
    String source = Files.readString(ROBOT_SOURCE);
    String periodic = between(source, "public void robotPeriodic()", "public void disabledInit()");

    assertTrue(
        periodic.indexOf("suppressHardwareEvidenceForFmsNoThrow()")
            < periodic.indexOf("if (!m_runtimeSafetyLatch.healthy())"));
    assertTrue(periodic.contains("m_deviceEvidenceSuppressedForFms = false"));
    assertTrue(
        periodic.indexOf("m_deviceEvidenceSuppressedForFms = false")
            < periodic.indexOf("SparkDeviceEvidence.publish(sparkEvidence)"));
    assertTrue(periodic.contains("m_nextDeviceEvidenceTimestamp = now + 0.5"));
    assertTrue(periodic.contains("SparkDeviceEvidence::publishSuppressedForFms"));
    assertTrue(periodic.contains("CtreDeviceEvidence::publishSuppressedForFms"));
    assertTrue(periodic.contains("CanDeviceEvidenceSummary::publishSuppressedForFms"));
    assertTrue(
        periodic.indexOf("publishRuntimeFaultEvidenceUnavailableNoThrow()")
            < periodic.indexOf("enforceLatchedStop()"));
    assertTrue(periodic.contains("SparkDeviceEvidence.publishUnavailable(\"RUNTIME_FAULT\")"));
    assertTrue(periodic.contains("CtreDeviceEvidence.publishUnavailable(\"RUNTIME_FAULT\")"));
    assertTrue(periodic.contains("Hardware/SPARK Health\", \"RUNTIME_FAULT"));
    assertTrue(periodic.contains("Hardware/CTRE Health\", \"RUNTIME_FAULT"));
    assertTrue(periodic.contains("m_deviceEvidenceUnavailableForRuntimeFault = false"));
    assertTrue(periodic.contains("m_deviceEvidenceSuppressedForFms = false"));
  }

  private static String between(String source, String start, String end) {
    int startIndex = source.indexOf(start);
    int endIndex = source.indexOf(end, startIndex + start.length());
    assertTrue(startIndex >= 0 && endIndex > startIndex);
    return source.substring(startIndex, endIndex);
  }
}
