package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

class DeviceEvidenceLifecycleArchitectureTest {
  private static final Path ROBOT_SOURCE = Path.of(
      "src", "main", "java", "frc", "robot", "Robot.java");
  private static final Path ROBOT_CONTAINER_SOURCE = Path.of(
      "src", "main", "java", "frc", "robot", "RobotContainer.java");
  private static final Path DRIVE_CONTAINER_SOURCE = Path.of(
      "src", "main", "java", "frc", "robot", "containers", "DriveBaseContainer.java");
  private static final Path AUTO_CONTAINER_SOURCE = Path.of(
      "src", "main", "java", "frc", "robot", "containers", "AutoContainer.java");

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

  @Test
  void idleArmCleanupCannotContinuouslyStopTheFeederOrBreakFutureControlledHst()
      throws IOException {
    String source = Files.readString(ROBOT_CONTAINER_SOURCE);
    String discard = between(
        source,
        "public void discardPreparedUnhomedDiagnosticSession()",
        "public Command getAutonomousCommand()");
    int tokenGuard = discard.indexOf(
        "m_unhomedDiagnosticTarget == null && m_feederManualRetestToken != null");
    int feederDisarm = discard.indexOf("m_feeder.disarmManualControlledRetest()");
    int guardedBlockEnd = discard.indexOf("}", feederDisarm);

    assertTrue(tokenGuard >= 0, "prepared Feeder cleanup must require an actual token");
    assertTrue(feederDisarm > tokenGuard && guardedBlockEnd > feederDisarm);
  }

  @Test
  void processHeartbeatGuardsSchedulerAndRuntimeFaultsRevokeBeforeStopping()
      throws IOException {
    String robot = Files.readString(ROBOT_SOURCE);
    String container = Files.readString(ROBOT_CONTAINER_SOURCE);
    String periodic = between(robot, "public void robotPeriodic()", "public void disabledInit()");
    String latch = between(robot, "private void latchRuntimeFault(", "private void enforceLatchedStop()");
    String enforce = between(robot, "private void enforceLatchedStop()", "public void simulationPeriodic()");
    String globalStop = between(
        container,
        "private StopSession beginIndependentOutputStopSession()",
        "/** Releases process-owned simulation/native resources");

    int heartbeat = periodic.indexOf("serviceOutputSafetyHeartbeat()");
    int schedulerStart = periodic.indexOf("beginCommandSchedulerRun()");
    int schedulerRun = periodic.indexOf("CommandScheduler.getInstance().run()");
    int schedulerComplete = periodic.indexOf("completeCommandSchedulerRun(schedulerRunEpoch)");
    assertTrue(
        heartbeat >= 0
            && heartbeat < schedulerStart
            && schedulerStart < schedulerRun
            && schedulerRun < schedulerComplete,
        "only a normally returned scheduler run may support the next heartbeat");
    assertTrue(
        latch.indexOf("tripOutputSafety") < latch.indexOf("enforceLatchedStop()"));
    assertTrue(
        enforce.indexOf("tripOutputSafety") < enforce.indexOf("stopAll()"));
    assertTrue(globalStop.contains("SparkMAXContainer.requestOutputStops(sparkIds)"));
    assertTrue(globalStop.contains("drivetrain.requestIdleWithToken()"));
    assertTrue(globalStop.contains("SparkMAXContainer.serviceAll()"));
    assertTrue(globalStop.contains("drivetrain.getStopEvidence("));
    assertTrue(robot.contains("m_robotContainer.isOutputSafetyReadyForEnable()"));
  }

  @Test
  void disabledHstArmIsDeferredUntilTheFirstAuthorizedTestHeartbeat() throws IOException {
    String robot = Files.readString(ROBOT_SOURCE);
    String testInit = between(robot, "public void testInit()", "public void testPeriodic()");
    String periodic = between(robot, "public void robotPeriodic()", "public void disabledInit()");
    String deferredStart = between(
        robot,
        "private void startPendingHardwareSelfTestIfAuthorized()",
        "private void clearUnhomedDiagnosticArm()");
    String hstSessionFactory = between(
        Files.readString(ROBOT_CONTAINER_SOURCE),
        "private DiagnosticOutputSession createHardwareSelfTestOutputSession(",
        "private Command rejectedHardwareSelfTestCommand(");

    assertTrue(testInit.contains("m_pendingHardwareSelfTestExpiresAt = selfTestExpiresAt"));
    assertFalse(testInit.contains("getHardwareSelfTestCommand(selfTestExpiresAt)"));
    assertTrue(periodic.contains("serviceOutputSafetyHeartbeat()"));
    assertTrue(periodic.contains("startPendingHardwareSelfTestIfAuthorized()"));
    assertTrue(periodic.contains("beginCommandSchedulerRun()"));
    assertTrue(
        periodic.indexOf("serviceOutputSafetyHeartbeat()")
            < periodic.indexOf("startPendingHardwareSelfTestIfAuthorized()"));
    assertTrue(
        periodic.indexOf("startPendingHardwareSelfTestIfAuthorized()")
            < periodic.indexOf("beginCommandSchedulerRun()"));
    assertTrue(deferredStart.contains("Phase.ARMED"));
    assertTrue(deferredStart.contains("processOutputSafety().outputAuthorized()"));
    assertTrue(hstSessionFactory.contains("armExpiresAtSeconds <= now + HardwareTestConstants.ARM_LIFETIME_SECONDS"));
    assertTrue(hstSessionFactory.contains("HARDWARE_SELF_TEST_SESSION_LIFETIME_SECONDS"));
    assertTrue(deferredStart.contains(
        "m_pendingHardwareSelfTestExpiresAt = Double.NEGATIVE_INFINITY"));
    assertTrue(deferredStart.contains(
        "CommandScheduler.getInstance().schedule(m_hardwareSelfTest)"));
    assertTrue(
        deferredStart.indexOf("m_pendingHardwareSelfTestExpiresAt = Double.NEGATIVE_INFINITY")
            < deferredStart.indexOf("CommandScheduler.getInstance().schedule(m_hardwareSelfTest)"));
  }

  @Test
  void activeAutonomousRequiresLiveOutputAuthorizationBeforeDependencyReadiness()
      throws IOException {
    String robot = Files.readString(ROBOT_SOURCE);
    String container = Files.readString(ROBOT_CONTAINER_SOURCE);
    String driveContainer = Files.readString(DRIVE_CONTAINER_SOURCE);
    String autoContainer = Files.readString(AUTO_CONTAINER_SOURCE);

    String autonomousPeriodic = between(
        robot, "public void autonomousPeriodic()", "private void abortActiveAutonomousIfUnsafe()");
    assertFalse(autonomousPeriodic.contains("getOutputSafetySnapshot()"));

    String robotPeriodic = between(
        robot, "public void robotPeriodic()", "public void disabledInit()");
    int heartbeat = robotPeriodic.indexOf("serviceOutputSafetyHeartbeat()");
    int abort = robotPeriodic.indexOf("abortActiveAutonomousIfUnsafe()");
    int schedulerStart = robotPeriodic.indexOf("beginCommandSchedulerRun()");
    int scheduler = robotPeriodic.indexOf("CommandScheduler.getInstance().run()");
    int schedulerComplete =
        robotPeriodic.indexOf("completeCommandSchedulerRun(schedulerRunEpoch)");
    assertTrue(
        heartbeat >= 0
            && heartbeat < abort
            && abort < schedulerStart
            && schedulerStart < scheduler
            && scheduler < schedulerComplete,
        "active auto must see the prior completion-backed grant before this scheduler pass");

    String activeAutoGate = between(
        robot, "private void abortActiveAutonomousIfUnsafe()", "public void autonomousExit()");
    int snapshot = activeAutoGate.indexOf("getOutputSafetySnapshot()");
    int policy = activeAutoGate.indexOf(
        "shouldAbortActiveAutonomous(outputSafetySnapshot)");
    int cancel = activeAutoGate.indexOf("m_autonomousCommand.cancel()");
    int stop = activeAutoGate.indexOf("m_robotContainer.stopAll()");
    assertTrue(snapshot >= 0 && snapshot < policy && policy < cancel && cancel < stop);

    String containerForwarder = between(
        container,
        "public boolean shouldAbortActiveAutonomous(",
        "public void refreshAutonomousStatus()");
    assertTrue(containerForwarder.contains(
        "m_DriveBaseContainer.shouldAbortActiveAutonomous(outputSafetySnapshot)"));
    assertTrue(driveContainer.contains(
        "autoContainer.shouldAbortActiveAutonomous(outputSafetySnapshot)"));

    String runtimeGate = between(
        autoContainer,
        "public boolean shouldAbortActiveAutonomous(",
        "/** Keeps preflight state visible");
    assertTrue(
        runtimeGate.indexOf("evaluateActiveOutputSafety(outputSafetySnapshot)")
            < runtimeGate.indexOf("currentReadiness()"));
    assertTrue(runtimeGate.contains("abortActiveAutonomous(outputSafety.reason())"));

    String managedAuto = between(
        autoContainer, "return freshSelectedAuto", ".withName(\"Managed Auto:");
    int finishClassification = managedAuto.indexOf("runState.finished(interrupted)");
    int abortedGuard = managedAuto.indexOf("AutonomousRunState.Phase.ABORTED");
    int completedPublication = managedAuto.indexOf("COMPLETED: ");
    assertTrue(
        finishClassification >= 0
            && finishClassification < abortedGuard
            && abortedGuard < completedPublication,
        "finallyDo must preserve ABORTED before considering COMPLETED");
  }

  private static String between(String source, String start, String end) {
    int startIndex = source.indexOf(start);
    int endIndex = source.indexOf(end, startIndex + start.length());
    assertTrue(startIndex >= 0 && endIndex > startIndex);
    return source.substring(startIndex, endIndex);
  }
}
