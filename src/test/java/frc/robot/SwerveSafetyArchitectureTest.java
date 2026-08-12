package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import org.junit.jupiter.api.Test;

class SwerveSafetyArchitectureTest {
  private static final Path DRIVETRAIN_SOURCE = Path.of(
      "src", "main", "java", "frc", "robot", "subsystems", "CommandSwerveDrivetrain.java");
  private static final Path DRIVE_CONTAINER_SOURCE = Path.of(
      "src", "main", "java", "frc", "robot", "containers", "DriveBaseContainer.java");
  private static final Path ALIGNMENT_COMMAND_SOURCE = Path.of(
      "src", "main", "java", "frc", "robot", "commands", "AlignmentCommand.java");

  @Test
  void failClosedPathsNeverUseCtreIdleBecauseItLeavesThePreviousRequestLatched()
      throws IOException {
    String drivetrain = Files.readString(DRIVETRAIN_SOURCE);
    String container = Files.readString(DRIVE_CONTAINER_SOURCE);

    assertFalse(drivetrain.contains("new SwerveRequest.Idle"));
    assertFalse(container.contains("new SwerveRequest.Idle"));
    assertTrue(drivetrain.contains("module.apply(driveNeutral, steerNeutral)"));
    assertTrue(drivetrain.contains("appliedGeneration.get() == generation"));
    assertTrue(drivetrain.contains("kNeutralRetryPeriodSeconds"));
    assertTrue(container.contains("drivetrain.safeIdleCommand()"));
  }

  @Test
  void everyReusableDriveCommandRequestsRealNeutralWhenItEnds() throws IOException {
    String drivetrain = Files.readString(DRIVETRAIN_SOURCE);
    String alignment = Files.readString(ALIGNMENT_COMMAND_SOURCE);

    assertTrue(drivetrain.contains("}).finallyDo(interrupted -> requestIdle());"));
    assertTrue(alignment.contains("drive.requestIdle();"));
    assertFalse(alignment.contains("drive.drive(0, 0, 0"));
  }

  @Test
  void modeStopsAndModuleFaultsInvalidatePreviousDriverNeutralEvidence() throws IOException {
    String container = Files.readString(DRIVE_CONTAINER_SOURCE);
    String robotContainer = Files.readString(Path.of(
        "src", "main", "java", "frc", "robot", "RobotContainer.java"));

    assertTrue(container.contains("public void blockDriverInputsUntilNeutral()"));
    assertTrue(container.contains("case MODULES_UNHEALTHY -> {"));
    assertTrue(container.contains("blockDriverInputsUntilNeutral();"));
    int seedRequest = container.indexOf("drivetrain.seedFieldCentric();");
    int seedNeutralRearm = container.indexOf("blockDriverInputsUntilNeutral();", seedRequest);
    assertTrue(seedRequest >= 0, "field-heading seed request must remain explicit");
    assertTrue(
        seedNeutralRearm > seedRequest && seedNeutralRearm - seedRequest < 500,
        "a successful field-heading seed must invalidate held-stick neutral evidence");
    assertTrue(robotContainer.contains("m_DriveBaseContainer.blockDriverInputsUntilNeutral();"));
  }

  @Test
  void criticalSignalHealthUsesNonblockingSilentRefreshes() throws IOException {
    String drivetrain = Files.readString(DRIVETRAIN_SOURCE);

    assertTrue(drivetrain.contains("drive.getPosition(false)"));
    assertTrue(drivetrain.contains("drive.getVelocity(false)"));
    assertTrue(drivetrain.contains("steer.getPosition(false)"));
    assertTrue(drivetrain.contains("steer.getVelocity(false)"));
    assertTrue(drivetrain.contains("encoder.getPosition(false)"));
    assertTrue(drivetrain.contains("encoder.getAbsolutePosition(false)"));
    assertTrue(drivetrain.contains("getPigeon2().getYaw(false)"));
    assertTrue(drivetrain.contains("getPigeon2().getAngularVelocityZWorld(false)"));
    assertTrue(drivetrain.contains("refresh(false)"));
    assertFalse(drivetrain.contains("refreshAll("));
    assertFalse(drivetrain.contains("waitForAll("));
    assertFalse(drivetrain.contains("setUpdateFrequency("));
    assertTrue(drivetrain.contains("getStateCopy()"));
    assertFalse(drivetrain.contains("getState()"));
  }
}
