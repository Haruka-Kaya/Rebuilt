package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.constants.ConfiguredOperatorControls;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;

class ConfiguredOperatorControlsTest {
  private static final Path ROBOT_CONTAINER_SOURCE = Path.of(
      "src", "main", "java", "frc", "robot", "RobotContainer.java");
  private static final Path DRIVE_CONTAINER_SOURCE = Path.of(
      "src", "main", "java", "frc", "robot", "containers", "DriveBaseContainer.java");

  @Test
  void productionBindingsMatchTheDocumentedPs5RawButtonMap() {
    assertEquals(
        List.of(1, 4, 5, 6, 7, 8, 9, 12, 14),
        ConfiguredOperatorControls.driverSafetyButtons());
    assertEquals(List.of(1, 2, 3, 4), ConfiguredOperatorControls.climberFaceButtons());
    assertEquals(14, ConfiguredOperatorControls.maximumDriverButton());
    assertEquals(5, ConfiguredOperatorControls.maximumOperatorButton());
    assertEquals(10, ConfiguredOperatorControls.maximumMaintenanceButton());
    assertTrue(ConfiguredOperatorControls.configuredSummary().contains("L1=intake"));
    assertTrue(ConfiguredOperatorControls.configuredSummary().contains("Create+(L1-/R1+)"));
    assertTrue(ConfiguredOperatorControls.configuredSummary().contains("Options+"));
  }

  @Test
  void mechanismAndDriveBindingsUseTheSharedControlManifest() throws IOException {
    String robotContainer = Files.readString(ROBOT_CONTAINER_SOURCE);
    String driveContainer = Files.readString(DRIVE_CONTAINER_SOURCE);

    for (String binding : List.of(
        "DRIVER_INTAKE",
        "DRIVER_OUTPUT",
        "DRIVER_REV",
        "DRIVER_FIRE",
        "DRIVER_RETRACT_FALLBACK",
        "DRIVER_AUTO_AIM_FALLBACK",
        "OPERATOR_RETRACT",
        "MAINTENANCE_AUTO_AIM",
        "UNHOMED_DIAGNOSTIC_DEADMAN",
        "UNHOMED_DIAGNOSTIC_NEGATIVE",
        "UNHOMED_DIAGNOSTIC_POSITIVE",
        "CLIMBER_DEADMAN",
        "CLIMBER_LEFT_POSITIVE",
        "CLIMBER_LEFT_NEGATIVE",
        "CLIMBER_RIGHT_POSITIVE",
        "CLIMBER_RIGHT_NEGATIVE")) {
      assertTrue(
          robotContainer.contains("ConfiguredOperatorControls." + binding),
          () -> "RobotContainer drifted from the control manifest: " + binding);
    }

    for (String binding : List.of(
        "DRIVER_SEED_FIELD", "DRIVER_JUMP_BUMP", "DRIVER_WHEEL_LOCK")) {
      assertTrue(
          driveContainer.contains("ConfiguredOperatorControls." + binding),
          () -> "DriveBaseContainer drifted from the control manifest: " + binding);
    }
  }
}
