package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.constants.ConfiguredCanHardware;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashSet;
import java.util.List;
import org.junit.jupiter.api.Test;

class ConfiguredCanHardwareTest {
  private static final Path TUNER_CONSTANTS_SOURCE = Path.of(
      "src", "main", "java", "frc", "robot", "constants", "TunerConstants.java");
  private static final Path ROBOT_CONSTANTS_SOURCE = Path.of(
      "src", "main", "java", "frc", "robot", "constants", "Constants.java");

  @Test
  void configuredCanMapIsCompleteUniqueAndWithinTheLegalDeviceRange() {
    List<Integer> expectedSparkIds = List.of(30, 31, 32, 33, 34, 35, 36, 37, 38, 39);
    List<Integer> expectedCtreIds = List.of(
        20, 40, 41, 42, 43, 50, 51, 52, 53, 54, 55, 56, 57);

    assertEquals(expectedSparkIds, ConfiguredCanHardware.sparkDeviceIds());
    assertEquals(expectedCtreIds, ConfiguredCanHardware.ctreDeviceIds());
    assertEquals("CTRE20,40-43,50-57", ConfiguredCanHardware.ctreCoverageLabel());
    assertEquals(
        "SPARK30-39; CTRE20,40-43,50-57; PHYSICAL_INVENTORY_UNVERIFIED",
        ConfiguredCanHardware.configuredSummary());
    assertEquals(
        ConfiguredCanHardware.allDeviceIds().size(),
        new HashSet<>(ConfiguredCanHardware.allDeviceIds()).size(),
        "Every configured device on the shared CAN bus must have a unique ID");
    assertTrue(
        ConfiguredCanHardware.allDeviceIds().stream().allMatch(id -> id >= 0 && id <= 62),
        "CAN IDs must stay within the legal 0..62 range");
  }

  @Test
  void generatedAndMechanismConstantsKeepUsingTheSharedCanManifest() throws IOException {
    String tunerConstants = Files.readString(TUNER_CONSTANTS_SOURCE);
    String robotConstants = Files.readString(ROBOT_CONSTANTS_SOURCE);

    for (String configuredName : List.of(
        "PIGEON_ID",
        "FRONT_LEFT_DRIVE_ID",
        "FRONT_LEFT_STEER_ID",
        "FRONT_LEFT_ENCODER_ID",
        "FRONT_RIGHT_DRIVE_ID",
        "FRONT_RIGHT_STEER_ID",
        "FRONT_RIGHT_ENCODER_ID",
        "BACK_LEFT_DRIVE_ID",
        "BACK_LEFT_STEER_ID",
        "BACK_LEFT_ENCODER_ID",
        "BACK_RIGHT_DRIVE_ID",
        "BACK_RIGHT_STEER_ID",
        "BACK_RIGHT_ENCODER_ID")) {
      assertTrue(
          tunerConstants.contains("ConfiguredCanHardware." + configuredName),
          () -> "TunerConstants drifted from the shared CAN manifest: " + configuredName);
    }

    for (String configuredName : List.of(
        "INTAKE_ACTUATOR_ID",
        "INTAKE_ROLLER_ID",
        "FEEDER_ID",
        "CONVEYOR_ID",
        "CLIMBER_LEFT_ID",
        "CLIMBER_RIGHT_ID",
        "SHOOTER_LEADER_ID",
        "SHOOTER_FOLLOWER_ID",
        "SHOOTER_ACTUATOR_ID",
        "TURRET_ID")) {
      assertTrue(
          robotConstants.contains("ConfiguredCanHardware." + configuredName),
          () -> "Constants drifted from the shared CAN manifest: " + configuredName);
    }
  }
}
