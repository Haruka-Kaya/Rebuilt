package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.constants.ConfiguredCanHardware;
import frc.robot.constants.ConfiguredCanHardware.DeviceType;
import frc.robot.constants.ConfiguredCanHardware.HardwareRole;
import frc.robot.constants.ConfiguredCanHardware.Vendor;
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
  void everyConfiguredIdHasOneTypedOperatorFacingInventoryRecord() {
    assertEquals(23, ConfiguredCanHardware.devices().size());
    assertEquals(
        ConfiguredCanHardware.allDeviceIds(),
        ConfiguredCanHardware.devices().stream()
            .map(ConfiguredCanHardware.Device::canId)
            .toList());
    assertEquals(
        ConfiguredCanHardware.sparkDeviceIds(),
        ConfiguredCanHardware.devices().stream()
            .filter(device -> device.vendor() == Vendor.REV)
            .map(ConfiguredCanHardware.Device::canId)
            .toList());
    assertEquals(
        ConfiguredCanHardware.ctreDeviceIds(),
        ConfiguredCanHardware.devices().stream()
            .filter(device -> device.vendor() == Vendor.CTRE)
            .map(ConfiguredCanHardware.Device::canId)
            .toList());

    var pigeon = ConfiguredCanHardware.byCanId(20).orElseThrow();
    var encoder = ConfiguredCanHardware.byCanId(40).orElseThrow();
    var drive = ConfiguredCanHardware.byCanId(51).orElseThrow();
    assertEquals(DeviceType.GYRO, pigeon.type());
    assertEquals(HardwareRole.SWERVE_GYRO, pigeon.role());
    assertEquals(DeviceType.ABSOLUTE_ENCODER, encoder.type());
    assertEquals(HardwareRole.SWERVE_ABSOLUTE_ENCODER, encoder.role());
    assertEquals(DeviceType.MOTOR_CONTROLLER, drive.type());
    assertEquals(HardwareRole.SWERVE_DRIVE_MOTOR, drive.role());
    assertEquals(List.of(), drive.dependencyCanIds());
    assertEquals(List.of(40), ConfiguredCanHardware.byCanId(50).orElseThrow().dependencyCanIds());
    assertEquals(List.of(41), ConfiguredCanHardware.byCanId(52).orElseThrow().dependencyCanIds());
    assertEquals(List.of(42), ConfiguredCanHardware.byCanId(54).orElseThrow().dependencyCanIds());
    assertEquals(List.of(43), ConfiguredCanHardware.byCanId(56).orElseThrow().dependencyCanIds());
    assertEquals(List.of(37), ConfiguredCanHardware.byCanId(36).orElseThrow().dependencyCanIds());
    assertEquals(List.of(36), ConfiguredCanHardware.byCanId(37).orElseThrow().dependencyCanIds());
    assertTrue(ConfiguredCanHardware.devices().stream().noneMatch(device -> device.label().isBlank()));
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
