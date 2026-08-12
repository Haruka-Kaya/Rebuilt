package frc.robot.constants;

import java.util.ArrayList;
import java.util.List;

/**
 * Single source of truth for the CAN IDs configured by this software.
 *
 * <p>This is not proof of the physical robot inventory. The configured map must still be checked
 * against the wiring/CAD and live device discovery before calibrated motion is enabled.
 */
public final class ConfiguredCanHardware {
  public static final int PIGEON_ID = 20;

  public static final int INTAKE_ACTUATOR_ID = 30;
  public static final int INTAKE_ROLLER_ID = 31;
  public static final int FEEDER_ID = 32;
  public static final int CONVEYOR_ID = 33;
  public static final int CLIMBER_LEFT_ID = 34;
  public static final int CLIMBER_RIGHT_ID = 35;
  public static final int SHOOTER_LEADER_ID = 36;
  public static final int SHOOTER_FOLLOWER_ID = 37;
  public static final int SHOOTER_ACTUATOR_ID = 38;
  public static final int TURRET_ID = 39;

  public static final int FRONT_LEFT_ENCODER_ID = 40;
  public static final int FRONT_RIGHT_ENCODER_ID = 41;
  public static final int BACK_LEFT_ENCODER_ID = 42;
  public static final int BACK_RIGHT_ENCODER_ID = 43;

  public static final int FRONT_LEFT_STEER_ID = 50;
  public static final int FRONT_LEFT_DRIVE_ID = 51;
  public static final int FRONT_RIGHT_STEER_ID = 52;
  public static final int FRONT_RIGHT_DRIVE_ID = 53;
  public static final int BACK_LEFT_STEER_ID = 54;
  public static final int BACK_LEFT_DRIVE_ID = 55;
  public static final int BACK_RIGHT_STEER_ID = 56;
  public static final int BACK_RIGHT_DRIVE_ID = 57;

  private static final List<Integer> SPARK_DEVICE_IDS = List.of(
      INTAKE_ACTUATOR_ID,
      INTAKE_ROLLER_ID,
      FEEDER_ID,
      CONVEYOR_ID,
      CLIMBER_LEFT_ID,
      CLIMBER_RIGHT_ID,
      SHOOTER_LEADER_ID,
      SHOOTER_FOLLOWER_ID,
      SHOOTER_ACTUATOR_ID,
      TURRET_ID);

  private static final List<Integer> SWERVE_ENCODER_IDS = List.of(
      FRONT_LEFT_ENCODER_ID,
      FRONT_RIGHT_ENCODER_ID,
      BACK_LEFT_ENCODER_ID,
      BACK_RIGHT_ENCODER_ID);

  private static final List<Integer> SWERVE_STEER_IDS = List.of(
      FRONT_LEFT_STEER_ID,
      FRONT_RIGHT_STEER_ID,
      BACK_LEFT_STEER_ID,
      BACK_RIGHT_STEER_ID);

  private static final List<Integer> SWERVE_DRIVE_IDS = List.of(
      FRONT_LEFT_DRIVE_ID,
      FRONT_RIGHT_DRIVE_ID,
      BACK_LEFT_DRIVE_ID,
      BACK_RIGHT_DRIVE_ID);

  private static final List<Integer> CTRE_DEVICE_IDS = concatenate(
      List.of(PIGEON_ID), SWERVE_ENCODER_IDS, SWERVE_STEER_IDS, SWERVE_DRIVE_IDS);

  private static final List<Integer> ALL_DEVICE_IDS = concatenate(
      SPARK_DEVICE_IDS, CTRE_DEVICE_IDS);

  private ConfiguredCanHardware() {}

  public static List<Integer> sparkDeviceIds() {
    return SPARK_DEVICE_IDS;
  }

  public static List<Integer> swerveEncoderIds() {
    return SWERVE_ENCODER_IDS;
  }

  public static List<Integer> swerveSteerIds() {
    return SWERVE_STEER_IDS;
  }

  public static List<Integer> swerveDriveIds() {
    return SWERVE_DRIVE_IDS;
  }

  public static List<Integer> ctreDeviceIds() {
    return CTRE_DEVICE_IDS;
  }

  public static List<Integer> allDeviceIds() {
    return ALL_DEVICE_IDS;
  }

  /** Returns the compact configured CTRE range used by operator diagnostics. */
  public static String ctreCoverageLabel() {
    return "CTRE" + compactRanges(CTRE_DEVICE_IDS);
  }

  public static String configuredSummary() {
    return "SPARK" + compactRanges(SPARK_DEVICE_IDS)
        + "; " + ctreCoverageLabel()
        + "; PHYSICAL_INVENTORY_UNVERIFIED";
  }

  @SafeVarargs
  private static List<Integer> concatenate(List<Integer>... groups) {
    List<Integer> result = new ArrayList<>();
    for (List<Integer> group : groups) {
      result.addAll(group);
    }
    // Keep duplicates so the manifest test can detect an accidental CAN-ID collision.
    return result.stream().sorted().toList();
  }

  private static String compactRanges(List<Integer> ids) {
    List<Integer> sorted = ids.stream().distinct().sorted().toList();
    StringBuilder result = new StringBuilder();
    for (int index = 0; index < sorted.size();) {
      int start = sorted.get(index);
      int end = start;
      while (index + 1 < sorted.size() && sorted.get(index + 1) == end + 1) {
        end = sorted.get(++index);
      }
      if (!result.isEmpty()) {
        result.append(',');
      }
      result.append(start);
      if (end != start) {
        result.append('-').append(end);
      }
      index++;
    }
    return result.toString();
  }
}
