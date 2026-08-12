package frc.robot.constants;

import java.util.List;
import java.util.Objects;
import java.util.Optional;

/**
 * Single source of truth for the CAN IDs configured by this software.
 *
 * <p>This is not proof of the physical robot inventory. The configured map must still be checked
 * against the wiring/CAD and live device discovery before calibrated motion is enabled.
 */
public final class ConfiguredCanHardware {
  public enum Vendor {
    REV,
    CTRE
  }

  public enum DeviceType {
    MOTOR_CONTROLLER,
    GYRO,
    ABSOLUTE_ENCODER
  }

  public enum HardwareRole {
    MECHANISM_MOTOR,
    SWERVE_DRIVE_MOTOR,
    SWERVE_STEER_MOTOR,
    SWERVE_ABSOLUTE_ENCODER,
    SWERVE_GYRO
  }

  /**
   * Static software inventory only; live device evidence is published separately.
   *
   * <p>{@code dependencyCanIds} lists controllers or sensors whose configured relationship gates
   * this device's software role. The shooter pair is intentionally mutual: ID37 follows ID36,
   * while ID36 output is refused unless its required follower ID37 is ready.
   */
  public record Device(
      int canId,
      String label,
      Vendor vendor,
      DeviceType type,
      HardwareRole role,
      List<Integer> dependencyCanIds) {
    public Device {
      if (canId < 0 || canId > 62) {
        throw new IllegalArgumentException("CAN ID must be within 0..62");
      }
      Objects.requireNonNull(label, "label");
      if (label.isBlank()) {
        throw new IllegalArgumentException("label must not be blank");
      }
      Objects.requireNonNull(vendor, "vendor");
      Objects.requireNonNull(type, "type");
      Objects.requireNonNull(role, "role");
      dependencyCanIds = List.copyOf(dependencyCanIds);
      if (dependencyCanIds.stream().anyMatch(id -> id < 0 || id > 62 || id == canId)
          || dependencyCanIds.stream().distinct().count() != dependencyCanIds.size()) {
        throw new IllegalArgumentException("dependency CAN IDs must be unique, legal, and external");
      }
    }
  }

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

  private static final List<Device> DEVICES = List.of(
      device(
          PIGEON_ID,
          "swerve pigeon",
          Vendor.CTRE,
          DeviceType.GYRO,
          HardwareRole.SWERVE_GYRO),
      mechanismMotor(INTAKE_ACTUATOR_ID, "intake actuator"),
      mechanismMotor(INTAKE_ROLLER_ID, "intake roller"),
      mechanismMotor(FEEDER_ID, "feeder"),
      mechanismMotor(CONVEYOR_ID, "conveyor"),
      mechanismMotor(CLIMBER_LEFT_ID, "climber left"),
      mechanismMotor(CLIMBER_RIGHT_ID, "climber right"),
      mechanismMotor(
          SHOOTER_LEADER_ID, "shooter flywheel leader", SHOOTER_FOLLOWER_ID),
      mechanismMotor(
          SHOOTER_FOLLOWER_ID, "shooter flywheel follower", SHOOTER_LEADER_ID),
      mechanismMotor(SHOOTER_ACTUATOR_ID, "shooter actuator"),
      mechanismMotor(TURRET_ID, "turret"),
      swerveEncoder(FRONT_LEFT_ENCODER_ID, "front-left encoder"),
      swerveEncoder(FRONT_RIGHT_ENCODER_ID, "front-right encoder"),
      swerveEncoder(BACK_LEFT_ENCODER_ID, "back-left encoder"),
      swerveEncoder(BACK_RIGHT_ENCODER_ID, "back-right encoder"),
      swerveMotor(
          FRONT_LEFT_STEER_ID,
          "front-left steer",
          HardwareRole.SWERVE_STEER_MOTOR,
          FRONT_LEFT_ENCODER_ID),
      swerveMotor(FRONT_LEFT_DRIVE_ID, "front-left drive", HardwareRole.SWERVE_DRIVE_MOTOR),
      swerveMotor(
          FRONT_RIGHT_STEER_ID,
          "front-right steer",
          HardwareRole.SWERVE_STEER_MOTOR,
          FRONT_RIGHT_ENCODER_ID),
      swerveMotor(FRONT_RIGHT_DRIVE_ID, "front-right drive", HardwareRole.SWERVE_DRIVE_MOTOR),
      swerveMotor(
          BACK_LEFT_STEER_ID,
          "back-left steer",
          HardwareRole.SWERVE_STEER_MOTOR,
          BACK_LEFT_ENCODER_ID),
      swerveMotor(BACK_LEFT_DRIVE_ID, "back-left drive", HardwareRole.SWERVE_DRIVE_MOTOR),
      swerveMotor(
          BACK_RIGHT_STEER_ID,
          "back-right steer",
          HardwareRole.SWERVE_STEER_MOTOR,
          BACK_RIGHT_ENCODER_ID),
      swerveMotor(BACK_RIGHT_DRIVE_ID, "back-right drive", HardwareRole.SWERVE_DRIVE_MOTOR));

  private static final List<Integer> SPARK_DEVICE_IDS = idsWhere(
      device -> device.vendor() == Vendor.REV);
  private static final List<Integer> SWERVE_ENCODER_IDS = idsWhere(
      device -> device.role() == HardwareRole.SWERVE_ABSOLUTE_ENCODER);
  private static final List<Integer> SWERVE_STEER_IDS = idsWhere(
      device -> device.role() == HardwareRole.SWERVE_STEER_MOTOR);
  private static final List<Integer> SWERVE_DRIVE_IDS = idsWhere(
      device -> device.role() == HardwareRole.SWERVE_DRIVE_MOTOR);
  private static final List<Integer> CTRE_DEVICE_IDS = idsWhere(
      device -> device.vendor() == Vendor.CTRE);
  private static final List<Integer> ALL_DEVICE_IDS = DEVICES.stream().map(Device::canId).toList();

  static {
    List<Integer> ids = DEVICES.stream().map(Device::canId).toList();
    if (!ids.equals(ids.stream().sorted().toList()) || ids.stream().distinct().count() != ids.size()) {
      throw new IllegalStateException("configured CAN devices must be unique and sorted");
    }
  }

  private ConfiguredCanHardware() {}

  public static List<Device> devices() {
    return DEVICES;
  }

  public static Optional<Device> byCanId(int canId) {
    return DEVICES.stream().filter(device -> device.canId() == canId).findFirst();
  }

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

  private static Device mechanismMotor(int canId, String label) {
    return device(
        canId, label, Vendor.REV, DeviceType.MOTOR_CONTROLLER, HardwareRole.MECHANISM_MOTOR);
  }

  private static Device mechanismMotor(int canId, String label, int dependencyCanId) {
    return new Device(
        canId,
        label,
        Vendor.REV,
        DeviceType.MOTOR_CONTROLLER,
        HardwareRole.MECHANISM_MOTOR,
        List.of(dependencyCanId));
  }

  private static Device swerveEncoder(int canId, String label) {
    return device(
        canId,
        label,
        Vendor.CTRE,
        DeviceType.ABSOLUTE_ENCODER,
        HardwareRole.SWERVE_ABSOLUTE_ENCODER);
  }

  private static Device swerveMotor(int canId, String label, HardwareRole role) {
    return device(canId, label, Vendor.CTRE, DeviceType.MOTOR_CONTROLLER, role);
  }

  private static Device swerveMotor(
      int canId, String label, HardwareRole role, int dependencyCanId) {
    return new Device(
        canId,
        label,
        Vendor.CTRE,
        DeviceType.MOTOR_CONTROLLER,
        role,
        List.of(dependencyCanId));
  }

  private static Device device(
      int canId, String label, Vendor vendor, DeviceType type, HardwareRole role) {
    return new Device(canId, label, vendor, type, role, List.of());
  }

  private static List<Integer> idsWhere(java.util.function.Predicate<Device> predicate) {
    return DEVICES.stream().filter(predicate).map(Device::canId).toList();
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
