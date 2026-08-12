package frc.robot.constants;

import frc.robot.constants.Constants.ManipulatorConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

/**
 * Static reachability manifest for every configured motor controller.
 *
 * <p>This describes software routes and motor-local static blockers, not every aggregate command
 * interlock, live CAN health, or proof that the physical mechanism is safe. Runtime health and
 * diagnostic evidence remain authoritative.
 */
public final class ConfiguredMotorCapabilities {
  public enum Blocker {
    UNREFERENCED,
    UPSTREAM_UNREFERENCED,
    KNOWN_STALL,
    UPSTREAM_INTERLOCK,
    DESIGN_UNVERIFIED,
    AUTONOMOUS_CALIBRATION,
    NO_ROUTE
  }

  public enum DiagnosticRoute {
    HST_LOW_OUTPUT_STAGE,
    HST_CONTROLLED_RETEST_STAGE,
    HST_FLYWHEEL_PAIR_STAGE,
    HST_FOLLOWER_ISOLATED_STAGE,
    MANUAL_ARMED_PULSE_ONLY,
    HST_SWERVE_GROUP
  }

  public record ModeAccess(String route, Set<Blocker> blockers) {
    public ModeAccess {
      Objects.requireNonNull(route, "route");
      if (route.isBlank()) {
        throw new IllegalArgumentException("route must not be blank");
      }
      blockers = Set.copyOf(blockers);
    }

    public boolean reachable() {
      return blockers.isEmpty();
    }
  }

  public record MotorCapability(
      int canId,
      String label,
      ModeAccess teleop,
      ModeAccess autonomous,
      List<DiagnosticRoute> diagnostics,
      Set<Blocker> diagnosticBlockers) {
    public MotorCapability {
      if (canId < 0 || canId > 62) {
        throw new IllegalArgumentException("CAN ID must be within 0..62");
      }
      Objects.requireNonNull(label, "label");
      if (label.isBlank()) {
        throw new IllegalArgumentException("label must not be blank");
      }
      Objects.requireNonNull(teleop, "teleop");
      Objects.requireNonNull(autonomous, "autonomous");
      diagnostics = List.copyOf(diagnostics);
      diagnosticBlockers = Set.copyOf(diagnosticBlockers);
      if (diagnostics.isEmpty()) {
        throw new IllegalArgumentException("every motor must have a diagnostic route");
      }
    }

    public boolean diagnosticReachable() {
      return diagnosticBlockers.isEmpty();
    }
  }

  private static final ModeAccess TELEOP_INTAKE_ACTUATOR = blocked(
      "Intake/Output/Retract", Blocker.UNREFERENCED);
  private static final ModeAccess AUTO_SLURP_UNREFERENCED = blocked(
      "Slurp registered but not deployed",
      Blocker.AUTONOMOUS_CALIBRATION,
      Blocker.UNREFERENCED,
      Blocker.NO_ROUTE);
  private static final ModeAccess TELEOP_INTAKE_PATH = blocked(
      "Intake/Output", Blocker.UPSTREAM_UNREFERENCED);
  private static final ModeAccess AUTO_SLURP_UPSTREAM = blocked(
      "Slurp registered but not deployed",
      Blocker.AUTONOMOUS_CALIBRATION,
      Blocker.UPSTREAM_UNREFERENCED,
      Blocker.NO_ROUTE);
  private static final ModeAccess AUTO_ADVANCED_FIRE = blocked(
      "Advanced Fire", Blocker.AUTONOMOUS_CALIBRATION);
  private static final ModeAccess TELEOP_NO_CLIMBER_ROUTE = blocked(
      "NONE", Blocker.NO_ROUTE, Blocker.DESIGN_UNVERIFIED);
  private static final ModeAccess AUTO_NO_CLIMBER_ROUTE = blocked(
      "NONE", Blocker.NO_ROUTE, Blocker.DESIGN_UNVERIFIED);
  private static final ModeAccess TELEOP_FLYWHEEL = available("Rev flywheel pair");
  private static final ModeAccess TELEOP_SWERVE = available("Swerve drive/assist");
  private static final ModeAccess AUTO_SWERVE = blocked(
      "PathPlanner", Blocker.AUTONOMOUS_CALIBRATION);

  private static final List<MotorCapability> CAPABILITIES = List.of(
      capability(
          ConfiguredCanHardware.INTAKE_ACTUATOR_ID,
          "intake actuator",
          TELEOP_INTAKE_ACTUATOR,
          AUTO_SLURP_UNREFERENCED,
          DiagnosticRoute.MANUAL_ARMED_PULSE_ONLY),
      capability(
          ConfiguredCanHardware.INTAKE_ROLLER_ID,
          "intake roller",
          TELEOP_INTAKE_PATH,
          AUTO_SLURP_UPSTREAM,
          DiagnosticRoute.HST_LOW_OUTPUT_STAGE),
      new MotorCapability(
          ConfiguredCanHardware.FEEDER_ID,
          "feeder",
          blocked("Fire", Blocker.KNOWN_STALL),
          blocked("Advanced Fire", Blocker.AUTONOMOUS_CALIBRATION, Blocker.KNOWN_STALL),
          ManipulatorConstants.FEEDER_CONTROLLED_RETEST_ENABLED
              ? List.of(
                  DiagnosticRoute.HST_CONTROLLED_RETEST_STAGE,
                  DiagnosticRoute.MANUAL_ARMED_PULSE_ONLY)
              : List.of(DiagnosticRoute.MANUAL_ARMED_PULSE_ONLY),
          Set.of()),
      capability(
          ConfiguredCanHardware.CONVEYOR_ID,
          "conveyor",
          blocked("Intake/Output/Fire", Blocker.UPSTREAM_INTERLOCK),
          blocked(
              "Slurp/Advanced Fire",
              Blocker.AUTONOMOUS_CALIBRATION,
              Blocker.UPSTREAM_INTERLOCK),
          DiagnosticRoute.HST_LOW_OUTPUT_STAGE),
      capability(
          ConfiguredCanHardware.CLIMBER_LEFT_ID,
          "climber left",
          TELEOP_NO_CLIMBER_ROUTE,
          AUTO_NO_CLIMBER_ROUTE,
          DiagnosticRoute.MANUAL_ARMED_PULSE_ONLY),
      capability(
          ConfiguredCanHardware.CLIMBER_RIGHT_ID,
          "climber right",
          TELEOP_NO_CLIMBER_ROUTE,
          AUTO_NO_CLIMBER_ROUTE,
          DiagnosticRoute.MANUAL_ARMED_PULSE_ONLY),
      capability(
          ConfiguredCanHardware.SHOOTER_LEADER_ID,
          "shooter flywheel leader",
          TELEOP_FLYWHEEL,
          AUTO_ADVANCED_FIRE,
          DiagnosticRoute.HST_FLYWHEEL_PAIR_STAGE),
      new MotorCapability(
          ConfiguredCanHardware.SHOOTER_FOLLOWER_ID,
          "shooter flywheel follower",
          TELEOP_FLYWHEEL,
          AUTO_ADVANCED_FIRE,
          List.of(
              DiagnosticRoute.HST_FLYWHEEL_PAIR_STAGE,
              DiagnosticRoute.HST_FOLLOWER_ISOLATED_STAGE),
          Set.of()),
      capability(
          ConfiguredCanHardware.SHOOTER_ACTUATOR_ID,
          "shooter actuator",
          blocked("Rev", Blocker.UNREFERENCED),
          blocked(
              "Advanced Fire", Blocker.AUTONOMOUS_CALIBRATION, Blocker.UNREFERENCED),
          DiagnosticRoute.MANUAL_ARMED_PULSE_ONLY),
      capability(
          ConfiguredCanHardware.TURRET_ID,
          "turret",
          blocked("Auto Aim", Blocker.UNREFERENCED),
          blocked(
              "Advanced Fire", Blocker.AUTONOMOUS_CALIBRATION, Blocker.UNREFERENCED),
          DiagnosticRoute.MANUAL_ARMED_PULSE_ONLY),
      swerveCapability(ConfiguredCanHardware.FRONT_LEFT_STEER_ID, "front-left steer"),
      swerveCapability(ConfiguredCanHardware.FRONT_LEFT_DRIVE_ID, "front-left drive"),
      swerveCapability(ConfiguredCanHardware.FRONT_RIGHT_STEER_ID, "front-right steer"),
      swerveCapability(ConfiguredCanHardware.FRONT_RIGHT_DRIVE_ID, "front-right drive"),
      swerveCapability(ConfiguredCanHardware.BACK_LEFT_STEER_ID, "back-left steer"),
      swerveCapability(ConfiguredCanHardware.BACK_LEFT_DRIVE_ID, "back-left drive"),
      swerveCapability(ConfiguredCanHardware.BACK_RIGHT_STEER_ID, "back-right steer"),
      swerveCapability(ConfiguredCanHardware.BACK_RIGHT_DRIVE_ID, "back-right drive"));

  static {
    List<Integer> ids = CAPABILITIES.stream().map(MotorCapability::canId).toList();
    if (!ids.equals(ids.stream().sorted().toList())
        || new LinkedHashSet<>(ids).size() != ids.size()) {
      throw new IllegalStateException("motor capability CAN IDs must be unique and sorted");
    }
  }

  private ConfiguredMotorCapabilities() {}

  public static List<MotorCapability> all() {
    return CAPABILITIES;
  }

  public static Optional<MotorCapability> byCanId(int canId) {
    return CAPABILITIES.stream().filter(capability -> capability.canId() == canId).findFirst();
  }

  public static Set<Integer> manualDiagnosticCanIds() {
    return idsWithDiagnostic(DiagnosticRoute.MANUAL_ARMED_PULSE_ONLY);
  }

  public static Set<Integer> idsWithDiagnostic(DiagnosticRoute route) {
    return CAPABILITIES.stream()
        .filter(capability -> capability.diagnostics().contains(route))
        .map(MotorCapability::canId)
        .collect(Collectors.toUnmodifiableSet());
  }

  /** Existing operator-facing HST format, generated from the capability entries. */
  public static String hardwareSelfTestCoverageSummary() {
    String feederStatus = ManipulatorConstants.FEEDER_CONTROLLED_RETEST_ENABLED
        ? "CONTROLLED_RETEST_STAGE"
        : "HST_SKIP_KNOWN_STALL+MANUAL_ARMED_REPAIR_RETEST_ONLY";
    return String.join(
        "; ",
        sparkCoverage(ConfiguredCanHardware.INTAKE_ACTUATOR_ID, "MANUAL_ARMED_PULSE_ONLY"),
        sparkCoverage(ConfiguredCanHardware.INTAKE_ROLLER_ID, "LOW_OUTPUT_STAGE"),
        sparkCoverage(ConfiguredCanHardware.FEEDER_ID, feederStatus),
        sparkCoverage(ConfiguredCanHardware.CONVEYOR_ID, "LOW_OUTPUT_STAGE"),
        sparkGroupCoverage(
            List.of(ConfiguredCanHardware.CLIMBER_LEFT_ID, ConfiguredCanHardware.CLIMBER_RIGHT_ID),
            "climber",
            "MANUAL_ARMED_PULSE_ONLY"),
        sparkGroupCoverage(
            List.of(
                ConfiguredCanHardware.SHOOTER_LEADER_ID,
                ConfiguredCanHardware.SHOOTER_FOLLOWER_ID),
            "flywheel",
            "PAIR_STAGE"),
        "Spark" + ConfiguredCanHardware.SHOOTER_FOLLOWER_ID + " follower=ISOLATED_STAGE",
        sparkCoverage(ConfiguredCanHardware.SHOOTER_ACTUATOR_ID, "MANUAL_ARMED_PULSE_ONLY"),
        sparkCoverage(ConfiguredCanHardware.TURRET_ID, "MANUAL_ARMED_PULSE_ONLY"),
        ConfiguredCanHardware.ctreCoverageLabel()
            + " swerve=LOW_OUTPUT_MOTION_OBSERVED_ONLY");
  }

  public static String normalMotionSummary() {
    return "STATIC_ROUTE_ONLY; LIVE_HEALTH_SEPARATE; TELEOP_ROUTE_OPEN="
        + compactIds(idsWhere(capability -> capability.teleop().reachable()))
        + "; TELEOP_ROUTE_BLOCKED="
        + compactIds(idsWhere(capability -> !capability.teleop().reachable()))
        + "; AUTO_ROUTE_OPEN="
        + compactIds(idsWhere(capability -> capability.autonomous().reachable()))
        + "; AUTO_ROUTE_BLOCKED="
        + compactIds(idsWhere(capability -> !capability.autonomous().reachable()));
  }

  public static String blockedSummary() {
    String teleop = blockedModeSummary(true);
    String autonomous = blockedModeSummary(false);
    String diagnostic = CAPABILITIES.stream()
        .filter(capability -> !capability.diagnosticReachable())
        .map(capability -> "ID" + capability.canId() + ':'
            + joinBlockers(capability.diagnosticBlockers()))
        .collect(Collectors.joining(","));
    return "TELEOP=" + emptyAsNone(teleop)
        + "; AUTO=" + emptyAsNone(autonomous)
        + "; DIAGNOSTIC=" + emptyAsNone(diagnostic);
  }

  private static MotorCapability capability(
      int canId,
      String label,
      ModeAccess teleop,
      ModeAccess autonomous,
      DiagnosticRoute diagnostic) {
    return new MotorCapability(
        canId, label, teleop, autonomous, List.of(diagnostic), Set.of());
  }

  private static MotorCapability swerveCapability(int canId, String label) {
    return capability(
        canId, label, TELEOP_SWERVE, AUTO_SWERVE, DiagnosticRoute.HST_SWERVE_GROUP);
  }

  private static ModeAccess available(String route) {
    return new ModeAccess(route, Set.of());
  }

  private static ModeAccess blocked(String route, Blocker... blockers) {
    return new ModeAccess(route, Set.copyOf(Arrays.asList(blockers)));
  }

  private static String sparkCoverage(int canId, String status) {
    MotorCapability capability = byCanId(canId).orElseThrow();
    return "Spark" + canId + ' ' + capability.label() + '=' + status;
  }

  private static String sparkGroupCoverage(List<Integer> canIds, String label, String status) {
    for (int canId : canIds) {
      if (byCanId(canId).isEmpty()) {
        throw new IllegalStateException("missing grouped motor capability ID" + canId);
      }
    }
    return "Spark" + canIds.stream().map(String::valueOf).collect(Collectors.joining("/"))
        + ' ' + label + '=' + status;
  }

  private static List<Integer> idsWhere(
      java.util.function.Predicate<MotorCapability> predicate) {
    return CAPABILITIES.stream()
        .filter(predicate)
        .map(MotorCapability::canId)
        .toList();
  }

  private static String blockedModeSummary(boolean teleop) {
    return CAPABILITIES.stream()
        .filter(capability -> !(teleop ? capability.teleop() : capability.autonomous()).reachable())
        .map(capability -> {
          ModeAccess access = teleop ? capability.teleop() : capability.autonomous();
          return "ID" + capability.canId() + ':' + joinBlockers(access.blockers());
        })
        .collect(Collectors.joining(","));
  }

  private static String joinBlockers(Set<Blocker> blockers) {
    return blockers.stream().sorted().map(Enum::name).collect(Collectors.joining("+"));
  }

  private static String emptyAsNone(String value) {
    return value.isEmpty() ? "NONE" : value;
  }

  private static String compactIds(Iterable<Integer> ids) {
    List<Integer> sorted = new ArrayList<>();
    ids.forEach(sorted::add);
    sorted = sorted.stream().distinct().sorted().toList();
    if (sorted.isEmpty()) {
      return "NONE";
    }
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
