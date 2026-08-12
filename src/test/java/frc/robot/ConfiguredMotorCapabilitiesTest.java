package frc.robot;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.commands.HardwareSelfTestCommand;
import frc.robot.commands.ManualUnhomedActuatorDiagnosticCommand;
import frc.robot.constants.ConfiguredCanHardware;
import frc.robot.constants.ConfiguredMotorCapabilities;
import frc.robot.constants.ConfiguredMotorCapabilities.Blocker;
import frc.robot.constants.ConfiguredMotorCapabilities.DiagnosticRoute;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

class ConfiguredMotorCapabilitiesTest {
  @Test
  void manifestCoversEveryConfiguredMotorExactlyOnceInCanOrder() {
    List<Integer> expectedMotorIds = Stream.of(
            ConfiguredCanHardware.sparkDeviceIds(),
            ConfiguredCanHardware.swerveSteerIds(),
            ConfiguredCanHardware.swerveDriveIds())
        .flatMap(List::stream)
        .sorted()
        .toList();
    List<Integer> actualMotorIds = ConfiguredMotorCapabilities.all().stream()
        .map(ConfiguredMotorCapabilities.MotorCapability::canId)
        .toList();

    assertAll(
        () -> assertEquals(18, actualMotorIds.size()),
        () -> assertEquals(expectedMotorIds, actualMotorIds),
        () -> assertEquals(actualMotorIds.size(), new HashSet<>(actualMotorIds).size()),
        () -> ConfiguredMotorCapabilities.all().forEach(capability -> {
          assertFalse(capability.label().isBlank(), "ID" + capability.canId());
          assertFalse(capability.teleop().route().isBlank(), "ID" + capability.canId());
          assertFalse(capability.autonomous().route().isBlank(), "ID" + capability.canId());
          assertFalse(capability.diagnostics().isEmpty(), "ID" + capability.canId());
        }));
  }

  @Test
  void manualDiagnosticTargetsStayExactlySynchronizedWithTheCapabilityManifest() {
    Set<Integer> commandTargetIds = Arrays.stream(
            ManualUnhomedActuatorDiagnosticCommand.Target.values())
        .map(ManualUnhomedActuatorDiagnosticCommand.Target::canId)
        .collect(java.util.stream.Collectors.toUnmodifiableSet());

    assertEquals(Set.of(30, 34, 35, 38, 39), commandTargetIds);
    assertEquals(commandTargetIds, ConfiguredMotorCapabilities.manualDiagnosticCanIds());
  }

  @Test
  void groupedDiagnosticRoutesDescribeEvidenceWithoutClaimingPerMotorIsolation() {
    assertAll(
        () -> assertEquals(
            Set.of(31, 33),
            ConfiguredMotorCapabilities.idsWithDiagnostic(
                DiagnosticRoute.HST_LOW_OUTPUT_STAGE)),
        () -> assertEquals(
            Set.of(36, 37),
            ConfiguredMotorCapabilities.idsWithDiagnostic(
                DiagnosticRoute.HST_FLYWHEEL_PAIR_STAGE)),
        () -> assertEquals(
            Set.of(37),
            ConfiguredMotorCapabilities.idsWithDiagnostic(
                DiagnosticRoute.HST_FOLLOWER_ISOLATED_STAGE)),
        () -> assertEquals(
            Set.of(50, 51, 52, 53, 54, 55, 56, 57),
            ConfiguredMotorCapabilities.idsWithDiagnostic(DiagnosticRoute.HST_SWERVE_GROUP)));
  }

  @Test
  void currentReachabilityAndKnownBlockersRemainFailClosed() {
    var feeder = ConfiguredMotorCapabilities.byCanId(32).orElseThrow();
    var intakeActuator = ConfiguredMotorCapabilities.byCanId(30).orElseThrow();
    var intakeRoller = ConfiguredMotorCapabilities.byCanId(31).orElseThrow();
    var shooterLeader = ConfiguredMotorCapabilities.byCanId(36).orElseThrow();

    assertAll(
        () -> assertEquals(
            "STATIC_ROUTE_ONLY; LIVE_HEALTH_SEPARATE; TELEOP_ROUTE_OPEN=36-37,50-57; "
                + "TELEOP_ROUTE_BLOCKED=30-35,38-39; AUTO_ROUTE_OPEN=NONE; "
                + "AUTO_ROUTE_BLOCKED=30-39,50-57",
            ConfiguredMotorCapabilities.normalMotionSummary()),
        () -> assertTrue(intakeActuator.teleop().blockers().contains(Blocker.UNREFERENCED)),
        () -> assertFalse(intakeActuator.teleop().reachable()),
        () -> assertTrue(intakeActuator.autonomous().blockers().contains(Blocker.NO_ROUTE),
            "Slurp is registered but is not referenced by a deployed autonomous routine"),
        () -> assertTrue(intakeRoller.autonomous().blockers().contains(Blocker.NO_ROUTE)),
        () -> assertTrue(shooterLeader.teleop().reachable()),
        () -> assertTrue(feeder.teleop().blockers().contains(Blocker.KNOWN_STALL)),
        () -> assertFalse(feeder.diagnosticReachable(),
            "controlled feeder retest must stay blocked until its commissioning flag changes"));
  }

  @Test
  void hardwareSelfTestCoverageIsGeneratedByTheCapabilityManifest() {
    String expected = ConfiguredMotorCapabilities.hardwareSelfTestCoverageSummary();

    assertAll(
        () -> assertEquals(expected, HardwareSelfTestCommand.getCoverageManifest()),
        () -> assertTrue(expected.contains("Spark34/35 climber=MANUAL_ARMED_PULSE_ONLY")),
        () -> assertTrue(expected.contains("Spark36/37 flywheel=PAIR_STAGE")),
        () -> assertTrue(expected.contains(
            ConfiguredCanHardware.ctreCoverageLabel()
                + " swerve=LOW_OUTPUT_MOTION_OBSERVED_ONLY")));
  }
}
