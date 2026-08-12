package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.commands.HardwareSelfTestCommand;
import frc.robot.commands.ManualUnhomedActuatorDiagnosticCommand;
import frc.robot.commands.ManualUnhomedActuatorDiagnosticCommand.Direction;
import frc.robot.commands.ManualUnhomedActuatorDiagnosticCommand.Target;
import frc.robot.constants.ConfiguredCanHardware;
import frc.robot.constants.ConfiguredOperatorControls;
import frc.robot.constants.ConfiguredOperatorActions.Action;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.constants.Constants.DebugConstants;
import frc.robot.constants.Constants.HardwareTestConstants;
import frc.robot.constants.Constants.ManipulatorConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.utils.CtreDeviceEvidence.Metric;
import frc.robot.utils.SparkMAXContainer.OutputStopBatch;
import frc.robot.utils.SparkSimulationHandle.SimulationSnapshot;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.parallel.Execution;
import org.junit.jupiter.api.parallel.ExecutionMode;
import org.junit.jupiter.api.parallel.ResourceLock;

/** End-to-end proof that the production binding reaches SPARK output and stops fail-closed. */
@Execution(ExecutionMode.SAME_THREAD)
@ResourceLock("WPILIB_GLOBALS")
class RobotContainerSimulationIntegrationTest {
  private static final int DRIVER_PORT = OIConstants.kDriverControllerPort;
  private static final int SHOOTER_LEADER_ID = ConfiguredCanHardware.SHOOTER_LEADER_ID;
  private static final int SHOOTER_FOLLOWER_ID = ConfiguredCanHardware.SHOOTER_FOLLOWER_ID;
  private static final Translation2d[] SWERVE_MODULE_LOCATIONS = {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
  };
  private static final SwerveDriveKinematics SWERVE_KINEMATICS =
      new SwerveDriveKinematics(SWERVE_MODULE_LOCATIONS);

  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  @Test
  void driverRevBindingCommandsThePairStopsAndRequiresNeutralAfterRecovery()
      throws InterruptedException {
    CommandScheduler scheduler = CommandScheduler.getInstance();
    RobotContainer container = null;
    configureDisabledDriverStation();
    try {
      SparkMAXContainer.configureProcessDefaults();
      container = new RobotContainer();
      RobotContainer activeContainer = container;

      assertEquals(
          ConfiguredCanHardware.sparkDeviceIds().size(),
          SparkRawCommandEchoSimulation.stepConfiguredControllers(false).size());
      assertTrue(
          await(12.0, activeContainer, () -> ConfiguredCanHardware.sparkDeviceIds().stream()
              .allMatch(id -> SparkMAXContainer.getDiagnosticSnapshotForId(id)
                  .map(snapshot -> snapshot.ready())
                  .orElse(false))),
          SparkMAXContainer::getDeviceAvailabilitySummary);
      assertTrue(
          await(
              8.0,
              activeContainer,
              () -> activeContainer.getOutputSafetySnapshot().phase()
                  == RobotOutputSafetySupervisor.Phase.READY_DISABLED),
          () -> activeContainer.getOutputSafetySnapshot().toString());

      enableTeleop(false);
      assertOutputSafetyArmed(activeContainer);
      long followerEpochBeforeRev = SparkMAXContainer.getDeviceEvidenceSnapshots().stream()
          .filter(snapshot -> snapshot.canId() == SHOOTER_FOLLOWER_ID)
          .findFirst()
          .orElseThrow()
          .outputEpoch();
      setDriverButton(ConfiguredOperatorControls.DRIVER_REV, true);

      assertTrue(await(3.0, activeContainer, () -> pairEchoesRequestedVelocity(500.0)),
          () -> pairSummary("initial rev binding did not reach the SPARK pair"));
      var followerEvidence = SparkMAXContainer.getDeviceEvidenceSnapshots().stream()
          .filter(snapshot -> snapshot.canId() == SHOOTER_FOLLOWER_ID)
          .findFirst()
          .orElseThrow();
      assertFalse(
          followerEvidence.lastRequestAccepted(),
          "a follower observation must not be reported as a direct setpoint API acceptance");
      assertEquals(
          "FOLLOWER_OUTPUT_EXPECTED_FROM_LEADER_NOT_DIRECT_API_ACCEPTANCE",
          followerEvidence.lastRequestReason());
      assertTrue(
          followerEvidence.outputEpoch() > followerEpochBeforeRev,
          "the follower evidence epoch must advance with the accepted leader request");
      assertEquals(
          "ACTIVE",
          SmartDashboard.getString(OperatorActionEvidence.stateKey(Action.REV), "MISSING"));
      assertTrue(
          SmartDashboard.getString(OperatorActionEvidence.reasonKey(Action.REV), "")
              .contains("HOOD_UNREFERENCED"),
          "Rev evidence must explain that only the flywheel pair was accepted");
      assertRawControllersStopped(ConfiguredCanHardware.SHOOTER_ACTUATOR_ID);

      setDriverButton(ConfiguredOperatorControls.DRIVER_REV, false);
      assertTrue(await(3.0, activeContainer, RobotContainerSimulationIntegrationTest::pairRawStopped),
          () -> pairSummary("RevUpCommand.end did not stop the SPARK pair"));
      assertEquals(
          "STOPPED",
          SmartDashboard.getString(OperatorActionEvidence.stateKey(Action.REV), "MISSING"));
      assertEquals(
          "INPUT_RELEASED",
          SmartDashboard.getString(OperatorActionEvidence.reasonKey(Action.REV), "MISSING"));
      OutputStopBatch firstStop = SparkMAXContainer.requestOutputStops(
          SHOOTER_LEADER_ID, SHOOTER_FOLLOWER_ID);
      assertTrue(await(3.0, activeContainer, () -> firstStop.snapshot().confirmed()),
          () -> firstStop.snapshot().summary());
      assertPairRawStopped();
      assertRawControllersStopped(ConfiguredCanHardware.SHOOTER_ACTUATOR_ID);

      // Start once more, then inject the vendor CAN-fault bit while the physical input is held.
      pump(activeContainer, 2);
      setDriverButton(ConfiguredOperatorControls.DRIVER_REV, true);
      assertTrue(await(3.0, activeContainer, () -> pairEchoesRequestedVelocity(500.0)),
          () -> pairSummary("second rev did not start"));
      SparkSimulationHandle leader = simulationHandle(SHOOTER_LEADER_ID);
      leader.setCanFault(true, false);
      assertTrue(await(3.0, activeContainer, () -> {
        SimulationSnapshot snapshot = leader.observe();
        SimulationSnapshot follower = simulationHandle(SHOOTER_FOLLOWER_ID).observe();
        return !SparkMAXContainer.getDiagnosticSnapshotForId(SHOOTER_LEADER_ID)
                .orElseThrow().ready()
            && snapshot.setpoint() == 0.0
            && snapshot.appliedOutput() == 0.0
            && snapshot.velocity() == 0.0
            && follower.setpoint() == 0.0
            && follower.appliedOutput() == 0.0
            && follower.velocity() == 0.0;
      }), () -> pairSummary("CAN fault did not revoke output"));
      assertEquals(
          "BLOCKED",
          SmartDashboard.getString(OperatorActionEvidence.stateKey(Action.REV), "MISSING"));

      // Recovery configuration is disabled-only. Keep the button held throughout recovery.
      enableDisabled();
      leader.setCanFault(false, false);
      assertTrue(
          await(8.0, activeContainer, () -> ConfiguredCanHardware.sparkDeviceIds().stream()
              .allMatch(id -> SparkMAXContainer.getDiagnosticSnapshotForId(id)
                  .map(snapshot -> snapshot.ready())
                  .orElse(false))),
          SparkMAXContainer::getDeviceAvailabilitySummary);
      assertOutputSafetyReadyDisabled(activeContainer);

      enableTeleop(true);
      for (int recoveryCycle = 0; recoveryCycle < 8; recoveryCycle++) {
        pump(activeContainer, 1);
        assertPairRawStopped();
        assertEquals(
            0.0,
            simulationHandle(SHOOTER_LEADER_ID).observe().setpoint(),
            "held input restarted the recovered leader during cycle " + recoveryCycle);
        assertEquals(
            "BLOCKED",
            SmartDashboard.getString(OperatorActionEvidence.stateKey(Action.REV), "MISSING"),
            "held recovery input lost its blocked evidence during cycle " + recoveryCycle);
      }
      assertFalse(
          pairEchoesRequestedVelocity(500.0),
          "a held button must not restart the recovered mechanism");

      // One neutral observation rearms the gate; only a new rising edge may restart motion.
      setDriverButton(ConfiguredOperatorControls.DRIVER_REV, false);
      pump(activeContainer, 3);
      assertOutputSafetyArmed(activeContainer);
      setDriverButton(ConfiguredOperatorControls.DRIVER_REV, true);
      assertTrue(await(3.0, activeContainer, () -> pairEchoesRequestedVelocity(500.0)),
          () -> pairSummary("release and fresh press did not rearm the binding"));
      assertEquals(
          "ACTIVE",
          SmartDashboard.getString(OperatorActionEvidence.stateKey(Action.REV), "MISSING"));

      // The same production binding layer also reaches the one-shot unreferenced diagnostic.
      // Command echo proves output acceptance, but deliberately reports no invented motion.
      setDriverButton(ConfiguredOperatorControls.DRIVER_REV, false);
      assertTrue(await(3.0, activeContainer, RobotContainerSimulationIntegrationTest::pairRawStopped),
          () -> pairSummary("recovered RevUpCommand.end did not stop the SPARK pair"));
      OutputStopBatch recoveredStop = SparkMAXContainer.requestOutputStops(
          SHOOTER_LEADER_ID, SHOOTER_FOLLOWER_ID);
      assertTrue(await(3.0, activeContainer, () -> recoveredStop.snapshot().confirmed()),
          () -> recoveredStop.snapshot().summary());

      // Ambiguous intake-path gestures are rejected as a group and require a full release.
      pump(activeContainer, 3);
      setDriverButton(ConfiguredOperatorControls.DRIVER_INTAKE, true);
      setDriverButton(ConfiguredOperatorControls.DRIVER_OUTPUT, true);
      pump(activeContainer, 2);
      assertActionBlocked(Action.INTAKE, "CONFLICTING_INTAKE_PATH_INPUTS");
      assertActionBlocked(Action.OUTPUT, "CONFLICTING_INTAKE_PATH_INPUTS");
      assertRawSetpointsZero(
          ConfiguredCanHardware.INTAKE_ACTUATOR_ID,
          ConfiguredCanHardware.INTAKE_ROLLER_ID,
          ConfiguredCanHardware.CONVEYOR_ID);
      setDriverButton(ConfiguredOperatorControls.DRIVER_INTAKE, false);
      pump(activeContainer, 2);
      assertEquals(
          "BLOCKED",
          SmartDashboard.getString(OperatorActionEvidence.stateKey(Action.OUTPUT), "MISSING"),
          "one held button must not restart after an ambiguous gesture");
      assertTrue(
          SmartDashboard.getString(OperatorActionEvidence.reasonKey(Action.OUTPUT), "")
              .contains("RELEASE_ALL_INTAKE_PATH_INPUTS_AFTER_CONFLICT"),
          "held conflict input must preserve the exact release-all recovery instruction");
      setDriverButton(ConfiguredOperatorControls.DRIVER_OUTPUT, false);
      pump(activeContainer, 3);

      // A fresh single press reaches the command, which reports the exact unreferenced blocker.
      setDriverButton(ConfiguredOperatorControls.DRIVER_INTAKE, true);
      pump(activeContainer, 2);
      assertActionBlocked(Action.INTAKE, "ACTUATOR_UNREFERENCED");
      assertRawSetpointsZero(
          ConfiguredCanHardware.INTAKE_ACTUATOR_ID,
          ConfiguredCanHardware.INTAKE_ROLLER_ID,
          ConfiguredCanHardware.CONVEYOR_ID);
      setDriverButton(ConfiguredOperatorControls.DRIVER_INTAKE, false);
      pump(activeContainer, 3);
      assertActionStopped(Action.INTAKE);
      assertRawControllersStopped(
          ConfiguredCanHardware.INTAKE_ACTUATOR_ID,
          ConfiguredCanHardware.INTAKE_ROLLER_ID,
          ConfiguredCanHardware.CONVEYOR_ID);

      // R1 has its own production route and must report the same unavailable reference without
      // allowing the roller or conveyor to move before the actuator is commissioned.
      setDriverButton(ConfiguredOperatorControls.DRIVER_OUTPUT, true);
      pump(activeContainer, 2);
      assertActionBlocked(Action.OUTPUT, "ACTUATOR_UNREFERENCED");
      assertRawControllersStopped(
          ConfiguredCanHardware.INTAKE_ACTUATOR_ID,
          ConfiguredCanHardware.INTAKE_ROLLER_ID,
          ConfiguredCanHardware.CONVEYOR_ID);
      setDriverButton(ConfiguredOperatorControls.DRIVER_OUTPUT, false);
      pump(activeContainer, 3);
      assertActionStopped(Action.OUTPUT);

      // A present operator controller owns retract; the driver fallback must explain why it was
      // ignored instead of silently scheduling a second route.
      setDriverButton(ConfiguredOperatorControls.DRIVER_RETRACT_FALLBACK, true);
      pump(activeContainer, 2);
      assertActionBlocked(Action.RETRACT, "DEDICATED_OPERATOR_PRESENT_USE_OPERATOR_L1");
      assertRawControllersStopped(ConfiguredCanHardware.INTAKE_ACTUATOR_ID);
      setDriverButton(ConfiguredOperatorControls.DRIVER_RETRACT_FALLBACK, false);
      pump(activeContainer, 3);
      assertActionStopped(Action.RETRACT);

      // The dedicated Operator L1 route reaches ID30 and reports the uncommissioned reference.
      setOperatorButton(ConfiguredOperatorControls.OPERATOR_RETRACT, true);
      pump(activeContainer, 2);
      assertActionBlocked(Action.RETRACT, "ACTUATOR_UNREFERENCED");
      assertRawControllersStopped(ConfiguredCanHardware.INTAKE_ACTUATOR_ID);
      setOperatorButton(ConfiguredOperatorControls.OPERATOR_RETRACT, false);
      pump(activeContainer, 3);
      assertActionStopped(Action.RETRACT);
      assertRawControllersStopped(ConfiguredCanHardware.INTAKE_ACTUATOR_ID);

      // Fire preserves all simultaneous interlock blockers instead of hiding the known feeder fault.
      setDriverButton(ConfiguredOperatorControls.DRIVER_FIRE, true);
      pump(activeContainer, 2);
      assertActionBlocked(Action.FIRE, "SHOOTER_NOT_READY");
      assertTrue(
          SmartDashboard.getString(OperatorActionEvidence.reasonKey(Action.FIRE), "")
              .contains("FEEDER_KNOWN_STALL"));
      assertRawSetpointsZero(
          ConfiguredCanHardware.FEEDER_ID, ConfiguredCanHardware.CONVEYOR_ID);
      setDriverButton(ConfiguredOperatorControls.DRIVER_FIRE, false);
      pump(activeContainer, 3);
      assertActionStopped(Action.FIRE);
      assertRawControllersStopped(
          ConfiguredCanHardware.FEEDER_ID, ConfiguredCanHardware.CONVEYOR_ID);

      // A populated maintenance controller owns auto-aim; the driver fallback is rejected with an
      // actionable reason before the dedicated route reports the reference failure without motion.
      setDriverButton(ConfiguredOperatorControls.DRIVER_AUTO_AIM_FALLBACK, true);
      pump(activeContainer, 2);
      assertActionBlocked(
          Action.AUTO_AIM, "DEDICATED_CONTROLLER_PRESENT_USE_DEDICATED_CONTROL");
      assertRawControllersStopped(ConfiguredCanHardware.TURRET_ID);
      setDriverButton(ConfiguredOperatorControls.DRIVER_AUTO_AIM_FALLBACK, false);
      pump(activeContainer, 3);
      assertActionStopped(Action.AUTO_AIM);

      setMaintenanceButton(ConfiguredOperatorControls.MAINTENANCE_AUTO_AIM, true);
      pump(activeContainer, 2);
      assertActionBlocked(Action.AUTO_AIM, "TURRET_UNREFERENCED");
      assertRawSetpointsZero(ConfiguredCanHardware.TURRET_ID);
      setMaintenanceButton(ConfiguredOperatorControls.MAINTENANCE_AUTO_AIM, false);
      pump(activeContainer, 3);
      assertActionStopped(Action.AUTO_AIM);
      assertRawControllersStopped(ConfiguredCanHardware.TURRET_ID);

      // A repaired ID32 may receive one isolated 3% pulse, but the known-stall block remains.
      prepareFeederDiagnosticSelection();
      enableDisabledTest();
      assertOutputSafetyReadyDisabled(activeContainer);
      double feederDiagnosticExpiresAt =
          Timer.getFPGATimestamp() + HardwareTestConstants.ARM_LIFETIME_SECONDS;
      SmartDashboard.putBoolean(
          ManualUnhomedActuatorDiagnosticCommand.FEEDER_REPAIR_VERIFIED_KEY, false);
      assertFalse(activeContainer.prepareUnhomedDiagnosticSession(
          Target.FEEDER,
          Direction.POSITIVE,
          feederDiagnosticExpiresAt),
          "ID32 repair attestation must be checked by the production session boundary");
      // Restore the complete exact-one selection after the negative boundary assertion. Other
      // dashboard-focused suites deliberately reset these shared NT keys in their cleanup.
      prepareFeederDiagnosticSelection();
      assertTrue(activeContainer.prepareUnhomedDiagnosticSession(
          Target.FEEDER,
          Direction.POSITIVE,
          feederDiagnosticExpiresAt),
          () -> "feeder prepare rejected: guard="
              + SmartDashboard.getString("Feeder/Manual Retest Guard", "MISSING")
              + " target=" + ManualUnhomedActuatorDiagnosticCommand.readExactlyOneTarget()
              + " targetBits=" + java.util.List.of(
                  SmartDashboard.getBoolean(
                      ManualUnhomedActuatorDiagnosticCommand.TARGET_INTAKE_KEY, false),
                  SmartDashboard.getBoolean(
                      ManualUnhomedActuatorDiagnosticCommand.TARGET_FEEDER_KEY, false),
                  SmartDashboard.getBoolean(
                      ManualUnhomedActuatorDiagnosticCommand.TARGET_CLIMBER_LEFT_KEY, false),
                  SmartDashboard.getBoolean(
                      ManualUnhomedActuatorDiagnosticCommand.TARGET_CLIMBER_RIGHT_KEY, false),
                  SmartDashboard.getBoolean(
                      ManualUnhomedActuatorDiagnosticCommand.TARGET_SHOOTER_KEY, false),
                  SmartDashboard.getBoolean(
                      ManualUnhomedActuatorDiagnosticCommand.TARGET_TURRET_KEY, false))
              + " direction=" + ManualUnhomedActuatorDiagnosticCommand.readExactlyOneDirection()
              + " repair=" + SmartDashboard.getBoolean(
                  ManualUnhomedActuatorDiagnosticCommand.FEEDER_REPAIR_VERIFIED_KEY, false)
              + " dsDisabled=" + DriverStation.isDisabled()
              + " dsTest=" + DriverStation.isTest()
              + " fms=" + DriverStation.isFMSAttached());
      activeContainer.stopAllPreservingPreparedUnhomedDiagnosticSession();
      activeContainer.armUnhomedDiagnosticSession(
          Target.FEEDER, Direction.POSITIVE, feederDiagnosticExpiresAt);
      enableTest();
      assertOutputSafetyArmed(activeContainer);
      setMaintenanceButton(ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_DEADMAN, true);
      setMaintenanceButton(ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_POSITIVE, true);
      assertTrue(await(3.0, activeContainer, () -> Math.abs(
          simulationHandle(ConfiguredCanHardware.FEEDER_ID).observe().appliedOutput() - 0.03)
          < 1e-9),
          () -> "manual feeder retest duty did not reach ID32: "
              + simulationHandle(ConfiguredCanHardware.FEEDER_ID).observe());
      SimulationSnapshot feederPulse =
          simulationHandle(ConfiguredCanHardware.FEEDER_ID).observe();
      assertEquals(0.0, feederPulse.velocity(), "raw echo must not invent feeder motion");
      assertEquals(0.0, feederPulse.motorCurrentAmps(), "raw echo must not invent feeder current");
      assertTrue(
          awaitWithoutScheduler(
              3.0,
              activeContainer,
              () -> !ProcessOutputSafety.isOutputAuthorized()
                  && rawControllerStopped(ConfiguredCanHardware.FEEDER_ID)),
          () -> "heartbeats without scheduler completion kept ID32 authorized: safety="
              + activeContainer.getOutputSafetySnapshot()
              + " raw=" + simulationHandle(ConfiguredCanHardware.FEEDER_ID).observe());
      assertEquals(
          "ROBOT_LOOP_HEARTBEAT_EXPIRED",
          activeContainer.getOutputSafetySnapshot().schedulerFaultReason());
      setMaintenanceButton(ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_POSITIVE, false);
      setMaintenanceButton(ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_DEADMAN, false);

      // The consumed arm cannot be reused by releasing and pressing the same gesture again.
      pump(activeContainer, 3);
      setMaintenanceButton(ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_DEADMAN, true);
      setMaintenanceButton(ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_POSITIVE, true);
      for (int reuseCycle = 0; reuseCycle < 8; reuseCycle++) {
        pump(activeContainer, 1);
        assertTrue(
            rawControllerStopped(ConfiguredCanHardware.FEEDER_ID),
            "consumed feeder arm restarted during cycle " + reuseCycle + ": "
                + simulationHandle(ConfiguredCanHardware.FEEDER_ID).observe());
      }
      setMaintenanceButton(ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_POSITIVE, false);
      setMaintenanceButton(ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_DEADMAN, false);
      enableDisabled();
      assertTrue(await(8.0, activeContainer, () -> ConfiguredCanHardware.sparkDeviceIds().stream()
          .allMatch(id -> SparkMAXContainer.getDiagnosticSnapshotForId(id)
              .map(snapshot -> snapshot.ready())
              .orElse(false))),
          SparkMAXContainer::getDeviceAvailabilitySummary);

      // A fresh arm also proves the independent 8A known-stall cutoff without scheduler ticks.
      prepareFeederDiagnosticSelection();
      enableDisabledTest();
      assertOutputSafetyReadyDisabled(activeContainer);
      double currentCutoffExpiresAt =
          Timer.getFPGATimestamp() + HardwareTestConstants.ARM_LIFETIME_SECONDS;
      assertTrue(activeContainer.prepareUnhomedDiagnosticSession(
          Target.FEEDER, Direction.POSITIVE, currentCutoffExpiresAt));
      activeContainer.stopAllPreservingPreparedUnhomedDiagnosticSession();
      activeContainer.armUnhomedDiagnosticSession(
          Target.FEEDER, Direction.POSITIVE, currentCutoffExpiresAt);
      enableTest();
      assertOutputSafetyArmed(activeContainer);
      setMaintenanceButton(ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_DEADMAN, true);
      setMaintenanceButton(ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_POSITIVE, true);
      assertTrue(await(3.0, activeContainer, () -> Math.abs(
          simulationHandle(ConfiguredCanHardware.FEEDER_ID).observe().appliedOutput() - 0.03)
          < 1e-9));
      SimulationSnapshot beforeCurrentInjection =
          simulationHandle(ConfiguredCanHardware.FEEDER_ID).observe();
      simulationHandle(ConfiguredCanHardware.FEEDER_ID).injectRawTelemetry(
          new SparkSimulationHandle.RawTelemetry(
              0.03,
              ManipulatorConstants.FEEDER_CURRENT_LIMIT_AMPS * 0.8,
              500.0,
              beforeCurrentInjection.position(),
              12.0));
      assertTrue(awaitWithoutScheduler(3.0, activeContainer, () -> SmartDashboard.getString(
          "Feeder/Manual Retest Guard", "")
              .contains("CURRENT_CUTOFF_STOP_REQUESTED")
          && simulationHandle(ConfiguredCanHardware.FEEDER_ID).observe().setpoint() == 0.0),
          () -> SmartDashboard.getString("Feeder/Manual Retest Guard", "MISSING_GUARD"));
      setMaintenanceButton(ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_POSITIVE, false);
      setMaintenanceButton(ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_DEADMAN, false);
      assertTrue(await(5.0, activeContainer, () -> rawControllerStopped(
          ConfiguredCanHardware.FEEDER_ID)),
          () -> "current cutoff did not finish zeroing ID32: "
              + simulationHandle(ConfiguredCanHardware.FEEDER_ID).observe());
      assertTrue(await(5.0, activeContainer, () -> SmartDashboard.getString(
          ManualUnhomedActuatorDiagnosticCommand.STATUS_KEY, "")
              .contains("STOP_CONFIRMED")),
          () -> SmartDashboard.getString(
              ManualUnhomedActuatorDiagnosticCommand.STATUS_KEY, "MISSING_STATUS"));

      // A successful isolated retest never unlocks the normal Fire path.
      enableTeleop(false);
      pump(activeContainer, 3);
      setDriverButton(ConfiguredOperatorControls.DRIVER_FIRE, true);
      pump(activeContainer, 2);
      assertActionBlocked(Action.FIRE, "FEEDER_KNOWN_STALL");
      assertTrue(
          rawControllerStopped(ConfiguredCanHardware.FEEDER_ID),
          () -> "normal Fire moved ID32 after retest: "
              + simulationHandle(ConfiguredCanHardware.FEEDER_ID).observe());
      setDriverButton(ConfiguredOperatorControls.DRIVER_FIRE, false);
      pump(activeContainer, 3);

      prepareTurretDiagnosticSelection();
      enableDisabledTest();
      assertOutputSafetyReadyDisabled(activeContainer);
      double turretDiagnosticExpiresAt =
          Timer.getFPGATimestamp() + HardwareTestConstants.ARM_LIFETIME_SECONDS;
      assertTrue(activeContainer.prepareUnhomedDiagnosticSession(
          Target.TURRET,
          Direction.POSITIVE,
          turretDiagnosticExpiresAt));
      activeContainer.stopAllPreservingPreparedUnhomedDiagnosticSession();
      activeContainer.armUnhomedDiagnosticSession(
          Target.TURRET, Direction.POSITIVE, turretDiagnosticExpiresAt);
      double referenceEpochBefore = SmartDashboard.getNumber("Turret/Continuity Epoch", -1.0);
      enableTest();
      assertOutputSafetyArmed(activeContainer);
      setMaintenanceButton(ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_DEADMAN, true);
      setMaintenanceButton(ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_POSITIVE, true);
      assertTrue(await(3.0, activeContainer, () -> Math.abs(
          simulationHandle(ConfiguredCanHardware.TURRET_ID).observe().appliedOutput() - 0.03)
          < 1e-9),
          () -> "manual turret duty did not reach ID39: "
              + simulationHandle(ConfiguredCanHardware.TURRET_ID).observe());
      SimulationSnapshot turretPulse =
          simulationHandle(ConfiguredCanHardware.TURRET_ID).observe();
      assertEquals(0.0, turretPulse.velocity(), "raw echo must not invent turret motion");
      assertEquals(0.0, turretPulse.motorCurrentAmps(), "raw echo must not invent turret current");
      setMaintenanceButton(ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_POSITIVE, false);
      setMaintenanceButton(ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_DEADMAN, false);
      assertTrue(await(5.0, activeContainer, () -> SmartDashboard.getString(
          ManualUnhomedActuatorDiagnosticCommand.STATUS_KEY, "")
              .contains("INTERLOCK_RELEASED_STOP_CONFIRMED")),
          () -> SmartDashboard.getString(
              ManualUnhomedActuatorDiagnosticCommand.STATUS_KEY, "MISSING_STATUS"));
      assertFalse(
          SmartDashboard.getString(ManualUnhomedActuatorDiagnosticCommand.STATUS_KEY, "")
              .contains("PASS_OBSERVED"),
          "raw command echo must not claim physical turret motion");
      assertEquals(
          "UNREFERENCED", SmartDashboard.getString("Turret/Reference State", "MISSING"));
      assertEquals(
          referenceEpochBefore,
          SmartDashboard.getNumber("Turret/Continuity Epoch", -2.0),
          "raw command echo must never mint or mutate a mechanism reference");

      // A blocked robot loop cannot refresh any normal or diagnostic output indefinitely. The
      // independent 5 ms supervisor revokes the process gate, orders zero after any in-flight
      // vendor call, and advances the raw simulation response without CommandScheduler.
      setMaintenanceButton(ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_POSITIVE, false);
      setMaintenanceButton(ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_DEADMAN, false);
      enableTeleop(false);
      assertOutputSafetyArmed(activeContainer);
      setDriverButton(ConfiguredOperatorControls.DRIVER_REV, true);
      assertTrue(await(3.0, activeContainer, () -> pairEchoesRequestedVelocity(500.0)),
          () -> pairSummary("heartbeat-timeout setup did not start Rev"));
      assertTrue(
          awaitWithoutRobotLoop(
              3.0,
              () -> !ProcessOutputSafety.isOutputAuthorized() && pairRawStopped()),
          () -> pairSummary("independent robot-loop heartbeat did not stop the pair"));
      assertTrue(
          activeContainer.getOutputSafetySnapshot().phase()
                  == RobotOutputSafetySupervisor.Phase.STOPPING
              || activeContainer.getOutputSafetySnapshot().phase()
                  == RobotOutputSafetySupervisor.Phase.TRIPPED,
          () -> activeContainer.getOutputSafetySnapshot().toString());
      assertFalse(SmartDashboard.getBoolean("Runtime/Scheduler Healthy", true));
    } finally {
      try {
        setDriverButton(ConfiguredOperatorControls.DRIVER_REV, false);
        enableDisabled();
        if (container != null) {
          scheduler.cancelAll();
          container.stopAll();
          int[] ids = ConfiguredCanHardware.sparkDeviceIds().stream().mapToInt(Integer::intValue)
              .toArray();
          OutputStopBatch finalStop = SparkMAXContainer.requestOutputStops(ids);
          RobotContainer closingContainer = container;
          assertTrue(await(5.0, closingContainer, () -> finalStop.snapshot().confirmed()),
              () -> finalStop.snapshot().summary());
        }
      } finally {
        if (container != null) {
          container.close();
          container.close();
        }
        scheduler.cancelAll();
        scheduler.getDefaultButtonLoop().clear();
        scheduler.setActiveButtonLoop(scheduler.getDefaultButtonLoop());
        scheduler.unregisterAllSubsystems();
        scheduler.clearComposedCommands();
        AutoBuilder.resetForTesting();
        NamedCommands.clearAll();
        try {
          assertTrue(awaitCleanup(5.0), "SPARK simulation registry did not quiesce");
        } finally {
          DriverStationSim.resetData();
          DriverStationSim.notifyNewData();
        }
      }
    }
  }

  @Test
  void hardwareSelfTestRunsEveryAutomaticStageAndConfirmsEveryRequiredStop()
      throws InterruptedException {
    CommandScheduler scheduler = CommandScheduler.getInstance();
    RobotContainer container = null;
    configureDisabledDriverStation();
    try {
      SparkMAXContainer.configureProcessDefaults();
      container = new RobotContainer();
      RobotContainer activeContainer = container;

      assertTrue(
          await(12.0, activeContainer, () -> ConfiguredCanHardware.sparkDeviceIds().stream()
              .allMatch(id -> SparkMAXContainer.getDiagnosticSnapshotForId(id)
                  .map(snapshot -> snapshot.ready())
                  .orElse(false))),
          SparkMAXContainer::getDeviceAvailabilitySummary);
      assertOutputSafetyReadyDisabled(activeContainer);

      enableTest();
      assertOutputSafetyArmed(activeContainer);
      Command selfTest = activeContainer.getHardwareSelfTestCommand();
      scheduler.schedule(selfTest);

      assertTrue(
          await(
              3.0,
              activeContainer,
              () -> SmartDashboard.getBoolean(HardwareSelfTestCommand.RUNNING_KEY, false)),
          "Hardware Self-Test did not start");
      HardwareSelfTestSwerveObservation swerveObservation =
          new HardwareSelfTestSwerveObservation();
      assertTrue(
          await(
              30.0,
              activeContainer,
              () -> {
                swerveObservation.observe(activeContainer);
                return !selfTest.isScheduled()
                    && !SmartDashboard.getBoolean(HardwareSelfTestCommand.RUNNING_KEY, true);
              }),
          () -> hardwareSelfTestSummary("Hardware Self-Test did not finish"));

      assertEquals(
          "ATTENTION_REQUIRED",
          SmartDashboard.getString("Hardware Self-Test/Overall", "MISSING"),
          () -> hardwareSelfTestSummary("unexpected overall result"));
      assertEquals(
          "NONE",
          SmartDashboard.getString("Hardware Self-Test/Abort Reason", "MISSING"),
          () -> hardwareSelfTestSummary("automatic sequence aborted"));
      assertEquals(
          "DESKTOP_SIMULATION_RAW_COMMAND_ECHO_NO_MECHANISM_PHYSICS",
          SmartDashboard.getString(HardwareSelfTestCommand.EVIDENCE_SOURCE_KEY, "MISSING"));
      assertEveryConfiguredCanResultReady();

      assertHardwareSelfTestResult("SPARK_ID30_INTAKE_ACTUATOR", "SKIPPED");
      assertHardwareSelfTestResult("SPARK_ID32_FEEDER", "BLOCKED_KNOWN_FAULT");
      assertHardwareSelfTestResult("SPARK_ID34_35_CLIMBER", "SKIPPED");
      assertHardwareSelfTestResult("SPARK_ID38_SHOOTER_ACTUATOR", "SKIPPED");
      assertHardwareSelfTestResult("SPARK_ID39_TURRET", "SKIPPED");

      assertHardwareSelfTestResult("SPARK_ID31_INTAKE_ROLLER", "INCONCLUSIVE_NO_MOTION");
      assertHardwareSelfTestResult("SPARK_ID33_CONVEYOR", "INCONCLUSIVE_NO_MOTION");
      assertHardwareSelfTestResult("SPARK_ID36_FLYWHEEL_LEADER", "INCONCLUSIVE_NO_MOTION");
      assertHardwareSelfTestResult(
          "SPARK_ID37_FLYWHEEL_FOLLOWER_PAIR", "INCONCLUSIVE_NO_MOTION");
      assertHardwareSelfTestResult("SPARK_ID37_FOLLOWER_ISOLATED", "INCONCLUSIVE_NO_MOTION");

      swerveObservation.assertEveryMotorReceivedEachStage();
      assertHardwareSelfTestReachedSimulationTerminalAssessment("SWERVE_FORWARD");
      assertHardwareSelfTestReachedSimulationTerminalAssessment("SWERVE_STRAFE");
      assertHardwareSelfTestReachedSimulationTerminalAssessment("SWERVE_ROTATE");

      assertHardwareSelfTestStopConfirmed("GLOBAL_START");
      assertHardwareSelfTestStopConfirmed("SPARK_ID31_INTAKE_ROLLER_SPARK_STOP");
      assertEquals(
          "NOT_RUN",
          SmartDashboard.getString(
              "Hardware Self-Test/SPARK_ID32_FEEDER_CONTROLLED_RETEST_SPARK_STOP/Stop Result",
              "MISSING"),
          "the compile-time-disabled automatic feeder retest must remain skipped");
      assertHardwareSelfTestStopConfirmed("SPARK_ID33_CONVEYOR_SPARK_STOP");
      assertHardwareSelfTestStopConfirmed("SPARK_ID36_37_FLYWHEEL_PAIR_SPARK_STOP");
      assertHardwareSelfTestStopConfirmed("SPARK_ID37_FOLLOWER_ISOLATED_SPARK_STOP");
      assertHardwareSelfTestStopConfirmed("SWERVE_FORWARD_SWERVE_STOP");
      assertHardwareSelfTestStopConfirmed("SWERVE_STRAFE_SWERVE_STOP");
      assertHardwareSelfTestStopConfirmed("SWERVE_ROTATE_SWERVE_STOP");
      assertHardwareSelfTestStopConfirmed("GLOBAL_END");

      int[] allSparkIds = ConfiguredCanHardware.sparkDeviceIds().stream()
          .mapToInt(Integer::intValue)
          .toArray();
      assertRawControllersStopped(allSparkIds);
      OutputStopBatch verifiedEndStop = SparkMAXContainer.requestOutputStops(allSparkIds);
      assertTrue(
          await(5.0, activeContainer, () -> verifiedEndStop.snapshot().confirmed()),
          () -> verifiedEndStop.snapshot().summary());
    } finally {
      try {
        enableDisabled();
        if (container != null) {
          scheduler.cancelAll();
          container.stopAll();
          int[] ids = ConfiguredCanHardware.sparkDeviceIds().stream().mapToInt(Integer::intValue)
              .toArray();
          OutputStopBatch finalStop = SparkMAXContainer.requestOutputStops(ids);
          RobotContainer closingContainer = container;
          assertTrue(await(5.0, closingContainer, () -> finalStop.snapshot().confirmed()),
              () -> finalStop.snapshot().summary());
        }
      } finally {
        if (container != null) {
          container.close();
          container.close();
        }
        scheduler.cancelAll();
        scheduler.getDefaultButtonLoop().clear();
        scheduler.setActiveButtonLoop(scheduler.getDefaultButtonLoop());
        scheduler.unregisterAllSubsystems();
        scheduler.clearComposedCommands();
        AutoBuilder.resetForTesting();
        NamedCommands.clearAll();
        try {
          assertTrue(awaitCleanup(5.0), "SPARK simulation registry did not quiesce");
        } finally {
          DriverStationSim.resetData();
          DriverStationSim.notifyNewData();
        }
      }
    }
  }

  @Test
  void normalDriveSeedWheelLockAndJumpBumpReachCtreOutputsAndStop()
      throws InterruptedException {
    CommandScheduler scheduler = CommandScheduler.getInstance();
    RobotContainer container = null;
    configureDisabledDriverStation();
    try {
      SparkMAXContainer.configureProcessDefaults();
      container = new RobotContainer();
      RobotContainer activeContainer = container;

      assertTrue(
          await(12.0, activeContainer, () -> ConfiguredCanHardware.sparkDeviceIds().stream()
              .allMatch(id -> SparkMAXContainer.getDiagnosticSnapshotForId(id)
                  .map(snapshot -> snapshot.ready())
                  .orElse(false))),
          SparkMAXContainer::getDeviceAvailabilitySummary);
      assertOutputSafetyReadyDisabled(activeContainer);

      // A stick held across enable must never inherit the previous neutral authorization.
      setDriverAxis(1, 0.65);
      enableTeleop(false);
      assertOutputSafetyArmed(activeContainer);
      assertTrue(
          await(3.0, activeContainer, () -> actionIsBlocked(Action.DRIVE, "RELEASE_TO_ARM")),
          () -> actionSummary(Action.DRIVE));
      Map<Integer, Map<Metric, Double>> heldEnableBaseline =
          captureSwerveOutputBaseline(activeContainer);
      assertTrue(
          await(3.0, activeContainer, () -> allSwerveMotorsHaveFreshZeroOutput(
              activeContainer, heldEnableBaseline)),
          () -> swerveOutputSummary(activeContainer, heldEnableBaseline));

      // One neutral sample arms the shared gate; only a later fresh deflection may drive.
      setDriverAxis(1, 0.0);
      assertTrue(
          await(2.0, activeContainer, () -> actionIsStopped(Action.DRIVE)),
          () -> actionSummary(Action.DRIVE));
      setDriverAxis(1, 0.65);
      assertTrue(
          await(3.0, activeContainer, () -> actionIsActive(Action.DRIVE)),
          () -> actionSummary(Action.DRIVE));
      Map<Integer, Map<Metric, Double>> driveBaseline =
          captureSwerveOutputBaseline(activeContainer);
      SwerveDriveState forwardStateBaseline = activeContainer.getSwerveDriveStateCopy();
      assertTrue(
          await(4.0, activeContainer, () -> actionIsActive(Action.DRIVE)
              && configuredDriveIdsHaveFreshNonzeroOutput(activeContainer, driveBaseline)
              && hasPostRequestTargetPattern(
                  activeContainer, forwardStateBaseline, TargetPattern.NEGATIVE_X_TRANSLATION)),
          () -> actionSummary(Action.DRIVE) + " "
              + swerveOutputSummary(activeContainer, driveBaseline) + " "
              + swerveStateSummary(activeContainer, forwardStateBaseline));

      // Exercise every normal drive axis independently. The post-request module target vectors are
      // converted back through the configured Tuner kinematics, so swapped axes or signs fail even
      // if all eight Talons still happen to receive fresh output frames.
      setDriverAxis(1, 0.0);
      assertTrue(await(2.0, activeContainer, () -> actionIsStopped(Action.DRIVE)),
          () -> actionSummary(Action.DRIVE));
      setDriverAxis(0, 0.55);
      assertTrue(
          await(3.0, activeContainer, () -> actionIsActive(Action.DRIVE)),
          () -> actionSummary(Action.DRIVE));
      Map<Integer, Map<Metric, Double>> strafeBaseline =
          captureSwerveOutputBaseline(activeContainer);
      SwerveDriveState strafeStateBaseline = activeContainer.getSwerveDriveStateCopy();
      assertTrue(
          await(4.0, activeContainer, () -> actionIsActive(Action.DRIVE)
              && configuredDriveIdsHaveFreshNonzeroOutput(activeContainer, strafeBaseline)
              && configuredSteerIdsHaveFreshNonzeroOutput(activeContainer, strafeBaseline)
              && hasPostRequestTargetPattern(
                  activeContainer, strafeStateBaseline, TargetPattern.NEGATIVE_Y_TRANSLATION)),
          () -> actionSummary(Action.DRIVE) + " "
              + swerveOutputSummary(activeContainer, strafeBaseline) + " "
              + swerveStateSummary(activeContainer, strafeStateBaseline));

      setDriverAxis(0, 0.0);
      assertTrue(await(2.0, activeContainer, () -> actionIsStopped(Action.DRIVE)),
          () -> actionSummary(Action.DRIVE));
      setDriverAxis(2, 0.50);
      assertTrue(
          await(3.0, activeContainer, () -> actionIsActive(Action.DRIVE)),
          () -> actionSummary(Action.DRIVE));
      Map<Integer, Map<Metric, Double>> rotationBaseline =
          captureSwerveOutputBaseline(activeContainer);
      SwerveDriveState rotationStateBaseline = activeContainer.getSwerveDriveStateCopy();
      assertTrue(
          await(4.0, activeContainer, () -> actionIsActive(Action.DRIVE)
              && configuredDriveIdsHaveFreshNonzeroOutput(activeContainer, rotationBaseline)
              && configuredSteerIdsHaveFreshNonzeroOutput(activeContainer, rotationBaseline)
              && hasPostRequestTargetPattern(
                  activeContainer, rotationStateBaseline, TargetPattern.NEGATIVE_ROTATION)),
          () -> actionSummary(Action.DRIVE) + " "
              + swerveOutputSummary(activeContainer, rotationBaseline) + " "
              + swerveStateSummary(activeContainer, rotationStateBaseline));

      setDriverAxis(2, 0.0);
      assertTrue(
          await(3.0, activeContainer, () -> actionIsStopped(Action.DRIVE)
              && drivetrainIsSettled(activeContainer.getSwerveDriveStateCopy())),
          () -> actionSummary(Action.DRIVE) + " state="
              + swerveStateSummary(activeContainer, rotationStateBaseline));

      // Seed is a pose operation, not a motor-output claim. It must preserve translation and the
      // raw gyro heading while resetting only the field-relative pose rotation on a later DAQ.
      SwerveDriveState seedInjectionBaseline = activeContainer.getSwerveDriveStateCopy();
      try (SimYawOverride seedYaw = new SimYawOverride(
          activeContainer, rawYawForPoseHeading(seedInjectionBaseline, 0.20))) {
        assertTrue(
            await(3.0, activeContainer, () -> hasPostRequestHeadingNear(
                activeContainer, seedInjectionBaseline, 0.20, 0.02)),
            () -> "simulation Pigeon did not reach the deterministic pre-seed heading: "
                + swerveStateSummary(activeContainer, seedInjectionBaseline));
        SwerveDriveState seedPoseBaseline = activeContainer.getSwerveDriveStateCopy();
        assertTrue(Math.abs(seedPoseBaseline.Pose.getRotation().getRadians()) > 0.10,
            () -> "pre-seed heading was not nonzero: " + describeState(seedPoseBaseline));
        setDriverButton(ConfiguredOperatorControls.DRIVER_SEED_FIELD, true);
        assertTrue(
            await(3.0, activeContainer, () -> actionIsCompleted(
                Action.SEED_FIELD, "HEADING_SEED_API_RETURNED_NOT_POSE_PROOF")),
            () -> actionSummary(Action.SEED_FIELD));
        SwerveDriveState seedCompletionBaseline = activeContainer.getSwerveDriveStateCopy();
        assertTrue(
            await(3.0, activeContainer, () -> seedAppliedToPostDaqState(
                activeContainer.getSwerveDriveStateCopy(),
                seedPoseBaseline,
                seedCompletionBaseline)),
            () -> actionSummary(Action.SEED_FIELD) + " requestReference="
                + describeState(seedPoseBaseline) + " "
                + swerveStateSummary(activeContainer, seedCompletionBaseline));
        setDriverButton(ConfiguredOperatorControls.DRIVER_SEED_FIELD, false);
        pump(activeContainer, 3);
      }

      setDriverAxis(1, 0.65);
      assertTrue(
          await(3.0, activeContainer, () -> actionIsActive(Action.DRIVE)),
          () -> actionSummary(Action.DRIVE));
      Map<Integer, Map<Metric, Double>> preSeedDriveBaseline =
          captureSwerveOutputBaseline(activeContainer);
      assertTrue(
          await(4.0, activeContainer, () -> actionIsActive(Action.DRIVE)
              && configuredDriveIdsHaveFreshNonzeroOutput(
                  activeContainer, preSeedDriveBaseline)),
          () -> actionSummary(Action.DRIVE) + " "
              + swerveOutputSummary(activeContainer, preSeedDriveBaseline));

      // Seeding changes the field-relative frame. The held stick must be neutralized and must not
      // restart in the new frame until it has been released and deflected again.
      setDriverButton(ConfiguredOperatorControls.DRIVER_SEED_FIELD, true);
      assertTrue(
          await(3.0, activeContainer, () -> actionIsCompleted(
              Action.SEED_FIELD, "HEADING_SEED_API_RETURNED_NOT_POSE_PROOF")),
          () -> actionSummary(Action.SEED_FIELD));
      Map<Integer, Map<Metric, Double>> seedBaseline =
          captureSwerveOutputBaseline(activeContainer);
      assertTrue(
          await(3.0, activeContainer, () -> actionIsBlocked(Action.DRIVE, "RELEASE_TO_ARM")
              && allSwerveMotorsHaveFreshZeroOutput(activeContainer, seedBaseline)),
          () -> actionSummary(Action.DRIVE) + " "
              + swerveOutputSummary(activeContainer, seedBaseline));
      setDriverButton(ConfiguredOperatorControls.DRIVER_SEED_FIELD, false);
      pump(activeContainer, 3);
      assertTrue(actionIsBlocked(Action.DRIVE, "RELEASE_TO_ARM"),
          () -> actionSummary(Action.DRIVE));

      setDriverAxis(1, 0.0);
      assertTrue(
          await(2.0, activeContainer, () -> actionIsStopped(Action.DRIVE)),
          () -> actionSummary(Action.DRIVE));
      setDriverAxis(1, 0.55);
      assertTrue(
          await(3.0, activeContainer, () -> actionIsActive(Action.DRIVE)),
          () -> actionSummary(Action.DRIVE));
      Map<Integer, Map<Metric, Double>> reseededDriveBaseline =
          captureSwerveOutputBaseline(activeContainer);
      assertTrue(
          await(4.0, activeContainer, () -> actionIsActive(Action.DRIVE)
              && configuredDriveIdsHaveFreshNonzeroOutput(
                  activeContainer, reseededDriveBaseline)),
          () -> actionSummary(Action.DRIVE) + " "
              + swerveOutputSummary(activeContainer, reseededDriveBaseline));

      // Touchpad has explicit priority over R3. Every steer Talon must receive a fresh brake
      // response, while the held R3 cannot auto-start after Touchpad is released.
      setDriverAxis(1, 0.0);
      assertTrue(
          await(2.0, activeContainer, () -> actionIsStopped(Action.DRIVE)),
          () -> actionSummary(Action.DRIVE));
      setDriverButton(ConfiguredOperatorControls.DRIVER_WHEEL_LOCK, true);
      setDriverButton(ConfiguredOperatorControls.DRIVER_JUMP_BUMP, true);
      assertTrue(
          await(4.0, activeContainer, () -> actionIsActive(
              Action.WHEEL_LOCK, "BRAKE_REQUEST_SUBMITTED_NOT_MOTION_PROOF")
              && actionIsBlocked(Action.JUMP_BUMP, "WHEEL_LOCK_PRIORITY")),
          () -> actionSummary(Action.WHEEL_LOCK) + " " + actionSummary(Action.JUMP_BUMP));
      Map<Integer, Map<Metric, Double>> wheelLockBaseline =
          captureSwerveOutputBaseline(activeContainer);
      SwerveDriveState wheelLockStateBaseline = activeContainer.getSwerveDriveStateCopy();
      assertTrue(
          await(4.0, activeContainer, () -> actionIsActive(
              Action.WHEEL_LOCK, "BRAKE_REQUEST_SUBMITTED_NOT_MOTION_PROOF")
              && actionIsBlocked(Action.JUMP_BUMP, "WHEEL_LOCK_PRIORITY")
              && configuredSteerIdsHaveFreshNonzeroOutput(
                  activeContainer, wheelLockBaseline)
              && hasPostRequestXLockTargets(activeContainer, wheelLockStateBaseline)),
          () -> actionSummary(Action.WHEEL_LOCK) + " " + actionSummary(Action.JUMP_BUMP)
              + " " + swerveOutputSummary(activeContainer, wheelLockBaseline) + " "
              + swerveStateSummary(activeContainer, wheelLockStateBaseline));

      setDriverButton(ConfiguredOperatorControls.DRIVER_WHEEL_LOCK, false);
      assertTrue(
          await(3.0, activeContainer, () -> actionIsStopped(Action.WHEEL_LOCK)
              && actionIsBlocked(Action.JUMP_BUMP, "WHEEL_LOCK_PRIORITY")),
          () -> actionSummary(Action.WHEEL_LOCK) + " " + actionSummary(Action.JUMP_BUMP));
      Map<Integer, Map<Metric, Double>> priorityReleaseBaseline =
          captureSwerveOutputBaseline(activeContainer);
      assertTrue(
          await(3.0, activeContainer, () -> actionIsBlocked(
              Action.JUMP_BUMP, "WHEEL_LOCK_PRIORITY")
              && configuredDriveIdsHaveFreshZeroOutput(
                  activeContainer, priorityReleaseBaseline)),
          () -> actionSummary(Action.JUMP_BUMP) + " "
              + swerveOutputSummary(activeContainer, priorityReleaseBaseline));

      // Give Jump Bump an unambiguous starting quadrant. The CTRE command-output model above proves
      // rotation requests, while this read-only simulation seam sets a deterministic Pigeon heading
      // so the snap target and convergence can be asserted without depending on simulated friction.
      setDriverButton(ConfiguredOperatorControls.DRIVER_JUMP_BUMP, false);
      assertTrue(
          await(3.0, activeContainer, () -> actionIsStopped(Action.JUMP_BUMP)),
          () -> actionSummary(Action.JUMP_BUMP));
      SwerveDriveState preJumpHeadingBaseline = activeContainer.getSwerveDriveStateCopy();
      try (SimYawOverride jumpYaw = new SimYawOverride(
          activeContainer, rawYawForPoseHeading(preJumpHeadingBaseline, 0.30))) {
        assertTrue(
            await(3.0, activeContainer, () -> hasPostRequestHeadingNear(
                activeContainer, preJumpHeadingBaseline, 0.30, 0.02)),
            () -> "simulation Pigeon heading did not reach the pre-snap value: "
                + swerveStateSummary(activeContainer, preJumpHeadingBaseline));

        SwerveDriveState jumpStartState = activeContainer.getSwerveDriveStateCopy();
        assertTrue(jumpStartState != null && jumpStartState.Pose != null,
            () -> describeState(jumpStartState));
        double jumpStartHeading = jumpStartState.Pose.getRotation().getRadians();
        assertTrue(Double.isFinite(jumpStartHeading), () -> describeState(jumpStartState));
        assertTrue(distanceFromBumpSelectionBoundary(jumpStartHeading) > 0.12,
            () -> "Jump Bump start remained too close to a quadrant boundary: "
                + describeState(jumpStartState));
        double expectedJumpHeading = closestDiagonalHeading(jumpStartHeading);
        assertTrue(Math.abs(moduloAngleError(expectedJumpHeading, jumpStartHeading)) > 0.12,
            () -> "Jump Bump start did not leave a meaningful snap error: start="
                + jumpStartHeading + " expected=" + expectedJumpHeading);

        // A completely new R3 press may run Jump Bump. Capture both baselines only after ACTIVE,
        // then require later output/DAQ progress and convergence while R3 remains held.
        startJumpBumpAfterAtMostOneRecoveredDesktopHealthTransient(activeContainer);
        Map<Integer, Map<Metric, Double>> jumpBaseline =
            captureSwerveOutputBaseline(activeContainer);
        SwerveDriveState jumpStateBaseline = activeContainer.getSwerveDriveStateCopy();
        assertTrue(
            await(4.0, activeContainer, () -> actionIsActive(
                Action.JUMP_BUMP, "BUMP_HEADING_API_RETURNED_NOT_REQUEST_OR_MOTION_PROOF")
                && configuredDriveIdsHaveFreshNonzeroOutput(activeContainer, jumpBaseline)
                && hasPostRequestRotationToward(
                    activeContainer,
                    jumpStateBaseline,
                    jumpStartHeading,
                    expectedJumpHeading)),
            () -> actionSummary(Action.JUMP_BUMP) + " "
                + swerveOutputSummary(activeContainer, jumpBaseline) + " "
                + swerveStateSummary(activeContainer, jumpStateBaseline));
        jumpYaw.setRawYaw(rawYawForPoseHeading(
            activeContainer.getSwerveDriveStateCopy(), expectedJumpHeading));
        assertTrue(
            await(3.0, activeContainer, () -> actionIsActive(
                Action.JUMP_BUMP, "BUMP_HEADING_API_RETURNED_NOT_REQUEST_OR_MOTION_PROOF")
                && headingConvergedToPostRequestState(
                    activeContainer.getSwerveDriveStateCopy(),
                    jumpStateBaseline,
                    expectedJumpHeading,
                    Math.toRadians(3.0))),
            () -> "R3-held Jump Bump did not converge modulo 2pi: expected="
                + expectedJumpHeading + " " + actionSummary(Action.JUMP_BUMP) + " "
                + swerveStateSummary(activeContainer, jumpStateBaseline));
      }

      setDriverButton(ConfiguredOperatorControls.DRIVER_JUMP_BUMP, false);
      assertTrue(
          await(3.0, activeContainer, () -> actionIsStopped(Action.JUMP_BUMP)),
          () -> actionSummary(Action.JUMP_BUMP));
      Map<Integer, Map<Metric, Double>> jumpReleaseBaseline =
          captureSwerveOutputBaseline(activeContainer);
      assertTrue(
          await(3.0, activeContainer, () -> configuredDriveIdsHaveFreshZeroOutput(
              activeContainer, jumpReleaseBaseline)),
          () -> actionSummary(Action.JUMP_BUMP) + " "
              + swerveOutputSummary(activeContainer, jumpReleaseBaseline));

      // Enabled neutral driving may retain a small steer-angle hold. Disabled OutputSafety is the
      // authoritative all-eight-motor neutral barrier and must still complete after the sequence.
      enableDisabled();
      assertOutputSafetyReadyDisabled(activeContainer);
    } finally {
      try {
        setDriverAxis(0, 0.0);
        setDriverAxis(1, 0.0);
        setDriverAxis(2, 0.0);
        setDriverButton(ConfiguredOperatorControls.DRIVER_SEED_FIELD, false);
        setDriverButton(ConfiguredOperatorControls.DRIVER_WHEEL_LOCK, false);
        setDriverButton(ConfiguredOperatorControls.DRIVER_JUMP_BUMP, false);
        enableDisabled();
        if (container != null) {
          scheduler.cancelAll();
          container.stopAll();
          int[] ids = ConfiguredCanHardware.sparkDeviceIds().stream().mapToInt(Integer::intValue)
              .toArray();
          OutputStopBatch finalStop = SparkMAXContainer.requestOutputStops(ids);
          RobotContainer closingContainer = container;
          assertTrue(await(5.0, closingContainer, () -> finalStop.snapshot().confirmed()),
              () -> finalStop.snapshot().summary());
        }
      } finally {
        if (container != null) {
          container.close();
          container.close();
        }
        scheduler.cancelAll();
        scheduler.getDefaultButtonLoop().clear();
        scheduler.setActiveButtonLoop(scheduler.getDefaultButtonLoop());
        scheduler.unregisterAllSubsystems();
        scheduler.clearComposedCommands();
        AutoBuilder.resetForTesting();
        NamedCommands.clearAll();
        try {
          assertTrue(awaitCleanup(5.0), "SPARK simulation registry did not quiesce");
        } finally {
          DriverStationSim.resetData();
          DriverStationSim.notifyNewData();
        }
      }
    }
  }

  private static void assertHardwareSelfTestResult(String target, String expected) {
    assertEquals(
        expected,
        SmartDashboard.getString(
            "Hardware Self-Test/" + target + "/Motion Result", "MISSING"),
        () -> hardwareSelfTestSummary("unexpected result for " + target));
  }

  private static void assertHardwareSelfTestReachedSimulationTerminalAssessment(String target) {
    String result = SmartDashboard.getString(
        "Hardware Self-Test/" + target + "/Motion Result", "MISSING");
    if (result.equals("PASS_OBSERVED") || result.equals("INCONCLUSIVE_NO_MOTION")) {
      return;
    }
    String reason = SmartDashboard.getString(
        "Hardware Self-Test/" + target + "/Reason", "MISSING");
    assertTrue(
        result.equals("PASS_OBSERVED") || result.equals("INCONCLUSIVE_NO_MOTION"),
        () -> hardwareSelfTestSummary(
        target + " did not produce valid terminal evidence reason=" + reason));
  }

  private static void assertHardwareSelfTestStopConfirmed(String stopName) {
    assertEquals(
        "CONFIRMED",
        SmartDashboard.getString(
            "Hardware Self-Test/" + stopName + "/Stop Result", "MISSING"),
        () -> hardwareSelfTestSummary("stop was not confirmed for " + stopName));
  }

  private static void assertEveryConfiguredCanResultReady() {
    for (var device : ConfiguredCanHardware.devices()) {
      String vendorPrefix = device.vendor() == ConfiguredCanHardware.Vendor.REV
          ? "SPARK_ID" : "CTRE_ID";
      String topicPrefix = "Hardware Self-Test/" + vendorPrefix + device.canId() + "/";
      assertEquals(
          "PASS_READY",
          SmartDashboard.getString(topicPrefix + "CAN Result", "MISSING"),
          () -> "CAN " + device.canId() + " result/reason="
              + SmartDashboard.getString(topicPrefix + "CAN Result", "MISSING") + "/"
              + SmartDashboard.getString(topicPrefix + "CAN Reason", "MISSING"));
      assertFalse(
          SmartDashboard.getString(topicPrefix + "CAN Reason", "").isBlank(),
          "CAN " + device.canId() + " must retain its per-device evidence reason");
    }
    String allCan = SmartDashboard.getString("Hardware Self-Test/All CAN Results", "MISSING");
    for (int canId : ConfiguredCanHardware.allDeviceIds()) {
      assertTrue(allCan.contains(canId + "=PASS_READY("),
          () -> "all-CAN summary omitted ID" + canId + ": " + allCan);
    }
  }

  private static String hardwareSelfTestSummary(String prefix) {
    return prefix
        + " overall=" + SmartDashboard.getString("Hardware Self-Test/Overall", "MISSING")
        + " abort=" + SmartDashboard.getString("Hardware Self-Test/Abort Reason", "MISSING")
        + " results=" + SmartDashboard.getString("Hardware Self-Test/Results", "MISSING");
  }

  private static final class HardwareSelfTestSwerveObservation {
    private static final Set<Integer> DRIVE_IDS = Set.copyOf(
        ConfiguredCanHardware.swerveDriveIds());
    private static final Set<Integer> STEER_IDS = Set.copyOf(
        ConfiguredCanHardware.swerveSteerIds());
    private static final Set<Integer> MOTOR_IDS = configuredSwerveMotorIds();
    private static final java.util.List<String> STAGES = java.util.List.of(
        "SWERVE_FORWARD", "SWERVE_STRAFE", "SWERVE_ROTATE");
    private final Map<String, Set<Integer>> nonzeroDriveIdsByStage = new LinkedHashMap<>();
    private final Map<String, Set<Integer>> nonzeroSteerIdsByStage = new LinkedHashMap<>();
    private final Map<String, Set<Integer>> freshOutputIdsByStage = new LinkedHashMap<>();
    private final Map<String, Map<Integer, Map<Metric, Double>>> baselines =
        new LinkedHashMap<>();

    HardwareSelfTestSwerveObservation() {
      STAGES.forEach(stage -> {
        nonzeroDriveIdsByStage.put(stage, new LinkedHashSet<>());
        nonzeroSteerIdsByStage.put(stage, new LinkedHashSet<>());
        freshOutputIdsByStage.put(stage, new LinkedHashSet<>());
      });
    }

    void observe(RobotContainer container) {
      String stage = activeStage();
      if (stage == null) {
        return;
      }
      var snapshots = container.getSwerveDeviceEvidenceSnapshots();
      if (!baselines.containsKey(stage)) {
        Map<Integer, Map<Metric, Double>> stageBaseline = new LinkedHashMap<>();
        for (var snapshot : snapshots) {
          if (MOTOR_IDS.contains(snapshot.canId())) {
            stageBaseline.put(
                snapshot.canId(),
                Map.of(
                    Metric.DUTY_CYCLE,
                    progressTimestamp(snapshot, Metric.DUTY_CYCLE),
                    Metric.MOTOR_VOLTAGE_VOLTS,
                    progressTimestamp(snapshot, Metric.MOTOR_VOLTAGE_VOLTS)));
          }
        }
        baselines.put(stage, stageBaseline);
        return;
      }

      Map<Integer, Map<Metric, Double>> stageBaseline = baselines.get(stage);
      for (var snapshot : snapshots) {
        if (!MOTOR_IDS.contains(snapshot.canId())) {
          continue;
        }
        Map<Metric, Double> motorBaseline = stageBaseline.get(snapshot.canId());
        if (motorBaseline == null
            || !advancedAfterBaseline(snapshot, Metric.DUTY_CYCLE, motorBaseline)
            || !advancedAfterBaseline(snapshot, Metric.MOTOR_VOLTAGE_VOLTS, motorBaseline)) {
          continue;
        }
        freshOutputIdsByStage.get(stage).add(snapshot.canId());
        double duty = snapshot.value(Metric.DUTY_CYCLE).orElse(0.0);
        double voltage = snapshot.value(Metric.MOTOR_VOLTAGE_VOLTS).orElse(0.0);
        if (DRIVE_IDS.contains(snapshot.canId())
            && (Math.abs(duty) > 1e-4 || Math.abs(voltage) > 1e-3)) {
          nonzeroDriveIdsByStage.get(stage).add(snapshot.canId());
        }
        if (STEER_IDS.contains(snapshot.canId())
            && (Math.abs(duty) > 1e-4 || Math.abs(voltage) > 1e-3)) {
          nonzeroSteerIdsByStage.get(stage).add(snapshot.canId());
        }
      }
    }

    void assertEveryMotorReceivedEachStage() {
      for (String stage : STAGES) {
        assertEquals(
            MOTOR_IDS,
            freshOutputIdsByStage.get(stage),
            () -> stage + " did not produce post-stage output frames for all eight Talons: "
                + freshOutputIdsByStage);
        assertEquals(
            DRIVE_IDS,
            nonzeroDriveIdsByStage.get(stage),
            () -> stage + " did not reach every drive Talon with post-stage nonzero evidence: "
                + nonzeroDriveIdsByStage);
      }
      for (String stage : java.util.List.of("SWERVE_STRAFE", "SWERVE_ROTATE")) {
        assertEquals(
            STEER_IDS,
            nonzeroSteerIdsByStage.get(stage),
            () -> stage + " did not reach every steer Talon with post-stage nonzero evidence: "
                + nonzeroSteerIdsByStage);
      }
    }

    private static boolean advancedAfterBaseline(
        CtreDeviceEvidence.Snapshot snapshot,
        Metric metric,
        Map<Metric, Double> baseline) {
      double before = baseline.getOrDefault(metric, Double.NaN);
      double after = progressTimestamp(snapshot, metric);
      return Double.isFinite(before) && Double.isFinite(after) && after > before;
    }

    private static double progressTimestamp(
        CtreDeviceEvidence.Snapshot snapshot, Metric metric) {
      return snapshot.observations().stream()
          .filter(observation -> observation.metric() == metric)
          .filter(CtreDeviceEvidence.SignalObservation::fresh)
          .mapToDouble(CtreDeviceEvidence.SignalObservation::progressTimestampSeconds)
          .findFirst()
          .orElse(Double.NaN);
    }

    private static Set<Integer> configuredSwerveMotorIds() {
      Set<Integer> ids = new LinkedHashSet<>(ConfiguredCanHardware.swerveDriveIds());
      ids.addAll(ConfiguredCanHardware.swerveSteerIds());
      return Set.copyOf(ids);
    }

    private static String activeStage() {
      if (stopResult("SPARK_ID37_FOLLOWER_ISOLATED_SPARK_STOP").equals("CONFIRMED")
          && motionResult("SWERVE_FORWARD").equals("NOT_RUN")) {
        return "SWERVE_FORWARD";
      }
      if (stopResult("SWERVE_FORWARD_SWERVE_STOP").equals("CONFIRMED")
          && motionResult("SWERVE_STRAFE").equals("NOT_RUN")) {
        return "SWERVE_STRAFE";
      }
      if (stopResult("SWERVE_STRAFE_SWERVE_STOP").equals("CONFIRMED")
          && motionResult("SWERVE_ROTATE").equals("NOT_RUN")) {
        return "SWERVE_ROTATE";
      }
      return null;
    }

    private static String stopResult(String name) {
      return SmartDashboard.getString(
          "Hardware Self-Test/" + name + "/Stop Result", "MISSING");
    }

    private static String motionResult(String name) {
      return SmartDashboard.getString(
          "Hardware Self-Test/" + name + "/Motion Result", "MISSING");
    }
  }

  private static boolean pairEchoesRequestedVelocity(double expected) {
    SimulationSnapshot leader = simulationHandle(SHOOTER_LEADER_ID).observe();
    SimulationSnapshot follower = simulationHandle(SHOOTER_FOLLOWER_ID).observe();
    return Math.abs(leader.setpoint() - expected) < 1e-9
        && Math.abs(leader.velocity() - expected) < 1e-9
        && Math.abs(follower.velocity() + expected) < 1e-9;
  }

  private enum TargetPattern {
    NEGATIVE_X_TRANSLATION,
    NEGATIVE_Y_TRANSLATION,
    NEGATIVE_ROTATION,
    POSITIVE_ROTATION
  }

  private static boolean hasPostRequestTargetPattern(
      RobotContainer container, SwerveDriveState baseline, TargetPattern pattern) {
    SwerveDriveState state = container.getSwerveDriveStateCopy();
    if (!isPostRequestState(state, baseline) || !hasCompleteModuleTargets(state)) {
      return false;
    }
    ChassisSpeeds target = SWERVE_KINEMATICS.toChassisSpeeds(state.ModuleTargets);
    if (!Double.isFinite(target.vxMetersPerSecond)
        || !Double.isFinite(target.vyMetersPerSecond)
        || !Double.isFinite(target.omegaRadiansPerSecond)) {
      return false;
    }
    double maximumTranslation = TunerConstants.kSpeedAt12Volts.baseUnitMagnitude()
        * DebugConstants.MAX_SWERVE_TRANSLATION_FRACTION + 1e-6;
    double maximumRotation = DebugConstants.MAX_SWERVE_ROTATION_RADIANS_PER_SECOND + 1e-6;
    boolean withinFixedSafetyCap = Math.hypot(
        target.vxMetersPerSecond, target.vyMetersPerSecond) <= maximumTranslation
        && Math.abs(target.omegaRadiansPerSecond) <= maximumRotation;
    return withinFixedSafetyCap && switch (pattern) {
      case NEGATIVE_X_TRANSLATION -> target.vxMetersPerSecond < -0.02
          && Math.abs(target.vyMetersPerSecond) <= 0.005
          && Math.abs(target.omegaRadiansPerSecond) <= 0.01;
      case NEGATIVE_Y_TRANSLATION -> target.vyMetersPerSecond < -0.02
          && Math.abs(target.vxMetersPerSecond) <= 0.005
          && Math.abs(target.omegaRadiansPerSecond) <= 0.01;
      case NEGATIVE_ROTATION -> target.omegaRadiansPerSecond < -0.01
          && Math.abs(target.vxMetersPerSecond) <= 0.005
          && Math.abs(target.vyMetersPerSecond) <= 0.005;
      case POSITIVE_ROTATION -> target.omegaRadiansPerSecond > 0.005
          && Math.abs(target.vxMetersPerSecond) <= 0.005
          && Math.abs(target.vyMetersPerSecond) <= 0.005;
    };
  }

  private static boolean hasPostRequestXLockTargets(
      RobotContainer container, SwerveDriveState baseline) {
    SwerveDriveState state = container.getSwerveDriveStateCopy();
    if (!isPostRequestState(state, baseline) || !hasCompleteModuleTargets(state)) {
      return false;
    }
    for (int index = 0; index < state.ModuleTargets.length; index++) {
      SwerveModuleState target = state.ModuleTargets[index];
      Translation2d location = SWERVE_MODULE_LOCATIONS[index];
      double inwardAngle = Math.atan2(-location.getY(), -location.getX());
      double lineAngleError = Math.IEEEremainder(
          target.angle.getRadians() - inwardAngle, Math.PI);
      if (Math.abs(target.speedMetersPerSecond) > 1e-6
          || Math.abs(lineAngleError) > Math.toRadians(2.0)) {
        return false;
      }
    }
    return true;
  }

  private static boolean hasPostRequestHeadingNear(
      RobotContainer container,
      SwerveDriveState baseline,
      double expectedHeadingRadians,
      double toleranceRadians) {
    SwerveDriveState state = container.getSwerveDriveStateCopy();
    return isPostRequestState(state, baseline)
        && state.Pose != null
        && Double.isFinite(state.Pose.getRotation().getRadians())
        && Math.abs(moduloAngleError(
            expectedHeadingRadians,
            state.Pose.getRotation().getRadians())) <= toleranceRadians;
  }

  private static boolean hasPostRequestRotationToward(
      RobotContainer container,
      SwerveDriveState baseline,
      double requestHeadingRadians,
      double expectedHeadingRadians) {
    SwerveDriveState state = container.getSwerveDriveStateCopy();
    if (!isPostRequestState(state, baseline) || !hasCompleteModuleTargets(state)) {
      return false;
    }
    ChassisSpeeds target = SWERVE_KINEMATICS.toChassisSpeeds(state.ModuleTargets);
    double expectedError = moduloAngleError(expectedHeadingRadians, requestHeadingRadians);
    return Double.isFinite(target.omegaRadiansPerSecond)
        && Math.abs(target.vxMetersPerSecond) <= 0.005
        && Math.abs(target.vyMetersPerSecond) <= 0.005
        && Math.abs(target.omegaRadiansPerSecond) > 0.005
        && target.omegaRadiansPerSecond * expectedError > 0.0;
  }

  private static boolean seedAppliedToPostDaqState(
      SwerveDriveState state,
      SwerveDriveState requestReference,
      SwerveDriveState completionBaseline) {
    return requestReference != null
        && requestReference.Pose != null
        && requestReference.RawHeading != null
        && isPostRequestState(state, completionBaseline)
        && state.Pose != null
        && state.RawHeading != null
        && Math.abs(state.Pose.getRotation().getRadians()) <= 0.01
        && state.Pose.getTranslation().getDistance(
            requestReference.Pose.getTranslation()) <= 0.01
        && Math.abs(Math.IEEEremainder(
            state.RawHeading.minus(requestReference.RawHeading).getRadians(),
            2.0 * Math.PI)) <= 0.01;
  }

  private static boolean headingConvergedToPostRequestState(
      SwerveDriveState state,
      SwerveDriveState baseline,
      double expectedHeadingRadians,
      double toleranceRadians) {
    return isPostRequestState(state, baseline)
        && state.Pose != null
        && Double.isFinite(expectedHeadingRadians)
        && Double.isFinite(toleranceRadians)
        && toleranceRadians >= 0.0
        && Math.abs(moduloAngleError(
            expectedHeadingRadians,
            state.Pose.getRotation().getRadians())) <= toleranceRadians;
  }

  private static double closestDiagonalHeading(double currentHeadingRadians) {
    if (!Double.isFinite(currentHeadingRadians)) {
      return Double.NaN;
    }
    double wrappedHeading = Math.IEEEremainder(currentHeadingRadians, 2.0 * Math.PI);
    double quadrantStart = Math.floor(wrappedHeading / (Math.PI / 2.0)) * (Math.PI / 2.0);
    return Math.IEEEremainder(quadrantStart + Math.PI / 4.0, 2.0 * Math.PI);
  }

  private static double distanceFromBumpSelectionBoundary(double headingRadians) {
    return Math.abs(Math.IEEEremainder(headingRadians, Math.PI / 2.0));
  }

  private static double moduloAngleError(double expectedRadians, double actualRadians) {
    return Math.IEEEremainder(expectedRadians - actualRadians, 2.0 * Math.PI);
  }

  private static double rawYawForPoseHeading(
      SwerveDriveState referenceState, double requestedPoseHeadingRadians) {
    if (referenceState == null
        || referenceState.Pose == null
        || referenceState.RawHeading == null
        || !Double.isFinite(requestedPoseHeadingRadians)) {
      return Double.NaN;
    }
    double poseHeading = referenceState.Pose.getRotation().getRadians();
    double rawHeading = referenceState.RawHeading.getRadians();
    if (!Double.isFinite(poseHeading) || !Double.isFinite(rawHeading)) {
      return Double.NaN;
    }
    return Math.IEEEremainder(
        rawHeading + requestedPoseHeadingRadians - poseHeading,
        2.0 * Math.PI);
  }

  private static final class SimYawOverride implements AutoCloseable {
    private final RobotContainer container;
    private final Notifier notifier;
    private volatile double rawYawRadians;

    SimYawOverride(RobotContainer container, double initialRawYawRadians) {
      this.container = container;
      if (!setRawYaw(initialRawYawRadians)) {
        throw new IllegalStateException("simulation Pigeon rejected the yaw override");
      }
      notifier = new Notifier(this::apply);
      notifier.startPeriodic(0.001);
    }

    boolean setRawYaw(double requestedRawYawRadians) {
      if (!Double.isFinite(requestedRawYawRadians)) {
        return false;
      }
      rawYawRadians = Math.IEEEremainder(requestedRawYawRadians, 2.0 * Math.PI);
      return apply();
    }

    private boolean apply() {
      try {
        return container.setSwerveSimRawYawForTesting(rawYawRadians);
      } catch (RuntimeException exception) {
        return false;
      }
    }

    @Override
    public void close() {
      notifier.close();
    }
  }

  private static boolean drivetrainIsSettled(SwerveDriveState state) {
    return state != null
        && state.Speeds != null
        && Math.abs(state.Speeds.vxMetersPerSecond) <= 0.02
        && Math.abs(state.Speeds.vyMetersPerSecond) <= 0.02
        && Math.abs(state.Speeds.omegaRadiansPerSecond) <= 0.03;
  }

  private static boolean isPostRequestState(
      SwerveDriveState state, SwerveDriveState baseline) {
    return state != null
        && baseline != null
        && Double.isFinite(state.Timestamp)
        && Double.isFinite(baseline.Timestamp)
        && state.Timestamp > baseline.Timestamp
        && state.SuccessfulDaqs > baseline.SuccessfulDaqs;
  }

  private static boolean hasCompleteModuleTargets(SwerveDriveState state) {
    if (state.ModuleTargets == null
        || state.ModuleTargets.length != SWERVE_MODULE_LOCATIONS.length) {
      return false;
    }
    for (SwerveModuleState target : state.ModuleTargets) {
      if (target == null
          || target.angle == null
          || !Double.isFinite(target.speedMetersPerSecond)
          || !Double.isFinite(target.angle.getRadians())) {
        return false;
      }
    }
    return true;
  }

  private static String swerveStateSummary(
      RobotContainer container, SwerveDriveState baseline) {
    return "stateBaseline=" + describeState(baseline)
        + " stateNow=" + describeState(container.getSwerveDriveStateCopy());
  }

  private static String describeState(SwerveDriveState state) {
    if (state == null) {
      return "null";
    }
    String targets = state.ModuleTargets == null
        ? "null" : java.util.Arrays.toString(state.ModuleTargets);
    ChassisSpeeds targetSpeeds = hasCompleteModuleTargets(state)
        ? SWERVE_KINEMATICS.toChassisSpeeds(state.ModuleTargets) : null;
    return "{timestamp=" + state.Timestamp
        + ",daqs=" + state.SuccessfulDaqs
        + ",pose=" + state.Pose
        + ",rawHeading=" + state.RawHeading
        + ",speeds=" + state.Speeds
        + ",targetSpeeds=" + targetSpeeds
        + ",targets=" + targets + "}";
  }

  private static boolean actionIsActive(Action action) {
    return "ACTIVE".equals(SmartDashboard.getString(
        OperatorActionEvidence.stateKey(action), "MISSING"));
  }

  private static boolean actionIsActive(Action action, String reason) {
    return actionIsActive(action) && SmartDashboard.getString(
        OperatorActionEvidence.reasonKey(action), "").contains(reason);
  }

  private static boolean actionIsBlocked(Action action, String reason) {
    return "BLOCKED".equals(SmartDashboard.getString(
        OperatorActionEvidence.stateKey(action), "MISSING"))
        && SmartDashboard.getString(
            OperatorActionEvidence.reasonKey(action), "").contains(reason);
  }

  private static boolean actionIsStopped(Action action) {
    return "STOPPED".equals(SmartDashboard.getString(
        OperatorActionEvidence.stateKey(action), "MISSING"));
  }

  private static boolean actionIsCompleted(Action action, String reason) {
    return "COMPLETED".equals(SmartDashboard.getString(
        OperatorActionEvidence.stateKey(action), "MISSING"))
        && SmartDashboard.getString(
            OperatorActionEvidence.reasonKey(action), "").contains(reason);
  }

  private static String actionSummary(Action action) {
    return action + "=" + SmartDashboard.getString(
        OperatorActionEvidence.stateKey(action), "MISSING") + "/"
        + SmartDashboard.getString(
            OperatorActionEvidence.reasonKey(action), "MISSING");
  }

  private static void startJumpBumpAfterAtMostOneRecoveredDesktopHealthTransient(
      RobotContainer container) throws InterruptedException {
    setDriverButton(ConfiguredOperatorControls.DRIVER_JUMP_BUMP, true);
    if (await(4.0, container, () -> actionIsActive(
        Action.JUMP_BUMP, "BUMP_HEADING_API_RETURNED_NOT_REQUEST_OR_MOTION_PROOF"))) {
      return;
    }

    // Phoenix desktop status updates can invalidate aggregate readiness for one scheduler poll.
    // Production correctly blocks and rearms on that transition. Exercise that exact recovery once
    // instead of weakening the action assertion or treating a persistent unhealthy state as success.
    assertTrue(
        actionIsBlocked(Action.JUMP_BUMP, "DRIVETRAIN_UNHEALTHY"),
        () -> actionSummary(Action.JUMP_BUMP));
    setDriverButton(ConfiguredOperatorControls.DRIVER_JUMP_BUMP, false);
    assertTrue(
        await(4.0, container, () -> actionIsStopped(Action.JUMP_BUMP)
            && everyConfiguredCtreSnapshotReady(container)),
        () -> actionSummary(Action.JUMP_BUMP) + " snapshots="
            + container.getSwerveDeviceEvidenceSnapshots());
    setDriverButton(ConfiguredOperatorControls.DRIVER_JUMP_BUMP, true);
    assertTrue(
        await(4.0, container, () -> actionIsActive(
            Action.JUMP_BUMP, "BUMP_HEADING_API_RETURNED_NOT_REQUEST_OR_MOTION_PROOF")),
        () -> actionSummary(Action.JUMP_BUMP));
  }

  private static boolean everyConfiguredCtreSnapshotReady(RobotContainer container) {
    var snapshots = container.getSwerveDeviceEvidenceSnapshots();
    Set<Integer> readyIds = snapshots.stream()
        .filter(snapshot -> snapshot.ready())
        .map(CtreDeviceEvidence.Snapshot::canId)
        .collect(java.util.stream.Collectors.toSet());
    return readyIds.equals(Set.copyOf(ConfiguredCanHardware.ctreDeviceIds()));
  }

  private static Map<Integer, Map<Metric, Double>> captureSwerveOutputBaseline(
      RobotContainer container) {
    Map<Integer, Map<Metric, Double>> baseline = new LinkedHashMap<>();
    Set<Integer> motorIds = configuredSwerveMotorIdsForNormalTest();
    for (var snapshot : container.getSwerveDeviceEvidenceSnapshots()) {
      if (motorIds.contains(snapshot.canId())) {
        baseline.put(
            snapshot.canId(),
            Map.of(
                Metric.DUTY_CYCLE,
                swerveProgressTimestamp(snapshot, Metric.DUTY_CYCLE),
                Metric.MOTOR_VOLTAGE_VOLTS,
                swerveProgressTimestamp(snapshot, Metric.MOTOR_VOLTAGE_VOLTS)));
      }
    }
    return baseline;
  }

  private static boolean configuredDriveIdsHaveFreshNonzeroOutput(
      RobotContainer container, Map<Integer, Map<Metric, Double>> baseline) {
    return swerveOutputIdsMatching(container, baseline, false).containsAll(
        ConfiguredCanHardware.swerveDriveIds());
  }

  private static boolean configuredSteerIdsHaveFreshNonzeroOutput(
      RobotContainer container, Map<Integer, Map<Metric, Double>> baseline) {
    return swerveOutputIdsMatching(container, baseline, false).containsAll(
        ConfiguredCanHardware.swerveSteerIds());
  }

  private static boolean configuredDriveIdsHaveFreshZeroOutput(
      RobotContainer container, Map<Integer, Map<Metric, Double>> baseline) {
    return swerveOutputIdsMatching(container, baseline, true).containsAll(
        ConfiguredCanHardware.swerveDriveIds());
  }

  private static boolean allSwerveMotorsHaveFreshZeroOutput(
      RobotContainer container, Map<Integer, Map<Metric, Double>> baseline) {
    return swerveOutputIdsMatching(container, baseline, true).equals(
        configuredSwerveMotorIdsForNormalTest());
  }

  private static Set<Integer> swerveOutputIdsMatching(
      RobotContainer container,
      Map<Integer, Map<Metric, Double>> baseline,
      boolean requireZero) {
    Set<Integer> matched = new LinkedHashSet<>();
    Set<Integer> motorIds = configuredSwerveMotorIdsForNormalTest();
    for (var snapshot : container.getSwerveDeviceEvidenceSnapshots()) {
      if (!motorIds.contains(snapshot.canId())) {
        continue;
      }
      Map<Metric, Double> before = baseline.get(snapshot.canId());
      if (before == null
          || !swerveOutputAdvanced(snapshot, Metric.DUTY_CYCLE, before)
          || !swerveOutputAdvanced(snapshot, Metric.MOTOR_VOLTAGE_VOLTS, before)) {
        continue;
      }
      double duty = snapshot.value(Metric.DUTY_CYCLE).orElse(Double.NaN);
      double voltage = snapshot.value(Metric.MOTOR_VOLTAGE_VOLTS).orElse(Double.NaN);
      if (!Double.isFinite(duty) || !Double.isFinite(voltage)) {
        continue;
      }
      boolean matches = requireZero
          ? Math.abs(duty) <= 0.01 && Math.abs(voltage) <= 0.25
          : Math.abs(duty) > 1e-4 || Math.abs(voltage) > 1e-3;
      if (matches) {
        matched.add(snapshot.canId());
      }
    }
    return matched;
  }

  private static boolean swerveOutputAdvanced(
      CtreDeviceEvidence.Snapshot snapshot,
      Metric metric,
      Map<Metric, Double> baseline) {
    double before = baseline.getOrDefault(metric, Double.NaN);
    double after = swerveProgressTimestamp(snapshot, metric);
    return Double.isFinite(before) && Double.isFinite(after) && after > before;
  }

  private static double swerveProgressTimestamp(
      CtreDeviceEvidence.Snapshot snapshot, Metric metric) {
    return snapshot.observations().stream()
        .filter(observation -> observation.metric() == metric)
        .filter(CtreDeviceEvidence.SignalObservation::fresh)
        .mapToDouble(CtreDeviceEvidence.SignalObservation::progressTimestampSeconds)
        .findFirst()
        .orElse(Double.NaN);
  }

  private static Set<Integer> configuredSwerveMotorIdsForNormalTest() {
    Set<Integer> ids = new LinkedHashSet<>(ConfiguredCanHardware.swerveDriveIds());
    ids.addAll(ConfiguredCanHardware.swerveSteerIds());
    return Set.copyOf(ids);
  }

  private static String swerveOutputSummary(
      RobotContainer container, Map<Integer, Map<Metric, Double>> baseline) {
    return "baseline=" + baseline + " snapshots=" + container.getSwerveDeviceEvidenceSnapshots();
  }

  private static void assertPairRawStopped() {
    assertTrue(pairRawStopped(), () -> pairSummary("SPARK pair remained nonzero"));
  }

  private static void assertActionBlocked(Action action, String expectedReason) {
    assertEquals(
        "BLOCKED",
        SmartDashboard.getString(OperatorActionEvidence.stateKey(action), "MISSING"));
    assertTrue(
        SmartDashboard.getString(OperatorActionEvidence.reasonKey(action), "")
            .contains(expectedReason),
        () -> action + " reason="
            + SmartDashboard.getString(OperatorActionEvidence.reasonKey(action), "MISSING"));
  }

  private static void assertActionStopped(Action action) {
    assertEquals(
        "STOPPED",
        SmartDashboard.getString(OperatorActionEvidence.stateKey(action), "MISSING"));
    assertEquals(
        "INPUT_RELEASED",
        SmartDashboard.getString(OperatorActionEvidence.reasonKey(action), "MISSING"));
  }

  private static void assertRawSetpointsZero(int... canIds) {
    for (int canId : canIds) {
      assertEquals(
          0.0,
          simulationHandle(canId).observe().setpoint(),
          "CAN " + canId + " retained a nonzero setpoint");
    }
  }

  private static void assertRawControllersStopped(int... canIds) {
    for (int canId : canIds) {
      assertTrue(
          rawControllerStopped(canId),
          () -> "CAN " + canId + " retained raw output " + simulationHandle(canId).observe());
    }
  }

  private static boolean rawControllerStopped(int canId) {
    SimulationSnapshot snapshot = simulationHandle(canId).observe();
    return snapshot.setpoint() == 0.0
        && snapshot.appliedOutput() == 0.0
        && snapshot.velocity() == 0.0;
  }

  private static boolean pairRawStopped() {
    SimulationSnapshot leader = simulationHandle(SHOOTER_LEADER_ID).observe();
    SimulationSnapshot follower = simulationHandle(SHOOTER_FOLLOWER_ID).observe();
    return leader.appliedOutput() == 0.0
        && leader.velocity() == 0.0
        && follower.appliedOutput() == 0.0
        && follower.velocity() == 0.0;
  }

  private static SparkSimulationHandle simulationHandle(int canId) {
    return SparkMAXContainer.getSimulationHandleForId(canId).orElseThrow();
  }

  private static String pairSummary(String prefix) {
    return prefix + " leader=" + simulationHandle(SHOOTER_LEADER_ID).observe()
        + " follower=" + simulationHandle(SHOOTER_FOLLOWER_ID).observe()
        + " health=" + SparkMAXContainer.getDeviceAvailabilitySummary();
  }

  private static boolean await(
      double timeoutSeconds, RobotContainer container, BooleanSupplier condition)
      throws InterruptedException {
    long deadline = System.nanoTime() + (long) (timeoutSeconds * 1_000_000_000L);
    do {
      pump(container, 1);
      if (condition.getAsBoolean()) {
        return true;
      }
    } while (System.nanoTime() < deadline);
    return condition.getAsBoolean();
  }

  private static boolean awaitCleanup(double timeoutSeconds) throws InterruptedException {
    long deadline = System.nanoTime() + (long) (timeoutSeconds * 1_000_000_000L);
    do {
      SparkMAXContainer.serviceAll();
      if (SparkMAXContainer.cleanupSimulationDevicesForTesting()) {
        return true;
      }
      Thread.sleep(10L);
    } while (System.nanoTime() < deadline);
    return SparkMAXContainer.cleanupSimulationDevicesForTesting();
  }

  private static boolean awaitWithoutRobotLoop(
      double timeoutSeconds, BooleanSupplier condition) throws InterruptedException {
    long deadline = System.nanoTime() + (long) (timeoutSeconds * 1_000_000_000L);
    do {
      if (condition.getAsBoolean()) {
        return true;
      }
      Thread.sleep(5L);
    } while (System.nanoTime() < deadline);
    return condition.getAsBoolean();
  }

  private static boolean awaitWithoutScheduler(
      double timeoutSeconds, RobotContainer container, BooleanSupplier condition)
      throws InterruptedException {
    long deadline = System.nanoTime() + (long) (timeoutSeconds * 1_000_000_000L);
    do {
      container.serviceOutputSafetyHeartbeat();
      if (condition.getAsBoolean()) {
        return true;
      }
      Thread.sleep(5L);
    } while (System.nanoTime() < deadline);
    return condition.getAsBoolean();
  }

  private static void pump(RobotContainer container, int iterations) throws InterruptedException {
    for (int iteration = 0; iteration < iterations; iteration++) {
      SparkMAXContainer.serviceAll();
      container.updateTeleopSafetyState();
      container.serviceOutputSafetyHeartbeat();
      long schedulerRunEpoch = container.beginCommandSchedulerRun();
      CommandScheduler.getInstance().run();
      container.completeCommandSchedulerRun(schedulerRunEpoch);
      container.simulationPeriodic();
      Thread.sleep(20L);
    }
  }

  private static void assertOutputSafetyArmed(RobotContainer container)
      throws InterruptedException {
    assertTrue(
        await(
            5.0,
            container,
            () -> container.getOutputSafetySnapshot().phase()
                    == RobotOutputSafetySupervisor.Phase.ARMED
                && ProcessOutputSafety.isOutputAuthorized()),
        () -> container.getOutputSafetySnapshot().toString());
  }

  private static void assertOutputSafetyReadyDisabled(RobotContainer container)
      throws InterruptedException {
    assertTrue(
        await(
            5.0,
            container,
            () -> container.getOutputSafetySnapshot().phase()
                == RobotOutputSafetySupervisor.Phase.READY_DISABLED),
        () -> container.getOutputSafetySnapshot().toString());
  }

  private static void configureDisabledDriverStation() {
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setFmsAttached(false);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.setEnabled(false);
    DriverStationSim.setJoystickAxisCount(DRIVER_PORT, 6);
    DriverStationSim.setJoystickButtonCount(DRIVER_PORT, 15);
    DriverStationSim.setJoystickPOVCount(DRIVER_PORT, 1);
    DriverStationSim.setJoystickAxisCount(OIConstants.kOperatorControllerPort, 6);
    DriverStationSim.setJoystickButtonCount(OIConstants.kOperatorControllerPort, 5);
    DriverStationSim.setJoystickAxisCount(OIConstants.kMaintenanceControllerPort, 6);
    DriverStationSim.setJoystickButtonCount(OIConstants.kMaintenanceControllerPort, 10);
    DriverStationSim.notifyNewData();
  }

  private static void enableTeleop(boolean revHeld) {
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setJoystickButton(
        DRIVER_PORT, ConfiguredOperatorControls.DRIVER_REV, revHeld);
    DriverStationSim.notifyNewData();
  }

  private static void enableDisabled() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  private static void enableDisabledTest() {
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  private static void enableTest() {
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  private static void setDriverButton(int button, boolean pressed) {
    DriverStationSim.setJoystickButton(DRIVER_PORT, button, pressed);
    DriverStationSim.notifyNewData();
  }

  private static void setDriverAxis(int axis, double value) {
    DriverStationSim.setJoystickAxis(DRIVER_PORT, axis, value);
    DriverStationSim.notifyNewData();
  }

  private static void setOperatorButton(int button, boolean pressed) {
    DriverStationSim.setJoystickButton(
        OIConstants.kOperatorControllerPort, button, pressed);
    DriverStationSim.notifyNewData();
  }

  private static void setMaintenanceButton(int button, boolean pressed) {
    DriverStationSim.setJoystickButton(OIConstants.kMaintenanceControllerPort, button, pressed);
    DriverStationSim.notifyNewData();
  }

  private static void prepareTurretDiagnosticSelection() {
    SmartDashboard.putBoolean(HardwareSelfTestCommand.RUNNING_KEY, false);
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.PHYSICAL_CLEARANCE_KEY, true);
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.MOTOR_TYPE_VERIFIED_KEY, true);
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.TARGET_INTAKE_KEY, false);
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.TARGET_FEEDER_KEY, false);
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.TARGET_CLIMBER_LEFT_KEY, false);
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.TARGET_CLIMBER_RIGHT_KEY, false);
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.TARGET_SHOOTER_KEY, false);
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.TARGET_TURRET_KEY, true);
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.DIRECTION_NEGATIVE_KEY, false);
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.DIRECTION_POSITIVE_KEY, true);
  }

  private static void prepareFeederDiagnosticSelection() {
    SmartDashboard.putBoolean(HardwareSelfTestCommand.RUNNING_KEY, false);
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.PHYSICAL_CLEARANCE_KEY, true);
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.MOTOR_TYPE_VERIFIED_KEY, true);
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.FEEDER_REPAIR_VERIFIED_KEY, true);
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.TARGET_INTAKE_KEY, false);
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.TARGET_FEEDER_KEY, true);
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.TARGET_CLIMBER_LEFT_KEY, false);
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.TARGET_CLIMBER_RIGHT_KEY, false);
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.TARGET_SHOOTER_KEY, false);
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.TARGET_TURRET_KEY, false);
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.DIRECTION_NEGATIVE_KEY, false);
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.DIRECTION_POSITIVE_KEY, true);
  }
}
