package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.commands.HardwareSelfTestCommand;
import frc.robot.commands.ManualUnhomedActuatorDiagnosticCommand;
import frc.robot.commands.ManualUnhomedActuatorDiagnosticCommand.Direction;
import frc.robot.commands.ManualUnhomedActuatorDiagnosticCommand.Target;
import frc.robot.constants.ConfiguredCanHardware;
import frc.robot.constants.ConfiguredOperatorControls;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.utils.SparkMAXContainer.OutputStopBatch;
import frc.robot.utils.SparkSimulationHandle.SimulationSnapshot;
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

      enableTeleop(false);
      pump(activeContainer, 3);
      setDriverButton(ConfiguredOperatorControls.DRIVER_REV, true);

      assertTrue(await(3.0, activeContainer, () -> pairEchoesRequestedVelocity(500.0)),
          () -> pairSummary("initial rev binding did not reach the SPARK pair"));

      setDriverButton(ConfiguredOperatorControls.DRIVER_REV, false);
      assertTrue(await(3.0, activeContainer, RobotContainerSimulationIntegrationTest::pairRawStopped),
          () -> pairSummary("RevUpCommand.end did not stop the SPARK pair"));
      OutputStopBatch firstStop = SparkMAXContainer.requestOutputStops(
          SHOOTER_LEADER_ID, SHOOTER_FOLLOWER_ID);
      assertTrue(await(3.0, activeContainer, () -> firstStop.snapshot().confirmed()),
          () -> firstStop.snapshot().summary());
      assertPairRawStopped();

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

      // Recovery configuration is disabled-only. Keep the button held throughout recovery.
      enableDisabled();
      leader.setCanFault(false, false);
      assertTrue(
          await(8.0, activeContainer, () -> ConfiguredCanHardware.sparkDeviceIds().stream()
              .allMatch(id -> SparkMAXContainer.getDiagnosticSnapshotForId(id)
                  .map(snapshot -> snapshot.ready())
                  .orElse(false))),
          SparkMAXContainer::getDeviceAvailabilitySummary);

      enableTeleop(true);
      for (int recoveryCycle = 0; recoveryCycle < 8; recoveryCycle++) {
        pump(activeContainer, 1);
        assertPairRawStopped();
        assertEquals(
            0.0,
            simulationHandle(SHOOTER_LEADER_ID).observe().setpoint(),
            "held input restarted the recovered leader during cycle " + recoveryCycle);
      }
      assertFalse(
          pairEchoesRequestedVelocity(500.0),
          "a held button must not restart the recovered mechanism");

      // One neutral observation rearms the gate; only a new rising edge may restart motion.
      setDriverButton(ConfiguredOperatorControls.DRIVER_REV, false);
      pump(activeContainer, 3);
      setDriverButton(ConfiguredOperatorControls.DRIVER_REV, true);
      assertTrue(await(3.0, activeContainer, () -> pairEchoesRequestedVelocity(500.0)),
          () -> pairSummary("release and fresh press did not rearm the binding"));

      // The same production binding layer also reaches the one-shot unreferenced diagnostic.
      // Command echo proves output acceptance, but deliberately reports no invented motion.
      setDriverButton(ConfiguredOperatorControls.DRIVER_REV, false);
      assertTrue(await(3.0, activeContainer, RobotContainerSimulationIntegrationTest::pairRawStopped),
          () -> pairSummary("recovered RevUpCommand.end did not stop the SPARK pair"));
      OutputStopBatch recoveredStop = SparkMAXContainer.requestOutputStops(
          SHOOTER_LEADER_ID, SHOOTER_FOLLOWER_ID);
      assertTrue(await(3.0, activeContainer, () -> recoveredStop.snapshot().confirmed()),
          () -> recoveredStop.snapshot().summary());
      prepareTurretDiagnosticSelection();
      enableDisabledTest();
      activeContainer.armUnhomedDiagnosticSession(Target.TURRET, Direction.POSITIVE);
      double referenceEpochBefore = SmartDashboard.getNumber("Turret/Continuity Epoch", -1.0);
      enableTest();
      pump(activeContainer, 3);
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

  private static boolean pairEchoesRequestedVelocity(double expected) {
    SimulationSnapshot leader = simulationHandle(SHOOTER_LEADER_ID).observe();
    SimulationSnapshot follower = simulationHandle(SHOOTER_FOLLOWER_ID).observe();
    return Math.abs(leader.setpoint() - expected) < 1e-9
        && Math.abs(leader.velocity() - expected) < 1e-9
        && Math.abs(follower.velocity() + expected) < 1e-9;
  }

  private static void assertPairRawStopped() {
    assertTrue(pairRawStopped(), () -> pairSummary("SPARK pair remained nonzero"));
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

  private static void pump(RobotContainer container, int iterations) throws InterruptedException {
    for (int iteration = 0; iteration < iterations; iteration++) {
      SparkMAXContainer.serviceAll();
      container.updateTeleopSafetyState();
      CommandScheduler.getInstance().run();
      container.simulationPeriodic();
      Thread.sleep(20L);
    }
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
}
