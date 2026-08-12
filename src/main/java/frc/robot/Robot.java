// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.HardwareSelfTestCommand;
import frc.robot.commands.ManualUnhomedActuatorDiagnosticCommand;
import frc.robot.commands.ManualUnhomedActuatorDiagnosticCommand.Direction;
import frc.robot.commands.ManualUnhomedActuatorDiagnosticCommand.Target;
import frc.robot.utils.AsyncDiagnosticSink;
import frc.robot.utils.OneShotTimedArmGate;
import frc.robot.utils.RuntimeSafetyLatch;
import frc.robot.utils.SparkMAXContainer;
import frc.robot.constants.Constants.HardwareTestConstants;
import frc.robot.constants.ConfiguredCanHardware;
import frc.robot.constants.ConfiguredMotorCapabilities;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command m_hardwareSelfTest;

  private RobotContainer m_robotContainer;
  private double m_nextDiagnosticTimestamp;
  private double m_nextOperatorStatusTimestamp;
  private boolean m_hardwareHealthSuppressedForFms;
  private final RuntimeSafetyLatch m_runtimeSafetyLatch = new RuntimeSafetyLatch();
  private final OneShotTimedArmGate m_selfTestArmGate = new OneShotTimedArmGate(
      HardwareTestConstants.ARM_LIFETIME_SECONDS);
  private final OneShotTimedArmGate m_unhomedDiagnosticArmGate = new OneShotTimedArmGate(
      HardwareTestConstants.ARM_LIFETIME_SECONDS);
  private Target m_unhomedDiagnosticTargetSnapshot;
  private Direction m_unhomedDiagnosticDirectionSnapshot;
  private Target m_lastUnhomedDiagnosticTargetSelection;
  private Direction m_lastUnhomedDiagnosticDirectionSelection;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    SparkMAXContainer.configureProcessDefaults();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    SmartDashboard.putBoolean("Hardware Self-Test/Armed", false);
    ManualUnhomedActuatorDiagnosticCommand.initializeDashboard();
    SmartDashboard.putBoolean(HardwareSelfTestCommand.RUNNING_KEY, false);
    SmartDashboard.putBoolean("Hub Active", false);
    SmartDashboard.putBoolean("Runtime/Scheduler Healthy", true);
    SmartDashboard.putString("Runtime/Fault", "HEALTHY");
    SmartDashboard.putString(
        "Hardware/CAN Configured", ConfiguredCanHardware.configuredSummary());
    SmartDashboard.putString(
        "Hardware/Motor Capability Summary",
        ConfiguredMotorCapabilities.normalMotionSummary());
    SmartDashboard.putString(
        "Hardware/Motor Blockers", ConfiguredMotorCapabilities.blockedSummary());
    SmartDashboard.putString("Hardware/SPARK Health", "WAITING_FOR_SAMPLE");
    SmartDashboard.putString("Hardware/CTRE Health", "WAITING_FOR_SAMPLE");
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    if (!m_runtimeSafetyLatch.healthy()) {
      enforceLatchedStop();
      return;
    }

    try {
      SparkMAXContainer.serviceAll();
      m_robotContainer.updateTeleopSafetyState();

      // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
      // commands, running already-scheduled commands, removing finished or interrupted commands,
      // and running subsystem periodic() methods.  This must be called from the robot's periodic
      // block in order for anything in the Command-based framework to work.
      CommandScheduler.getInstance().run();
      m_robotContainer.refreshAutonomousStatus();

      updateDiagnosticArmGates();

      double now = Timer.getFPGATimestamp();
      if (now >= m_nextOperatorStatusTimestamp) {
        SmartDashboard.putBoolean(
            "Hub Active", DriverStation.isEnabled() && Telemetry.isHubActive());
        m_nextOperatorStatusTimestamp = now + 0.10;
      }

      if (!DriverStation.isFMSAttached() && now >= m_nextDiagnosticTimestamp) {
        var canStatus = RobotController.getCANStatus();
        String sparkHealth = m_robotContainer.getSparkDeviceHealthSummary();
        String ctreHealth = m_robotContainer.getSwerveDeviceHealthSummary();
        SmartDashboard.putString("Hardware/SPARK Health", sparkHealth);
        SmartDashboard.putString("Hardware/CTRE Health", ctreHealth);
        AsyncDiagnosticSink.log(String.format(
            "DIAGNOSTICS ds=%s enabled=%s voltage=%.2fV canUtil=%.1f%% busOff=%d txFull=%d rxErr=%d txErr=%d "
                + "sticks=[0:'%s' a%d b%d; 1:'%s' a%d b%d; 2:'%s' a%d b%d] "
                + "spark=%s ctre=[%s]",
            DriverStation.isDSAttached(), DriverStation.isEnabled(), RobotController.getBatteryVoltage(),
            canStatus.percentBusUtilization * 100.0, canStatus.busOffCount, canStatus.txFullCount,
            canStatus.receiveErrorCount, canStatus.transmitErrorCount,
            DriverStation.getJoystickName(0), DriverStation.getStickAxisCount(0),
            DriverStation.getStickButtonCount(0),
            DriverStation.getJoystickName(1), DriverStation.getStickAxisCount(1),
            DriverStation.getStickButtonCount(1),
            DriverStation.getJoystickName(2), DriverStation.getStickAxisCount(2),
            DriverStation.getStickButtonCount(2),
            sparkHealth,
            ctreHealth));
        m_hardwareHealthSuppressedForFms = false;
        m_nextDiagnosticTimestamp = now + 5.0;
      } else if (DriverStation.isFMSAttached() && !m_hardwareHealthSuppressedForFms) {
        SmartDashboard.putString("Hardware/SPARK Health", "SUPPRESSED_FMS");
        SmartDashboard.putString("Hardware/CTRE Health", "SUPPRESSED_FMS");
        m_hardwareHealthSuppressedForFms = true;
      }
    } catch (RuntimeException exception) {
      latchRuntimeFault(exception);
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    runLifecycleSafely(() -> {
      if (m_hardwareSelfTest != null) {
        m_hardwareSelfTest.cancel();
        m_hardwareSelfTest = null;
      }
      clearSelfTestArm();
      clearUnhomedDiagnosticArm();
      clearUnhomedDiagnosticVerifications();
      m_robotContainer.stopAll();
    });
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    runLifecycleSafely(() -> {
      clearSelfTestArm();
      clearUnhomedDiagnosticArm();
      clearUnhomedDiagnosticVerifications();
      m_robotContainer.stopAll();
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
      if (m_autonomousCommand != null) {
        CommandScheduler.getInstance().schedule(m_autonomousCommand);
      }
    });
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    runLifecycleSafely(() -> {
      if (m_autonomousCommand == null) {
        return;
      }
      if (!CommandScheduler.getInstance().isScheduled(m_autonomousCommand)) {
        m_autonomousCommand = null;
        return;
      }
      if (m_robotContainer.shouldAbortActiveAutonomous()) {
        m_autonomousCommand.cancel();
        m_autonomousCommand = null;
        m_robotContainer.stopAll();
      }
    });
  }

  @Override
  public void autonomousExit() {
    runLifecycleSafely(() -> {
      if (m_autonomousCommand != null) {
        m_autonomousCommand.cancel();
        m_autonomousCommand = null;
      }
      m_robotContainer.stopAll();
    });
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    runLifecycleSafely(() -> {
      if (m_autonomousCommand != null) {
        m_autonomousCommand.cancel();
        m_autonomousCommand = null;
      }
      clearSelfTestArm();
      clearUnhomedDiagnosticArm();
      clearUnhomedDiagnosticVerifications();
      m_robotContainer.stopAll();
    });
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    runLifecycleSafely(() -> {
      // Cancels all running commands at the start of test mode.
      CommandScheduler.getInstance().cancelAll();
      boolean selfTestRequested = SmartDashboard.getBoolean(
          "Hardware Self-Test/Armed", false);
      boolean unhomedDiagnosticRequested = SmartDashboard.getBoolean(
          ManualUnhomedActuatorDiagnosticCommand.ARM_KEY, false);
      boolean conflictingArms = selfTestRequested && unhomedDiagnosticRequested;
      double now = Timer.getFPGATimestamp();
      boolean armAccepted = !conflictingArms
          && !DriverStation.isFMSAttached()
          && selfTestRequested
          && m_selfTestArmGate.consume(now);
      Target diagnosticTarget = m_unhomedDiagnosticTargetSnapshot;
      Direction diagnosticDirection = m_unhomedDiagnosticDirectionSnapshot;
      boolean unhomedDiagnosticAccepted = !conflictingArms
          && !DriverStation.isFMSAttached()
          && unhomedDiagnosticRequested
          && diagnosticTarget != null
          && diagnosticDirection != null
          && ManualUnhomedActuatorDiagnosticCommand.readExactlyOneTarget()
              .filter(target -> target == diagnosticTarget)
              .isPresent()
          && ManualUnhomedActuatorDiagnosticCommand.readExactlyOneDirection()
              .filter(direction -> direction == diagnosticDirection)
              .isPresent()
          && SmartDashboard.getBoolean(
              ManualUnhomedActuatorDiagnosticCommand.PHYSICAL_CLEARANCE_KEY, false)
          && SmartDashboard.getBoolean(
              ManualUnhomedActuatorDiagnosticCommand.MOTOR_TYPE_VERIFIED_KEY, false)
          && m_unhomedDiagnosticArmGate.consume(now);
      clearSelfTestArm();
      clearUnhomedDiagnosticArm();
      m_robotContainer.stopAll();
      if (conflictingArms) {
        SmartDashboard.putString(
            ManualUnhomedActuatorDiagnosticCommand.STATUS_KEY,
            "ARM_CONFLICT_REJECTED");
      } else if (unhomedDiagnosticRequested && !unhomedDiagnosticAccepted) {
        SmartDashboard.putString(
            ManualUnhomedActuatorDiagnosticCommand.STATUS_KEY,
            "ARM_REJECTED_RELEASE_AND_RETRY_DISABLED");
      }
      if (armAccepted) {
        m_hardwareSelfTest = m_robotContainer.getHardwareSelfTestCommand();
        CommandScheduler.getInstance().schedule(m_hardwareSelfTest);
      } else if (unhomedDiagnosticAccepted) {
        m_robotContainer.armUnhomedDiagnosticSession(diagnosticTarget, diagnosticDirection);
      }
    });
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {
    runLifecycleSafely(() -> {
      if (m_hardwareSelfTest != null) {
        m_hardwareSelfTest.cancel();
        m_hardwareSelfTest = null;
      }
      clearSelfTestArm();
      clearUnhomedDiagnosticArm();
      clearUnhomedDiagnosticVerifications();
      m_robotContainer.stopAll();
    });
  }

  private void clearSelfTestArm() {
    SmartDashboard.putBoolean("Hardware Self-Test/Armed", false);
    SmartDashboard.putBoolean("Hardware Self-Test/Arm Valid", false);
    m_selfTestArmGate.requireRelease();
  }

  private void clearUnhomedDiagnosticArm() {
    SmartDashboard.putBoolean(ManualUnhomedActuatorDiagnosticCommand.ARM_KEY, false);
    SmartDashboard.putBoolean(ManualUnhomedActuatorDiagnosticCommand.ARM_VALID_KEY, false);
    SmartDashboard.putString(
        ManualUnhomedActuatorDiagnosticCommand.STATUS_KEY, "DISARMED");
    m_unhomedDiagnosticArmGate.requireRelease();
    m_unhomedDiagnosticTargetSnapshot = null;
    m_unhomedDiagnosticDirectionSnapshot = null;
  }

  private void clearUnhomedDiagnosticVerifications() {
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.PHYSICAL_CLEARANCE_KEY, false);
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.MOTOR_TYPE_VERIFIED_KEY, false);
  }

  private void updateDiagnosticArmGates() {
    boolean selfTestRequested = SmartDashboard.getBoolean(
        "Hardware Self-Test/Armed", false);
    boolean unhomedRequested = SmartDashboard.getBoolean(
        ManualUnhomedActuatorDiagnosticCommand.ARM_KEY, false);
    boolean conflictingArms = selfTestRequested && unhomedRequested;
    boolean disabledTestWithoutFms = DriverStation.isDisabled()
        && DriverStation.isTest()
        && !DriverStation.isFMSAttached();
    double now = Timer.getFPGATimestamp();

    boolean selfTestArmValid;
    boolean unhomedArmValid = false;
    var selectedTarget = ManualUnhomedActuatorDiagnosticCommand.readExactlyOneTarget();
    var selectedDirection = ManualUnhomedActuatorDiagnosticCommand.readExactlyOneDirection();
    Target currentTargetSelection = selectedTarget.orElse(null);
    Direction currentDirectionSelection = selectedDirection.orElse(null);
    boolean selectionChanged = currentTargetSelection != m_lastUnhomedDiagnosticTargetSelection
        || currentDirectionSelection != m_lastUnhomedDiagnosticDirectionSelection;
    if (selectionChanged) {
      // Clearance and motor-type evidence applies to one exact mechanism and direction only.
      clearUnhomedDiagnosticVerifications();
      m_lastUnhomedDiagnosticTargetSelection = currentTargetSelection;
      m_lastUnhomedDiagnosticDirectionSelection = currentDirectionSelection;
    }
    boolean unhomedVerifications = SmartDashboard.getBoolean(
        ManualUnhomedActuatorDiagnosticCommand.PHYSICAL_CLEARANCE_KEY, false)
        && SmartDashboard.getBoolean(
            ManualUnhomedActuatorDiagnosticCommand.MOTOR_TYPE_VERIFIED_KEY, false);

    if (conflictingArms) {
      m_selfTestArmGate.invalidate(true);
      m_unhomedDiagnosticArmGate.invalidate(true);
      selfTestArmValid = false;
      if (DriverStation.isDisabled() && DriverStation.isTest()) {
        SmartDashboard.putString(
            ManualUnhomedActuatorDiagnosticCommand.STATUS_KEY,
            "ARM_CONFLICT_RELEASE_BOTH");
      }
    } else {
      selfTestArmValid = m_selfTestArmGate.observe(
          selfTestRequested,
          disabledTestWithoutFms && !unhomedRequested,
          now);

      if (!unhomedRequested) {
        m_unhomedDiagnosticTargetSnapshot = null;
        m_unhomedDiagnosticDirectionSnapshot = null;
      }
      boolean selectionChangedWhileArmed = unhomedRequested
          && (m_unhomedDiagnosticTargetSnapshot != null
              || m_unhomedDiagnosticDirectionSnapshot != null)
          && (selectedTarget.isEmpty()
              || selectedDirection.isEmpty()
              || selectedTarget.get() != m_unhomedDiagnosticTargetSnapshot
              || selectedDirection.get() != m_unhomedDiagnosticDirectionSnapshot);
      if (selectionChangedWhileArmed) {
        m_unhomedDiagnosticArmGate.invalidate(true);
      } else {
        unhomedArmValid = m_unhomedDiagnosticArmGate.observe(
            unhomedRequested,
            disabledTestWithoutFms
                && !selfTestRequested
                && unhomedVerifications
                && selectedTarget.isPresent()
                && selectedDirection.isPresent(),
            now);
        if (unhomedArmValid && m_unhomedDiagnosticTargetSnapshot == null) {
          m_unhomedDiagnosticTargetSnapshot = selectedTarget.orElseThrow();
          m_unhomedDiagnosticDirectionSnapshot = selectedDirection.orElseThrow();
        }
      }

      if (DriverStation.isDisabled() && DriverStation.isTest()) {
        if (selectionChangedWhileArmed) {
          SmartDashboard.putString(
              ManualUnhomedActuatorDiagnosticCommand.STATUS_KEY,
              "TARGET_OR_DIRECTION_CHANGED_RELEASE_ARM");
        } else if (unhomedArmValid) {
          SmartDashboard.putString(
              ManualUnhomedActuatorDiagnosticCommand.STATUS_KEY,
              "ARM_VALID_" + m_unhomedDiagnosticTargetSnapshot.label() + "_"
                  + m_unhomedDiagnosticDirectionSnapshot.name());
        } else if (unhomedRequested
            && (selectedTarget.isEmpty() || selectedDirection.isEmpty())) {
          SmartDashboard.putString(
              ManualUnhomedActuatorDiagnosticCommand.STATUS_KEY,
              "SELECT_EXACTLY_ONE_TARGET_AND_DIRECTION");
        } else if (unhomedRequested && !unhomedVerifications) {
          SmartDashboard.putString(
              ManualUnhomedActuatorDiagnosticCommand.STATUS_KEY,
              "VERIFY_CLEARANCE_AND_MOTOR_TYPE");
        }
      }
    }

    SmartDashboard.putBoolean("Hardware Self-Test/Arm Valid", selfTestArmValid);
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.ARM_VALID_KEY, unhomedArmValid);
  }

  private void runLifecycleSafely(Runnable action) {
    if (!m_runtimeSafetyLatch.healthy()) {
      enforceLatchedStop();
      return;
    }
    try {
      action.run();
    } catch (RuntimeException exception) {
      latchRuntimeFault(exception);
    }
  }

  private void latchRuntimeFault(RuntimeException exception) {
    RuntimeSafetyLatch.Snapshot fault = m_runtimeSafetyLatch.latch(exception);
    enforceLatchedStop();
    try {
      SmartDashboard.putBoolean("Runtime/Scheduler Healthy", false);
      SmartDashboard.putString("Runtime/Fault", fault.reason());
    } catch (RuntimeException ignored) {
      // The output stop remains authoritative if NetworkTables itself is the failed component.
    }
    AsyncDiagnosticSink.log("RUNTIME FAULT latched=" + fault.reason());
  }

  private void enforceLatchedStop() {
    m_robotContainer.stopAll();
    try {
      // SPARK stops are queued so the dedicated worker can preserve zero/nonzero ordering.
      SparkMAXContainer.serviceAll();
    } catch (RuntimeException ignored) {
      // Keep retrying the stop request on later robot periods without re-entering the scheduler.
    }
  }
}
