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
import frc.robot.utils.AsyncDiagnosticSink;
import frc.robot.utils.OneShotTimedArmGate;
import frc.robot.utils.RuntimeSafetyLatch;
import frc.robot.utils.SparkMAXContainer;
import frc.robot.constants.Constants.HardwareTestConstants;

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
  private final RuntimeSafetyLatch m_runtimeSafetyLatch = new RuntimeSafetyLatch();
  private final OneShotTimedArmGate m_selfTestArmGate = new OneShotTimedArmGate(
      HardwareTestConstants.ARM_LIFETIME_SECONDS);

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
    SmartDashboard.putBoolean(HardwareSelfTestCommand.RUNNING_KEY, false);
    SmartDashboard.putBoolean("Hub Active", false);
    SmartDashboard.putBoolean("Runtime/Scheduler Healthy", true);
    SmartDashboard.putString("Runtime/Fault", "HEALTHY");
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

      boolean armRequested = SmartDashboard.getBoolean("Hardware Self-Test/Armed", false);
      boolean armValid = m_selfTestArmGate.observe(
          armRequested,
          DriverStation.isDisabled()
              && DriverStation.isTest()
              && !DriverStation.isFMSAttached(),
          Timer.getFPGATimestamp());
      SmartDashboard.putBoolean("Hardware Self-Test/Arm Valid", armValid);

      double now = Timer.getFPGATimestamp();
      if (now >= m_nextOperatorStatusTimestamp) {
        SmartDashboard.putBoolean(
            "Hub Active", DriverStation.isEnabled() && Telemetry.isHubActive());
        m_nextOperatorStatusTimestamp = now + 0.10;
      }

      if (!DriverStation.isFMSAttached() && now >= m_nextDiagnosticTimestamp) {
        var canStatus = RobotController.getCANStatus();
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
            m_robotContainer.getSparkDeviceHealthSummary(),
            m_robotContainer.getSwerveDeviceHealthSummary()));
        m_nextDiagnosticTimestamp = now + 5.0;
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
      m_robotContainer.stopAll();
      boolean armAccepted = !DriverStation.isFMSAttached()
          && SmartDashboard.getBoolean("Hardware Self-Test/Armed", false)
          && m_selfTestArmGate.consume(Timer.getFPGATimestamp());
      clearSelfTestArm();
      if (armAccepted) {
        m_hardwareSelfTest = m_robotContainer.getHardwareSelfTestCommand();
        CommandScheduler.getInstance().schedule(m_hardwareSelfTest);
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
      m_robotContainer.stopAll();
    });
  }

  private void clearSelfTestArm() {
    SmartDashboard.putBoolean("Hardware Self-Test/Armed", false);
    SmartDashboard.putBoolean("Hardware Self-Test/Arm Valid", false);
    m_selfTestArmGate.requireRelease();
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
