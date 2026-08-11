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
import frc.robot.utils.SparkMAXContainer;

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
    SparkMAXContainer.serviceAll();

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    if (!DriverStation.isFMSAttached() && Timer.getFPGATimestamp() >= m_nextDiagnosticTimestamp) {
      var canStatus = RobotController.getCANStatus();
      System.out.printf(
          "DIAGNOSTICS ds=%s enabled=%s voltage=%.2fV canUtil=%.1f%% busOff=%d txFull=%d rxErr=%d txErr=%d "
              + "sticks=[0:'%s' a%d b%d; 1:'%s' a%d b%d; 2:'%s' a%d b%d] "
              + "spark=%s ctre=[%s]%n",
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
          m_robotContainer.getSwerveDeviceHealthSummary());
      m_nextDiagnosticTimestamp = Timer.getFPGATimestamp() + 5.0;
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    if (m_hardwareSelfTest != null) {
      m_hardwareSelfTest.cancel();
      m_hardwareSelfTest = null;
    }
    m_robotContainer.stopAll();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
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
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (m_robotContainer.shouldAbortActiveAutonomous()) {
      if (m_autonomousCommand != null) {
        m_autonomousCommand.cancel();
        m_autonomousCommand = null;
      }
      m_robotContainer.stopAll();
    }
  }

  @Override
  public void autonomousExit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      m_autonomousCommand = null;
    }
    m_robotContainer.stopAll();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      m_autonomousCommand = null;
    }
    m_robotContainer.stopAll();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.stopAll();
    if (!DriverStation.isFMSAttached()
        && SmartDashboard.getBoolean("Hardware Self-Test/Armed", false)) {
      SmartDashboard.putBoolean("Hardware Self-Test/Armed", false);
      m_hardwareSelfTest = m_robotContainer.getHardwareSelfTestCommand();
      CommandScheduler.getInstance().schedule(m_hardwareSelfTest);
    }
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {
    if (m_hardwareSelfTest != null) {
      m_hardwareSelfTest.cancel();
      m_hardwareSelfTest = null;
    }
    m_robotContainer.stopAll();
  }
}
