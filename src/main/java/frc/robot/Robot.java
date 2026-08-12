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
import frc.robot.utils.SparkDeviceEvidence;
import frc.robot.utils.CtreDeviceEvidence;
import frc.robot.utils.CanDeviceEvidenceSummary;
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
  private double m_nextDeviceEvidenceTimestamp;
  private double m_nextOperatorStatusTimestamp;
  private boolean m_hardwareHealthSuppressedForFms;
  private boolean m_deviceEvidenceSuppressedForFms;
  private boolean m_deviceEvidenceUnavailableForRuntimeFault;
  private final RuntimeSafetyLatch m_runtimeSafetyLatch = new RuntimeSafetyLatch();
  private final OneShotTimedArmGate m_selfTestArmGate = new OneShotTimedArmGate(
      HardwareTestConstants.ARM_LIFETIME_SECONDS);
  private final OneShotTimedArmGate m_unhomedDiagnosticArmGate = new OneShotTimedArmGate(
      HardwareTestConstants.ARM_LIFETIME_SECONDS);
  private Target m_unhomedDiagnosticTargetSnapshot;
  private Direction m_unhomedDiagnosticDirectionSnapshot;
  private double m_unhomedDiagnosticSnapshotExpiresAt = Double.NEGATIVE_INFINITY;
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
    SmartDashboard.putString(
        CanDeviceEvidenceSummary.SCOPE_KEY, CanDeviceEvidenceSummary.SCOPE);
    SmartDashboard.putString(
        CanDeviceEvidenceSummary.SUMMARY_KEY, "WAITING_FOR_DEVICE_EVIDENCE");
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
    suppressHardwareEvidenceForFmsNoThrow();
    if (!m_runtimeSafetyLatch.healthy()) {
      publishRuntimeFaultEvidenceUnavailableNoThrow();
      enforceLatchedStop();
      return;
    }

    try {
      SparkMAXContainer.serviceAll();
      m_robotContainer.updateTeleopSafetyState();

      // Renew immediately before scheduler execution. If this call is not repeated within the
      // bounded window, an independent Notifier revokes all later nonzero vendor calls and keeps
      // retrying a globally confirmed stop without re-entering CommandScheduler.
      m_robotContainer.serviceOutputSafetyHeartbeat();

      // TimedRobot calls autonomousPeriodic() before this common periodic block. Evaluate the
      // active-auto interlock only after this cycle's heartbeat has converted READY_DISABLED into
      // a live ARMED grant, and before any autonomous command executes in the scheduler.
      abortActiveAutonomousIfUnsafe();

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
        // Invalidate the prior FMS-suppressed latch before the first live write. If any later
        // publisher throws and the runtime latch trips, a subsequent FMS attach must still retry
        // suppression rather than preserving a partial live update.
        m_hardwareHealthSuppressedForFms = false;
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
        m_nextDiagnosticTimestamp = now + 5.0;
      }
      if (!DriverStation.isFMSAttached() && now >= m_nextDeviceEvidenceTimestamp) {
        m_deviceEvidenceSuppressedForFms = false;
        var sparkEvidence = SparkMAXContainer.getDeviceEvidenceSnapshots();
        var ctreEvidence = m_robotContainer.getSwerveDeviceEvidenceSnapshots();
        SparkDeviceEvidence.publish(sparkEvidence);
        CtreDeviceEvidence.publish(ctreEvidence);
        CanDeviceEvidenceSummary.publish(sparkEvidence, ctreEvidence);
        m_nextDeviceEvidenceTimestamp = now + 0.5;
      }
    } catch (RuntimeException exception) {
      latchRuntimeFault(exception);
    }
  }

  private void suppressHardwareEvidenceForFmsNoThrow() {
    boolean fmsAttached;
    try {
      fmsAttached = DriverStation.isFMSAttached();
    } catch (RuntimeException exception) {
      return;
    }
    if (!fmsAttached) {
      return;
    }
    // A later detach while the runtime latch remains active must republish RUNTIME_FAULT rather
    // than leaving SUPPRESSED_FMS frozen indefinitely.
    m_deviceEvidenceUnavailableForRuntimeFault = false;
    if (!m_hardwareHealthSuppressedForFms) {
      boolean sparkSuppressed = dashboardWriteNoThrow(
          () -> SmartDashboard.putString("Hardware/SPARK Health", "SUPPRESSED_FMS"));
      boolean ctreSuppressed = dashboardWriteNoThrow(
          () -> SmartDashboard.putString("Hardware/CTRE Health", "SUPPRESSED_FMS"));
      m_hardwareHealthSuppressedForFms = sparkSuppressed && ctreSuppressed;
    }
    if (!m_deviceEvidenceSuppressedForFms) {
      boolean sparkSuppressed = dashboardWriteNoThrow(SparkDeviceEvidence::publishSuppressedForFms);
      boolean ctreSuppressed = dashboardWriteNoThrow(CtreDeviceEvidence::publishSuppressedForFms);
      boolean summarySuppressed = dashboardWriteNoThrow(
          CanDeviceEvidenceSummary::publishSuppressedForFms);
      m_deviceEvidenceSuppressedForFms =
          sparkSuppressed && ctreSuppressed && summarySuppressed;
    }
  }

  private static boolean dashboardWriteNoThrow(Runnable write) {
    try {
      write.run();
      return true;
    } catch (RuntimeException exception) {
      return false;
    }
  }

  private void publishRuntimeFaultEvidenceUnavailableNoThrow() {
    boolean fmsAttached;
    try {
      fmsAttached = DriverStation.isFMSAttached();
    } catch (RuntimeException exception) {
      return;
    }
    if (fmsAttached || m_deviceEvidenceUnavailableForRuntimeFault) {
      return;
    }
    // Clear the FMS latch before the first write so a partial failure still forces the next attach
    // to retry complete suppression.
    m_deviceEvidenceSuppressedForFms = false;
    m_hardwareHealthSuppressedForFms = false;
    boolean sparkHealthUnavailable = dashboardWriteNoThrow(
        () -> SmartDashboard.putString("Hardware/SPARK Health", "RUNTIME_FAULT"));
    boolean ctreHealthUnavailable = dashboardWriteNoThrow(
        () -> SmartDashboard.putString("Hardware/CTRE Health", "RUNTIME_FAULT"));
    boolean sparkUnavailable = dashboardWriteNoThrow(
        () -> SparkDeviceEvidence.publishUnavailable("RUNTIME_FAULT"));
    boolean ctreUnavailable = dashboardWriteNoThrow(
        () -> CtreDeviceEvidence.publishUnavailable("RUNTIME_FAULT"));
    boolean summaryUnavailable = dashboardWriteNoThrow(
        () -> CanDeviceEvidenceSummary.publishUnavailable("RUNTIME_FAULT"));
    m_deviceEvidenceUnavailableForRuntimeFault =
        sparkHealthUnavailable
            && ctreHealthUnavailable
            && sparkUnavailable
            && ctreUnavailable
            && summaryUnavailable;
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
      }
    });
  }

  private void abortActiveAutonomousIfUnsafe() {
    if (!DriverStation.isAutonomousEnabled() || m_autonomousCommand == null) {
      return;
    }
    if (!CommandScheduler.getInstance().isScheduled(m_autonomousCommand)) {
      m_autonomousCommand = null;
      return;
    }
    var outputSafetySnapshot = m_robotContainer.getOutputSafetySnapshot();
    if (m_robotContainer.shouldAbortActiveAutonomous(outputSafetySnapshot)) {
      m_autonomousCommand.cancel();
      m_autonomousCommand = null;
      m_robotContainer.stopAll();
    }
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
          && m_robotContainer.isOutputSafetyReadyForEnable()
          && selfTestRequested
          && m_selfTestArmGate.consume(now);
      Target diagnosticTarget = m_unhomedDiagnosticTargetSnapshot;
      Direction diagnosticDirection = m_unhomedDiagnosticDirectionSnapshot;
      double diagnosticExpiresAt = m_unhomedDiagnosticSnapshotExpiresAt;
      boolean unhomedDiagnosticAccepted = !conflictingArms
          && !DriverStation.isFMSAttached()
          && m_robotContainer.isOutputSafetyReadyForEnable()
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
          && ManualUnhomedActuatorDiagnosticCommand.targetSpecificVerificationSatisfied(
              diagnosticTarget)
          && m_unhomedDiagnosticArmGate.consume(now);
      clearSelfTestArm();
      clearUnhomedDiagnosticArm(!unhomedDiagnosticAccepted);
      if (unhomedDiagnosticAccepted) {
        m_robotContainer.stopAllPreservingPreparedUnhomedDiagnosticSession();
      } else {
        m_robotContainer.stopAll();
      }
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
        m_robotContainer.armUnhomedDiagnosticSession(
            diagnosticTarget, diagnosticDirection, diagnosticExpiresAt);
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
    clearUnhomedDiagnosticArm(true);
  }

  private void clearUnhomedDiagnosticArm(boolean discardPreparedSession) {
    SmartDashboard.putBoolean(ManualUnhomedActuatorDiagnosticCommand.ARM_KEY, false);
    SmartDashboard.putBoolean(ManualUnhomedActuatorDiagnosticCommand.ARM_VALID_KEY, false);
    SmartDashboard.putString(
        ManualUnhomedActuatorDiagnosticCommand.STATUS_KEY, "DISARMED");
    m_unhomedDiagnosticArmGate.requireRelease();
    m_unhomedDiagnosticTargetSnapshot = null;
    m_unhomedDiagnosticDirectionSnapshot = null;
    m_unhomedDiagnosticSnapshotExpiresAt = Double.NEGATIVE_INFINITY;
    if (discardPreparedSession) {
      m_robotContainer.discardPreparedUnhomedDiagnosticSession();
    }
  }

  private void clearUnhomedDiagnosticVerifications() {
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.PHYSICAL_CLEARANCE_KEY, false);
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.MOTOR_TYPE_VERIFIED_KEY, false);
    SmartDashboard.putBoolean(
        ManualUnhomedActuatorDiagnosticCommand.FEEDER_REPAIR_VERIFIED_KEY, false);
  }

  private void updateDiagnosticArmGates() {
    boolean selfTestRequested = SmartDashboard.getBoolean(
        "Hardware Self-Test/Armed", false);
    boolean unhomedRequested = SmartDashboard.getBoolean(
        ManualUnhomedActuatorDiagnosticCommand.ARM_KEY, false);
    boolean conflictingArms = selfTestRequested && unhomedRequested;
    boolean disabledTestWithoutFms = DriverStation.isDisabled()
        && DriverStation.isTest()
        && !DriverStation.isFMSAttached()
        && m_robotContainer.isOutputSafetyReadyForEnable();
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
        m_unhomedDiagnosticSnapshotExpiresAt = Double.NEGATIVE_INFINITY;
        m_robotContainer.discardPreparedUnhomedDiagnosticSession();
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
        m_robotContainer.discardPreparedUnhomedDiagnosticSession();
      } else {
        unhomedArmValid = m_unhomedDiagnosticArmGate.observe(
            unhomedRequested,
            disabledTestWithoutFms
                && !selfTestRequested
                && unhomedVerifications
                && selectedTarget.isPresent()
                && selectedDirection.isPresent()
                && ManualUnhomedActuatorDiagnosticCommand
                    .targetSpecificVerificationSatisfied(selectedTarget.orElse(null)),
            now);
        if (unhomedArmValid && m_unhomedDiagnosticTargetSnapshot == null) {
          m_unhomedDiagnosticTargetSnapshot = selectedTarget.orElseThrow();
          m_unhomedDiagnosticDirectionSnapshot = selectedDirection.orElseThrow();
          m_unhomedDiagnosticSnapshotExpiresAt =
              m_unhomedDiagnosticArmGate.expiresAtSeconds();
          boolean prepared = m_robotContainer.prepareUnhomedDiagnosticSession(
              m_unhomedDiagnosticTargetSnapshot,
              m_unhomedDiagnosticDirectionSnapshot,
              m_unhomedDiagnosticSnapshotExpiresAt);
          if (!prepared) {
            m_unhomedDiagnosticArmGate.invalidate(true);
            m_unhomedDiagnosticTargetSnapshot = null;
            m_unhomedDiagnosticDirectionSnapshot = null;
            m_unhomedDiagnosticSnapshotExpiresAt = Double.NEGATIVE_INFINITY;
            unhomedArmValid = false;
            SmartDashboard.putString(
                ManualUnhomedActuatorDiagnosticCommand.STATUS_KEY,
                "PREPARE_REJECTED_RELEASE_ARM");
          }
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
        } else if (unhomedRequested && (!unhomedVerifications
            || !ManualUnhomedActuatorDiagnosticCommand
                .targetSpecificVerificationSatisfied(selectedTarget.orElse(null)))) {
          SmartDashboard.putString(
              ManualUnhomedActuatorDiagnosticCommand.STATUS_KEY,
              selectedTarget.filter(target -> target == Target.FEEDER).isPresent()
                  ? "VERIFY_CLEARANCE_MOTOR_TYPE_AND_ID32_REPAIR"
                  : "VERIFY_CLEARANCE_AND_MOTOR_TYPE");
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
    m_robotContainer.tripOutputSafety("RUNTIME_FAULT_" + fault.reason());
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
    m_robotContainer.tripOutputSafety("RUNTIME_FAULT_LATCHED");
    m_robotContainer.stopAll();
    try {
      // SPARK stops are queued so the dedicated worker can preserve zero/nonzero ordering.
      SparkMAXContainer.serviceAll();
    } catch (RuntimeException ignored) {
      // Keep retrying the stop request on later robot periods without re-entering the scheduler.
    }
  }

  /** Advances the explicit non-physical SPARK command echo used only by desktop simulation. */
  @Override
  public void simulationPeriodic() {
    if (!m_runtimeSafetyLatch.healthy()) {
      try {
        // Never re-enter the poisoned scheduler, but do not freeze raw simulated telemetry at a
        // previously nonzero value while the authoritative queued stop is being retried.
        m_robotContainer.simulationPeriodic(false);
      } catch (RuntimeException ignored) {
        // enforceLatchedStop remains authoritative when the simulation façade itself fails.
      }
      enforceLatchedStop();
      return;
    }
    runLifecycleSafely(m_robotContainer::simulationPeriodic);
  }
}
