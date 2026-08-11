// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.FireCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.HardwareSelfTestCommand;
import frc.robot.commands.JumpBumpCommand;
import frc.robot.commands.RevUpCommand;
import frc.robot.commands.OutputCommand;
import frc.robot.commands.RetractIntakeCommand;
import frc.robot.constants.Constants.LimelightConstants;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.containers.DriveBaseContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ClimberDiagnosticLatch.MotorSide;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.SparkMAXContainer;
import frc.robot.utils.NeutralAfterEnableGate;
import frc.robot.utils.AsyncDiagnosticSink;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private enum IntakePathAction {
    INTAKE,
    OUTPUT,
    FIRE,
    RETRACT
  }

  private final NeutralAfterEnableGate m_teleopInputGate = new NeutralAfterEnableGate();
  private final NeutralAfterEnableGate m_climberInputGate = new NeutralAfterEnableGate();
  private long m_teleopSafetySourceSignature;

  // The driver's controller
  private final CommandPS5Controller m_driverController = new CommandPS5Controller(OIConstants.kDriverControllerPort);
  private final CommandPS5Controller m_operatorController = new CommandPS5Controller(OIConstants.kOperatorControllerPort);
  private final CommandPS5Controller m_maintenanceController = new CommandPS5Controller(OIConstants.kMaintenanceControllerPort);

  // The robot's subsystems
  private final VisionSubsystem m_turretVision = new VisionSubsystem(
      LimelightConstants.TURRET_LIMELIGHT_NAME,
      LimelightConstants.PIPELINE_APRILTAG);

  private final CommandSwerveDrivetrain drivetrain;

  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final TurretSubsystem m_turret = new TurretSubsystem(m_turretVision);
  private final ConveyorSubsystem m_conveyor = new ConveyorSubsystem();
  private final FeederSubsystem m_feeder = new FeederSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();

  // The robot's commands
  private final JumpBumpCommand jumpBump;

  private final IntakeCommand slurp = new IntakeCommand(m_intake, m_conveyor);
  private final OutputCommand spit = new OutputCommand(m_intake, m_conveyor);

  private final RetractIntakeCommand back_in_shell = new RetractIntakeCommand(m_intake);

  private final FireCommand fire = new FireCommand(m_feeder, m_conveyor, m_shooter);
  private final RevUpCommand revWheel = new RevUpCommand(m_shooter);


  // Something?
  private final DriveBaseContainer m_DriveBaseContainer; 

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_DriveBaseContainer = new DriveBaseContainer(
        m_driverController, m_turret, m_shooter, m_feeder, m_conveyor, m_intake, m_climber);
    drivetrain = m_DriveBaseContainer.drivetrain;

    jumpBump = new JumpBumpCommand(
        drivetrain,
        m_driverController,
        m_DriveBaseContainer::driverInputsAllowed);

    // Configure the button bindings (put this last)
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link PS5Controller}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Probably change (all of) this

    // Note;
    /*
     * Driver controls driving, intake, aiming, reving, and shooting
     * 
     */

    exclusiveIntakePathButton(IntakePathAction.INTAKE).whileTrue(slurp);
    exclusiveIntakePathButton(IntakePathAction.OUTPUT).whileTrue(spit);

    availableButton(m_driverController, OIConstants.kDriverControllerPort, 12)
        .and(availableButton(m_driverController, OIConstants.kDriverControllerPort, 14).negate())
        .whileTrue(jumpBump);

    availableButton(m_driverController, OIConstants.kDriverControllerPort, 7).whileTrue(revWheel);
    exclusiveIntakePathButton(IntakePathAction.FIRE).whileTrue(fire);

    exclusiveIntakePathButton(IntakePathAction.RETRACT).whileTrue(back_in_shell);

    operatorOrDriverButton(
        m_maintenanceController,
        OIConstants.kMaintenanceControllerPort,
        5,
        4)
        .whileTrue(new RunCommand(() -> m_turret.autoAimWithLimelight(), m_turret)
            .finallyDo(interrupted -> m_turret.stop()));

    configureClimberDiagnosticBindings();
  }

  private Trigger availableButton(CommandPS5Controller controller, int port, int button) {
    return new Trigger(() -> teleopInputsAllowed()
        && DriverStation.getStickButtonCount(port) >= button
        && controller.getHID().getRawButton(button));
  }

  private Trigger exclusiveIntakePathButton(IntakePathAction requestedAction) {
    return new Trigger(() -> {
      if (!teleopInputsAllowed()) {
        return false;
      }
      boolean intake = rawButtonPressed(m_driverController, OIConstants.kDriverControllerPort, 5);
      boolean output = rawButtonPressed(m_driverController, OIConstants.kDriverControllerPort, 6);
      boolean firePressed = rawButtonPressed(m_driverController, OIConstants.kDriverControllerPort, 8);
      boolean retract = operatorOrDriverPressed(
          m_operatorController,
          OIConstants.kOperatorControllerPort,
          5,
          1);
      int pressedCount = (intake ? 1 : 0)
          + (output ? 1 : 0)
          + (firePressed ? 1 : 0)
          + (retract ? 1 : 0);
      if (pressedCount > 1) {
        m_teleopInputGate.blockUntilNeutral();
        return false;
      }
      if (pressedCount != 1) {
        return false;
      }
      return switch (requestedAction) {
        case INTAKE -> intake;
        case OUTPUT -> output;
        case FIRE -> firePressed;
        case RETRACT -> retract;
      };
    });
  }

  private Trigger operatorOrDriverButton(
      CommandPS5Controller dedicatedController,
      int dedicatedPort,
      int dedicatedButton,
      int driverFallbackButton) {
    return new Trigger(() -> teleopInputsAllowed()
        && operatorOrDriverPressed(
            dedicatedController, dedicatedPort, dedicatedButton, driverFallbackButton));
  }

  private boolean teleopInputsAllowed() {
    int[] driverButtons = {1, 4, 5, 6, 7, 8, 9, 12, 14};
    boolean anyPressed = false;
    for (int button : driverButtons) {
      anyPressed |= rawButtonPressed(
          m_driverController, OIConstants.kDriverControllerPort, button);
    }
    anyPressed |= rawButtonPressed(m_operatorController, OIConstants.kOperatorControllerPort, 5);
    anyPressed |= rawButtonPressed(
        m_maintenanceController, OIConstants.kMaintenanceControllerPort, 5);
    return m_teleopInputGate.allow(
        DriverStation.isTeleopEnabled(), m_teleopSafetySourceSignature, anyPressed);
  }

  /** Samples controller topology and mechanism health once before each scheduler iteration. */
  public void updateTeleopSafetyState() {
    long signature = DriverStation.getStickButtonCount(OIConstants.kDriverControllerPort) & 0xffL;
    signature |= (DriverStation.getStickButtonCount(OIConstants.kOperatorControllerPort) & 0xffL)
        << 8;
    signature |= (DriverStation.getStickButtonCount(OIConstants.kMaintenanceControllerPort) & 0xffL)
        << 16;
    signature |= m_intake.isActuatorReferenced() ? 1L << 24 : 0L;
    signature |= m_shooter.isActuatorReferenced() ? 1L << 25 : 0L;
    signature |= m_turret.isPositionControlReadyForAutonomousAim() ? 1L << 26 : 0L;
    signature |= m_turret.isVisionReadyForAutonomousAim() ? 1L << 27 : 0L;
    signature |= m_feeder.isReady() ? 1L << 28 : 0L;
    signature |= m_conveyor.isReady() ? 1L << 29 : 0L;
    signature |= SparkMAXContainer.getReadyCanIdMask();
    m_teleopSafetySourceSignature = signature;
  }

  private boolean operatorOrDriverPressed(
      CommandPS5Controller dedicatedController,
      int dedicatedPort,
      int dedicatedButton,
      int driverFallbackButton) {
    if (DriverStation.getStickButtonCount(dedicatedPort) >= dedicatedButton) {
      return dedicatedController.getHID().getRawButton(dedicatedButton);
    }
    return rawButtonPressed(
        m_driverController, OIConstants.kDriverControllerPort, driverFallbackButton);
  }

  private static boolean rawButtonPressed(
      CommandPS5Controller controller, int port, int button) {
    return DriverStation.getStickButtonCount(port) >= button
        && controller.getHID().getRawButton(button);
  }

  private void configureClimberDiagnosticBindings() {
    bindClimberDiagnostic(1, MotorSide.LEFT, 1.0);
    bindClimberDiagnostic(2, MotorSide.LEFT, -1.0);
    bindClimberDiagnostic(3, MotorSide.RIGHT, 1.0);
    bindClimberDiagnostic(4, MotorSide.RIGHT, -1.0);
  }

  private void bindClimberDiagnostic(int faceButton, MotorSide side, double sign) {
    boolean[] active = {false};
    new Trigger(() -> climberDiagnosticPressed(faceButton))
        .whileTrue(new FunctionalCommand(
            () -> active[0] = m_climber.canStartDiagnostic(),
            () -> {
              if (active[0]) {
                active[0] = m_climber.runDiagnostic(
                    side, sign * ClimberConstants.DIAGNOSTIC_MAX_DUTY_CYCLE);
              }
            },
            interrupted -> {
              active[0] = false;
              m_climber.stop();
            },
            () -> !active[0],
            m_climber)
            .withTimeout(ClimberConstants.DIAGNOSTIC_PULSE_SECONDS)
            .finallyDo(interrupted -> m_climber.stop()));
  }

  private boolean climberDiagnosticPressed(int selectedFaceButton) {
    CommandPS5Controller controller = DriverStation.getStickButtonCount(
        OIConstants.kMaintenanceControllerPort) >= 10
            ? m_maintenanceController
            : m_driverController;
    int port = controller == m_maintenanceController
        ? OIConstants.kMaintenanceControllerPort
        : OIConstants.kDriverControllerPort;
    boolean deadmanPressed = rawButtonPressed(controller, port, 10);
    int pressedFaces = 0;
    for (int button = 1; button <= 4; button++) {
      if (rawButtonPressed(controller, port, button)) {
        pressedFaces++;
      }
    }
    if (pressedFaces > 1) {
      m_climberInputGate.blockUntilNeutral();
    }
    boolean interlocksEnabled = DriverStation.isTestEnabled()
        && !DriverStation.isFMSAttached()
        && SmartDashboard.getBoolean(ClimberSubsystem.DIAGNOSTIC_ARM_KEY, false)
        && SmartDashboard.getBoolean(ClimberSubsystem.MOTOR_TYPE_VERIFIED_KEY, false);
    boolean anyPressed = deadmanPressed || pressedFaces > 0;
    int sourceSignature = (port << 16) | DriverStation.getStickButtonCount(port);
    if (!m_climberInputGate.allow(interlocksEnabled, sourceSignature, anyPressed)) {
      return false;
    }
    return deadmanPressed
        && pressedFaces == 1
        && rawButtonPressed(controller, port, selectedFaceButton);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return this.m_DriveBaseContainer.GetAutonCommand();
  }

  public boolean shouldAbortActiveAutonomous() {
    return m_DriveBaseContainer.shouldAbortActiveAutonomous();
  }

  public void refreshAutonomousStatus() {
    m_DriveBaseContainer.refreshAutonomousStatus();
  }

  public Command getHardwareSelfTestCommand() {
    return HardwareSelfTestCommand.create(
        drivetrain, m_intake, m_conveyor, m_feeder, m_shooter, m_turret, m_climber);
  }

  public String getSwerveDeviceHealthSummary() {
    return drivetrain.getDeviceHealthSummary();
  }

  public String getSparkDeviceHealthSummary() {
    return SparkMAXContainer.getDeviceAvailabilitySummary();
  }

  public void stopAll() {
    stopSafely("swerve", drivetrain::requestIdle);
    stopSafely("intake", m_intake::stopAll);
    stopSafely("conveyor", m_conveyor::stop);
    stopSafely("feeder", m_feeder::stop);
    stopSafely("shooter", m_shooter::stop);
    stopSafely("turret", m_turret::stop);
    stopSafely("climber", m_climber::stop);
    stopSafely(
        "climber diagnostic arm",
        () -> SmartDashboard.putBoolean(ClimberSubsystem.DIAGNOSTIC_ARM_KEY, false));
  }

  private static void stopSafely(String target, Runnable stopAction) {
    try {
      stopAction.run();
    } catch (RuntimeException exception) {
      AsyncDiagnosticSink.log(
          "STOP FAILED target=" + target + " error=" + exception.getClass().getSimpleName());
    }
  }
}
