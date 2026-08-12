// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.FireCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.HardwareSelfTestCommand;
import frc.robot.commands.JumpBumpCommand;
import frc.robot.commands.ManualUnhomedActuatorDiagnosticCommand;
import frc.robot.commands.ManualUnhomedActuatorDiagnosticCommand.Direction;
import frc.robot.commands.ManualUnhomedActuatorDiagnosticCommand.Target;
import frc.robot.commands.RevUpCommand;
import frc.robot.commands.OutputCommand;
import frc.robot.commands.RetractIntakeCommand;
import frc.robot.constants.Constants.LimelightConstants;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.constants.ConfiguredOperatorControls;
import frc.robot.constants.Constants.HardwareTestConstants;
import frc.robot.containers.DriveBaseContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.SparkMAXContainer;
import frc.robot.utils.SparkRawCommandEchoSimulation;
import frc.robot.utils.NeutralAfterEnableGate;
import frc.robot.utils.AsyncDiagnosticSink;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.concurrent.atomic.AtomicBoolean;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements AutoCloseable {
  private final AtomicBoolean closed = new AtomicBoolean();
  private enum IntakePathAction {
    INTAKE,
    OUTPUT,
    FIRE,
    RETRACT
  }

  private final NeutralAfterEnableGate m_teleopInputGate = new NeutralAfterEnableGate();
  private final NeutralAfterEnableGate m_unhomedDiagnosticInputGate =
      new NeutralAfterEnableGate();
  private long m_teleopSafetySourceSignature;
  private Target m_unhomedDiagnosticTarget;
  private Direction m_unhomedDiagnosticDirection;
  private double m_unhomedDiagnosticExpiresAt = Double.NEGATIVE_INFINITY;

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

    SmartDashboard.putString(
        "Controls/Configured", ConfiguredOperatorControls.configuredSummary());
    if (RobotBase.isSimulation()) {
      SmartDashboard.putString(
          "Simulation/SPARK Model", SparkRawCommandEchoSimulation.SOURCE);
    }

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

    availableButton(
        m_driverController,
        OIConstants.kDriverControllerPort,
        ConfiguredOperatorControls.DRIVER_JUMP_BUMP)
        .and(availableButton(
            m_driverController,
            OIConstants.kDriverControllerPort,
            ConfiguredOperatorControls.DRIVER_WHEEL_LOCK).negate())
        .whileTrue(jumpBump);

    availableButton(
        m_driverController,
        OIConstants.kDriverControllerPort,
        ConfiguredOperatorControls.DRIVER_REV).whileTrue(revWheel);
    exclusiveIntakePathButton(IntakePathAction.FIRE).whileTrue(fire);

    exclusiveIntakePathButton(IntakePathAction.RETRACT).whileTrue(back_in_shell);

    operatorOrDriverButton(
        m_maintenanceController,
        OIConstants.kMaintenanceControllerPort,
        ConfiguredOperatorControls.MAINTENANCE_AUTO_AIM,
        ConfiguredOperatorControls.DRIVER_AUTO_AIM_FALLBACK)
        .whileTrue(new RunCommand(() -> m_turret.autoAimWithLimelight(), m_turret)
            .finallyDo(interrupted -> m_turret.stop()));

    configureUnhomedDiagnosticBindings();
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
      boolean intake = rawButtonPressed(
          m_driverController,
          OIConstants.kDriverControllerPort,
          ConfiguredOperatorControls.DRIVER_INTAKE);
      boolean output = rawButtonPressed(
          m_driverController,
          OIConstants.kDriverControllerPort,
          ConfiguredOperatorControls.DRIVER_OUTPUT);
      boolean firePressed = rawButtonPressed(
          m_driverController,
          OIConstants.kDriverControllerPort,
          ConfiguredOperatorControls.DRIVER_FIRE);
      boolean retract = operatorOrDriverPressed(
          m_operatorController,
          OIConstants.kOperatorControllerPort,
          ConfiguredOperatorControls.OPERATOR_RETRACT,
          ConfiguredOperatorControls.DRIVER_RETRACT_FALLBACK);
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
    boolean anyPressed = false;
    for (int button : ConfiguredOperatorControls.driverSafetyButtons()) {
      anyPressed |= rawButtonPressed(
          m_driverController, OIConstants.kDriverControllerPort, button);
    }
    anyPressed |= rawButtonPressed(
        m_operatorController,
        OIConstants.kOperatorControllerPort,
        ConfiguredOperatorControls.OPERATOR_RETRACT);
    anyPressed |= rawButtonPressed(
        m_maintenanceController,
        OIConstants.kMaintenanceControllerPort,
        ConfiguredOperatorControls.MAINTENANCE_AUTO_AIM);
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

  private void configureUnhomedDiagnosticBindings() {
    for (Target target : Target.values()) {
      bindUnhomedDiagnostic(
          target,
          Direction.NEGATIVE,
          ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_NEGATIVE,
          ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_POSITIVE);
      bindUnhomedDiagnostic(
          target,
          Direction.POSITIVE,
          ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_POSITIVE,
          ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_NEGATIVE);
    }
  }

  private void bindUnhomedDiagnostic(
      Target target,
      Direction direction,
      int selectedDirectionButton,
      int oppositeDirectionButton) {
    new Trigger(() -> unhomedDiagnosticPressed(target, direction, selectedDirectionButton))
        // The command owns the requirements until post-stop evidence is confirmed (or times out).
        // Releasing the gesture is observed through interlocksHeld and starts POST_STOP; it must
        // not cancel the command before that evidence can be collected.
        .onTrue(new ManualUnhomedActuatorDiagnosticCommand(
            target,
            direction.duty(),
            () -> unhomedDiagnosticInterlocksHeld(
                target, direction, selectedDirectionButton, oppositeDirectionButton),
            m_intake,
            m_shooter,
            m_turret,
            m_climber,
            drivetrain,
            m_intake,
            m_conveyor,
            m_feeder,
            m_shooter,
            m_turret,
            m_climber)
            // One consumed dashboard arm authorizes at most one scheduled pulse. A second pulse
            // requires returning to Disabled Test and creating a new target snapshot.
            .finallyDo(interrupted -> disarmUnhomedDiagnosticSession())
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
  }

  private boolean unhomedDiagnosticPressed(
      Target target, Direction direction, int selectedDirectionButton) {
    if (target != m_unhomedDiagnosticTarget || direction != m_unhomedDiagnosticDirection) {
      return false;
    }
    boolean deadman = maintenanceButtonPressed(
        ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_DEADMAN);
    boolean negative = maintenanceButtonPressed(
        ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_NEGATIVE);
    boolean positive = maintenanceButtonPressed(
        ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_POSITIVE);
    int directionsPressed = (negative ? 1 : 0) + (positive ? 1 : 0);
    if (directionsPressed > 1) {
      m_unhomedDiagnosticInputGate.blockUntilNeutral();
    }
    boolean anyPressed = deadman || negative || positive;
    long sourceSignature = DriverStation.getStickButtonCount(
        OIConstants.kMaintenanceControllerPort)
        | ((long) target.ordinal() << 16)
        | ((long) direction.ordinal() << 20);
    if (!m_unhomedDiagnosticInputGate.allow(
        unhomedDiagnosticSessionAllowed(target), sourceSignature, anyPressed)) {
      return false;
    }
    return deadman
        && directionsPressed == 1
        && maintenanceButtonPressed(selectedDirectionButton);
  }

  private boolean unhomedDiagnosticInterlocksHeld(
      Target target,
      Direction direction,
      int selectedDirectionButton,
      int oppositeDirectionButton) {
    return unhomedDiagnosticSessionAllowed(target, direction)
        && maintenanceButtonPressed(ConfiguredOperatorControls.UNHOMED_DIAGNOSTIC_DEADMAN)
        && maintenanceButtonPressed(selectedDirectionButton)
        && !maintenanceButtonPressed(oppositeDirectionButton);
  }

  private boolean unhomedDiagnosticSessionAllowed(Target target) {
    return unhomedDiagnosticSessionAllowed(target, m_unhomedDiagnosticDirection);
  }

  private boolean unhomedDiagnosticSessionAllowed(Target target, Direction direction) {
    boolean unexpired = Double.isFinite(m_unhomedDiagnosticExpiresAt)
        && Timer.getFPGATimestamp() <= m_unhomedDiagnosticExpiresAt;
    if (!unexpired && m_unhomedDiagnosticTarget != null) {
      disarmUnhomedDiagnosticSession();
      SmartDashboard.putString(
          ManualUnhomedActuatorDiagnosticCommand.STATUS_KEY,
          "SESSION_EXPIRED_REARM_DISABLED");
    }
    boolean selectionAndVerificationMatch = target == m_unhomedDiagnosticTarget
        && direction == m_unhomedDiagnosticDirection
        && SmartDashboard.getBoolean(
            ManualUnhomedActuatorDiagnosticCommand.PHYSICAL_CLEARANCE_KEY, false)
        && SmartDashboard.getBoolean(
            ManualUnhomedActuatorDiagnosticCommand.MOTOR_TYPE_VERIFIED_KEY, false)
        && ManualUnhomedActuatorDiagnosticCommand.readExactlyOneTarget()
            .filter(selected -> selected == target)
            .isPresent()
        && ManualUnhomedActuatorDiagnosticCommand.readExactlyOneDirection()
            .filter(selected -> selected == direction)
            .isPresent();
    if (!selectionAndVerificationMatch && m_unhomedDiagnosticTarget != null) {
      disarmUnhomedDiagnosticSession();
      SmartDashboard.putString(
          ManualUnhomedActuatorDiagnosticCommand.STATUS_KEY,
          "SESSION_INVALIDATED_REARM_DISABLED");
      return false;
    }
    return unexpired
        && selectionAndVerificationMatch
        && DriverStation.isTestEnabled()
        && !DriverStation.isFMSAttached()
        && !SmartDashboard.getBoolean(HardwareSelfTestCommand.RUNNING_KEY, false);
  }

  private boolean maintenanceButtonPressed(int button) {
    return rawButtonPressed(
        m_maintenanceController, OIConstants.kMaintenanceControllerPort, button);
  }

  /** Starts the already consumed, disabled-mode target snapshot for this Test session. */
  public void armUnhomedDiagnosticSession(Target target, Direction direction) {
    m_unhomedDiagnosticTarget = target;
    m_unhomedDiagnosticDirection = direction;
    m_unhomedDiagnosticExpiresAt = Timer.getFPGATimestamp()
        + HardwareTestConstants.ARM_LIFETIME_SECONDS;
    SmartDashboard.putString(
        ManualUnhomedActuatorDiagnosticCommand.STATUS_KEY,
        "ARMED_" + target.label() + "_" + direction.name() + "_RELEASE_CONTROLS");
    SmartDashboard.putString(
        ManualUnhomedActuatorDiagnosticCommand.STOP_EVIDENCE_KEY, "NOT_RUN");
    m_unhomedDiagnosticInputGate.blockUntilNeutral();
  }

  public void disarmUnhomedDiagnosticSession() {
    m_unhomedDiagnosticTarget = null;
    m_unhomedDiagnosticDirection = null;
    m_unhomedDiagnosticExpiresAt = Double.NEGATIVE_INFINITY;
    m_unhomedDiagnosticInputGate.blockUntilNeutral();
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

  /** Applies raw command echo telemetry; it is not a mechanism physics model or reference source. */
  public void simulationPeriodic() {
    simulationPeriodic(DriverStation.isEnabled());
  }

  /** Applies one simulation response with an explicit output permission. */
  public void simulationPeriodic(boolean outputsAllowed) {
    SparkRawCommandEchoSimulation.stepConfiguredControllers(outputsAllowed);
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
    disarmUnhomedDiagnosticSession();
    stopSafely("swerve", drivetrain::requestIdle);
    stopSafely("intake", m_intake::stopAll);
    stopSafely("conveyor", m_conveyor::stop);
    stopSafely("feeder", m_feeder::stop);
    stopSafely("shooter", m_shooter::stop);
    stopSafely("turret", m_turret::stop);
    stopSafely("climber", m_climber::stop);
  }

  /** Releases process-owned simulation/native resources after all outputs are requested neutral. */
  @Override
  public void close() {
    if (!closed.compareAndSet(false, true)) {
      return;
    }
    try {
      stopAll();
    } finally {
      m_DriveBaseContainer.close();
    }
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
