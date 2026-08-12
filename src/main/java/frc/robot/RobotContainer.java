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
import frc.robot.utils.OneShotMotorRetestLease.Token;
import frc.robot.commands.RevUpCommand;
import frc.robot.commands.OutputCommand;
import frc.robot.commands.RetractIntakeCommand;
import frc.robot.constants.Constants.LimelightConstants;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.constants.ConfiguredOperatorControls;
import frc.robot.constants.ConfiguredOperatorActions.Action;
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
import frc.robot.utils.OperatorActionEvidence;
import frc.robot.utils.AsyncDiagnosticSink;
import frc.robot.utils.RobotOutputSafetySupervisor;
import frc.robot.utils.RobotOutputSafetySupervisor.StopSession;
import frc.robot.utils.RobotOutputSafetySupervisor.Snapshot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.EnumSet;

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
  private final OperatorActionEvidence m_operatorActionEvidence =
      new OperatorActionEvidence();
  private final EnumSet<Action> m_conflictingIntakeActions = EnumSet.noneOf(Action.class);
  private boolean m_jumpBumpRequiresRelease;
  private long m_teleopSafetySourceSignature;
  private Target m_unhomedDiagnosticTarget;
  private Direction m_unhomedDiagnosticDirection;
  private double m_unhomedDiagnosticExpiresAt = Double.NEGATIVE_INFINITY;
  private Token m_feederManualRetestToken;
  private Target m_preparedUnhomedDiagnosticTarget;
  private Direction m_preparedUnhomedDiagnosticDirection;
  private double m_preparedUnhomedDiagnosticExpiresAt = Double.NEGATIVE_INFINITY;

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

  private final IntakeCommand slurp = new IntakeCommand(
      m_intake, m_conveyor, m_operatorActionEvidence);
  private final OutputCommand spit = new OutputCommand(
      m_intake, m_conveyor, m_operatorActionEvidence);

  private final RetractIntakeCommand back_in_shell = new RetractIntakeCommand(
      m_intake, m_operatorActionEvidence);

  private final FireCommand fire = new FireCommand(
      m_feeder, m_conveyor, m_shooter, m_operatorActionEvidence);
  private final RevUpCommand revWheel = new RevUpCommand(
      m_shooter, m_operatorActionEvidence);


  // Something?
  private final DriveBaseContainer m_DriveBaseContainer; 
  private final RobotOutputSafetySupervisor m_outputSafetySupervisor;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_DriveBaseContainer = new DriveBaseContainer(
        m_driverController, m_turret, m_shooter, m_feeder, m_conveyor, m_intake, m_climber,
        m_operatorActionEvidence);
    drivetrain = m_DriveBaseContainer.drivetrain;

    jumpBump = new JumpBumpCommand(
        drivetrain,
        m_driverController,
        m_DriveBaseContainer::driverInputsAllowed,
        m_DriveBaseContainer::blockDriverInputsUntilNeutral,
        m_operatorActionEvidence);

    SmartDashboard.putString(
        "Controls/Configured", ConfiguredOperatorControls.configuredSummary());
    if (RobotBase.isSimulation()) {
      SmartDashboard.putString(
          "Simulation/SPARK Model", SparkRawCommandEchoSimulation.SOURCE);
    }

    // Configure the button bindings (put this last)
    configureButtonBindings();
    m_outputSafetySupervisor = new RobotOutputSafetySupervisor(
        this::beginIndependentOutputStopSession);
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
        ConfiguredOperatorControls.DRIVER_JUMP_BUMP,
        Action.JUMP_BUMP)
        .and(new Trigger(this::jumpBumpHasPriority))
        .whileTrue(jumpBump);

    availableButton(
        m_driverController,
        OIConstants.kDriverControllerPort,
        ConfiguredOperatorControls.DRIVER_REV,
        Action.REV).whileTrue(revWheel);
    exclusiveIntakePathButton(IntakePathAction.FIRE).whileTrue(fire);

    exclusiveIntakePathButton(IntakePathAction.RETRACT).whileTrue(back_in_shell);

    operatorOrDriverButton(
        m_maintenanceController,
        OIConstants.kMaintenanceControllerPort,
        ConfiguredOperatorControls.MAINTENANCE_AUTO_AIM,
        ConfiguredOperatorControls.DRIVER_AUTO_AIM_FALLBACK,
        Action.AUTO_AIM)
        .whileTrue(new RunCommand(this::runAutoAimWithEvidence, m_turret)
            .beforeStarting(() -> m_operatorActionEvidence.requested(Action.AUTO_AIM))
            .finallyDo(interrupted -> {
              m_turret.stop();
              m_operatorActionEvidence.commandEnded(Action.AUTO_AIM, interrupted);
            }));

    configureUnhomedDiagnosticBindings();
  }

  private Trigger availableButton(
      CommandPS5Controller controller, int port, int button, Action action) {
    return new Trigger(() -> {
      boolean inputsAllowed = teleopInputsAllowed();
      boolean pressed = rawButtonPressed(controller, port, button);
      if (!pressed) {
        if (action == Action.JUMP_BUMP) {
          m_jumpBumpRequiresRelease = false;
        }
        recordReleased(action);
        return false;
      }
      if (!inputsAllowed) {
        m_operatorActionEvidence.blocked(action, mechanismGateBlockReason());
        return false;
      }
      return true;
    });
  }

  private Trigger exclusiveIntakePathButton(IntakePathAction requestedAction) {
    return new Trigger(() -> {
      boolean inputsAllowed = teleopInputsAllowed();
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
      boolean ignoredRetractFallback = dedicatedControllerPresent(
              OIConstants.kOperatorControllerPort,
              ConfiguredOperatorControls.OPERATOR_RETRACT)
          && rawButtonPressed(
              m_driverController,
              OIConstants.kDriverControllerPort,
              ConfiguredOperatorControls.DRIVER_RETRACT_FALLBACK)
          && !rawButtonPressed(
              m_operatorController,
              OIConstants.kOperatorControllerPort,
              ConfiguredOperatorControls.OPERATOR_RETRACT);
      int pressedCount = (intake ? 1 : 0)
          + (output ? 1 : 0)
          + (firePressed ? 1 : 0)
          + (retract ? 1 : 0);
      Action action = operatorAction(requestedAction);
      boolean requestedPressed = switch (requestedAction) {
        case INTAKE -> intake;
        case OUTPUT -> output;
        case FIRE -> firePressed;
        case RETRACT -> retract;
      };
      if (requestedAction == IntakePathAction.RETRACT && ignoredRetractFallback) {
        m_operatorActionEvidence.blocked(
            Action.RETRACT, "DEDICATED_OPERATOR_PRESENT_USE_OPERATOR_L1");
        return false;
      }
      if (pressedCount > 1) {
        m_teleopInputGate.blockUntilNeutral();
        recordIntakePathConflict(intake, output, firePressed, retract);
        return false;
      }
      if (pressedCount == 0) {
        clearReleasedIntakeConflicts();
        recordReleased(action);
        return false;
      }
      if (!inputsAllowed) {
        if (requestedPressed) {
          m_operatorActionEvidence.blocked(
              action,
              m_conflictingIntakeActions.contains(action)
                  ? "RELEASE_ALL_INTAKE_PATH_INPUTS_AFTER_CONFLICT"
                  : mechanismGateBlockReason());
        }
        return false;
      }
      if (!requestedPressed) {
        recordReleased(action);
      }
      return requestedPressed;
    });
  }

  private Trigger operatorOrDriverButton(
      CommandPS5Controller dedicatedController,
      int dedicatedPort,
      int dedicatedButton,
      int driverFallbackButton,
      Action action) {
    return new Trigger(() -> {
      boolean inputsAllowed = teleopInputsAllowed();
      boolean dedicatedPresent = dedicatedControllerPresent(dedicatedPort, dedicatedButton);
      boolean dedicatedPressed = rawButtonPressed(
          dedicatedController, dedicatedPort, dedicatedButton);
      boolean fallbackPressed = rawButtonPressed(
          m_driverController, OIConstants.kDriverControllerPort, driverFallbackButton);
      if (dedicatedPresent && fallbackPressed && !dedicatedPressed) {
        m_operatorActionEvidence.blocked(
            action, "DEDICATED_CONTROLLER_PRESENT_USE_DEDICATED_CONTROL");
        return false;
      }
      boolean pressed = operatorOrDriverPressed(
          dedicatedController, dedicatedPort, dedicatedButton, driverFallbackButton);
      if (!pressed) {
        recordReleased(action);
        return false;
      }
      if (!inputsAllowed) {
        m_operatorActionEvidence.blocked(action, mechanismGateBlockReason());
        return false;
      }
      return true;
    });
  }

  private boolean jumpBumpHasPriority() {
    boolean jumpPressed = rawButtonPressed(
        m_driverController,
        OIConstants.kDriverControllerPort,
        ConfiguredOperatorControls.DRIVER_JUMP_BUMP);
    boolean wheelLockPressed = rawButtonPressed(
        m_driverController,
        OIConstants.kDriverControllerPort,
        ConfiguredOperatorControls.DRIVER_WHEEL_LOCK);
    if (m_jumpBumpRequiresRelease) {
      return false;
    }
    if (jumpPressed && wheelLockPressed) {
      m_jumpBumpRequiresRelease = true;
      m_operatorActionEvidence.blocked(Action.JUMP_BUMP, "WHEEL_LOCK_PRIORITY");
      return false;
    }
    return true;
  }

  private void runAutoAimWithEvidence() {
    switch (m_turret.autoAimWithLimelight()) {
      case ALIGNED -> m_operatorActionEvidence.active(Action.AUTO_AIM, "ALIGNED");
      case CONFIRMING_ALIGNMENT -> m_operatorActionEvidence.active(
          Action.AUTO_AIM, "CONFIRMING_ALIGNMENT");
      case WAITING_FOR_NEW_FRAME -> m_operatorActionEvidence.active(
          Action.AUTO_AIM, "WAITING_FOR_NEW_FRAME");
      case COMMANDING_CORRECTION -> m_operatorActionEvidence.active(
          Action.AUTO_AIM, "CORRECTION_COMMAND_ACCEPTED_NOT_MOTION_PROOF");
      case CORRECTION_AT_TARGET -> m_operatorActionEvidence.active(
          Action.AUTO_AIM, "CORRECTION_AT_TARGET");
      case UNREFERENCED -> m_operatorActionEvidence.blocked(
          Action.AUTO_AIM, "TURRET_UNREFERENCED");
      case ALLIANCE_UNKNOWN -> m_operatorActionEvidence.blocked(
          Action.AUTO_AIM, "ALLIANCE_UNKNOWN");
      case VISION_NOT_READY -> m_operatorActionEvidence.blocked(
          Action.AUTO_AIM, "VISION_NOT_READY");
      case NO_VALID_TARGET -> m_operatorActionEvidence.blocked(
          Action.AUTO_AIM, "NO_VALID_TARGET");
      case WRONG_ALLIANCE_OR_NON_HUB_TAG -> m_operatorActionEvidence.blocked(
          Action.AUTO_AIM, "WRONG_ALLIANCE_OR_NON_HUB_TAG");
      case POSITION_UNAVAILABLE -> m_operatorActionEvidence.blocked(
          Action.AUTO_AIM, "POSITION_UNAVAILABLE");
      case COMMAND_REJECTED -> m_operatorActionEvidence.blocked(
          Action.AUTO_AIM, "POSITION_COMMAND_REJECTED");
    }
  }

  private static Action operatorAction(IntakePathAction action) {
    return switch (action) {
      case INTAKE -> Action.INTAKE;
      case OUTPUT -> Action.OUTPUT;
      case FIRE -> Action.FIRE;
      case RETRACT -> Action.RETRACT;
    };
  }

  private void recordIntakePathConflict(
      boolean intake, boolean output, boolean firePressed, boolean retract) {
    if (intake) {
      recordIntakePathConflict(Action.INTAKE);
    }
    if (output) {
      recordIntakePathConflict(Action.OUTPUT);
    }
    if (firePressed) {
      recordIntakePathConflict(Action.FIRE);
    }
    if (retract) {
      recordIntakePathConflict(Action.RETRACT);
    }
  }

  private void recordIntakePathConflict(Action action) {
    m_conflictingIntakeActions.add(action);
    m_operatorActionEvidence.blocked(action, "CONFLICTING_INTAKE_PATH_INPUTS");
  }

  private void clearReleasedIntakeConflicts() {
    for (Action action : m_conflictingIntakeActions) {
      m_operatorActionEvidence.stopped(action, "INPUTS_RELEASED_AFTER_CONFLICT");
    }
    m_conflictingIntakeActions.clear();
  }

  private void recordReleased(Action action) {
    OperatorActionEvidence.Snapshot snapshot = m_operatorActionEvidence.snapshot(action);
    if (snapshot != null && snapshot.state() != OperatorActionEvidence.State.STOPPED) {
      m_operatorActionEvidence.stopped(action, "INPUT_RELEASED");
    }
  }

  private static String mechanismGateBlockReason() {
    return DriverStation.isTeleopEnabled()
        ? "RELEASE_TO_ARM_AFTER_ENABLE_OR_DEPENDENCY_CHANGE"
        : "NOT_TELEOP";
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

  private static boolean dedicatedControllerPresent(int port, int requiredButton) {
    return DriverStation.getStickButtonCount(port) >= requiredButton;
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
            m_feeder,
            () -> m_feederManualRetestToken,
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
        && unhomedDiagnosticSelectionVerified(target, direction)
        && (target != Target.FEEDER
            || (m_feederManualRetestToken != null
                && m_feeder.isManualControlledRetestSessionValid(
                    m_feederManualRetestToken, direction.duty())));
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
  public boolean prepareUnhomedDiagnosticSession(
      Target target, Direction direction, double absoluteExpiresAtSeconds) {
    discardPreparedUnhomedDiagnosticSession();
    double now = Timer.getFPGATimestamp();
    boolean valid = target != null
        && direction != null
        && DriverStation.isDisabled()
        && DriverStation.isTest()
        && !DriverStation.isFMSAttached()
        && unhomedDiagnosticSelectionVerified(target, direction)
        && Double.isFinite(absoluteExpiresAtSeconds)
        && absoluteExpiresAtSeconds > now
        && absoluteExpiresAtSeconds <= now + HardwareTestConstants.ARM_LIFETIME_SECONDS;
    if (!valid) {
      return false;
    }
    Token feederToken = null;
    if (target == Target.FEEDER) {
      feederToken = m_feeder.armManualControlledRetest(
          direction.duty(), absoluteExpiresAtSeconds).orElse(null);
      if (feederToken == null) {
        return false;
      }
    }
    m_preparedUnhomedDiagnosticTarget = target;
    m_preparedUnhomedDiagnosticDirection = direction;
    m_preparedUnhomedDiagnosticExpiresAt = absoluteExpiresAtSeconds;
    m_feederManualRetestToken = feederToken;
    return true;
  }

  public void armUnhomedDiagnosticSession(
      Target target, Direction direction, double absoluteExpiresAtSeconds) {
    double now = Timer.getFPGATimestamp();
    if (target == null
        || direction == null
        || target != m_preparedUnhomedDiagnosticTarget
        || direction != m_preparedUnhomedDiagnosticDirection
        || !unhomedDiagnosticSelectionVerified(target, direction)
        || Double.compare(
            absoluteExpiresAtSeconds, m_preparedUnhomedDiagnosticExpiresAt) != 0
        || !Double.isFinite(absoluteExpiresAtSeconds)
        || absoluteExpiresAtSeconds <= now
        || absoluteExpiresAtSeconds > now + HardwareTestConstants.ARM_LIFETIME_SECONDS
        || (target == Target.FEEDER && m_feederManualRetestToken == null)) {
      discardPreparedUnhomedDiagnosticSession();
      SmartDashboard.putString(
          ManualUnhomedActuatorDiagnosticCommand.STATUS_KEY,
          "ARM_EXPIRED_OR_INVALID_REARM_DISABLED");
      return;
    }
    m_unhomedDiagnosticTarget = target;
    m_unhomedDiagnosticDirection = direction;
    m_unhomedDiagnosticExpiresAt = absoluteExpiresAtSeconds;
    m_preparedUnhomedDiagnosticTarget = null;
    m_preparedUnhomedDiagnosticDirection = null;
    m_preparedUnhomedDiagnosticExpiresAt = Double.NEGATIVE_INFINITY;
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
    m_feederManualRetestToken = null;
    m_feeder.disarmManualControlledRetest();
    m_preparedUnhomedDiagnosticTarget = null;
    m_preparedUnhomedDiagnosticDirection = null;
    m_preparedUnhomedDiagnosticExpiresAt = Double.NEGATIVE_INFINITY;
    m_unhomedDiagnosticInputGate.blockUntilNeutral();
  }

  private static boolean unhomedDiagnosticSelectionVerified(
      Target target, Direction direction) {
    return target != null
        && direction != null
        && SmartDashboard.getBoolean(
            ManualUnhomedActuatorDiagnosticCommand.PHYSICAL_CLEARANCE_KEY, false)
        && SmartDashboard.getBoolean(
            ManualUnhomedActuatorDiagnosticCommand.MOTOR_TYPE_VERIFIED_KEY, false)
        && ManualUnhomedActuatorDiagnosticCommand.readExactlyOneTarget()
            .filter(selected -> selected == target)
            .isPresent()
        && ManualUnhomedActuatorDiagnosticCommand.readExactlyOneDirection()
            .filter(selected -> selected == direction)
            .isPresent()
        && ManualUnhomedActuatorDiagnosticCommand.targetSpecificVerificationSatisfied(target);
  }

  public void discardPreparedUnhomedDiagnosticSession() {
    if (m_unhomedDiagnosticTarget == null && m_feederManualRetestToken != null) {
      m_feederManualRetestToken = null;
      m_feeder.disarmManualControlledRetest();
    }
    m_preparedUnhomedDiagnosticTarget = null;
    m_preparedUnhomedDiagnosticDirection = null;
    m_preparedUnhomedDiagnosticExpiresAt = Double.NEGATIVE_INFINITY;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return this.m_DriveBaseContainer.GetAutonCommand();
  }

  public boolean shouldAbortActiveAutonomous(Snapshot outputSafetySnapshot) {
    return m_DriveBaseContainer.shouldAbortActiveAutonomous(outputSafetySnapshot);
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

  public java.util.List<frc.robot.utils.CtreDeviceEvidence.Snapshot>
      getSwerveDeviceEvidenceSnapshots() {
    return drivetrain.getDeviceEvidenceSnapshots();
  }

  public String getSparkDeviceHealthSummary() {
    return SparkMAXContainer.getDeviceAvailabilitySummary();
  }

  /** Renews the independent process-wide authorization immediately before scheduler execution. */
  public void serviceOutputSafetyHeartbeat() {
    m_outputSafetySupervisor.heartbeat();
  }

  /** Revokes nonzero output before requesting the independent global stop sequence. */
  public void tripOutputSafety(String reason) {
    m_outputSafetySupervisor.forceTrip(reason);
  }

  public RobotOutputSafetySupervisor.Snapshot getOutputSafetySnapshot() {
    return m_outputSafetySupervisor.snapshot();
  }

  /** Disabled arm gates may only become valid after fresh global zero evidence. */
  public boolean isOutputSafetyReadyForEnable() {
    return m_outputSafetySupervisor.snapshot().phase()
        == RobotOutputSafetySupervisor.Phase.READY_DISABLED;
  }

  public void stopAll() {
    stopAllInternal(false);
  }

  public void stopAllPreservingPreparedUnhomedDiagnosticSession() {
    stopAllInternal(true);
  }

  private void stopAllInternal(boolean preservePreparedUnhomedSession) {
    if (!preservePreparedUnhomedSession) {
      disarmUnhomedDiagnosticSession();
    } else {
      m_unhomedDiagnosticTarget = null;
      m_unhomedDiagnosticDirection = null;
      m_unhomedDiagnosticExpiresAt = Double.NEGATIVE_INFINITY;
      m_unhomedDiagnosticInputGate.blockUntilNeutral();
    }
    m_DriveBaseContainer.blockDriverInputsUntilNeutral();
    stopSafely("swerve", drivetrain::requestIdle);
    stopSafely("intake", m_intake::stopAll);
    stopSafely("conveyor", m_conveyor::stop);
    stopSafely("feeder", m_feeder::stopDiagnosticOutput);
    stopSafely("shooter", m_shooter::stop);
    stopSafely("turret", m_turret::stop);
    stopSafely("climber", m_climber::stop);
    m_operatorActionEvidence.allStopped("ROBOT_OUTPUT_STOP_REQUESTED");
  }

  /**
   * Creates fresh SPARK and CTRE stop tokens for the scheduler-independent heartbeat watchdog.
   * Every service call remains valid while the normal robot loop is stalled.
   */
  private StopSession beginIndependentOutputStopSession() {
    stopAll();
    int[] sparkIds = frc.robot.constants.ConfiguredCanHardware.sparkDeviceIds().stream()
        .mapToInt(Integer::intValue)
        .toArray();
    SparkMAXContainer.OutputStopBatch sparkStop =
        SparkMAXContainer.requestOutputStops(sparkIds);
    CommandSwerveDrivetrain.SwerveStopToken swerveStop =
        drivetrain.requestIdleWithToken();

    return new StopSession() {
      private boolean confirmed;
      private String summary = "GLOBAL_STOP_EVIDENCE_PENDING";
      private double nextEvidencePollAt = Double.NEGATIVE_INFINITY;

      @Override
      public void service() {
        SparkMAXContainer.serviceAll();
        if (RobotBase.isSimulation()) {
          // In desktop simulation there is no physical controller loop to turn a zero setpoint
          // into a fresh raw frame while robotPeriodic is intentionally stalled.
          SparkRawCommandEchoSimulation.stepConfiguredControllers(false);
        }
        double now = Timer.getFPGATimestamp();
        if (Double.isFinite(now) && now < nextEvidencePollAt) {
          return;
        }
        nextEvidencePollAt = Double.isFinite(now) ? now + 0.02 : Double.NEGATIVE_INFINITY;
        drivetrain.requestIdle();
        drivetrain.retryIdleIfNeeded(swerveStop);
        SparkMAXContainer.OutputStopSnapshot sparkEvidence = sparkStop.snapshot();
        CommandSwerveDrivetrain.SwerveStopEvidence swerveEvidence =
            drivetrain.getStopEvidence(
                swerveStop,
                HardwareTestConstants.MAX_STOPPED_SWERVE_SPEED_METERS_PER_SECOND);
        confirmed = sparkEvidence.confirmed() && swerveEvidence.confirmed();
        summary = "spark=" + sparkEvidence.summary() + " swerve=" + swerveEvidence.reason();
      }

      @Override
      public boolean confirmed() {
        return confirmed;
      }

      @Override
      public String summary() {
        return summary;
      }
    };
  }

  /** Releases process-owned simulation/native resources after all outputs are requested neutral. */
  @Override
  public void close() {
    if (!closed.compareAndSet(false, true)) {
      return;
    }
    // Revoke before the first stop request so a concurrently executing command cannot issue a
    // late nonzero request between stopAll() and supervisor teardown.
    m_outputSafetySupervisor.forceTrip("CONTAINER_CLOSED");
    try {
      stopAll();
    } finally {
      try {
        m_outputSafetySupervisor.close();
      } finally {
        try {
          m_feeder.close();
        } finally {
          m_DriveBaseContainer.close();
        }
      }
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
