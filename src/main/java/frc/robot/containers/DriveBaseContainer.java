package frc.robot.containers;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;
import java.util.concurrent.atomic.AtomicBoolean;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Telemetry;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.Constants.DebugConstants;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.constants.ConfiguredOperatorControls;
import frc.robot.constants.ConfiguredOperatorActions.Action;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain.ControlResult;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.NeutralAfterEnableGate;
import frc.robot.utils.OperatorActionEvidence;

public class DriveBaseContainer implements AutoCloseable {
    private final AtomicBoolean closed = new AtomicBoolean();
    public AutoContainer autoContainer;
    public static double speedFactor = .05;
    public static double rotationFactor = .05;
    
    static {
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Speed Factor", speedFactor);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Rotation Factor", rotationFactor);
    }

    public static DoubleSupplier MaxSpeed = () -> speedFactor * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static DoubleSupplier MaxAngularRate = () -> RotationsPerSecond.of(rotationFactor).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed.getAsDouble() * OIConstants.kDriveDeadband).withRotationalDeadband(MaxAngularRate.getAsDouble() * OIConstants.kDriveDeadband) // Add deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed.getAsDouble() * OIConstants.kDriveDeadband)
            .withRotationalDeadband(MaxAngularRate.getAsDouble() * OIConstants.kDriveDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final NeutralAfterEnableGate driveInputGate = new NeutralAfterEnableGate();
    private final OperatorActionEvidence actionEvidence;
    private String lastDriveInputBlockReason = "RELEASE_TO_ARM";
    private boolean lastDriveRequestWasNeutral;
    private boolean lastDriveUsedGyroFallback;
    private boolean seedFieldRequiresRelease;
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed.getAsDouble());
    CommandPS5Controller joystick;
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // private final TurretSubsystem m_turret;
    // private final ShooterSubsystem m_shooter;
    // private final FeederSubsystem m_feeder;
    // private final ConveyorSubsystem m_conveyor;
    // private final IntakeSubsystem m_intake;

    public DriveBaseContainer(CommandPS5Controller driverController, TurretSubsystem turret,
                            ShooterSubsystem shooter, FeederSubsystem feeder,
                            ConveyorSubsystem conveyor, IntakeSubsystem intake,
                            ClimberSubsystem climber) {
        this(driverController, turret, shooter, feeder, conveyor, intake, climber, null);
    }

    public DriveBaseContainer(CommandPS5Controller driverController, TurretSubsystem turret,
                            ShooterSubsystem shooter, FeederSubsystem feeder,
                            ConveyorSubsystem conveyor, IntakeSubsystem intake,
                            ClimberSubsystem climber, OperatorActionEvidence actionEvidence) {
        this.actionEvidence = actionEvidence;
        joystick = driverController;
        configureBindings();
        SmartDashboard.putBoolean("DriveBase Running",true);
        SmartDashboard.putBoolean("Swerve Output Enabled", DebugConstants.ALLOW_SWERVE_OUTPUT);

        SmartDashboard.putString("MESSAGE", "we are at autoSetup");

        // this.m_turret = turret;
        // this.m_shooter = shooter;
        // this.m_feeder = feeder;
        // this.m_conveyor = conveyor;
        // this.m_intake = intake;

        autoContainer = new AutoContainer(
            drivetrain, turret, shooter, feeder, conveyor, intake, climber);
    }

    public Command driveHider(){
            return drivetrain.applyRequest(() -> {
                if (!driverInputsAllowed()) {
                    lastDriveRequestWasNeutral = true;
                    return drivetrain.safeNeutralRequest();
                }
                double axisX = availableAxis(1);
                double axisY = availableAxis(0);
                double axisRotation = availableAxis(2);
                double maxSpeed = MaxSpeed.getAsDouble();
                double maxAngularRate = MaxAngularRate.getAsDouble();
                if (!Double.isFinite(axisX)
                        || !Double.isFinite(axisY)
                        || !Double.isFinite(axisRotation)
                        || !Double.isFinite(maxSpeed)
                        || !Double.isFinite(maxAngularRate)) {
                    driveInputGate.blockUntilNeutral();
                    lastDriveInputBlockReason = "INVALID_INPUT";
                    lastDriveRequestWasNeutral = true;
                    return drivetrain.safeNeutralRequest();
                }
                double velocityX = -axisX * maxSpeed;
                double velocityY = -axisY * maxSpeed;
                double rotation = -axisRotation * maxAngularRate;
                if (!Double.isFinite(velocityX)
                        || !Double.isFinite(velocityY)
                        || !Double.isFinite(rotation)) {
                    driveInputGate.blockUntilNeutral();
                    lastDriveInputBlockReason = "INVALID_SCALED_INPUT";
                    lastDriveRequestWasNeutral = true;
                    return drivetrain.safeNeutralRequest();
                }
                lastDriveRequestWasNeutral = Math.abs(axisX) <= OIConstants.kDriveDeadband
                    && Math.abs(axisY) <= OIConstants.kDriveDeadband
                    && Math.abs(axisRotation) <= OIConstants.kDriveDeadband;
                if (drivetrain.isGyroConnected()) {
                    lastDriveUsedGyroFallback = false;
                    return drive.withVelocityX(velocityX)
                        .withVelocityY(velocityY)
                        .withRotationalRate(rotation);
                }
                lastDriveUsedGyroFallback = true;
                return robotCentricDrive.withVelocityX(velocityX)
                    .withVelocityY(velocityY)
                    .withRotationalRate(rotation);
            }, this::publishDriveResult)
                .finallyDo(interrupted -> {
                    if (actionEvidence != null) {
                        actionEvidence.commandEnded(Action.DRIVE, interrupted);
                    }
                });
    }

    private double availableAxis(int axis) {
        return DriverStation.getStickAxisCount(OIConstants.kDriverControllerPort) > axis
            ? joystick.getHID().getRawAxis(axis)
            : 0.0;
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            driveHider()
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.safeIdleCommand().ignoringDisable(true)
        );

        final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

        // Touchpad is reserved for wheel-lock so it does not conflict with R1/intake output.
        Trigger brakeButton = availableButton(
            ConfiguredOperatorControls.DRIVER_WHEEL_LOCK, Action.WHEEL_LOCK);
        brakeButton.whileTrue(drivetrain.applyRequest(
            () -> brake,
            result -> publishAssistResult(Action.WHEEL_LOCK, result, "BRAKE_REQUEST_SUBMITTED"))
            .beforeStarting(() -> requestEvidence(Action.WHEEL_LOCK))
            .finallyDo(interrupted -> endEvidence(Action.WHEEL_LOCK, interrupted)));

        // Create resets the field-centric heading without colliding with mechanism controls.
        availableButton(ConfiguredOperatorControls.DRIVER_SEED_FIELD, Action.SEED_FIELD)
            .and(new Trigger(this::seedFieldHasPriority))
            .onTrue(drivetrain.runOnce(() -> {
                requestEvidence(Action.SEED_FIELD);
                if (!drivetrain.isGyroConnected()) {
                    blockedEvidence(Action.SEED_FIELD, "GYRO_UNAVAILABLE");
                    return;
                }
                try {
                    drivetrain.seedFieldCentric();
                    // Field-centric translation now uses a new heading reference.  Require a
                    // neutral sample before held sticks can command motion in that new frame.
                    blockDriverInputsUntilNeutral();
                    completedEvidence(
                        Action.SEED_FIELD, "HEADING_SEED_API_RETURNED_NOT_POSE_PROOF");
                } catch (RuntimeException exception) {
                    blockedEvidence(Action.SEED_FIELD, "HEADING_SEED_REQUEST_EXCEPTION");
                }
            }));

        drivetrain.registerTelemetry(logger::captureState);
    }

    private Trigger availableButton(int button, Action action) {
        return new Trigger(() -> {
            boolean pressed = rawButtonPressed(button);
            if (!pressed) {
                if (action == Action.SEED_FIELD) {
                    seedFieldRequiresRelease = false;
                }
                recordReleased(action);
                return false;
            }
            if (!driverInputsAllowed()) {
                blockedEvidence(action, lastDriveInputBlockReason);
                return false;
            }
            return true;
        });
    }

    private boolean seedFieldHasPriority() {
        boolean seedPressed = rawButtonPressed(ConfiguredOperatorControls.DRIVER_SEED_FIELD);
        boolean brakePressed = rawButtonPressed(ConfiguredOperatorControls.DRIVER_WHEEL_LOCK);
        boolean jumpPressed = rawButtonPressed(ConfiguredOperatorControls.DRIVER_JUMP_BUMP);
        if (seedFieldRequiresRelease) {
            return false;
        }
        if (seedPressed && (brakePressed || jumpPressed)) {
            seedFieldRequiresRelease = true;
            blockedEvidence(Action.SEED_FIELD, "DRIVE_ASSIST_PRIORITY");
            return false;
        }
        return true;
    }

    /** Shared neutral-after-enable gate for default and assisted driving commands. */
    public boolean driverInputsAllowed() {
        double axis0 = availableAxis(0);
        double axis1 = availableAxis(1);
        double axis2 = availableAxis(2);
        if (!Double.isFinite(axis0)
                || !Double.isFinite(axis1)
                || !Double.isFinite(axis2)) {
            driveInputGate.blockUntilNeutral();
            lastDriveInputBlockReason = "INVALID_INPUT";
            return false;
        }
        boolean anyAxisActive = Math.abs(axis0) > OIConstants.kDriveDeadband
            || Math.abs(axis1) > OIConstants.kDriveDeadband
            || Math.abs(axis2) > OIConstants.kDriveDeadband;
        boolean anyControlButton = rawButtonPressed(ConfiguredOperatorControls.DRIVER_SEED_FIELD)
            || rawButtonPressed(ConfiguredOperatorControls.DRIVER_JUMP_BUMP)
            || rawButtonPressed(ConfiguredOperatorControls.DRIVER_WHEEL_LOCK);
        int sourceSignature = DriverStation.getStickButtonCount(
            OIConstants.kDriverControllerPort)
            | (DriverStation.getStickAxisCount(OIConstants.kDriverControllerPort) << 8)
            // Field-centric and robot-centric interpret the same held stick differently. Treat a
            // gyro loss or recovery as a source change so both transitions require neutral input.
            | (drivetrain.isGyroConnected() ? 1 << 16 : 0);
        boolean teleopEnabled = DriverStation.isTeleopEnabled();
        boolean modulesHealthy = drivetrain.areAllModulesConnected();
        boolean allowed = driveInputGate.allow(
            teleopEnabled && modulesHealthy,
            sourceSignature,
            anyAxisActive || anyControlButton);
        if (!allowed) {
            lastDriveInputBlockReason = !teleopEnabled
                ? "NOT_TELEOP"
                : !modulesHealthy ? "DRIVETRAIN_MODULES_UNHEALTHY" : "RELEASE_TO_ARM";
        } else {
            lastDriveInputBlockReason = "NONE";
        }
        return allowed;
    }

    private boolean rawButtonPressed(int button) {
        return DriverStation.getStickButtonCount(OIConstants.kDriverControllerPort) >= button
            && joystick.getHID().getRawButton(button);
    }

    private void publishDriveResult(ControlResult result) {
        if (requiresNeutralRearm(result)) {
            blockDriverInputsUntilNeutral();
        }
        if (actionEvidence == null) {
            return;
        }
        switch (result) {
            case REQUEST_SUBMITTED -> {
                if (lastDriveRequestWasNeutral) {
                    actionEvidence.stopped(Action.DRIVE, "INPUT_NEUTRAL");
                } else if (lastDriveUsedGyroFallback) {
                    actionEvidence.active(
                        Action.DRIVE,
                        "ROBOT_CENTRIC_GYRO_FALLBACK_API_RETURNED_NOT_REQUEST_OR_MOTION_PROOF");
                } else {
                    actionEvidence.active(
                        Action.DRIVE,
                        "FIELD_CENTRIC_API_RETURNED_NOT_REQUEST_OR_MOTION_PROOF");
                }
            }
            case NEUTRAL_REQUESTED -> {
                if ("NONE".equals(lastDriveInputBlockReason)) {
                    actionEvidence.stopped(Action.DRIVE, "INPUT_NEUTRAL");
                } else {
                    actionEvidence.blocked(Action.DRIVE, lastDriveInputBlockReason);
                }
            }
            case OUTPUT_DISABLED -> {
                actionEvidence.blocked(Action.DRIVE, "SWERVE_OUTPUT_DISABLED");
            }
            case MODULES_UNHEALTHY -> {
                actionEvidence.blocked(Action.DRIVE, "DRIVETRAIN_MODULES_UNHEALTHY");
            }
            case INVALID_INPUT -> {
                actionEvidence.blocked(Action.DRIVE, "INVALID_INPUT");
            }
            case REQUEST_EXCEPTION -> {
                actionEvidence.blocked(Action.DRIVE, "SWERVE_REQUEST_EXCEPTION");
            }
        }
    }

    private void publishAssistResult(Action action, ControlResult result, String successReason) {
        if (requiresNeutralRearm(result)) {
            blockDriverInputsUntilNeutral();
        }
        if (actionEvidence == null) {
            return;
        }
        switch (result) {
            case REQUEST_SUBMITTED -> actionEvidence.active(
                action, successReason + "_NOT_MOTION_PROOF");
            case NEUTRAL_REQUESTED -> actionEvidence.blocked(action, "NEUTRAL_REQUESTED");
            case OUTPUT_DISABLED -> {
                actionEvidence.blocked(action, "SWERVE_OUTPUT_DISABLED");
            }
            case MODULES_UNHEALTHY -> {
                actionEvidence.blocked(action, "DRIVETRAIN_MODULES_UNHEALTHY");
            }
            case INVALID_INPUT -> {
                actionEvidence.blocked(action, "INVALID_INPUT");
            }
            case REQUEST_EXCEPTION -> {
                actionEvidence.blocked(action, "SWERVE_REQUEST_EXCEPTION");
            }
        }
    }

    private void requestEvidence(Action action) {
        if (actionEvidence != null) {
            actionEvidence.requested(action);
        }
    }

    private void activeEvidence(Action action, String reason) {
        if (actionEvidence != null) {
            actionEvidence.active(action, reason);
        }
    }

    private void completedEvidence(Action action, String reason) {
        if (actionEvidence != null) {
            actionEvidence.completed(action, reason);
        }
    }

    private void blockedEvidence(Action action, String reason) {
        if (actionEvidence != null) {
            actionEvidence.blocked(action, reason);
        }
    }

    private void endEvidence(Action action, boolean interrupted) {
        if (actionEvidence != null) {
            actionEvidence.commandEnded(action, interrupted);
        }
    }

    private void recordReleased(Action action) {
        if (actionEvidence == null) {
            return;
        }
        OperatorActionEvidence.Snapshot snapshot = actionEvidence.snapshot(action);
        if (snapshot != null && snapshot.state() != OperatorActionEvidence.State.STOPPED) {
            actionEvidence.stopped(action, "INPUT_RELEASED");
        }
    }

    /** Invalidates every prior neutral observation across mode and health transitions. */
    public void blockDriverInputsUntilNeutral() {
        driveInputGate.blockUntilNeutral();
        lastDriveInputBlockReason = "RELEASE_TO_ARM";
    }

    private static boolean requiresNeutralRearm(ControlResult result) {
        return switch (result) {
            case REQUEST_SUBMITTED, NEUTRAL_REQUESTED -> false;
            case OUTPUT_DISABLED, MODULES_UNHEALTHY, INVALID_INPUT, REQUEST_EXCEPTION -> true;
        };
    }

    public Command GetAutonCommand(){
        return this.autoContainer.getAutonomousCommand();
    }

    public boolean shouldAbortActiveAutonomous() {
        return autoContainer.shouldAbortActiveAutonomous();
    }

    public void refreshAutonomousStatus() {
        autoContainer.refreshReadinessStatus();
    }

    /** Releases simulation/native resources for repeatable integration tests. */
    @Override
    public void close() {
        if (!closed.compareAndSet(false, true)) {
            return;
        }
        try {
            logger.close();
        } finally {
            drivetrain.close();
        }
    }
}
