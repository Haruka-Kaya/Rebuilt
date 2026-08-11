package frc.robot.containers;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AdvancedFireCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.constants.Constants.AutoConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoContainer {
    private SendableChooser<Command> autoChooser;
    private final CommandSwerveDrivetrain drivetrain;

    private final TurretSubsystem m_turret;
    private final ShooterSubsystem m_shooter;
    private final FeederSubsystem m_feeder;
    private final ConveyorSubsystem m_conveyor;
    private final IntakeSubsystem m_intake;
    private final ClimberSubsystem m_climber;
    private final Command safeStopCommand;
    private String readinessStatus;
    private double nextStatusPublishTimestamp;
    private boolean chooserOperational;
    private String permanentBlockReason;

    public AutoContainer(CommandSwerveDrivetrain drivetrain, TurretSubsystem turret,
                            ShooterSubsystem shooter, FeederSubsystem feeder,
                            ConveyorSubsystem conveyor, IntakeSubsystem intake,
                            ClimberSubsystem climber) {
        this.drivetrain = drivetrain;

        this.m_turret = turret;
        this.m_shooter = shooter;
        this.m_feeder = feeder;
        this.m_conveyor = conveyor;
        this.m_intake = intake;
        this.m_climber = climber;
        this.safeStopCommand = Commands.run(
            () -> {
                drivetrain.requestIdle();
                m_turret.stop();
                m_shooter.stop();
                m_feeder.stop();
                m_conveyor.stop();
                m_intake.stopAll();
                m_climber.stop();
            },
            drivetrain, m_turret, m_shooter, m_feeder, m_conveyor, m_intake, m_climber)
            .finallyDo(interrupted -> {
                drivetrain.requestIdle();
                m_turret.stop();
                m_shooter.stop();
                m_feeder.stop();
                m_conveyor.stop();
                m_intake.stopAll();
                m_climber.stop();
            })
            .withName("Autonomous Safe Stop");

        this.configureAutoBindings();
    }

    private void configureAutoBindings() {
        if (!AutoConstants.CALIBRATED_AUTONOMOUS_ENABLED) {
            configureSafeChooser(AutoConstants.CALIBRATION_BLOCK_REASON);
            return;
        }
        if (!drivetrainConfigured()) {
            configureSafeChooser("BLOCKED: PathPlanner AutoBuilder is not configured");
            return;
        }

        NamedCommands.registerCommand("Advanced Fire", new AdvancedFireCommand(m_turret, m_shooter, m_feeder, m_conveyor));
        NamedCommands.registerCommand("Slurp", new IntakeCommand(m_intake, m_conveyor));

        try {
            autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
            chooserOperational = true;
            permanentBlockReason = null;
        } catch (RuntimeException e) {
            configureSafeChooser("BLOCKED: auto chooser load failed: " + e.getMessage());
            return;
        }
        publishCurrentReadiness();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private boolean drivetrainConfigured() {
        return drivetrain.isPathPlannerConfigured() && AutoBuilder.isConfigured();
    }

    private void configureSafeChooser(String reason) {
        chooserOperational = false;
        permanentBlockReason = reason;
        readinessStatus = reason;
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("SAFE STOP - calibration required", safeStopCommand);
        publishStatus(false);
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void publishStatus(boolean ready) {
        SmartDashboard.putBoolean("Autonomous/Ready", ready);
        SmartDashboard.putString("Autonomous/Status", readinessStatus);
    }

    public Command getAutonomousCommand() {
        if (!AutoConstants.CALIBRATED_AUTONOMOUS_ENABLED) {
            readinessStatus = AutoConstants.CALIBRATION_BLOCK_REASON;
            publishStatus(false);
            return safeStopCommand;
        }
        if (permanentBlockReason != null) {
            readinessStatus = permanentBlockReason;
            publishStatus(false);
            return safeStopCommand;
        }
        AutonomousReadiness.Result readiness = currentReadiness();
        if (!readiness.ready()) {
            readinessStatus = "BLOCKED: " + readiness.reason();
            publishStatus(false);
            return safeStopCommand;
        }
        readinessStatus = "READY: " + readiness.reason();
        publishStatus(true);
        Command selected = autoChooser == null ? null : autoChooser.getSelected();
        return selected == null ? safeStopCommand : selected;
    }

    /** Runtime interlock for a calibrated path that was already scheduled. */
    public boolean shouldAbortActiveAutonomous() {
        if (!AutoConstants.CALIBRATED_AUTONOMOUS_ENABLED) {
            return false;
        }
        if (permanentBlockReason != null) {
            readinessStatus = "ABORTED: " + withoutBlockedPrefix(permanentBlockReason);
            publishStatus(false);
            return true;
        }
        AutonomousReadiness.Result readiness = currentReadiness();
        boolean abort = !readiness.ready();
        if (abort) {
            readinessStatus = "ABORTED: " + readiness.reason();
            publishStatus(false);
        }
        return abort;
    }

    /** Keeps preflight state visible while disabled without waiting for autonomousInit(). */
    public void refreshReadinessStatus() {
        if (!DriverStation.isDisabled()) {
            return;
        }
        double now = Timer.getFPGATimestamp();
        if (now < nextStatusPublishTimestamp) {
            return;
        }
        nextStatusPublishTimestamp = now + 0.20;
        if (!AutoConstants.CALIBRATED_AUTONOMOUS_ENABLED) {
            readinessStatus = AutoConstants.CALIBRATION_BLOCK_REASON;
            publishStatus(false);
            return;
        }
        if (permanentBlockReason != null) {
            readinessStatus = permanentBlockReason;
            publishStatus(false);
            return;
        }
        publishCurrentReadiness();
    }

    private void publishCurrentReadiness() {
        AutonomousReadiness.Result readiness = currentReadiness();
        readinessStatus = (readiness.ready() ? "READY: " : "BLOCKED: ") + readiness.reason();
        publishStatus(readiness.ready());
    }

    private AutonomousReadiness.Result currentReadiness() {
        return AutonomousReadiness.evaluate(new AutonomousReadiness.Inputs(
            drivetrainConfigured(),
            chooserOperational,
            drivetrain.areAllDevicesConnected(),
            m_turret.isPositionControlReadyForAutonomousAim(),
            m_turret.isVisionReadyForAutonomousAim(),
            DriverStation.getAlliance().isPresent(),
            m_shooter.isOperationalForAutonomousShot(),
            m_conveyor.isReady(),
            m_feeder.isReady()));
    }

    private static String withoutBlockedPrefix(String reason) {
        return reason != null && reason.startsWith("BLOCKED: ")
            ? reason.substring("BLOCKED: ".length())
            : reason;
    }
}
