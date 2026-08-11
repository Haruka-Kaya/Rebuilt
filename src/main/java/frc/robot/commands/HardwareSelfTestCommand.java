package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** One-shot, low-output hardware test that only runs after the DS enters enabled Test mode. */
public final class HardwareSelfTestCommand {
    public static final String RUNNING_KEY = "Hardware Self-Test/Running";

    private HardwareSelfTestCommand() {}

    public static Command create(
            CommandSwerveDrivetrain drivetrain,
            FeederSubsystem feeder,
            ShooterSubsystem shooter,
            ClimberSubsystem climber) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                SmartDashboard.putBoolean(RUNNING_KEY, true);
                log("BEGIN", "low-output online-device test");
            }, climber),
            stage("FEEDER_ID32", feeder::runDiagnostic, feeder::stop, feeder::getDiagnosticStatus, 0.4, feeder),
            stage(
                "SHOOTER_FOLLOWER_ID37",
                shooter::runFollowerDiagnostic,
                shooter::stopFollowerDiagnostic,
                shooter::getFollowerDiagnosticStatus,
                0.8,
                shooter),
            stage(
                "SWERVE_FORWARD",
                () -> drivetrain.drive(0.015, 0, 0, false),
                () -> drivetrain.drive(0, 0, 0, false),
                drivetrain::getMotionDiagnosticSummary,
                1.0,
                drivetrain),
            stage(
                "SWERVE_STRAFE",
                () -> drivetrain.drive(0, 0.015, 0, false),
                () -> drivetrain.drive(0, 0, 0, false),
                drivetrain::getMotionDiagnosticSummary,
                1.0,
                drivetrain),
            stage(
                "SWERVE_ROTATE",
                () -> drivetrain.drive(0, 0, 0.12, false),
                () -> drivetrain.drive(0, 0, 0, false),
                drivetrain::getMotionDiagnosticSummary,
                1.0,
                drivetrain),
            Commands.runOnce(() -> log("SEQUENCE_COMPLETE", "review samples; outputs stopped"))
        ).finallyDo(interrupted -> {
            drivetrain.drive(0, 0, 0, false);
            feeder.stop();
            shooter.stopFollowerDiagnostic();
            climber.stop();
            SmartDashboard.putBoolean(RUNNING_KEY, false);
            log(interrupted ? "INTERRUPTED" : "END", "outputs forced to zero");
        }).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    private static Command stage(
            String name,
            Runnable action,
            Runnable stop,
            Supplier<String> status,
            double durationSeconds,
            Subsystem... requirements) {
        return Commands.sequence(
            Commands.runOnce(() -> log(name + "_START", status.get())),
            Commands.deadline(
                Commands.waitSeconds(durationSeconds),
                Commands.run(action, requirements),
                Commands.sequence(
                    Commands.waitSeconds(durationSeconds / 2.0),
                    Commands.runOnce(() -> log(name + "_SAMPLE", status.get())))),
            Commands.runOnce(stop, requirements),
            Commands.runOnce(() -> log(name + "_STOP", status.get())),
            Commands.waitSeconds(0.35));
    }

    private static void log(String stage, String details) {
        var canStatus = RobotController.getCANStatus();
        System.out.printf(
            "SELFTEST stage=%s voltage=%.2fV canUtil=%.1f%% details=[%s]%n",
            stage,
            RobotController.getBatteryVoltage(),
            canStatus.percentBusUtilization * 100.0,
            details);
    }
}
