package frc.robot.commands;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants.HardwareTestConstants;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.constants.Constants.ManipulatorConstants;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.diagnostics.HardwareDiagnosticEvaluator;
import frc.robot.diagnostics.HardwareDiagnosticEvaluator.MotionResult;
import frc.robot.diagnostics.HardwareDiagnosticEvaluator.Snapshot;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.SparkMAXContainer;

/** One-shot, low-output hardware test that only runs after the DS enters enabled Test mode. */
public final class HardwareSelfTestCommand {
    public static final String RUNNING_KEY = "Hardware Self-Test/Running";

    private static final double FOLLOWER_DIAGNOSTIC_DUTY = 0.08;
    private static final double FOLLOWER_DIAGNOSTIC_SECONDS = 1.20;

    private static final String COVERAGE_MANIFEST = String.join(
        "; ",
        "Spark30 intake actuator=SKIP_UNREFERENCED",
        "Spark31 intake roller=LOW_OUTPUT_STAGE",
        "Spark32 feeder=SKIP_KNOWN_STALL_SUSPECTED",
        "Spark33 conveyor=LOW_OUTPUT_STAGE",
        "Spark34/35 climber=MANUAL_ARMED_PULSE_ONLY",
        "Spark36/37 flywheel=PAIR_STAGE",
        "Spark37 follower=ISOLATED_STAGE",
        "Spark38 shooter actuator=SKIP_UNREFERENCED",
        "Spark39 turret=SKIP_UNREFERENCED",
        "CTRE20/40-57 swerve=LOW_OUTPUT_MOTION_OBSERVED_ONLY");

    private static final List<String> RESULT_TARGETS = List.of(
        "SPARK_ID30_INTAKE_ACTUATOR",
        "SPARK_ID31_INTAKE_ROLLER",
        "SPARK_ID32_FEEDER",
        "SPARK_ID33_CONVEYOR",
        "SPARK_ID34_35_CLIMBER",
        "SPARK_ID36_FLYWHEEL_LEADER",
        "SPARK_ID37_FLYWHEEL_FOLLOWER_PAIR",
        "SPARK_ID37_FOLLOWER_ISOLATED",
        "SPARK_ID38_SHOOTER_ACTUATOR",
        "SPARK_ID39_TURRET",
        "SWERVE_FORWARD",
        "SWERVE_STRAFE",
        "SWERVE_ROTATE");

    private HardwareSelfTestCommand() {}

    public static Command create(
            CommandSwerveDrivetrain drivetrain,
            IntakeSubsystem intake,
            ConveyorSubsystem conveyor,
            FeederSubsystem feeder,
            ShooterSubsystem shooter,
            TurretSubsystem turret,
            ClimberSubsystem climber) {
        Map<String, MotionResult> results = new LinkedHashMap<>();
        Map<Integer, Boolean> sparkCanResults = new LinkedHashMap<>();
        double duty = HardwareTestConstants.OPEN_LOOP_DUTY_CYCLE;

        Command sequence = Commands.sequence(
            Commands.runOnce(() -> {
                results.clear();
                sparkCanResults.clear();
                for (String target : RESULT_TARGETS) {
                    publishResult(results, target, MotionResult.NOT_RUN);
                }
                SmartDashboard.putBoolean(RUNNING_KEY, true);
                SmartDashboard.putNumber("Hardware Self-Test/Run ID", Timer.getFPGATimestamp());
                SmartDashboard.putString("Hardware Self-Test/Coverage", COVERAGE_MANIFEST);
                SmartDashboard.putString("Hardware Self-Test/Overall", "RUNNING");
                log("BEGIN", "motion evidence is not direction/calibration certification");
                log("COVERAGE", COVERAGE_MANIFEST);
                log("SPARK_CAN_STATUS", SparkMAXContainer.getDeviceAvailabilitySummary());
                recordSkipped(
                    results,
                    "SPARK_ID30_INTAKE_ACTUATOR",
                    "homing/reference sensor is not implemented");
                recordSkipped(
                    results,
                    "SPARK_ID32_FEEDER",
                    MotionResult.BLOCKED_KNOWN_FAULT,
                    "2026-08-10 log: 44.14A and approximately 0rpm; inspect jam/power branch first; "
                        + feeder.getDiagnosticStatus());
                recordSkipped(
                    results,
                    "SPARK_ID34_35_CLIMBER",
                    "automatic motion blocked; use separately armed 0.35s polarity pulse only; "
                        + climber.getDiagnosticStatus());
                recordSkipped(
                    results,
                    "SPARK_ID38_SHOOTER_ACTUATOR",
                    "homing/reference sensor is not implemented");
                recordSkipped(
                    results,
                    "SPARK_ID39_TURRET",
                    "homing/absolute reference is not implemented");
                captureSparkCanResults(sparkCanResults);
                SmartDashboard.putString("Hardware Self-Test/Results", results.toString());
            }, drivetrain, intake, conveyor, feeder, shooter, turret, climber),
            openLoopStage(
                "SPARK_ID31_INTAKE_ROLLER",
                () -> intake.getRollerDiagnosticSnapshot(false).ready(),
                () -> intake.runRollerDiagnostic(duty),
                intake::stopRoller,
                List.of(new DiagnosticTarget(
                    "SPARK_ID31_INTAKE_ROLLER",
                    duty,
                    IntakeConstants.ROLLER_CURRENT_LIMIT_AMPS,
                    intake::getRollerDiagnosticSnapshot)),
                HardwareTestConstants.OPEN_LOOP_STAGE_SECONDS,
                results,
                intake),
            openLoopStage(
                "SPARK_ID33_CONVEYOR",
                () -> conveyor.getDiagnosticSnapshot(false).ready(),
                () -> conveyor.runDiagnostic(duty),
                conveyor::stop,
                List.of(new DiagnosticTarget(
                    "SPARK_ID33_CONVEYOR",
                    duty,
                    ManipulatorConstants.CONVEYOR_CURRENT_LIMIT_AMPS,
                    conveyor::getDiagnosticSnapshot)),
                HardwareTestConstants.OPEN_LOOP_STAGE_SECONDS,
                results,
                conveyor),
            openLoopStage(
                "SPARK_ID36_37_FLYWHEEL_PAIR",
                () -> shooter.getFlywheelLeaderDiagnosticSnapshot(false).ready()
                    && shooter.getFlywheelFollowerDiagnosticSnapshot(false).ready(),
                () -> shooter.runFlywheelPairDiagnostic(duty),
                shooter::stopFlywheelPairDiagnostic,
                List.of(
                    new DiagnosticTarget(
                        "SPARK_ID36_FLYWHEEL_LEADER",
                        duty,
                        ShooterConstants.FLYWHEEL_CURRENT_LIMIT_AMPS,
                        shooter::getFlywheelLeaderDiagnosticSnapshot),
                    new DiagnosticTarget(
                        "SPARK_ID37_FLYWHEEL_FOLLOWER_PAIR",
                        -duty,
                        ShooterConstants.FLYWHEEL_CURRENT_LIMIT_AMPS,
                        shooter::getFlywheelFollowerDiagnosticSnapshot)),
                HardwareTestConstants.OPEN_LOOP_STAGE_SECONDS,
                results,
                shooter),
            openLoopStage(
                "SPARK_ID37_FOLLOWER_ISOLATED",
                () -> shooter.getFlywheelLeaderDiagnosticSnapshot(false).ready()
                    && shooter.getFlywheelFollowerDiagnosticSnapshot(false).ready(),
                () -> {
                    boolean accepted = shooter.runFollowerDiagnostic();
                    return accepted && shooter.isFollowerDiagnosticTransitionSafe();
                },
                shooter::stopFollowerDiagnostic,
                List.of(new DiagnosticTarget(
                    "SPARK_ID37_FOLLOWER_ISOLATED",
                    FOLLOWER_DIAGNOSTIC_DUTY,
                    ShooterConstants.FLYWHEEL_CURRENT_LIMIT_AMPS,
                    shooter::getIsolatedFollowerDiagnosticSnapshot)),
                FOLLOWER_DIAGNOSTIC_SECONDS,
                results,
                shooter),
            swerveStage(
                "SWERVE_FORWARD",
                drivetrain,
                () -> drivetrain.drive(0.015, 0, 0, false),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(
                    0.015 * TunerConstants.kSpeedAt12Volts.baseUnitMagnitude(), 0.0, 0.0),
                1.0,
                results),
            swerveStage(
                "SWERVE_STRAFE",
                drivetrain,
                () -> drivetrain.drive(0, 0.015, 0, false),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(
                    0.0, 0.015 * TunerConstants.kSpeedAt12Volts.baseUnitMagnitude(), 0.0),
                1.0,
                results),
            swerveStage(
                "SWERVE_ROTATE",
                drivetrain,
                () -> drivetrain.drive(0, 0, 0.12, false),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(0.0, 0.0, 0.12),
                1.0,
                results),
            Commands.runOnce(() -> {
                captureSparkCanResults(sparkCanResults);
                publishOverall(results, sparkCanResults);
            }));

        return sequence
            .until(() -> !testOutputsAllowed())
            .finallyDo(interrupted -> {
                drivetrain.requestIdle();
                intake.stopAll();
                conveyor.stop();
                feeder.stop();
                shooter.stop();
                turret.stop();
                climber.stop();
                SmartDashboard.putBoolean(RUNNING_KEY, false);
                if (interrupted || !testOutputsAllowed()) {
                    captureSparkCanResults(sparkCanResults);
                    SmartDashboard.putString("Hardware Self-Test/Overall", "INTERRUPTED_OUTPUTS_ZERO");
                }
                SmartDashboard.putString("Hardware Self-Test/Results", results.toString());
                log(interrupted ? "INTERRUPTED" : "END", "outputs forced to zero results=" + results);
            })
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    private static Command openLoopStage(
            String stageName,
            BooleanSupplier preflightReady,
            BooleanSupplier action,
            Runnable stop,
            List<DiagnosticTarget> targets,
            double durationSeconds,
            Map<String, MotionResult> report,
            Subsystem... requirements) {
        boolean[] eligible = {false};
        boolean[] attempted = {false};
        boolean[] accepted = {false};
        boolean[] aborted = {false};
        Map<String, MotionResult> terminalResults = new LinkedHashMap<>();

        return Commands.sequence(
            Commands.runOnce(() -> {
                attempted[0] = false;
                accepted[0] = false;
                aborted[0] = false;
                terminalResults.clear();
                eligible[0] = testOutputsAllowed() && preflightReady.getAsBoolean();
                log(stageName + "_START", "eligible=" + eligible[0] + " " + formatTargets(targets, false));
            }),
            Commands.deadline(
                Commands.waitSeconds(durationSeconds),
                Commands.run(() -> {
                    if (!eligible[0] || aborted[0] || !testOutputsAllowed()) {
                        stop.run();
                        aborted[0] = aborted[0] || !testOutputsAllowed();
                        return;
                    }
                    boolean commandAccepted = action.getAsBoolean();
                    attempted[0] = true;
                    accepted[0] = commandAccepted;
                    if (!commandAccepted) {
                        aborted[0] = true;
                        for (DiagnosticTarget target : targets) {
                            terminalResults.put(target.name(), MotionResult.FAIL_COMMAND_REJECTED);
                        }
                        stop.run();
                        return;
                    }

                    Map<String, MotionResult> liveResults = evaluateTargets(targets, true);
                    if (liveResults.containsValue(MotionResult.STALL_SUSPECTED)
                            || liveResults.containsValue(MotionResult.FAIL_DIRECTION_MISMATCH)) {
                        terminalResults.putAll(liveResults);
                        aborted[0] = true;
                        stop.run();
                        log(stageName + "_ABORT", "unsafe motion evidence " + liveResults);
                    }
                }, requirements),
                Commands.sequence(
                    Commands.waitSeconds(durationSeconds / 2.0),
                    Commands.runOnce(() -> log(
                        stageName + "_SAMPLE",
                        "results=" + evaluateTargets(targets, accepted[0])
                            + " telemetry=" + formatTargets(targets, accepted[0]))))),
            Commands.runOnce(() -> {
                Map<String, MotionResult> finalResults = new LinkedHashMap<>();
                if (!eligible[0]) {
                    for (DiagnosticTarget target : targets) {
                        finalResults.put(target.name(), MotionResult.FAIL_NOT_READY);
                    }
                } else if (!terminalResults.isEmpty()) {
                    finalResults.putAll(terminalResults);
                } else if (!attempted[0] || !accepted[0]) {
                    for (DiagnosticTarget target : targets) {
                        finalResults.put(target.name(), MotionResult.FAIL_COMMAND_REJECTED);
                    }
                } else {
                    finalResults.putAll(evaluateTargets(targets, true));
                }
                finalResults.forEach((name, result) -> publishResult(report, name, result));
                log(stageName + "_RESULT", finalResults.toString());
            }),
            Commands.runOnce(stop, requirements),
            Commands.runOnce(() -> log(stageName + "_STOP", formatTargets(targets, false))),
            Commands.waitSeconds(HardwareTestConstants.STAGE_SETTLE_SECONDS));
    }

    private static Command swerveStage(
            String name,
            CommandSwerveDrivetrain drivetrain,
            Runnable action,
            edu.wpi.first.math.kinematics.ChassisSpeeds expectedSpeeds,
            double durationSeconds,
            Map<String, MotionResult> report) {
        boolean[] eligible = {false};
        boolean[] connectionLost = {false};
        boolean[] motionObserved = {false};
        MotionResult[] terminalResult = {null};
        CommandSwerveDrivetrain.SwerveDiagnosticEvidence[] latestEvidence = {null};

        return Commands.sequence(
            Commands.runOnce(() -> {
                eligible[0] = testOutputsAllowed() && drivetrain.areAllDevicesConnected();
                connectionLost[0] = false;
                motionObserved[0] = false;
                terminalResult[0] = null;
                latestEvidence[0] = getSwerveEvidence(drivetrain, expectedSpeeds);
                log(name + "_START", "eligible=" + eligible[0] + " " + drivetrain.getDeviceHealthSummary());
            }),
            Commands.deadline(
                Commands.waitSeconds(durationSeconds),
                Commands.run(() -> {
                    if (!eligible[0]
                            || connectionLost[0]
                            || terminalResult[0] != null
                            || !testOutputsAllowed()
                            || !drivetrain.areAllDevicesConnected()) {
                        connectionLost[0] |= eligible[0]
                            && (!testOutputsAllowed() || !drivetrain.areAllDevicesConnected());
                        drivetrain.requestIdle();
                        return;
                    }
                    action.run();
                    latestEvidence[0] = getSwerveEvidence(drivetrain, expectedSpeeds);
                    if (!latestEvidence[0].ready() || !latestEvidence[0].telemetryFinite()) {
                        connectionLost[0] = true;
                        drivetrain.requestIdle();
                        return;
                    }
                    if (latestEvidence[0].overCurrent()) {
                        terminalResult[0] = MotionResult.STALL_SUSPECTED;
                        drivetrain.requestIdle();
                        log(name + "_ABORT", "swerve overcurrent " + latestEvidence[0]);
                        return;
                    }
                    if (motionObserved[0]
                            && latestEvidence[0].motionObserved()
                            && !latestEvidence[0].steeringAligned()) {
                        terminalResult[0] = MotionResult.FAIL_DIRECTION_MISMATCH;
                        drivetrain.requestIdle();
                        log(name + "_ABORT", "swerve vector deviated " + latestEvidence[0]);
                        return;
                    }
                    motionObserved[0] |= latestEvidence[0].motionObserved()
                        && latestEvidence[0].steeringAligned();
                }, drivetrain),
                Commands.sequence(
                    Commands.waitSeconds(durationSeconds / 2.0),
                    Commands.runOnce(() -> log(name + "_SAMPLE", drivetrain.getMotionDiagnosticSummary())))),
            Commands.runOnce(() -> {
                latestEvidence[0] = getSwerveEvidence(drivetrain, expectedSpeeds);
                MotionResult result;
                if (terminalResult[0] != null) {
                    result = terminalResult[0];
                } else if (!eligible[0] || connectionLost[0] || !latestEvidence[0].ready()) {
                    result = MotionResult.FAIL_NOT_READY;
                } else if (latestEvidence[0].motionObserved()
                        && !latestEvidence[0].steeringAligned()) {
                    result = MotionResult.FAIL_DIRECTION_MISMATCH;
                } else if (motionObserved[0]
                        && latestEvidence[0].motionObserved()
                        && latestEvidence[0].steeringAligned()) {
                    result = MotionResult.PASS_OBSERVED;
                } else {
                    result = MotionResult.INCONCLUSIVE_NO_MOTION;
                }
                publishResult(report, name, result);
                log(
                    name + "_RESULT",
                    result + " evidence=" + latestEvidence[0]
                        + " telemetry=" + drivetrain.getMotionDiagnosticSummary());
            }),
            Commands.runOnce(drivetrain::requestIdle, drivetrain),
            Commands.waitSeconds(HardwareTestConstants.STAGE_SETTLE_SECONDS));
    }

    private static CommandSwerveDrivetrain.SwerveDiagnosticEvidence getSwerveEvidence(
            CommandSwerveDrivetrain drivetrain,
            edu.wpi.first.math.kinematics.ChassisSpeeds expectedSpeeds) {
        return drivetrain.getSwerveDiagnosticEvidence(
            expectedSpeeds,
            HardwareTestConstants.MIN_SWERVE_MODULE_SPEED_METERS_PER_SECOND,
            HardwareTestConstants.MAX_SWERVE_VECTOR_ERROR_DEGREES,
            HardwareTestConstants.MAX_SWERVE_DIAGNOSTIC_DRIVE_CURRENT_AMPS,
            HardwareTestConstants.MAX_SWERVE_DIAGNOSTIC_STEER_CURRENT_AMPS);
    }

    private static Map<String, MotionResult> evaluateTargets(
            List<DiagnosticTarget> targets, boolean commandAccepted) {
        Map<String, MotionResult> results = new LinkedHashMap<>();
        for (DiagnosticTarget target : targets) {
            Snapshot snapshot = target.snapshot().apply(commandAccepted);
            results.put(
                target.name(),
                HardwareDiagnosticEvaluator.evaluateOpenLoop(
                    snapshot, target.requestedDuty(), target.currentLimitAmps()));
        }
        return results;
    }

    private static String formatTargets(
            List<DiagnosticTarget> targets, boolean commandAccepted) {
        StringBuilder summary = new StringBuilder();
        for (DiagnosticTarget target : targets) {
            if (!summary.isEmpty()) {
                summary.append("; ");
            }
            summary.append(target.snapshot().apply(commandAccepted));
        }
        return summary.toString();
    }

    private static void recordSkipped(
            Map<String, MotionResult> report, String name, String reason) {
        recordSkipped(report, name, MotionResult.SKIPPED, reason);
    }

    private static void recordSkipped(
            Map<String, MotionResult> report,
            String name,
            MotionResult result,
            String reason) {
        publishResult(report, name, result);
        SmartDashboard.putString("Hardware Self-Test/" + name + "/Reason", reason);
        log(name + "_SKIPPED", reason);
    }

    private static void publishResult(
            Map<String, MotionResult> report, String name, MotionResult result) {
        report.put(name, result);
        SmartDashboard.putString("Hardware Self-Test/" + name + "/Motion Result", result.name());
    }

    private static void captureSparkCanResults(Map<Integer, Boolean> canResults) {
        for (int canId = 30; canId <= 39; canId++) {
            boolean ready = SparkMAXContainer.getDiagnosticSnapshotForId(canId)
                .map(Snapshot::ready)
                .orElse(false);
            canResults.put(canId, ready);
            SmartDashboard.putString(
                "Hardware Self-Test/SPARK_ID" + canId + "/CAN Result",
                ready ? "PASS_READY" : "FAIL_NOT_READY");
        }
        SmartDashboard.putString("Hardware Self-Test/CAN Results", canResults.toString());
        log("SPARK_CAN_RESULT", canResults.toString());
    }

    private static void publishOverall(
            Map<String, MotionResult> results, Map<Integer, Boolean> sparkCanResults) {
        boolean attentionRequired = results.values().stream().anyMatch(result ->
            result == MotionResult.FAIL_NOT_READY
                || result == MotionResult.FAIL_COMMAND_REJECTED
                || result == MotionResult.FAIL_DIRECTION_MISMATCH
                || result == MotionResult.STALL_SUSPECTED
                || result == MotionResult.BLOCKED_KNOWN_FAULT);
        attentionRequired |= sparkCanResults.size() != 10
            || sparkCanResults.values().stream().anyMatch(ready -> !ready);
        boolean incomplete = results.values().stream().anyMatch(result ->
            result == MotionResult.INCONCLUSIVE_NO_MOTION
                || result == MotionResult.SKIPPED
                || result == MotionResult.NOT_RUN);
        String overall = attentionRequired
            ? "ATTENTION_REQUIRED"
            : incomplete ? "INCOMPLETE_DESIGN_OR_MOTION_EVIDENCE" : "MOTION_OBSERVED_ONLY";
        SmartDashboard.putString("Hardware Self-Test/Overall", overall);
        SmartDashboard.putString("Hardware Self-Test/Results", results.toString());
        log(
            "SEQUENCE_COMPLETE",
            "overall=" + overall + " motion=" + results + " sparkCAN=" + sparkCanResults);
    }

    private static boolean testOutputsAllowed() {
        return DriverStation.isTestEnabled() && !DriverStation.isFMSAttached();
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

    private record DiagnosticTarget(
        String name,
        double requestedDuty,
        double currentLimitAmps,
        Function<Boolean, Snapshot> snapshot) {}
}
