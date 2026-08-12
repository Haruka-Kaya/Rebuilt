package frc.robot.commands;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
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
import frc.robot.constants.ConfiguredCanHardware;
import frc.robot.constants.ConfiguredMotorCapabilities;
import frc.robot.constants.TunerConstants;
import frc.robot.diagnostics.HardwareDiagnosticEvaluator;
import frc.robot.diagnostics.HardwareDiagnosticEvaluator.MotionResult;
import frc.robot.diagnostics.HardwareDiagnosticEvaluator.Snapshot;
import frc.robot.diagnostics.SelfTestRunState;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.AsyncDiagnosticSink;
import frc.robot.utils.CtreDeviceEvidence;
import frc.robot.utils.SparkMAXContainer;
import frc.robot.utils.SparkMAXContainer.DeviceEvidenceSnapshot;

/** One-shot, low-output hardware test that only runs after the DS enters enabled Test mode. */
public final class HardwareSelfTestCommand {
    public static final String RUNNING_KEY = "Hardware Self-Test/Running";
    public static final String EVIDENCE_SOURCE_KEY = "Hardware Self-Test/Evidence Source";

    private static final double FOLLOWER_DIAGNOSTIC_DUTY = 0.08;
    private static final double FOLLOWER_DIAGNOSTIC_SECONDS = 1.20;
    private static final int[] ALL_SPARK_IDS = ConfiguredCanHardware.sparkDeviceIds().stream()
        .mapToInt(Integer::intValue)
        .toArray();

    private static final String COVERAGE_MANIFEST =
        ConfiguredMotorCapabilities.hardwareSelfTestCoverageSummary();

    private static final List<String> STOP_RESULT_NAMES = List.of(
        "GLOBAL_START",
        "SPARK_ID31_INTAKE_ROLLER_SPARK_STOP",
        "SPARK_ID32_FEEDER_CONTROLLED_RETEST_SPARK_STOP",
        "SPARK_ID33_CONVEYOR_SPARK_STOP",
        "SPARK_ID36_37_FLYWHEEL_PAIR_SPARK_STOP",
        "SPARK_ID37_FOLLOWER_ISOLATED_SPARK_STOP",
        "SWERVE_FORWARD_SWERVE_STOP",
        "SWERVE_STRAFE_SWERVE_STOP",
        "SWERVE_ROTATE_SWERVE_STOP",
        "GLOBAL_END");

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
        Map<Integer, CanEvidenceResult> allCanResults = new LinkedHashMap<>();
        SelfTestRunState runState = new SelfTestRunState();
        double duty = HardwareTestConstants.OPEN_LOOP_DUTY_CYCLE;

        Command sequence = Commands.sequence(
            Commands.runOnce(() -> {
                results.clear();
                sparkCanResults.clear();
                allCanResults.clear();
                for (String target : RESULT_TARGETS) {
                    publishResult(results, target, MotionResult.NOT_RUN);
                    SmartDashboard.putString(
                        "Hardware Self-Test/" + target + "/Reason", "NOT_RUN");
                }
                for (String stopResultName : STOP_RESULT_NAMES) {
                    SmartDashboard.putString(stopResultKey(stopResultName), "NOT_RUN");
                }
                SmartDashboard.putBoolean(RUNNING_KEY, true);
                SmartDashboard.putNumber("Hardware Self-Test/Run ID", Timer.getFPGATimestamp());
                SmartDashboard.putString("Hardware Self-Test/Coverage", COVERAGE_MANIFEST);
                SmartDashboard.putString(
                    EVIDENCE_SOURCE_KEY, evidenceSource(RobotBase.isSimulation()));
                SmartDashboard.putString("Hardware Self-Test/Overall", "RUNNING");
                SmartDashboard.putString("Hardware Self-Test/Abort Reason", "NONE");
                log("BEGIN", "motion evidence is not direction/calibration certification");
                log("COVERAGE", COVERAGE_MANIFEST);
                log("SPARK_CAN_STATUS", SparkMAXContainer.getDeviceAvailabilitySummary());
                recordSkipped(
                    results,
                    "SPARK_ID30_INTAKE_ACTUATOR",
                    "automatic motion blocked; use separately armed manual polarity pulse only");
                if (!feeder.isControlledRetestEnabled()) {
                    recordSkipped(
                        results,
                        "SPARK_ID32_FEEDER",
                        MotionResult.BLOCKED_KNOWN_FAULT,
                        "2026-08-10 log: 44.14A and approximately 0rpm; inspect jam/power branch first; "
                            + feeder.getDiagnosticStatus());
                }
                recordSkipped(
                    results,
                    "SPARK_ID34_35_CLIMBER",
                    "automatic motion blocked; use separately armed 0.35s polarity pulse only; "
                        + climber.getDiagnosticStatus());
                recordSkipped(
                    results,
                    "SPARK_ID38_SHOOTER_ACTUATOR",
                    "automatic motion blocked; use separately armed manual polarity pulse only");
                recordSkipped(
                    results,
                    "SPARK_ID39_TURRET",
                    "automatic motion blocked; use separately armed manual polarity pulse only");
                captureCanResults(sparkCanResults, allCanResults, drivetrain);
                SmartDashboard.putString("Hardware Self-Test/Results", results.toString());
            }, drivetrain, intake, conveyor, feeder, shooter, turret, climber),
            globalStopBarrier(
                "GLOBAL_START",
                drivetrain,
                intake,
                conveyor,
                feeder,
                shooter,
                turret,
                climber,
                runState),
            guardedStage(openLoopStage(
                "SPARK_ID31_INTAKE_ROLLER",
                () -> intake.getRollerDiagnosticSnapshot(false).ready(),
                () -> intake.runRollerDiagnostic(duty),
                () -> true,
                intake::stopRoller,
                new int[] {IntakeConstants.INTAKE_ROLLER_CAN_ID},
                List.of(new DiagnosticTarget(
                    "SPARK_ID31_INTAKE_ROLLER",
                    duty,
                    IntakeConstants.ROLLER_CURRENT_LIMIT_AMPS,
                    intake::getRollerDiagnosticSnapshot)),
                HardwareTestConstants.OPEN_LOOP_STAGE_SECONDS,
                results,
                runState,
                intake), runState),
            guardedStage(Commands.either(
                openLoopStage(
                    "SPARK_ID32_FEEDER_CONTROLLED_RETEST",
                    () -> feeder.getDiagnosticSnapshot(false).ready(),
                    () -> feeder.runControlledDiagnostic(duty),
                    () -> true,
                    feeder::stop,
                    new int[] {ManipulatorConstants.FEEDER_CAN_ID},
                    List.of(new DiagnosticTarget(
                        "SPARK_ID32_FEEDER",
                        duty,
                        ManipulatorConstants.FEEDER_CURRENT_LIMIT_AMPS,
                        feeder::getDiagnosticSnapshot)),
                    HardwareTestConstants.OPEN_LOOP_STAGE_SECONDS,
                    results,
                    runState,
                    feeder),
                Commands.none(),
                feeder::isControlledRetestEnabled), runState),
            guardedStage(openLoopStage(
                "SPARK_ID33_CONVEYOR",
                () -> conveyor.getDiagnosticSnapshot(false).ready(),
                () -> conveyor.runDiagnostic(duty),
                () -> true,
                conveyor::stop,
                new int[] {ManipulatorConstants.CONVEYOR_CAN_ID},
                List.of(new DiagnosticTarget(
                    "SPARK_ID33_CONVEYOR",
                    duty,
                    ManipulatorConstants.CONVEYOR_CURRENT_LIMIT_AMPS,
                    conveyor::getDiagnosticSnapshot)),
                HardwareTestConstants.OPEN_LOOP_STAGE_SECONDS,
                results,
                runState,
                conveyor), runState),
            guardedStage(openLoopStage(
                "SPARK_ID36_37_FLYWHEEL_PAIR",
                () -> shooter.getFlywheelLeaderDiagnosticSnapshot(false).ready()
                    && shooter.getFlywheelFollowerDiagnosticSnapshot(false).ready(),
                () -> shooter.runFlywheelPairDiagnostic(duty),
                () -> true,
                shooter::stopFlywheelPairDiagnostic,
                new int[] {
                    ShooterConstants.SHOOTER_1_CAN_ID,
                    ShooterConstants.SHOOTER_2_CAN_ID
                },
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
                runState,
                shooter), runState),
            guardedStage(openLoopStage(
                "SPARK_ID37_FOLLOWER_ISOLATED",
                () -> shooter.getFlywheelLeaderDiagnosticSnapshot(false).ready()
                    && shooter.getFlywheelFollowerDiagnosticSnapshot(false).ready(),
                () -> {
                    boolean accepted = shooter.runFollowerDiagnostic();
                    return accepted && shooter.isFollowerDiagnosticTransitionSafe();
                },
                shooter::isFollowerDiagnosticOutputSafe,
                shooter::stopFollowerDiagnostic,
                new int[] {
                    ShooterConstants.SHOOTER_1_CAN_ID,
                    ShooterConstants.SHOOTER_2_CAN_ID
                },
                List.of(new DiagnosticTarget(
                    "SPARK_ID37_FOLLOWER_ISOLATED",
                    FOLLOWER_DIAGNOSTIC_DUTY,
                    ShooterConstants.FLYWHEEL_CURRENT_LIMIT_AMPS,
                    shooter::getIsolatedFollowerDiagnosticSnapshot)),
                FOLLOWER_DIAGNOSTIC_SECONDS,
                results,
                runState,
                shooter), runState),
            guardedStage(swerveStage(
                "SWERVE_FORWARD",
                drivetrain,
                () -> drivetrain.drive(0.015, 0, 0, false),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(
                    0.015 * TunerConstants.kSpeedAt12Volts.baseUnitMagnitude(), 0.0, 0.0),
                1.0,
                results,
                runState), runState),
            guardedStage(swerveStage(
                "SWERVE_STRAFE",
                drivetrain,
                () -> drivetrain.drive(0, 0.015, 0, false),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(
                    0.0, 0.015 * TunerConstants.kSpeedAt12Volts.baseUnitMagnitude(), 0.0),
                1.0,
                results,
                runState), runState),
            guardedStage(swerveStage(
                "SWERVE_ROTATE",
                drivetrain,
                () -> drivetrain.drive(0, 0, 0.12, false),
                new edu.wpi.first.math.kinematics.ChassisSpeeds(0.0, 0.0, 0.12),
                1.0,
                results,
                runState), runState),
            globalStopBarrier(
                "GLOBAL_END",
                drivetrain,
                intake,
                conveyor,
                feeder,
                shooter,
                turret,
                climber,
                runState),
            Commands.runOnce(() -> {
                captureCanResults(sparkCanResults, allCanResults, drivetrain);
                publishOverall(results, allCanResults, runState);
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
                    captureCanResults(sparkCanResults, allCanResults, drivetrain);
                    SmartDashboard.putString(
                        "Hardware Self-Test/Overall", "INTERRUPTED_STOP_REQUESTED");
                    String abortReason = runState.abortReason();
                    if (abortReason.isBlank()) {
                        abortReason = !testOutputsAllowed()
                            ? "INTERRUPTED_TEST_OUTPUTS_NOT_ALLOWED"
                            : "INTERRUPTED_COMMAND_CANCELED";
                    }
                    SmartDashboard.putString("Hardware Self-Test/Abort Reason", abortReason);
                }
                SmartDashboard.putString("Hardware Self-Test/Results", results.toString());
                log(
                    interrupted ? "INTERRUPTED" : "END",
                    "stop requested; confirmation requires a completed barrier results=" + results);
            })
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    }

    private static Command openLoopStage(
            String stageName,
            BooleanSupplier preflightReady,
            BooleanSupplier action,
            BooleanSupplier outputSubmissionComplete,
            Runnable stop,
            int[] stopCanIds,
            List<DiagnosticTarget> targets,
            double durationSeconds,
            Map<String, MotionResult> report,
            SelfTestRunState runState,
            Subsystem... requirements) {
        boolean[] eligible = {false};
        boolean[] attempted = {false};
        boolean[] accepted = {false};
        boolean[] aborted = {false};
        Map<String, MotionResult> terminalResults = new LinkedHashMap<>();
        Map<String, MotionResult> latestResults = new LinkedHashMap<>();
        Map<String, TimedTargetState> timedTargets = new LinkedHashMap<>();

        return Commands.sequence(
            Commands.runOnce(() -> {
                attempted[0] = false;
                accepted[0] = false;
                aborted[0] = false;
                terminalResults.clear();
                latestResults.clear();
                timedTargets.clear();
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
                    if (!attempted[0]) {
                        boolean commandAccepted = action.getAsBoolean();
                        accepted[0] = commandAccepted;
                        if (!commandAccepted) {
                            aborted[0] = true;
                            for (DiagnosticTarget target : targets) {
                                terminalResults.put(
                                    target.name(), MotionResult.FAIL_COMMAND_REJECTED);
                            }
                            stop.run();
                            return;
                        }
                        if (!outputSubmissionComplete.getAsBoolean()) {
                            return;
                        }
                        captureTimedTargetStates(targets, timedTargets);
                        attempted[0] = true;
                    }
                    if (timedTargets.size() != targets.size()) {
                        aborted[0] = true;
                        for (DiagnosticTarget target : targets) {
                            terminalResults.put(target.name(), MotionResult.FAIL_NOT_READY);
                        }
                        stop.run();
                        return;
                    }
                    if (!timedTargetsStillReady(timedTargets)) {
                        aborted[0] = true;
                        for (DiagnosticTarget target : targets) {
                            terminalResults.put(target.name(), MotionResult.FAIL_NOT_READY);
                        }
                        stop.run();
                        return;
                    }

                    Map<String, MotionResult> liveResults = evaluateFreshTimedTargets(
                        targets, timedTargets);
                    latestResults.putAll(liveResults);
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
                        "results=" + latestResults
                            + " telemetry=" + formatTargets(targets, accepted[0]))))),
            Commands.runOnce(() -> {
                Map<String, MotionResult> finalResults = new LinkedHashMap<>();
                Map<String, MotionResult> currentResults = evaluateCurrentTimedTargets(
                    targets, timedTargets);
                if (!eligible[0]) {
                    for (DiagnosticTarget target : targets) {
                        finalResults.put(target.name(), MotionResult.FAIL_NOT_READY);
                    }
                } else if (!terminalResults.isEmpty()) {
                    finalResults.putAll(terminalResults);
                } else if (!attempted[0]
                        || !accepted[0]
                        || currentResults.size() != targets.size()) {
                    for (DiagnosticTarget target : targets) {
                        finalResults.put(target.name(), MotionResult.FAIL_NOT_READY);
                    }
                } else {
                    finalResults.putAll(currentResults);
                }
                finalResults.forEach((name, result) -> publishResult(report, name, result));
                log(stageName + "_RESULT", finalResults.toString());
            }),
            sparkStopBarrier(
                stageName,
                stop,
                stopCanIds,
                runState,
                requirements));
    }

    private static Command swerveStage(
            String name,
            CommandSwerveDrivetrain drivetrain,
            java.util.function.Supplier<CommandSwerveDrivetrain.ControlResult> action,
            edu.wpi.first.math.kinematics.ChassisSpeeds expectedSpeeds,
            double durationSeconds,
            Map<String, MotionResult> report,
            SelfTestRunState runState) {
        boolean[] eligible = {false};
        boolean[] connectionLost = {false};
        boolean[] motionObserved = {false};
        boolean[] simulationCurrentIgnored = {false};
        MotionResult[] terminalResult = {null};
        String[] assessmentReason = {"NOT_RUN"};
        CommandSwerveDrivetrain.SwerveDiagnosticEvidence[] latestEvidence = {null};
        CommandSwerveDrivetrain.SwerveDiagnosticBaseline[] baseline = {null};
        CommandSwerveDrivetrain.SwerveDiagnosticToken[] token = {null};

        return Commands.sequence(
            Commands.runOnce(() -> {
                eligible[0] = testOutputsAllowed() && drivetrain.areAllDevicesConnected();
                connectionLost[0] = false;
                motionObserved[0] = false;
                simulationCurrentIgnored[0] = false;
                terminalResult[0] = null;
                assessmentReason[0] = eligible[0]
                    ? "WAITING_FOR_POST_COMMAND_EVIDENCE"
                    : "INITIAL_DEVICE_READINESS_FAILED";
                baseline[0] = drivetrain.captureSwerveDiagnosticBaseline();
                token[0] = null;
                latestEvidence[0] = null;
                log(name + "_START", "eligible=" + eligible[0] + " " + drivetrain.getDeviceHealthSummary());
            }),
            Commands.deadline(
                Commands.waitSeconds(durationSeconds),
                Commands.run(() -> {
                    boolean outputsAllowed = testOutputsAllowed();
                    boolean devicesConnected = drivetrain.areAllDevicesConnected();
                    if (!eligible[0]
                            || connectionLost[0]
                            || terminalResult[0] != null
                            || !outputsAllowed
                            || !devicesConnected) {
                        if (eligible[0] && !outputsAllowed) {
                            assessmentReason[0] = "TEST_OUTPUT_AUTHORIZATION_LOST_DURING_STAGE";
                        } else if (eligible[0] && !devicesConnected) {
                            assessmentReason[0] = "DEVICE_CONNECTIVITY_LOST_DURING_STAGE";
                        }
                        connectionLost[0] |= eligible[0]
                            && (!outputsAllowed || !devicesConnected);
                        drivetrain.requestIdle();
                        return;
                    }
                    if (token[0] == null) {
                        CommandSwerveDrivetrain.ControlResult submission = action.get();
                        token[0] = drivetrain.completeSwerveDiagnosticRequest(
                            baseline[0], submission);
                        if (submission != CommandSwerveDrivetrain.ControlResult.REQUEST_SUBMITTED) {
                            connectionLost[0] = true;
                            assessmentReason[0] = "CONTROL_SUBMISSION_" + submission;
                            drivetrain.requestIdle();
                            log(name + "_ABORT", "control submission=" + submission);
                            return;
                        }
                    }
                    latestEvidence[0] = getSwerveEvidence(
                        drivetrain, expectedSpeeds, token[0]);
                    if (!latestEvidence[0].postCommandEvidenceReady()) {
                        if (latestEvidence[0].postCommandEvidenceTimedOut()) {
                            connectionLost[0] = true;
                            assessmentReason[0] = "POST_COMMAND_EVIDENCE_TIMEOUT";
                            drivetrain.requestIdle();
                        }
                        return;
                    }
                    if (!latestEvidence[0].ready() || !latestEvidence[0].telemetryFinite()) {
                        connectionLost[0] = true;
                        assessmentReason[0] = latestEvidence[0].ready()
                            ? "POST_COMMAND_TELEMETRY_NONFINITE"
                            : "POST_COMMAND_EVIDENCE_NOT_READY";
                        drivetrain.requestIdle();
                        return;
                    }
                    if (latestEvidence[0].overCurrent()) {
                        if (RobotBase.isSimulation()) {
                            // Phoenix desktop current is not backed by the verified robot load.
                            // Keep the stage running so command/output routing can be observed,
                            // while never promoting this value to physical stall evidence.
                            if (!simulationCurrentIgnored[0]) {
                                simulationCurrentIgnored[0] = true;
                                log(
                                    name + "_SIM_CURRENT_IGNORED",
                                    "not physical stall evidence " + latestEvidence[0]);
                            }
                        } else {
                            terminalResult[0] = MotionResult.STALL_SUSPECTED;
                            assessmentReason[0] = "LIVE_HARDWARE_OVERCURRENT";
                            drivetrain.requestIdle();
                            log(name + "_ABORT", "swerve overcurrent " + latestEvidence[0]);
                            return;
                        }
                    }
                    if (motionObserved[0]
                            && latestEvidence[0].motionObserved()
                            && !latestEvidence[0].steeringAligned()) {
                        terminalResult[0] = MotionResult.FAIL_DIRECTION_MISMATCH;
                        assessmentReason[0] = "SWERVE_VECTOR_DIRECTION_MISMATCH";
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
                latestEvidence[0] = getSwerveEvidence(
                    drivetrain, expectedSpeeds, token[0]);
                MotionResult result;
                if (terminalResult[0] != null) {
                    result = terminalResult[0];
                } else if (!eligible[0] || connectionLost[0] || !latestEvidence[0].ready()) {
                    result = MotionResult.FAIL_NOT_READY;
                    if (connectionLost[0]
                            && assessmentReason[0].equals("DEVICE_CONNECTIVITY_LOST_DURING_STAGE")
                            && drivetrain.areAllDevicesConnected()
                            && latestEvidence[0].ready()) {
                        assessmentReason[0] =
                            "DEVICE_CONNECTIVITY_LOST_DURING_STAGE_RECOVERED_BEFORE_FINAL_ASSESSMENT";
                    }
                } else if (latestEvidence[0].motionObserved()
                        && !latestEvidence[0].steeringAligned()) {
                    result = MotionResult.FAIL_DIRECTION_MISMATCH;
                    assessmentReason[0] = "FINAL_SWERVE_VECTOR_DIRECTION_MISMATCH";
                } else if (motionObserved[0]
                        && latestEvidence[0].motionObserved()
                        && latestEvidence[0].steeringAligned()) {
                    result = MotionResult.PASS_OBSERVED;
                    assessmentReason[0] = "POST_COMMAND_MOTION_AND_DIRECTION_OBSERVED";
                } else {
                    result = MotionResult.INCONCLUSIVE_NO_MOTION;
                    assessmentReason[0] = "POST_COMMAND_OUTPUT_EVIDENCE_WITHOUT_MOTION";
                }
                publishResult(report, name, result);
                SmartDashboard.putString(
                    "Hardware Self-Test/" + name + "/Reason", assessmentReason[0]);
                log(
                    name + "_RESULT",
                    result + " reason=" + assessmentReason[0]
                        + " evidence=" + latestEvidence[0]
                        + " telemetry=" + drivetrain.getMotionDiagnosticSummary());
            }),
            swerveStopBarrier(name, drivetrain, runState));
    }

    private static Command guardedStage(Command stage, SelfTestRunState runState) {
        return Commands.either(stage, Commands.none(), runState::mayContinue);
    }

    private static Command sparkStopBarrier(
            String stageName,
            Runnable stop,
            int[] canIds,
            SelfTestRunState runState,
            Subsystem... requirements) {
        return stopBarrier(
            stageName + "_SPARK_STOP",
            () -> {
                stop.run();
                SparkMAXContainer.OutputStopBatch batch =
                    SparkMAXContainer.requestOutputStops(canIds);
                return () -> {
                    SparkMAXContainer.OutputStopSnapshot snapshot = batch.snapshot();
                    return new StopPoll(snapshot.confirmed(), snapshot.summary());
                };
            },
            runState,
            requirements);
    }

    private static Command swerveStopBarrier(
            String stageName,
            CommandSwerveDrivetrain drivetrain,
            SelfTestRunState runState) {
        return stopBarrier(
            stageName + "_SWERVE_STOP",
            () -> {
                CommandSwerveDrivetrain.SwerveStopToken token = drivetrain.requestIdleWithToken();
                return () -> {
                    drivetrain.retryIdleIfNeeded(token);
                    CommandSwerveDrivetrain.SwerveStopEvidence evidence = drivetrain.getStopEvidence(
                        token,
                        HardwareTestConstants.MAX_STOPPED_SWERVE_SPEED_METERS_PER_SECOND);
                    return new StopPoll(evidence.confirmed(), evidence.reason());
                };
            },
            runState,
            drivetrain);
    }

    private static Command globalStopBarrier(
            String stageName,
            CommandSwerveDrivetrain drivetrain,
            IntakeSubsystem intake,
            ConveyorSubsystem conveyor,
            FeederSubsystem feeder,
            ShooterSubsystem shooter,
            TurretSubsystem turret,
            ClimberSubsystem climber,
            SelfTestRunState runState) {
        return stopBarrier(
            stageName,
            () -> {
                intake.stopAll();
                conveyor.stop();
                feeder.stop();
                shooter.stop();
                turret.stop();
                climber.stop();
                SparkMAXContainer.OutputStopBatch sparkBatch =
                    SparkMAXContainer.requestOutputStops(ALL_SPARK_IDS);
                CommandSwerveDrivetrain.SwerveStopToken swerveToken =
                    drivetrain.requestIdleWithToken();
                return () -> {
                    SparkMAXContainer.OutputStopSnapshot spark = sparkBatch.snapshot();
                    drivetrain.retryIdleIfNeeded(swerveToken);
                    CommandSwerveDrivetrain.SwerveStopEvidence swerve =
                        drivetrain.getStopEvidence(
                            swerveToken,
                            HardwareTestConstants.MAX_STOPPED_SWERVE_SPEED_METERS_PER_SECOND);
                    return new StopPoll(
                        spark.confirmed() && swerve.confirmed(),
                        "spark=" + spark.summary() + " swerve=" + swerve.reason());
                };
            },
            runState,
            drivetrain,
            intake,
            conveyor,
            feeder,
            shooter,
            turret,
            climber);
    }

    private static Command stopBarrier(
            String name,
            Supplier<StopMonitor> request,
            SelfTestRunState runState,
            Subsystem... requirements) {
        StopMonitor[] monitor = {null};
        StopPoll[] latest = {new StopPoll(false, "NOT_REQUESTED")};
        String dashboardKey = stopResultKey(name);

        return Commands.sequence(
            Commands.runOnce(() -> {
                latest[0] = new StopPoll(false, "PENDING");
                SmartDashboard.putString(dashboardKey, "PENDING");
                monitor[0] = request.get();
                log(name + "_REQUESTED", "stop requested");
            }, requirements),
            Commands.waitUntil(() -> {
                latest[0] = monitor[0].poll();
                return latest[0].confirmed();
            }).withTimeout(HardwareTestConstants.STOP_CONFIRM_TIMEOUT_SECONDS),
            Commands.runOnce(() -> {
                latest[0] = monitor[0].poll();
                if (latest[0].confirmed()) {
                    SmartDashboard.putString(dashboardKey, "CONFIRMED");
                    log(name + "_CONFIRMED", latest[0].summary());
                } else {
                    String reason = name + " timeout " + latest[0].summary();
                    runState.abort(reason);
                    SmartDashboard.putString(dashboardKey, "TIMEOUT " + latest[0].summary());
                    log(name + "_TIMEOUT", latest[0].summary());
                }
            }));
    }

    public static String getCoverageManifest() {
        return COVERAGE_MANIFEST;
    }

    public static List<String> getStopResultNames() {
        return STOP_RESULT_NAMES;
    }

    private static String stopResultKey(String name) {
        return "Hardware Self-Test/" + name + "/Stop Result";
    }

    private static CommandSwerveDrivetrain.SwerveDiagnosticEvidence getSwerveEvidence(
            CommandSwerveDrivetrain drivetrain,
            edu.wpi.first.math.kinematics.ChassisSpeeds expectedSpeeds) {
        return getSwerveEvidence(drivetrain, expectedSpeeds, null);
    }

    private static CommandSwerveDrivetrain.SwerveDiagnosticEvidence getSwerveEvidence(
            CommandSwerveDrivetrain drivetrain,
            edu.wpi.first.math.kinematics.ChassisSpeeds expectedSpeeds,
            CommandSwerveDrivetrain.SwerveDiagnosticToken token) {
        return drivetrain.getSwerveDiagnosticEvidence(
            expectedSpeeds,
            HardwareTestConstants.MIN_SWERVE_MODULE_SPEED_METERS_PER_SECOND,
            HardwareTestConstants.MAX_SWERVE_VECTOR_ERROR_DEGREES,
            HardwareTestConstants.MAX_SWERVE_DIAGNOSTIC_DRIVE_CURRENT_AMPS,
            HardwareTestConstants.MAX_SWERVE_DIAGNOSTIC_STEER_CURRENT_AMPS,
            token);
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

    private static void captureTimedTargetStates(
            List<DiagnosticTarget> targets,
            Map<String, TimedTargetState> states) {
        Map<String, Long> epochs = new LinkedHashMap<>();
        Map<String, Integer> canIds = new LinkedHashMap<>();
        for (DiagnosticTarget target : targets) {
            int canId = target.snapshot().apply(false).id();
            var timed = SparkMAXContainer.getTimedDiagnosticSnapshotForId(canId, true).orElse(null);
            if (timed == null) {
                continue;
            }
            canIds.put(target.name(), canId);
            epochs.put(target.name(), timed.currentOutputEpoch());
        }
        double commandCompletedAt = Timer.getFPGATimestamp();
        for (DiagnosticTarget target : targets) {
            Integer canId = canIds.get(target.name());
            Long epoch = epochs.get(target.name());
            if (canId != null && epoch != null) {
                states.put(
                    target.name(),
                    new TimedTargetState(canId, epoch, commandCompletedAt));
            }
        }
    }

    private static Map<String, MotionResult> evaluateFreshTimedTargets(
            List<DiagnosticTarget> targets,
            Map<String, TimedTargetState> states) {
        Map<String, MotionResult> results = new LinkedHashMap<>();
        for (DiagnosticTarget target : targets) {
            TimedTargetState state = states.get(target.name());
            if (state == null) {
                continue;
            }
            var timed = SparkMAXContainer.getTimedDiagnosticSnapshotForId(
                state.canId, true).orElse(null);
            if (!ManualUnhomedActuatorDiagnosticCommand.isCurrentPostCommandSample(
                    timed,
                    state.outputEpoch,
                    state.commandCompletedAtSeconds,
                    state.lastEvaluatedSampleAt)) {
                continue;
            }
            state.lastEvaluatedSampleAt = timed.sampledAtSeconds();
            // Evaluate the exact atomic sample whose epoch/timestamp passed the post-command gate.
            // Re-reading the subsystem cache here could race the worker and classify a different
            // frame than the one validated above.
            Snapshot snapshot = timed.snapshot();
            results.put(
                target.name(),
                HardwareDiagnosticEvaluator.evaluateOpenLoop(
                    snapshot, target.requestedDuty(), target.currentLimitAmps()));
        }
        return results;
    }

    private static boolean timedTargetsStillReady(Map<String, TimedTargetState> states) {
        if (states.isEmpty()) {
            return false;
        }
        for (TimedTargetState state : states.values()) {
            var timed = SparkMAXContainer.getTimedDiagnosticSnapshotForId(
                state.canId, true).orElse(null);
            if (timed == null
                    || timed.currentOutputEpoch() != state.outputEpoch
                    || !timed.snapshot().ready()) {
                return false;
            }
        }
        return true;
    }

    private static Map<String, MotionResult> evaluateCurrentTimedTargets(
            List<DiagnosticTarget> targets,
            Map<String, TimedTargetState> states) {
        Map<String, MotionResult> results = new LinkedHashMap<>();
        for (DiagnosticTarget target : targets) {
            TimedTargetState state = states.get(target.name());
            if (state == null) {
                continue;
            }
            var timed = SparkMAXContainer.getTimedDiagnosticSnapshotForId(
                state.canId, true).orElse(null);
            if (timed == null
                    || !timed.snapshot().ready()
                    || timed.sampleOutputEpoch() != state.outputEpoch
                    || timed.currentOutputEpoch() != state.outputEpoch
                    || !Double.isFinite(timed.sampledAtSeconds())
                    || timed.sampledAtSeconds() <= state.commandCompletedAtSeconds) {
                continue;
            }
            results.put(
                target.name(),
                HardwareDiagnosticEvaluator.evaluateOpenLoop(
                    timed.snapshot(), target.requestedDuty(), target.currentLimitAmps()));
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

    private static void captureCanResults(
            Map<Integer, Boolean> sparkCanResults,
            Map<Integer, CanEvidenceResult> allCanResults,
            CommandSwerveDrivetrain drivetrain) {
        Map<Integer, CanEvidenceResult> captured = evaluateCanEvidence(
            SparkMAXContainer.getDeviceEvidenceSnapshots(),
            drivetrain.getDeviceEvidenceSnapshots());
        sparkCanResults.clear();
        allCanResults.clear();
        allCanResults.putAll(captured);

        for (var device : ConfiguredCanHardware.devices()) {
            CanEvidenceResult result = captured.get(device.canId());
            String vendorPrefix = device.vendor() == ConfiguredCanHardware.Vendor.REV
                ? "SPARK_ID" : "CTRE_ID";
            String topicPrefix = "Hardware Self-Test/" + vendorPrefix + device.canId() + "/";
            SmartDashboard.putString(
                topicPrefix + "CAN Result", result.ready() ? "PASS_READY" : "FAIL_NOT_READY");
            SmartDashboard.putString(topicPrefix + "CAN Reason", result.reason());
            if (device.vendor() == ConfiguredCanHardware.Vendor.REV) {
                sparkCanResults.put(device.canId(), result.ready());
            }
        }
        // Compatibility: this topic intentionally remains the original SPARK-only boolean map.
        SmartDashboard.putString("Hardware Self-Test/CAN Results", sparkCanResults.toString());
        SmartDashboard.putString("Hardware Self-Test/All CAN Results", allCanResults.toString());
        log("SPARK_CAN_RESULT", sparkCanResults.toString());
        log("ALL_CAN_RESULT", allCanResults.toString());
    }

    static Map<Integer, CanEvidenceResult> evaluateCanEvidence(
            List<DeviceEvidenceSnapshot> sparkSnapshots,
            List<CtreDeviceEvidence.Snapshot> ctreSnapshots) {
        List<DeviceEvidenceSnapshot> safeSparkSnapshots = sparkSnapshots == null
            ? List.of() : sparkSnapshots;
        List<CtreDeviceEvidence.Snapshot> safeCtreSnapshots = ctreSnapshots == null
            ? List.of() : ctreSnapshots;
        Map<Integer, CanEvidenceResult> results = new LinkedHashMap<>();

        for (var device : ConfiguredCanHardware.devices()) {
            if (device.vendor() == ConfiguredCanHardware.Vendor.REV) {
                List<DeviceEvidenceSnapshot> matches = safeSparkSnapshots.stream()
                    .filter(snapshot -> snapshot != null && snapshot.canId() == device.canId())
                    .toList();
                if (matches.isEmpty()) {
                    results.put(device.canId(), new CanEvidenceResult(false, "DEVICE_NOT_REGISTERED"));
                } else if (matches.size() != 1) {
                    results.put(
                        device.canId(),
                        new CanEvidenceResult(false, "DUPLICATE_REGISTERED_CAN_ID"));
                } else {
                    DeviceEvidenceSnapshot snapshot = matches.get(0);
                    results.put(
                        device.canId(),
                        observedCanEvidence(snapshot.ready(), snapshot.reason()));
                }
            } else {
                List<CtreDeviceEvidence.Snapshot> matches = safeCtreSnapshots.stream()
                    .filter(snapshot -> snapshot != null && snapshot.canId() == device.canId())
                    .toList();
                if (matches.isEmpty()) {
                    results.put(
                        device.canId(),
                        new CanEvidenceResult(false, "DEVICE_EVIDENCE_NOT_CAPTURED"));
                } else if (matches.size() != 1) {
                    results.put(
                        device.canId(),
                        new CanEvidenceResult(false, "DUPLICATE_DEVICE_EVIDENCE"));
                } else {
                    CtreDeviceEvidence.Snapshot snapshot = matches.get(0);
                    results.put(
                        device.canId(),
                        observedCanEvidence(snapshot.ready(), snapshot.reason()));
                }
            }
        }
        return java.util.Collections.unmodifiableMap(results);
    }

    private static CanEvidenceResult observedCanEvidence(boolean ready, String reason) {
        if (reason == null || reason.isBlank()) {
            return new CanEvidenceResult(false, "INVALID_DEVICE_EVIDENCE_REASON");
        }
        return new CanEvidenceResult(ready, reason);
    }

    static String evidenceSource(boolean simulation) {
        return simulation
            ? "DESKTOP_SIMULATION_RAW_COMMAND_ECHO_NO_MECHANISM_PHYSICS"
            : "LIVE_HARDWARE";
    }

    private static void publishOverall(
            Map<String, MotionResult> results,
            Map<Integer, CanEvidenceResult> allCanResults,
            SelfTestRunState runState) {
        boolean attentionRequired = results.values().stream().anyMatch(result ->
            result == MotionResult.FAIL_NOT_READY
                || result == MotionResult.FAIL_COMMAND_REJECTED
                || result == MotionResult.FAIL_DIRECTION_MISMATCH
                || result == MotionResult.STALL_SUSPECTED
                || result == MotionResult.BLOCKED_KNOWN_FAULT);
        attentionRequired |= allCanResults.size() != ConfiguredCanHardware.devices().size()
            || allCanResults.values().stream().anyMatch(result -> !result.ready());
        attentionRequired |= !runState.mayContinue();
        boolean incomplete = results.values().stream().anyMatch(result ->
            result == MotionResult.INCONCLUSIVE_NO_MOTION
                || result == MotionResult.SKIPPED
                || result == MotionResult.NOT_RUN);
        String overall = attentionRequired
            ? "ATTENTION_REQUIRED"
            : incomplete ? "INCOMPLETE_DESIGN_OR_MOTION_EVIDENCE" : "MOTION_OBSERVED_ONLY";
        SmartDashboard.putString("Hardware Self-Test/Overall", overall);
        SmartDashboard.putString("Hardware Self-Test/Results", results.toString());
        SmartDashboard.putString(
            "Hardware Self-Test/Abort Reason",
            runState.abortReason().isBlank() ? "NONE" : runState.abortReason());
        log(
            "SEQUENCE_COMPLETE",
            "overall=" + overall + " motion=" + results + " allCAN=" + allCanResults);
    }

    private static boolean testOutputsAllowed() {
        return DriverStation.isTestEnabled() && !DriverStation.isFMSAttached();
    }

    private static void log(String stage, String details) {
        var canStatus = RobotController.getCANStatus();
        AsyncDiagnosticSink.log(String.format(
            "SELFTEST stage=%s voltage=%.2fV canUtil=%.1f%% details=[%s]",
            stage,
            RobotController.getBatteryVoltage(),
            canStatus.percentBusUtilization * 100.0,
            details));
    }

    private record DiagnosticTarget(
        String name,
        double requestedDuty,
        double currentLimitAmps,
        Function<Boolean, Snapshot> snapshot) {}

    record CanEvidenceResult(boolean ready, String reason) {
        CanEvidenceResult {
            if (reason == null || reason.isBlank()) {
                throw new IllegalArgumentException("CAN evidence reason must not be blank");
            }
        }

        @Override
        public String toString() {
            return ready ? "PASS_READY(" + reason + ")" : "FAIL_NOT_READY(" + reason + ")";
        }
    }

    private static final class TimedTargetState {
        final int canId;
        final long outputEpoch;
        final double commandCompletedAtSeconds;
        double lastEvaluatedSampleAt = Double.NEGATIVE_INFINITY;

        TimedTargetState(int canId, long outputEpoch, double commandCompletedAtSeconds) {
            this.canId = canId;
            this.outputEpoch = outputEpoch;
            this.commandCompletedAtSeconds = commandCompletedAtSeconds;
        }
    }

    @FunctionalInterface
    private interface StopMonitor {
        StopPoll poll();
    }

    private record StopPoll(boolean confirmed, String summary) {}
}
