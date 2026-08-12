package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DiagnosticOutputSession.PulsePermit;
import frc.robot.constants.Constants.HardwareTestConstants;
import frc.robot.constants.Constants.ManipulatorConstants;
import frc.robot.diagnostics.HardwareDiagnosticEvaluator;
import frc.robot.diagnostics.HardwareDiagnosticEvaluator.Snapshot;
import frc.robot.utils.DashboardApplyGate;
import frc.robot.utils.DashboardApplyGate.Decision;
import frc.robot.utils.OneShotMotorRetestLease;
import frc.robot.utils.OneShotMotorRetestLease.Token;
import frc.robot.utils.SparkMAXContainer;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicLong;

public class FeederSubsystem extends SubsystemBase {
    private static final String MANUAL_RETEST_GUARD_KEY = "Feeder/Manual Retest Guard";
    private static final double MANUAL_RETEST_WATCHDOG_PERIOD_SECONDS = 0.005;
    private static final String TUNING_APPLY_KEY = "Tuning/Feeder/Apply";
    private static final String TUNING_STATUS_KEY = "Tuning/Feeder/Status";
    private static final double MAX_DUTY_CYCLE = 0.20;

    private final SparkMAXContainer m_feeder = new SparkMAXContainer(ManipulatorConstants.FEEDER_CAN_ID);
    private final DashboardApplyGate tuningApplyGate = new DashboardApplyGate();
    private final OneShotMotorRetestLease manualRetestLease = new OneShotMotorRetestLease();
    private final Object manualRetestSafetyLock = new Object();
    private final AtomicLong manualRetestSetpointPermitGeneration = new AtomicLong(-1);
    private final Notifier manualRetestWatchdog = new Notifier(this::serviceManualRetestSafety);

    private double feedPercent = 0.15;
    private double rejectPercent = -0.15;
    private boolean manualRetestOutputActive;
    private boolean manualRetestCommandStarting;
    private boolean manualRetestSafetyStopActive;
    private long manualRetestWatchdogGeneration;
    private long manualRetestOutputEpoch = -1;
    private double manualRetestStartedAt = Double.NEGATIVE_INFINITY;
    private double manualRetestOutputDeadline = Double.NEGATIVE_INFINITY;
    private SparkMAXContainer.OutputStopBatch manualRetestSafetyStopBatch;
    private String manualRetestSafetyStopReason = "STOP_REQUESTED";
    private boolean automaticRetestOutputActive;
    private double automaticRetestExpiresAt = Double.NEGATIVE_INFINITY;

    public FeederSubsystem() {
        m_feeder.setBreakMode(false);
        m_feeder.setCurrentLimit(ManipulatorConstants.FEEDER_CURRENT_LIMIT_AMPS);

        SmartDashboard.putNumber("Set feeder feed percent", 0.15);
        SmartDashboard.putNumber("Set feeder reject percent", -0.15);
        SmartDashboard.putBoolean(TUNING_APPLY_KEY, false);
        SmartDashboard.putString(TUNING_STATUS_KEY, "ACTIVE_DEFAULTS");
        SmartDashboard.putBoolean(
            "Feeder/Known Fault Motion Blocked",
            ManipulatorConstants.FEEDER_MOTION_BLOCKED_KNOWN_STALL);
        SmartDashboard.putBoolean(
            "Feeder/Controlled Retest Enabled",
            ManipulatorConstants.FEEDER_CONTROLLED_RETEST_ENABLED);
        SmartDashboard.putString(MANUAL_RETEST_GUARD_KEY, "DISARMED");
    }

    public boolean feed() {
        if (isMotionBlocked()) {
            m_feeder.stop();
            return false;
        }
        return m_feeder.setDutyCycle(feedPercent);
    }

    public void reject() {
        if (isMotionBlocked()) {
            m_feeder.stop();
            return;
        }
        m_feeder.setDutyCycle(rejectPercent);
    }

    public void stop() {
        manualRetestLease.disarm();
        stopDiagnosticOutput();
    }

    /** Stops diagnostic output while preserving an armed, not-yet-consumed manual lease. */
    public void stopDiagnosticOutput() {
        // Revoke before waiting for either application lock. The SPARK output-lock check then
        // rejects a main-thread setpoint that has not started, or orders this stop after one whose
        // vendor call is already in flight.
        manualRetestSetpointPermitGeneration.set(-1);
        long watchdogGeneration;
        synchronized (manualRetestSafetyLock) {
            watchdogGeneration = ++manualRetestWatchdogGeneration;
            enterManualRetestSafetyStopLocked("EXPLICIT_STOP_REQUESTED");
            long capturedGeneration = watchdogGeneration;
            manualRetestWatchdog.setCallback(
                () -> serviceManualRetestSafety(capturedGeneration));
            manualRetestWatchdog.startPeriodic(MANUAL_RETEST_WATCHDOG_PERIOD_SECONDS);
        }
        try {
            SparkMAXContainer.serviceAll();
        } catch (RuntimeException ignored) {
            // The independent watchdog keeps retrying until cached zero evidence is confirmed.
        }
        automaticRetestOutputActive = false;
        automaticRetestExpiresAt = Double.NEGATIVE_INFINITY;
    }

    /**
     * Issues one opaque, signed authorization after the Disabled-mode operator verification.
     * Normal feed/reject remains blocked regardless of this lease or its eventual result.
     */
    public Optional<Token> armManualControlledRetest(
            double requestedDuty, double sessionExpiresAtSeconds) {
        manualRetestLease.disarm();
        stopDiagnosticOutput();
        double now = Timer.getFPGATimestamp();
        boolean boundedDeadline = Double.isFinite(sessionExpiresAtSeconds)
            && sessionExpiresAtSeconds <= now + HardwareTestConstants.ARM_LIFETIME_SECONDS;
        Optional<Token> token = manualRetestLease.arm(
            requestedDuty,
            HardwareTestConstants.UNHOMED_DIAGNOSTIC_MAX_DUTY_CYCLE,
            now,
            sessionExpiresAtSeconds,
            isMotionBlocked()
                && DriverStation.isDisabled()
                && DriverStation.isTest()
                && !DriverStation.isFMSAttached()
                && boundedDeadline);
        if (token.isPresent()) {
            // A previous explicit stop may already have acquired fresh zero evidence. Retire that
            // stop generation now so the newly armed lease is not timing-dependent on the 5ms
            // callback getting CPU before Test is enabled.
            serviceManualRetestSafety();
        }
        boolean armed = token.isPresent();
        if (armed) {
            SmartDashboard.putString(MANUAL_RETEST_GUARD_KEY, "ARMED_ONE_SHOT");
        } else {
            SmartDashboard.putString(MANUAL_RETEST_GUARD_KEY, "ARM_REJECTED");
        }
        return token;
    }

    /** Consumes the exact lease once and applies at most one 0.35-second, 3% pulse. */
    public boolean runManualControlledRetest(
            Token token, double requestedDuty, PulsePermit permit) {
        // The command's global pre-stop may become confirmed after this subsystem's periodic ran in
        // the same scheduler cycle. Re-evaluate the identical cached ID32 evidence before consuming
        // the one-shot lease so a confirmed neutral is not mistaken for a pending stop.
        serviceManualRetestSafety();
        double now = Timer.getFPGATimestamp();
        boolean allowed = isMotionBlocked()
            && DriverStation.isTestEnabled()
            && !DriverStation.isFMSAttached()
            && permit != null
            && permit.isValidFor(requestedDuty);
        boolean prepared = false;
        long watchdogGeneration = -1;
        synchronized (manualRetestSafetyLock) {
            if (!manualRetestSafetyStopActive
                    && manualRetestLease.consume(
                    token,
                    requestedDuty,
                    now,
                    HardwareTestConstants.UNHOMED_DIAGNOSTIC_PULSE_SECONDS,
                    allowed)) {
                // A concurrent or not-yet-confirmed stop owns the output until its exact zero
                // evidence completes. Never supersede that generation with a new nonzero pulse.
                manualRetestCommandStarting = true;
                manualRetestOutputActive = false;
                manualRetestSafetyStopActive = false;
                manualRetestSafetyStopBatch = null;
                watchdogGeneration = ++manualRetestWatchdogGeneration;
                manualRetestStartedAt = now;
                manualRetestOutputDeadline = manualRetestLease.outputExpiresAtSeconds(token);
                manualRetestSetpointPermitGeneration.set(watchdogGeneration);
                long capturedGeneration = watchdogGeneration;
                manualRetestWatchdog.setCallback(
                    () -> serviceManualRetestSafety(capturedGeneration));
                manualRetestWatchdog.startPeriodic(MANUAL_RETEST_WATCHDOG_PERIOD_SECONDS);
                prepared = true;
            }
        }

        boolean apiAccepted = false;
        SparkMAXContainer.TimedDiagnosticSnapshot timed = null;
        if (prepared) {
            long authorizationGeneration = watchdogGeneration;
            double authorizationDeadline;
            synchronized (manualRetestSafetyLock) {
                authorizationDeadline = manualRetestOutputDeadline;
            }
            try {
                apiAccepted = m_feeder.setDutyCycleIfAuthorized(
                    requestedDuty,
                    () -> {
                        double authorizationNow = Timer.getFPGATimestamp();
                        return manualRetestSetpointPermitGeneration.get()
                                == authorizationGeneration
                            && isMotionBlocked()
                            && DriverStation.isTestEnabled()
                            && !DriverStation.isFMSAttached()
                            && permit.isValidFor(requestedDuty)
                            && manualRetestLease.outputAuthorizationSnapshot(
                                token, requestedDuty, authorizationNow, true)
                            && authorizationNow <= authorizationDeadline;
                    });
                if (apiAccepted) {
                    timed = m_feeder.getTimedDiagnosticSnapshot(true);
                }
            } catch (RuntimeException ignored) {
                apiAccepted = false;
            }
        }

        boolean committed = false;
        if (prepared) {
            double committedAt;
            boolean stillAllowed;
            try {
                committedAt = Timer.getFPGATimestamp();
                stillAllowed = isMotionBlocked()
                    && DriverStation.isTestEnabled()
                    && !DriverStation.isFMSAttached()
                    && permit != null
                    && permit.isValidFor(requestedDuty);
            } catch (RuntimeException ignored) {
                committedAt = Double.POSITIVE_INFINITY;
                stillAllowed = false;
            }
            synchronized (manualRetestSafetyLock) {
                boolean pendingGenerationStillCurrent = watchdogGeneration
                        == manualRetestWatchdogGeneration
                    && manualRetestCommandStarting
                    && !manualRetestSafetyStopActive;
                boolean beforeAbsoluteDeadline = Double.isFinite(committedAt)
                    && Double.isFinite(manualRetestOutputDeadline)
                    && committedAt <= manualRetestOutputDeadline;
                if (apiAccepted
                        && timed != null
                        && pendingGenerationStillCurrent
                        && stillAllowed
                        && beforeAbsoluteDeadline) {
                    manualRetestCommandStarting = false;
                    manualRetestOutputActive = true;
                    manualRetestSafetyStopActive = false;
                    manualRetestOutputEpoch = timed.currentOutputEpoch();
                    committed = true;
                }
            }
        }

        if (!committed) {
            stop();
            SmartDashboard.putString(
                MANUAL_RETEST_GUARD_KEY, "COMMAND_REJECTED_OR_EXPIRED_STOP_REQUESTED");
            return false;
        }
        SmartDashboard.putString(
            MANUAL_RETEST_GUARD_KEY, "PULSE_ACTIVE_WATCHDOG_ARMED");
        return true;
    }

    public boolean isManualControlledRetestSessionValid(Token token, double requestedDuty) {
        return manualRetestLease.sessionValid(
            token,
            requestedDuty,
            Timer.getFPGATimestamp(),
            DriverStation.isTest() && !DriverStation.isFMSAttached());
    }

    public void disarmManualControlledRetest() {
        manualRetestLease.disarm();
        stopDiagnosticOutput();
    }

    /** Low-output Test-mode path used before normal motion is re-enabled after the known stall. */
    public boolean runControlledDiagnostic(double requestedDuty, PulsePermit permit) {
        if (!ManipulatorConstants.FEEDER_CONTROLLED_RETEST_ENABLED
                || !DriverStation.isTestEnabled()
                || DriverStation.isFMSAttached()
                || !Double.isFinite(requestedDuty)
                || Math.abs(requestedDuty) > HardwareTestConstants.OPEN_LOOP_DUTY_CYCLE
                || permit == null
                || !permit.isValidFor(requestedDuty)) {
            stop();
            return false;
        }
        automaticRetestOutputActive = true;
        automaticRetestExpiresAt = Timer.getFPGATimestamp()
            + HardwareTestConstants.OPEN_LOOP_STAGE_SECONDS;
        boolean accepted = m_feeder.setDutyCycleIfAuthorized(
            requestedDuty,
            () -> permit.isValidFor(requestedDuty)
                && DriverStation.isTestEnabled()
                && !DriverStation.isFMSAttached());
        if (!accepted) {
            stop();
        }
        return accepted;
    }

    public Snapshot getDiagnosticSnapshot(boolean commandAccepted) {
        return m_feeder.getDiagnosticSnapshot(commandAccepted);
    }

    public boolean isControlledRetestEnabled() {
        return ManipulatorConstants.FEEDER_CONTROLLED_RETEST_ENABLED;
    }

    public boolean isReady() {
        return !isMotionBlocked() && m_feeder.isReady();
    }

    public boolean isMotionBlockedByKnownStall() {
        return isMotionBlocked();
    }

    public String getDiagnosticStatus() {
        String status = m_feeder.getDiagnosticStatus();
        if (isMotionBlocked()) {
            status += "/MOTION_BLOCKED_KNOWN_STALL";
        }
        return isControlledRetestEnabled() ? status + "/CONTROLLED_RETEST_ENABLED" : status;
    }

    private static boolean isMotionBlocked() {
        return ManipulatorConstants.FEEDER_MOTION_BLOCKED_KNOWN_STALL;
    }

    @Override
    public void periodic() {
        long watchdogGeneration;
        synchronized (manualRetestSafetyLock) {
            watchdogGeneration = manualRetestWatchdogGeneration;
        }
        serviceManualRetestSafety(watchdogGeneration);
        double now = Timer.getFPGATimestamp();
        boolean testOutputsAllowed = DriverStation.isTestEnabled()
            && !DriverStation.isFMSAttached();
        if (automaticRetestOutputActive
                && (!testOutputsAllowed
                    || !Double.isFinite(automaticRetestExpiresAt)
                    || now > automaticRetestExpiresAt)) {
            disarmManualControlledRetest();
        }

        Decision decision = tuningApplyGate.poll(TUNING_APPLY_KEY);
        if (decision == Decision.NONE) {
            return;
        }
        if (decision != Decision.APPLY
                || !DriverStation.isDisabled()
                || DriverStation.isFMSAttached()) {
            publishActiveTuning();
            SmartDashboard.putString(TUNING_STATUS_KEY, "REJECTED_" + decision.name());
            return;
        }

        double requestedFeed = SmartDashboard.getNumber("Set feeder feed percent", feedPercent);
        double requestedReject = SmartDashboard.getNumber(
            "Set feeder reject percent", rejectPercent);
        if (!DashboardApplyGate.allFiniteInRange(
                new double[] {requestedFeed, requestedReject},
                new double[] {0.0, -MAX_DUTY_CYCLE},
                new double[] {MAX_DUTY_CYCLE, 0.0})) {
            publishActiveTuning();
            SmartDashboard.putString(TUNING_STATUS_KEY, "REJECTED_INVALID_OR_WRONG_SIGN");
            return;
        }
        feedPercent = requestedFeed;
        rejectPercent = requestedReject;
        SmartDashboard.putString(TUNING_STATUS_KEY, "QUEUED_DISABLED");
    }

    private void publishActiveTuning() {
        SmartDashboard.putNumber("Set feeder feed percent", feedPercent);
        SmartDashboard.putNumber("Set feeder reject percent", rejectPercent);
    }

    /** Independent safety service; the Notifier keeps enforcing zero if CommandScheduler stalls. */
    private void serviceManualRetestSafety() {
        long watchdogGeneration;
        synchronized (manualRetestSafetyLock) {
            watchdogGeneration = manualRetestWatchdogGeneration;
        }
        serviceManualRetestSafety(watchdogGeneration);
    }

    private void serviceManualRetestSafety(long expectedGeneration) {
        String statusToPublish = null;
        String confirmedStatus = null;
        try {
            double now = Timer.getFPGATimestamp();
            synchronized (manualRetestSafetyLock) {
                if (expectedGeneration != manualRetestWatchdogGeneration) {
                    return;
                }
                if (manualRetestOutputActive) {
                    boolean outputsAllowed = DriverStation.isTestEnabled()
                        && !DriverStation.isFMSAttached();
                    boolean leaseAllowsOutput = manualRetestLease.outputAllowed(now, outputsAllowed);
                    var timed = m_feeder.getTimedDiagnosticSnapshot(true);
                    boolean commandStillCurrent = timed.currentOutputEpoch()
                        == manualRetestOutputEpoch && timed.snapshot().ready();
                    boolean postCommandCurrent = timed.sampleOutputEpoch()
                        == manualRetestOutputEpoch
                        && Double.isFinite(timed.sampledAtSeconds())
                        && timed.sampledAtSeconds() > manualRetestStartedAt;
                    boolean knownStallCurrent = postCommandCurrent
                        && HardwareDiagnosticEvaluator.isCurrentAtOrAboveFraction(
                            timed.snapshot(),
                            ManipulatorConstants.FEEDER_CURRENT_LIMIT_AMPS,
                            0.8);
                    boolean deadlineReached = !Double.isFinite(manualRetestOutputDeadline)
                        || now >= manualRetestOutputDeadline;
                    if (!outputsAllowed || !leaseAllowsOutput || !commandStillCurrent
                        || knownStallCurrent || deadlineReached) {
                        manualRetestLease.disarm();
                        String stopReason = knownStallCurrent
                            ? "CURRENT_CUTOFF_STOP_REQUESTED"
                            : (!outputsAllowed ? "MODE_OR_FMS_LOST_STOP_REQUESTED"
                                : (deadlineReached ? "WATCHDOG_DEADLINE_STOP_REQUESTED"
                                    : "READY_OR_EPOCH_LOST_STOP_REQUESTED"));
                        enterManualRetestSafetyStopLocked(stopReason);
                        statusToPublish = stopReason;
                    }
                } else if (manualRetestCommandStarting) {
                    boolean outputsAllowed = DriverStation.isTestEnabled()
                        && !DriverStation.isFMSAttached();
                    boolean deadlineReached = !Double.isFinite(manualRetestOutputDeadline)
                        || now >= manualRetestOutputDeadline;
                    if (!outputsAllowed || deadlineReached) {
                        manualRetestLease.disarm();
                        String stopReason = !outputsAllowed
                            ? "MODE_OR_FMS_LOST_STOP_REQUESTED"
                            : "WATCHDOG_DEADLINE_STOP_REQUESTED";
                        enterManualRetestSafetyStopLocked(stopReason);
                        statusToPublish = stopReason;
                    }
                }
                if (manualRetestSafetyStopActive) {
                    ensureManualRetestStopBatchLocked();
                    if (manualRetestSafetyStopBatch != null
                            && manualRetestSafetyStopBatch.snapshot().confirmed()) {
                        manualRetestSafetyStopActive = false;
                        manualRetestSafetyStopBatch = null;
                        confirmedStatus = manualRetestSafetyStopReason + "_STOP_CONFIRMED";
                        manualRetestWatchdogGeneration++;
                    }
                }
            }
        } catch (RuntimeException exception) {
            synchronized (manualRetestSafetyLock) {
                if (expectedGeneration != manualRetestWatchdogGeneration) {
                    return;
                }
                manualRetestLease.disarm();
                String stopReason = "WATCHDOG_EXCEPTION_STOP_REQUESTED_"
                    + exception.getClass().getSimpleName();
                enterManualRetestSafetyStopLocked(stopReason);
                statusToPublish = stopReason;
            }
        }
        if (statusToPublish != null) {
            try {
                SmartDashboard.putString(MANUAL_RETEST_GUARD_KEY, statusToPublish);
            } catch (RuntimeException ignored) {
                // Dashboard evidence must never prevent the safety stop below.
            }
        }

        // Sampling stays active while the main loop is unavailable. During a stop this also wakes
        // the ordered-zero worker; the Notifier remains armed until cached output evidence confirms
        // that the exact nonzero epoch is neutral.
        try {
            SparkMAXContainer.serviceAll();
        } catch (RuntimeException ignored) {
            // The next watchdog tick retries without weakening the absolute deadline.
        }

        if (confirmedStatus != null) {
            try {
                SmartDashboard.putString(MANUAL_RETEST_GUARD_KEY, confirmedStatus);
            } catch (RuntimeException ignored) {
                // Stop confirmation remains represented by the cached batch evidence.
            }
        }
    }

    private void enterManualRetestSafetyStopLocked(String reason) {
        manualRetestSetpointPermitGeneration.set(-1);
        manualRetestOutputActive = false;
        manualRetestCommandStarting = false;
        manualRetestSafetyStopActive = true;
        manualRetestOutputEpoch = -1;
        manualRetestStartedAt = Double.NEGATIVE_INFINITY;
        manualRetestOutputDeadline = Double.NEGATIVE_INFINITY;
        manualRetestSafetyStopReason = reason;
        manualRetestSafetyStopBatch = null;
        ensureManualRetestStopBatchLocked();
    }

    private void ensureManualRetestStopBatchLocked() {
        if (manualRetestSafetyStopBatch != null) {
            return;
        }
        try {
            manualRetestSafetyStopBatch = SparkMAXContainer.requestOutputStops(
                ManipulatorConstants.FEEDER_CAN_ID);
        } catch (RuntimeException ignored) {
            try {
                m_feeder.stop();
            } catch (RuntimeException ignoredAgain) {
                // Keep the watchdog armed; a later tick retries creating the evidence batch.
            }
        }
    }

    public void close() {
        stop();
        manualRetestWatchdog.close();
    }
}
