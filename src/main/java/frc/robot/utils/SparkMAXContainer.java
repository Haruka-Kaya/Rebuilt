package frc.robot.utils;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;
import java.util.TreeMap;
import java.util.ArrayList;
import java.util.concurrent.CopyOnWriteArrayList;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.jni.REVLibJNI;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.PeriodicStatus0;
import com.revrobotics.spark.SparkLowLevel.PeriodicStatus1;
import com.revrobotics.spark.SparkLowLevel.PeriodicStatus2;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.diagnostics.HardwareDiagnosticEvaluator.Snapshot;
import frc.robot.utils.SparkRecoveryState.ServiceAction;
import frc.robot.utils.SparkOutputGate.ZeroDecision;
import frc.robot.utils.SparkResetGuard.Observation;

/**
 * Fail-closed SPARK MAX wrapper with asynchronous discovery and configuration recovery.
 *
 * <p>All desired configuration is retained while a device is offline. Nonzero output is accepted
 * only after the latest configuration revision and a fresh atomic status sample are verified.
 */
public class SparkMAXContainer implements MotorContainer {
  private static final int CAN_TIMEOUT_MILLISECONDS = 20;
  private static final double DISABLED_STABLE_SECONDS = 0.25;
  private static final double STATUS_SAMPLE_PERIOD_SECONDS = 0.10;
  private static final double STATUS_FRESHNESS_SECONDS = 0.50;
  private static final double FOLLOWER_TRANSITION_TIMEOUT_SECONDS = 0.50;
  private static final double FOLLOWER_DIAGNOSTIC_STOPPED_RPM = 2.0;
  private static final double MAX_DIAGNOSTIC_DUTY_CYCLE = 0.10;

  private static final List<SparkMAXContainer> DEVICES = new CopyOnWriteArrayList<>();
  private static final Object OUTPUT_ORDER_LOCK = new Object();
  private static final SparkRecoveryCoordinator RECOVERY_COORDINATOR =
      new SparkRecoveryCoordinator();
  private static final java.util.concurrent.atomic.AtomicBoolean PROCESS_DEFAULTS_CONFIGURED =
      new java.util.concurrent.atomic.AtomicBoolean();
  private static final java.util.concurrent.atomic.AtomicBoolean FOLLOWER_TOPOLOGY_FROZEN =
      new java.util.concurrent.atomic.AtomicBoolean();
  private static final SparkAsyncWorker IO_WORKER =
      SparkAsyncWorker.createDaemon("spark-recovery-worker");

  private static int sampleCursor;
  private static int zeroCursor;
  private static double disabledSince = Double.NaN;

  private enum FollowerDiagnosticMode {
    NONE,
    LEADER_ZERO_PENDING,
    PAUSE_PENDING,
    ACTIVE,
    RESUME_ZERO_PENDING,
    RESUME_PENDING
  }

  public enum PositionCommandStatus {
    REJECTED,
    MOVING,
    AT_TARGET
  }

  private final Object stateLock = new Object();
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController closedLoopController;
  private final SparkMaxConfig desiredConfig = new SparkMaxConfig();
  private final SparkRecoveryState recoveryState;
  private final SparkResetGuard resetGuard = new SparkResetGuard();
  private final SparkOutputGate outputGate = new SparkOutputGate();
  private final PositionReferenceGuard positionReferenceGuard = new PositionReferenceGuard();
  private final List<SparkMAXContainer> requiredFollowers = new ArrayList<>();
  private final int port;

  private long desiredRevision;
  private boolean desiredFollower;
  private boolean sampleInFlight;
  private boolean zeroInFlight;
  private int consecutiveSampleFailures;
  private double nextSampleAt;
  private double lastSampleAt = Double.NEGATIVE_INFINITY;
  private double lastZeroConfirmedAt = Double.NEGATIVE_INFINITY;
  private volatile int firmwareVersion;

  private double cachedAppliedOutput;
  private double cachedBusVoltage;
  private double cachedCurrent;
  private double cachedTemperatureCelsius;
  private double cachedPosition;
  private double cachedVelocity;
  private boolean cachedFollower;

  private double desiredP;
  private double desiredI;
  private double desiredD;
  private double desiredSmartCurrentLimit;
  private double desiredSecondaryCurrentLimit;
  private SparkMAXContainer followerLeader;
  private FollowerDiagnosticMode followerDiagnosticMode = FollowerDiagnosticMode.NONE;
  private double followerTransitionDeadline;

  /** Configures REV's process-wide defaults before any {@link SparkMax} is constructed. */
  public static void configureProcessDefaults() {
    if (PROCESS_DEFAULTS_CONFIGURED.compareAndSet(false, true)) {
      REVLibJNI.c_REVLib_SetBaseDefaultCanTimeoutMs(CAN_TIMEOUT_MILLISECONDS);
      REVLibJNI.c_REVLib_SetBaseDefaultCanRetries(0);
    }
  }

  /** Creates a brushless SPARK MAX container. */
  public SparkMAXContainer(int id) {
    this(id, true);
  }

  /** Creates a SPARK MAX container with the requested motor interface. */
  public SparkMAXContainer(int id, boolean isBrushless) {
    port = id;
    motor = new SparkMax(id, isBrushless ? MotorType.kBrushless : MotorType.kBrushed);
    motor.setCANTimeout(CAN_TIMEOUT_MILLISECONDS);
    motor.setCANMaxRetries(0);
    motor.setPeriodicFrameTimeout(0);
    encoder = isBrushless ? motor.getEncoder() : null;
    closedLoopController = motor.getClosedLoopController();

    desiredConfig.disableFollowerMode().idleMode(IdleMode.kCoast).inverted(false);
    desiredConfig.signals
        .faultsPeriodMs(20)
        .warningsPeriodMs(20)
        .faultsAlwaysOn(true)
        .warningsAlwaysOn(true);
    recoveryState = new SparkRecoveryState(Timer.getFPGATimestamp(), desiredRevision);
    DEVICES.add(this);
    logState("registered");
  }

  /**
   * Enqueues at most one asynchronous SPARK operation. This method never performs blocking CAN
   * reads or configuration on the robot main thread.
   */
  public static void serviceAll() {
    double now = Timer.getFPGATimestamp();
    boolean disabled = DriverStation.isDisabled();
    if (disabled) {
      if (!Double.isFinite(disabledSince)) {
        disabledSince = now;
      }
    } else {
      disabledSince = Double.NaN;
    }

    if (!IO_WORKER.isIdle() || DEVICES.isEmpty()) {
      return;
    }

    ZeroWork zeroWork = selectZeroWork(now);
    if (zeroWork != null) {
      if (!IO_WORKER.submit(() -> runZeroWork(zeroWork))) {
        zeroWork.device().zeroCancelled();
      }
      return;
    }

    boolean configurationAllowed = disabled
        && Double.isFinite(disabledSince)
        && now - disabledSince >= DISABLED_STABLE_SECONDS;
    OptionalInt selected = RECOVERY_COORDINATOR.selectConfiguration(
        now,
        DEVICES.size(),
        IO_WORKER.isIdle(),
        configurationAllowed,
        index -> DEVICES.get(index).hasConfigurationWork(now));

    if (selected.isPresent()) {
      ConfigurationWork work = DEVICES.get(selected.getAsInt()).beginConfigurationWork(now);
      if (work != null) {
        if (!IO_WORKER.submit(() -> runConfigurationWork(work))) {
          work.device().configurationCancelled();
        }
        return;
      }
    }

    SampleWork sample = selectSampleWork(now);
    if (sample != null) {
      if (!IO_WORKER.submit(() -> runSampleWork(sample))) {
        sample.device().sampleCancelled();
      }
    }
  }

  private static SampleWork selectSampleWork(double now) {
    int count = DEVICES.size();
    for (int offset = 0; offset < count; offset++) {
      int index = (sampleCursor + offset) % count;
      SampleWork work = DEVICES.get(index).beginSampleWork(now);
      if (work != null) {
        sampleCursor = (index + 1) % count;
        return work;
      }
    }
    return null;
  }

  private static ZeroWork selectZeroWork(double now) {
    int count = DEVICES.size();
    for (int offset = 0; offset < count; offset++) {
      int index = (zeroCursor + offset) % count;
      ZeroWork work = DEVICES.get(index).beginZeroWork(now);
      if (work != null) {
        zeroCursor = (index + 1) % count;
        return work;
      }
    }
    return null;
  }

  private ZeroWork beginZeroWork(double now) {
    synchronized (OUTPUT_ORDER_LOCK) {
      synchronized (stateLock) {
        return reserveZeroWorkLocked(now);
      }
    }
  }

  /** Called only while OUTPUT_ORDER_LOCK and this device's stateLock are held. */
  private ZeroWork reserveZeroWorkLocked(double now) {
    if (zeroInFlight || outputGate.decideZero(now) != ZeroDecision.ATTEMPT) {
      return null;
    }
    zeroInFlight = true;
    return new ZeroWork(this, outputGate.generation());
  }

  private static void runZeroWork(ZeroWork work) {
    SparkMAXContainer device = work.device();
    boolean execute;
    synchronized (OUTPUT_ORDER_LOCK) {
      synchronized (device.stateLock) {
        execute = device.zeroInFlight
            && device.outputGate.isZeroRequired()
            && device.outputGate.generation() == work.generation();
        if (!execute) {
          device.zeroInFlight = false;
        }
      }
    }
    if (!execute) {
      return;
    }

    // Vendor calls run with no application lock held. zeroInFlight blocks all nonzero commands.
    REVLibError result = device.closedLoopController.setSetpoint(0.0, ControlType.kDutyCycle);
    REVLibError resumeResult = null;
    if (result == REVLibError.kOk) {
      boolean resumeAfterZero;
      synchronized (OUTPUT_ORDER_LOCK) {
        synchronized (device.stateLock) {
          resumeAfterZero = device.followerDiagnosticMode
              == FollowerDiagnosticMode.RESUME_ZERO_PENDING;
        }
      }
      if (resumeAfterZero) {
        resumeResult = device.motor.resumeFollowerMode();
      }
    }

    SparkMAXContainer leaderToRevoke = null;
    String dependentFailure = null;
    ZeroWork leaderZeroWork;
    synchronized (OUTPUT_ORDER_LOCK) {
      synchronized (device.stateLock) {
        if (!device.zeroInFlight) {
          return;
        }
        double now = Timer.getFPGATimestamp();
        if (device.outputGate.generation() != work.generation()) {
          // A newer stop/config transition superseded this reservation. Keep the gate closed and
          // retry so an old completion can never certify a newer output epoch.
          device.zeroInFlight = false;
          device.outputGate.requireZero(now, true);
          if (result != REVLibError.kOk) {
            device.recordFailureLocked(now, "zero " + result, false);
          } else if (resumeResult != null && resumeResult != REVLibError.kOk) {
            device.recordFailureLocked(now, "follower resume " + resumeResult, false);
          }
        } else {
          device.completeZeroLocked(result, resumeResult, now);
        }
        if (result != REVLibError.kOk) {
          leaderToRevoke = device.desiredFollower ? device.followerLeader : null;
          dependentFailure = "zero " + result;
        } else if (resumeResult != null && resumeResult != REVLibError.kOk) {
          leaderToRevoke = device.desiredFollower ? device.followerLeader : null;
          dependentFailure = "follower resume " + resumeResult;
        }
      }
      leaderZeroWork = revokeLeaderForDependentLocked(
          leaderToRevoke,
          device.port,
          dependentFailure,
          Timer.getFPGATimestamp(),
          true);
    }
    if (leaderZeroWork != null) {
      runZeroWork(leaderZeroWork);
    }
    if (result != REVLibError.kOk) {
      device.logState("zero " + result);
    } else if (resumeResult != null && resumeResult != REVLibError.kOk) {
      device.logState("follower resume " + resumeResult);
    }
  }

  private void completeZeroLocked(
      REVLibError zeroResult, REVLibError resumeResult, double now) {
    zeroInFlight = false;
    if (zeroResult != REVLibError.kOk) {
      outputGate.zeroFailed(now);
      if (recoveryState.isConfigurationReady()) {
        recordFailureLocked(now, "zero " + zeroResult, false);
      }
      return;
    }

    outputGate.zeroSucceeded();
    lastZeroConfirmedAt = now;
    if (followerDiagnosticMode == FollowerDiagnosticMode.RESUME_ZERO_PENDING) {
      if (resumeResult == REVLibError.kOk) {
        followerDiagnosticMode = FollowerDiagnosticMode.RESUME_PENDING;
        followerTransitionDeadline = now + FOLLOWER_TRANSITION_TIMEOUT_SECONDS;
        nextSampleAt = now;
      } else if (resumeResult == null) {
        // The resume request arrived after this zero began; run a new ordered zero+resume item.
        outputGate.requireZero(now, true);
      } else {
        recordFailureLocked(now, "follower resume " + resumeResult, false);
      }
    }
  }

  private void zeroCancelled() {
    synchronized (OUTPUT_ORDER_LOCK) {
      synchronized (stateLock) {
        zeroInFlight = false;
      }
    }
  }

  private boolean hasConfigurationWork(double now) {
    synchronized (stateLock) {
      return recoveryState.peekServiceAction(now) != ServiceAction.NONE;
    }
  }

  private ConfigurationWork beginConfigurationWork(double now) {
    synchronized (OUTPUT_ORDER_LOCK) {
      synchronized (stateLock) {
        ServiceAction action = recoveryState.beginService(now);
        if (action == ServiceAction.NONE) {
          return null;
        }
        FOLLOWER_TOPOLOGY_FROZEN.set(true);
        SparkMaxConfig snapshot = new SparkMaxConfig().apply(desiredConfig);
        return new ConfigurationWork(
            this,
            action,
            snapshot,
            desiredRevision,
            desiredFollower,
            recoveryState.shouldPersist(action));
      }
    }
  }

  private SampleWork beginSampleWork(double now) {
    synchronized (stateLock) {
      if (!recoveryState.isConfigurationReady()
          || sampleInFlight
          || now < nextSampleAt) {
        return null;
      }
      sampleInFlight = true;
      return new SampleWork(this);
    }
  }

  private static void runConfigurationWork(ConfigurationWork work) {
    SparkMAXContainer device = work.device();
    if (!DriverStation.isDisabled()) {
      device.configurationCancelled();
      return;
    }

    try {
      PeriodicStatus1 preConfigurationStatus = null;
      if (work.action() == ServiceAction.PROBE_AND_APPLY_RESET) {
        if (device.motor.getPeriodicStatus0() == null) {
          device.configurationFailed("status0 timeout");
          return;
        }
        preConfigurationStatus = device.motor.getPeriodicStatus1();
        if (preConfigurationStatus == null) {
          device.configurationFailed("status1 timeout");
          return;
        }

        int version = device.motor.getFirmwareVersion();
        REVLibError firmwareError = device.motor.getLastError();
        if (version == 0 || firmwareError != REVLibError.kOk) {
          device.configurationFailed("firmware " + firmwareError);
          return;
        }
        device.firmwareVersion = version;
        device.logState("pre-config " + statusEvidenceSummary(preConfigurationStatus));

        if (preConfigurationStatus.hasResetStickyWarning) {
          if (!DriverStation.isDisabled()
              || !device.isConfigurationWorkCurrent(work.revision())) {
            device.configurationCancelled();
            return;
          }
          REVLibError clearError = device.motor.clearFaults();
          if (clearError != REVLibError.kOk) {
            device.configurationFailed("clear faults " + clearError);
            return;
          }
        }
      }

      if (!DriverStation.isDisabled() || !device.isConfigurationWorkCurrent(work.revision())) {
        device.configurationCancelled();
        return;
      }

      ResetMode resetMode = work.action() == ServiceAction.PROBE_AND_APPLY_RESET
          ? ResetMode.kResetSafeParameters
          : ResetMode.kNoResetSafeParameters;
      PersistMode persistMode = work.persist()
          ? PersistMode.kPersistParameters
          : PersistMode.kNoPersistParameters;
      if (work.persist()) {
        device.persistenceAttempted();
      }
      REVLibError configureError = device.motor.configure(work.config(), resetMode, persistMode);
      if (configureError != REVLibError.kOk) {
        device.configurationFailed("configure " + configureError);
        return;
      }
      if (work.persist()) {
        device.persistenceSucceeded(work.revision());
      }

      REVLibError zeroError = device.sendPostConfigurationZero();
      if (zeroError != REVLibError.kOk) {
        device.configurationFailed("post-config zero " + zeroError);
        return;
      }

      boolean actualFollower = device.motor.isFollower();
      REVLibError followerError = device.motor.getLastError();
      if (followerError != REVLibError.kOk || actualFollower != work.expectedFollower()) {
        device.configurationFailed(
            "follower verify " + followerError + " actual=" + actualFollower);
        return;
      }

      device.configurationSucceeded(work.revision(), work.action());
    } catch (Exception exception) {
      device.configurationFailed(
          exception.getClass().getSimpleName() + ": " + exception.getMessage());
    }
  }

  private boolean isConfigurationWorkCurrent(long revision) {
    synchronized (stateLock) {
      return recoveryState.isOperationInFlight()
          && recoveryState.getDesiredRevision() == revision;
    }
  }

  private void persistenceAttempted() {
    synchronized (stateLock) {
      recoveryState.persistenceAttempted();
    }
  }

  private void persistenceSucceeded(long revision) {
    synchronized (stateLock) {
      recoveryState.persistenceSucceeded(revision);
    }
  }

  private REVLibError sendPostConfigurationZero() {
    // Configuration is still marked in flight, so nonzero commands remain rejected while this
    // blocking vendor call runs. Do not hold application locks across REV JNI.
    REVLibError result = closedLoopController.setSetpoint(0.0, ControlType.kDutyCycle);
    double now = Timer.getFPGATimestamp();
    synchronized (OUTPUT_ORDER_LOCK) {
      synchronized (stateLock) {
        if (result == REVLibError.kOk) {
          outputGate.zeroSucceeded();
          lastZeroConfirmedAt = now;
        } else {
          outputGate.zeroFailed(now);
        }
        return result;
      }
    }
  }

  private static void runSampleWork(SampleWork work) {
    SparkMAXContainer device = work.device();
    try {
      PeriodicStatus0 status0 = device.motor.getPeriodicStatus0();
      if (status0 == null) {
        device.sampleFailed("periodic status 0 timeout");
        return;
      }
      PeriodicStatus1 status1 = device.motor.getPeriodicStatus1();
      if (status1 == null) {
        device.sampleFailed("periodic status 1 timeout");
        return;
      }
      PeriodicStatus2 status2 = device.encoder == null
          ? null
          : device.motor.getPeriodicStatus2();
      if (device.encoder != null && status2 == null) {
        device.sampleFailed("periodic status 2 timeout");
        return;
      }
      device.sampleSucceeded(status0, status1, status2);
    } catch (Exception exception) {
      device.sampleFailed(exception.getClass().getSimpleName() + ": " + exception.getMessage());
    }
  }

  private void configurationSucceeded(long revision, ServiceAction action) {
    double now = Timer.getFPGATimestamp();
    synchronized (OUTPUT_ORDER_LOCK) {
      synchronized (stateLock) {
        recoveryState.operationSucceeded(revision, now);
        if (action == ServiceAction.PROBE_AND_APPLY_RESET) {
          resetGuard.fullConfigurationCompleted(now);
          positionReferenceGuard.invalidate();
        }
        followerDiagnosticMode = FollowerDiagnosticMode.NONE;
        outputGate.zeroSucceeded();
        lastZeroConfirmedAt = now;
        invalidateSampleLocked(now);
      }
    }
    logState("configured revision=" + revision);
  }

  private void configurationCancelled() {
    synchronized (stateLock) {
      recoveryState.operationCancelled(Timer.getFPGATimestamp());
    }
  }

  private void configurationFailed(String error) {
    double now = Timer.getFPGATimestamp();
    ZeroWork zeroWork;
    ZeroWork leaderZeroWork;
    SparkMAXContainer leader;
    synchronized (OUTPUT_ORDER_LOCK) {
      synchronized (stateLock) {
        recordFailureLocked(now, error, false);
        zeroWork = reserveZeroWorkLocked(now);
        leader = desiredFollower ? followerLeader : null;
      }
      leaderZeroWork = revokeLeaderForDependentLocked(leader, port, error, now, true);
    }
    if (zeroWork != null) {
      runZeroWork(zeroWork);
    }
    if (leaderZeroWork != null) {
      runZeroWork(leaderZeroWork);
    }
    logState(error + " zero=worker");
  }

  private void sampleSucceeded(
      PeriodicStatus0 status0,
      PeriodicStatus1 status1,
      PeriodicStatus2 status2) {
    double now = Timer.getFPGATimestamp();
    String failure = null;
    boolean reset = false;
    boolean waitingForBaseline = false;
    ZeroWork zeroWork = null;
    ZeroWork leaderZeroWork;
    SparkMAXContainer leader = null;
    synchronized (OUTPUT_ORDER_LOCK) {
      synchronized (stateLock) {
        sampleInFlight = false;
        nextSampleAt = now + STATUS_SAMPLE_PERIOD_SECONDS;

        Observation resetObservation = resetGuard.observe(
            now, status1.hasResetWarning, status1.hasResetStickyWarning);
        if (resetObservation == Observation.WAITING_FOR_BASELINE) {
          waitingForBaseline = true;
        } else if (resetObservation == Observation.BASELINE_TIMEOUT) {
          failure = "reset baseline did not clear";
        } else if (resetObservation == Observation.RESET_DETECTED) {
          reset = true;
        } else {
          failure = fatalFaultSummary(status1);
          if (failure == null) {
            failure = validateFollowerStatusLocked(status1.isFollower, now);
          }
        }

        if (reset) {
          recordFailureLocked(now, "controller reset", true);
          zeroWork = reserveZeroWorkLocked(now);
          leader = desiredFollower ? followerLeader : null;
        } else if (failure != null) {
          recordFailureLocked(now, failure, false);
          zeroWork = reserveZeroWorkLocked(now);
          leader = desiredFollower ? followerLeader : null;
        } else if (!waitingForBaseline) {
          consecutiveSampleFailures = 0;
          cachedAppliedOutput = status0.appliedOutput;
          cachedBusVoltage = status0.voltage;
          cachedCurrent = status0.current;
          cachedTemperatureCelsius = status0.motorTemperature;
          cachedFollower = status1.isFollower;
          if (status2 != null) {
            cachedPosition = status2.primaryEncoderPosition;
            cachedVelocity = status2.primaryEncoderVelocity;
          }
          if (Math.abs(status0.appliedOutput) > 1e-9) {
            outputGate.observeNonzero();
          }
          lastSampleAt = now;
        }
      }
      leaderZeroWork = revokeLeaderForDependentLocked(
          leader, port, reset ? "controller reset" : failure, now, true);
    }
    if (zeroWork != null) {
      runZeroWork(zeroWork);
    }
    if (leaderZeroWork != null) {
      runZeroWork(leaderZeroWork);
    }

    if (reset) {
      logState("controller reset detected " + statusEvidenceSummary(status1)
          + " zero=worker");
    } else if (failure != null) {
      logState(failure + " " + statusEvidenceSummary(status1) + " zero=worker");
    }
  }

  private String validateFollowerStatusLocked(boolean actualFollower, double now) {
    if (!desiredFollower) {
      return actualFollower ? "unexpected follower mode" : null;
    }

    if (followerDiagnosticMode == FollowerDiagnosticMode.LEADER_ZERO_PENDING) {
      return actualFollower ? null : "follower mode lost while waiting for leader stop";
    }
    if (followerDiagnosticMode == FollowerDiagnosticMode.PAUSE_PENDING) {
      if (!actualFollower) {
        followerDiagnosticMode = FollowerDiagnosticMode.ACTIVE;
        return null;
      }
      return now > followerTransitionDeadline ? "follower pause timeout" : null;
    }
    if (followerDiagnosticMode == FollowerDiagnosticMode.ACTIVE) {
      return actualFollower ? "follower restarted during diagnostic" : null;
    }
    if (followerDiagnosticMode == FollowerDiagnosticMode.RESUME_ZERO_PENDING) {
      return null;
    }
    if (followerDiagnosticMode == FollowerDiagnosticMode.RESUME_PENDING) {
      if (actualFollower) {
        followerDiagnosticMode = FollowerDiagnosticMode.NONE;
        return null;
      }
      return now > followerTransitionDeadline ? "follower resume timeout" : null;
    }
    return actualFollower ? null : "follower mode lost";
  }

  private static String fatalFaultSummary(PeriodicStatus1 status) {
    if (status.otherFault
        || status.motorTypeFault
        || status.sensorFault
        || status.canFault
        || status.temperatureFault
        || status.drvFault
        || status.escEepromFault
        || status.firmwareFault) {
      return String.format(
          "fault other=%s motor=%s sensor=%s can=%s temp=%s driver=%s eeprom=%s firmware=%s",
          status.otherFault,
          status.motorTypeFault,
          status.sensorFault,
          status.canFault,
          status.temperatureFault,
          status.drvFault,
          status.escEepromFault,
          status.firmwareFault);
    }
    return null;
  }

  private void sampleFailed(String error) {
    boolean revoked = false;
    double now = Timer.getFPGATimestamp();
    ZeroWork zeroWork = null;
    ZeroWork leaderZeroWork;
    SparkMAXContainer leader = null;
    synchronized (OUTPUT_ORDER_LOCK) {
      synchronized (stateLock) {
        sampleInFlight = false;
        nextSampleAt = now + 0.05;
        consecutiveSampleFailures++;
        if (consecutiveSampleFailures >= 3
            || (Double.isFinite(lastSampleAt)
                && now - lastSampleAt > STATUS_FRESHNESS_SECONDS)) {
          recordFailureLocked(now, error, false);
          zeroWork = reserveZeroWorkLocked(now);
          leader = desiredFollower ? followerLeader : null;
          revoked = true;
        }
      }
      leaderZeroWork = revokeLeaderForDependentLocked(leader, port, error, now, true);
    }
    if (zeroWork != null) {
      runZeroWork(zeroWork);
    }
    if (leaderZeroWork != null) {
      runZeroWork(leaderZeroWork);
    }
    if (revoked) {
      logState(error + " zero=worker");
    }
  }

  private void sampleCancelled() {
    synchronized (stateLock) {
      sampleInFlight = false;
      nextSampleAt = Timer.getFPGATimestamp();
    }
  }

  private void invalidateSampleLocked(double now) {
    lastSampleAt = Double.NEGATIVE_INFINITY;
    nextSampleAt = now;
  }

  private void recordFailureLocked(double now, String error, boolean reset) {
    positionReferenceGuard.invalidate();
    if (reset) {
      recoveryState.resetDetected(now);
    } else {
      recoveryState.operationFailed(now, error);
    }
    followerDiagnosticMode = FollowerDiagnosticMode.NONE;
    invalidateSampleLocked(now);
    outputGate.requireZero(now, true);
  }

  /** Called only while OUTPUT_ORDER_LOCK is held and no other device stateLock is held. */
  private static ZeroWork revokeLeaderForDependentLocked(
      SparkMAXContainer leader,
      int followerPort,
      String reason,
      double now,
      boolean attemptZeroNow) {
    if (leader == null) {
      return null;
    }
    synchronized (leader.stateLock) {
      if (leader.recoveryState.isConfigurationReady()
          || leader.outputGate.outputMayBeNonzero()) {
        leader.recordFailureLocked(
            now, "dependent follower " + followerPort + " unavailable: " + reason, false);
      } else {
        leader.outputGate.requireZero(now, true);
      }
      if (attemptZeroNow) {
        return leader.reserveZeroWorkLocked(now);
      }
      return null;
    }
  }

  private static String statusEvidenceSummary(PeriodicStatus1 status) {
    return String.format(
        "reset(active=%s sticky=%s) follower=%s "
            + "fault(active=%s/%s/%s/%s/%s/%s/%s/%s "
            + "sticky=%s/%s/%s/%s/%s/%s/%s/%s) "
            + "warning(activeBrownout=%s activeOvercurrent=%s activeStall=%s "
            + "stickyBrownout=%s stickyOvercurrent=%s stickyStall=%s)",
        status.hasResetWarning,
        status.hasResetStickyWarning,
        status.isFollower,
        status.otherFault,
        status.motorTypeFault,
        status.sensorFault,
        status.canFault,
        status.temperatureFault,
        status.drvFault,
        status.escEepromFault,
        status.firmwareFault,
        status.otherStickyFault,
        status.motorTypeStickyFault,
        status.sensorStickyFault,
        status.canStickyFault,
        status.temperatureStickyFault,
        status.drvStickyFault,
        status.escEepromStickyFault,
        status.firmwareStickyFault,
        status.brownoutWarning,
        status.overcurrentWarning,
        status.stallWarning,
        status.brownoutStickyWarning,
        status.overcurrentStickyWarning,
        status.stallStickyWarning);
  }

  private void desiredConfigChanged() {
    double now = Timer.getFPGATimestamp();
    SparkMAXContainer leader;
    synchronized (OUTPUT_ORDER_LOCK) {
      synchronized (stateLock) {
        desiredRevision++;
        positionReferenceGuard.invalidate();
        recoveryState.desiredRevisionChanged(desiredRevision, now);
        followerDiagnosticMode = FollowerDiagnosticMode.NONE;
        lastSampleAt = Double.NEGATIVE_INFINITY;
        outputGate.requireZero(now, true);
        leader = desiredFollower ? followerLeader : null;
      }
      requestLeaderZeroLocked(leader, now, false);
    }
  }

  /** Returns true only when current desired config and a recent status frame are verified. */
  public boolean isAvailable() {
    return isReady();
  }

  public boolean isReady() {
    double now = Timer.getFPGATimestamp();
    String failure = null;
    SparkMAXContainer leader = null;
    boolean hardwareReady;
    boolean baseReady;
    boolean dependenciesReady;
    synchronized (OUTPUT_ORDER_LOCK) {
      synchronized (stateLock) {
        if (isStatusStaleLocked(now)) {
          failure = "status stale";
          recordFailureLocked(now, failure, false);
          leader = desiredFollower ? followerLeader : null;
        }
        hardwareReady = isBaseReadyLocked(now);
        baseReady = hardwareReady && followerDiagnosticMode == FollowerDiagnosticMode.NONE;
        if (!hardwareReady && outputGate.needsZeroCommand()) {
          outputGate.requireZero(now, false);
        }
      }
      revokeLeaderForDependentLocked(leader, port, failure, now, false);
      dependenciesReady = baseReady && requiredFollowersReadyLocked(now);
      if (baseReady && !dependenciesReady) {
        synchronized (stateLock) {
          if (outputGate.needsZeroCommand()) {
            outputGate.requireZero(now, false);
          }
        }
      }
    }
    if (failure != null) {
      logState(failure + " zero=queued");
    }
    return baseReady && dependenciesReady;
  }

  /** Called only while OUTPUT_ORDER_LOCK is held. */
  private boolean requiredFollowersReadyLocked(double now) {
    for (SparkMAXContainer follower : requiredFollowers) {
      synchronized (follower.stateLock) {
        if (follower.followerLeader != this
            || !follower.desiredFollower
            || follower.followerDiagnosticMode != FollowerDiagnosticMode.NONE
            || !follower.isBaseReadyLocked(now)) {
          return false;
        }
      }
    }
    return true;
  }

  /** Called only while OUTPUT_ORDER_LOCK is held. */
  private static void requestLeaderZeroLocked(
      SparkMAXContainer leader, double now, boolean retryImmediately) {
    if (leader == null) {
      return;
    }
    synchronized (leader.stateLock) {
      if (retryImmediately || leader.outputGate.needsZeroCommand()) {
        leader.outputGate.requireZero(now, retryImmediately);
      }
    }
  }

  public static String getDeviceAvailabilitySummary() {
    Map<Integer, String> summary = new TreeMap<>();
    for (SparkMAXContainer device : DEVICES) {
      summary.put(device.port, device.getHealthSummary());
    }
    return summary.toString();
  }

  private String getHealthSummary() {
    double now = Timer.getFPGATimestamp();
    synchronized (stateLock) {
      String state = recoveryState.getSummary();
      if (recoveryState.isConfigurationReady()
          && Double.isFinite(lastSampleAt)
          && now - lastSampleAt > STATUS_FRESHNESS_SECONDS) {
        state += "/STATUS_STALE";
      }
      if (recoveryState.isConfigurationReady() && !resetGuard.isArmed()) {
        state += "/RESET_BASELINE";
      }
      if (followerDiagnosticMode != FollowerDiagnosticMode.NONE) {
        state += "/DIAGNOSTIC_" + followerDiagnosticMode;
      }
      return state;
    }
  }

  public String getDiagnosticStatus() {
    synchronized (stateLock) {
      return String.format(
          "id=%d state=%s fw=0x%08X follower=%s applied=%.3f current=%.2fA "
              + "velocity=%.1frpm bus=%.2fV",
          port,
          getHealthSummary(),
          firmwareVersion,
          cachedFollower,
          cachedAppliedOutput,
          cachedCurrent,
          cachedVelocity,
          cachedBusVoltage);
    }
  }

  /** Returns one immutable telemetry sample for an external, read-only diagnostic evaluator. */
  public Snapshot getDiagnosticSnapshot(boolean commandAccepted) {
    double now = Timer.getFPGATimestamp();
    synchronized (OUTPUT_ORDER_LOCK) {
      synchronized (stateLock) {
        boolean ready = followerDiagnosticMode == FollowerDiagnosticMode.NONE
            && isBaseReadyLocked(now)
            && requiredFollowersReadyLocked(now);
        return new Snapshot(
            port,
            ready,
            cachedAppliedOutput,
            cachedCurrent,
            cachedVelocity,
            cachedBusVoltage,
            commandAccepted);
      }
    }
  }

  /** Finds a configured SPARK by CAN ID without exposing the mutable controller instance. */
  public static Optional<Snapshot> getDiagnosticSnapshotForId(int canId) {
    for (SparkMAXContainer device : DEVICES) {
      if (device.port == canId) {
        return Optional.of(device.getControllerTelemetrySnapshot());
      }
    }
    return Optional.empty();
  }

  private Snapshot getControllerTelemetrySnapshot() {
    double now = Timer.getFPGATimestamp();
    synchronized (stateLock) {
      boolean ready = recoveryState.isConfigurationReady()
          && Double.isFinite(lastSampleAt)
          && now - lastSampleAt <= STATUS_FRESHNESS_SECONDS;
      return new Snapshot(
          port,
          ready,
          cachedAppliedOutput,
          cachedCurrent,
          cachedVelocity,
          cachedBusVoltage,
          false);
    }
  }

  /** Same snapshot contract while an intentionally paused follower diagnostic is active. */
  public Snapshot getFollowerDiagnosticSnapshot(boolean commandAccepted) {
    double now = Timer.getFPGATimestamp();
    synchronized (OUTPUT_ORDER_LOCK) {
      double leaderStoppedAt = getLeaderStoppedAtLocked(followerLeader, now);
      synchronized (stateLock) {
        boolean diagnosticReady = followerDiagnosticMode == FollowerDiagnosticMode.ACTIVE
            && isBaseReadyLocked(now)
            && Double.isFinite(leaderStoppedAt);
        return new Snapshot(
            port,
            diagnosticReady,
            cachedAppliedOutput,
            cachedCurrent,
            cachedVelocity,
            cachedBusVoltage,
            commandAccepted);
      }
    }
  }

  /** True only while isolated follower output and its stopped-leader dependency remain verified. */
  public boolean isFollowerDiagnosticOutputSafe() {
    double now = Timer.getFPGATimestamp();
    synchronized (OUTPUT_ORDER_LOCK) {
      double leaderStoppedAt = getLeaderStoppedAtLocked(followerLeader, now);
      synchronized (stateLock) {
        return followerDiagnosticMode == FollowerDiagnosticMode.ACTIVE
            && isBaseReadyLocked(now)
            && Double.isFinite(leaderStoppedAt);
      }
    }
  }

  /**
   * Latches the whole isolated-diagnostic transition to a continuously healthy leader. A leader
   * loss before ACTIVE must require a new arm/gesture rather than resuming in the same stage.
   */
  public boolean isFollowerDiagnosticTransitionSafe() {
    double now = Timer.getFPGATimestamp();
    synchronized (OUTPUT_ORDER_LOCK) {
      SparkMAXContainer leader = followerLeader;
      boolean leaderControllerReady = leader != null
          && leader.getControllerTelemetrySnapshot().ready();
      double leaderStoppedAt = getLeaderStoppedAtLocked(leader, now);
      synchronized (stateLock) {
        if (!isBaseReadyLocked(now) || !leaderControllerReady) {
          return false;
        }
        return switch (followerDiagnosticMode) {
          case LEADER_ZERO_PENDING -> true;
          case PAUSE_PENDING, ACTIVE -> Double.isFinite(leaderStoppedAt);
          default -> false;
        };
      }
    }
  }

  /** Records PID configuration even while the device is offline. */
  @Override
  public void assignPIDValues() {
    assignPIDValues(0.1, 0.0, 0.0);
  }

  @Override
  public void assignPIDValues(double p, double i, double d) {
    if (!allFinite(p, i, d)) {
      DriverStation.reportWarning("Rejected non-finite PID for SPARK " + port, false);
      return;
    }
    synchronized (stateLock) {
      desiredP = p;
      desiredI = i;
      desiredD = d;
      desiredConfig.closedLoop.p(p).i(i).d(d);
    }
    desiredConfigChanged();
  }

  public void assignFF(
      double kS,
      double kV,
      double kA,
      double kG,
      double kCos,
      double kCosRatio) {
    if (!allFinite(kS, kV, kA, kG, kCos, kCosRatio)) {
      DriverStation.reportWarning("Rejected non-finite feedforward for SPARK " + port, false);
      return;
    }
    synchronized (stateLock) {
      desiredConfig.closedLoop.feedForward
          .kS(kS).kV(kV).kA(kA).kG(kG).kCos(kCos).kCosRatio(kCosRatio);
    }
    desiredConfigChanged();
  }

  @Override
  public void assignFF(double kS, double kV, double kA, double kG) {
    if (!allFinite(kS, kV, kA, kG)) {
      DriverStation.reportWarning("Rejected non-finite feedforward for SPARK " + port, false);
      return;
    }
    synchronized (stateLock) {
      desiredConfig.closedLoop.feedForward.kS(kS).kV(kV).kA(kA).kG(kG);
    }
    desiredConfigChanged();
  }

  /** Records follower CAN ID regardless of whether either controller is currently online. */
  @Override
  public void setupAsFollowerMotor(MotorContainer leader, boolean invert) {
    if (!(leader instanceof SparkMAXContainer sparkLeader)) {
      throw new IllegalArgumentException(
          "SparkMAXContainer can only follow another SparkMAXContainer");
    }
    if (sparkLeader == this) {
      throw new IllegalArgumentException("A SPARK cannot follow itself");
    }
    synchronized (OUTPUT_ORDER_LOCK) {
      if (FOLLOWER_TOPOLOGY_FROZEN.get()) {
        throw new IllegalStateException("Follower topology is frozen after SPARK service starts");
      }
      synchronized (stateLock) {
        if (!requiredFollowers.isEmpty()) {
          throw new IllegalArgumentException(
              "A SPARK leader with followers cannot become follower " + port);
        }
      }
      synchronized (sparkLeader.stateLock) {
        if (sparkLeader.desiredFollower) {
          throw new IllegalArgumentException(
              "Follower chains are not supported for SPARK " + port);
        }
      }
      for (SparkMAXContainer ancestor = sparkLeader;
          ancestor != null;
          ancestor = ancestor.followerLeader) {
        if (ancestor == this) {
          throw new IllegalArgumentException("Follower cycle involving SPARK " + port);
        }
      }
      SparkMAXContainer oldLeader;
      synchronized (stateLock) {
        oldLeader = followerLeader;
        followerLeader = sparkLeader;
        desiredFollower = true;
        desiredConfig.follow(sparkLeader.port, invert);
      }
      if (oldLeader != null && oldLeader != sparkLeader) {
        synchronized (oldLeader.stateLock) {
          oldLeader.requiredFollowers.remove(this);
        }
      }
      synchronized (sparkLeader.stateLock) {
        if (!sparkLeader.requiredFollowers.contains(this)) {
          sparkLeader.requiredFollowers.add(this);
        }
      }
    }
    desiredConfigChanged();
  }

  public void disableFollowerMode() {
    synchronized (OUTPUT_ORDER_LOCK) {
      if (FOLLOWER_TOPOLOGY_FROZEN.get()) {
        throw new IllegalStateException("Follower topology is frozen after SPARK service starts");
      }
      SparkMAXContainer oldLeader;
      synchronized (stateLock) {
        oldLeader = followerLeader;
        followerLeader = null;
        desiredFollower = false;
        desiredConfig.disableFollowerMode();
      }
      if (oldLeader != null) {
        synchronized (oldLeader.stateLock) {
          oldLeader.requiredFollowers.remove(this);
        }
      }
    }
    desiredConfigChanged();
  }

  @Override
  public void setGearRatio(double gearRatio) {
    if (!Double.isFinite(gearRatio) || gearRatio == 0.0) {
      DriverStation.reportWarning("Rejected invalid gear ratio for SPARK " + port, false);
      return;
    }
    synchronized (stateLock) {
      desiredConfig.encoder.positionConversionFactor(gearRatio);
    }
    desiredConfigChanged();
  }

  @Override
  public void setCurrentLimit(double limit) {
    if (!Double.isFinite(limit)) {
      DriverStation.reportWarning("Rejected non-finite current limit for SPARK " + port, false);
      return;
    }
    double safeLimit = Math.max(1.0, Math.min(100.0, Math.abs(limit)));
    synchronized (stateLock) {
      desiredSmartCurrentLimit = Math.floor(safeLimit);
      desiredSecondaryCurrentLimit = Math.min(100.0, safeLimit + 5.0);
      desiredConfig.smartCurrentLimit((int) desiredSmartCurrentLimit);
      desiredConfig.secondaryCurrentLimit(desiredSecondaryCurrentLimit);
    }
    desiredConfigChanged();
  }

  public void setInverted(boolean value) {
    synchronized (stateLock) {
      desiredConfig.inverted(value);
    }
    desiredConfigChanged();
  }

  public void setSmartCurrentLimit(int limit) {
    int safeLimit = (int) Math.max(1L, Math.min(100L, Math.abs((long) limit)));
    synchronized (stateLock) {
      desiredSmartCurrentLimit = safeLimit;
      desiredConfig.smartCurrentLimit(safeLimit);
    }
    desiredConfigChanged();
  }

  public void setSecondaryCurrentLimit(double limit) {
    if (!Double.isFinite(limit)) {
      DriverStation.reportWarning("Rejected non-finite current limit for SPARK " + port, false);
      return;
    }
    double safeLimit = Math.max(1.0, Math.min(100.0, Math.abs(limit)));
    synchronized (stateLock) {
      desiredSecondaryCurrentLimit = safeLimit;
      desiredConfig.secondaryCurrentLimit(safeLimit);
    }
    desiredConfigChanged();
  }

  @Override
  public void setBreakMode(boolean isBrakeMode) {
    synchronized (stateLock) {
      desiredConfig.idleMode(isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
    }
    desiredConfigChanged();
  }

  public void setMaxSpeed(double speed) {
    if (!Double.isFinite(speed)) {
      DriverStation.reportWarning("Rejected non-finite output range for SPARK " + port, false);
      return;
    }
    double safeSpeed = Math.min(1.0, Math.abs(speed));
    synchronized (stateLock) {
      desiredConfig.closedLoop.outputRange(-safeSpeed, safeSpeed);
    }
    desiredConfigChanged();
  }

  /** Sends open-loop duty cycle only while the latest config and status are ready. */
  public boolean setDutyCycle(double output) {
    return setDutyCycleInternal(output, false);
  }

  private boolean setDutyCycleInternal(double output, boolean diagnosticFollowerOutput) {
    if (!Double.isFinite(output)) {
      return false;
    }
    double clampedOutput = Math.max(-1.0, Math.min(1.0, output));
    if (Math.abs(clampedOutput) <= 1e-9) {
      requestZeroOutput();
      return true;
    }
    return trySetpoint(clampedOutput, ControlType.kDutyCycle, diagnosticFollowerOutput, null);
  }

  @Override
  @Deprecated(forRemoval = false)
  public boolean goToPostion(double position) {
    requestZeroOutput();
    return false;
  }

  @Override
  @Deprecated(forRemoval = false)
  public boolean goToPostion(double position, double deadband) {
    requestZeroOutput();
    return false;
  }

  /**
   * Commands a position only while an opaque reference token from this controller is still valid.
   * The return value means the command was accepted and the cached position is within deadband.
   */
  public boolean goToReferencedPosition(
      double position, double deadband, PositionReferenceGuard.Token reference) {
    return commandReferencedPosition(position, deadband, reference)
        == PositionCommandStatus.AT_TARGET;
  }

  public PositionCommandStatus commandReferencedPosition(
      double position, double deadband, PositionReferenceGuard.Token reference) {
    if (!Double.isFinite(position)
        || !Double.isFinite(deadband)
        || encoder == null
        || reference == null) {
      requestZeroOutput();
      return PositionCommandStatus.REJECTED;
    }
    if (!trySetpoint(position, ControlType.kPosition, false, reference)) {
      return PositionCommandStatus.REJECTED;
    }
    synchronized (stateLock) {
      if (!positionReferenceGuard.isValid(reference)) {
        return PositionCommandStatus.REJECTED;
      }
      return Math.abs(cachedPosition - position) <= Math.abs(deadband)
          ? PositionCommandStatus.AT_TARGET
          : PositionCommandStatus.MOVING;
    }
  }

  public double setVelocity(double velocity) {
    if (!Double.isFinite(velocity)) {
      return 0.0;
    }
    if (!trySetpoint(velocity, ControlType.kVelocity, false, null)) {
      return 0.0;
    }
    return getVelocity();
  }

  /** Stops output even when the device is not READY; nonzero requests are never queued. */
  public void stop() {
    FollowerDiagnosticMode mode;
    synchronized (stateLock) {
      mode = followerDiagnosticMode;
    }
    if (mode != FollowerDiagnosticMode.NONE) {
      endFollowerDiagnostic();
    } else {
      requestZeroOutput();
    }
  }

  private void requestZeroOutput() {
    double now = Timer.getFPGATimestamp();
    SparkMAXContainer leader;
    synchronized (OUTPUT_ORDER_LOCK) {
      synchronized (stateLock) {
        if (outputGate.needsZeroCommand()) {
          outputGate.requireZero(now, false);
        }
        leader = desiredFollower ? followerLeader : null;
      }
      requestLeaderZeroLocked(leader, now, false);
    }
  }

  private boolean trySetpoint(
      double value,
      ControlType controlType,
      boolean diagnosticFollowerOutput,
      PositionReferenceGuard.Token positionReference) {
    double now = Timer.getFPGATimestamp();
    String failure = null;
    SparkMAXContainer leader = null;
    boolean accepted = false;
    synchronized (OUTPUT_ORDER_LOCK) {
      boolean dependenciesReady = diagnosticFollowerOutput || requiredFollowersReadyLocked(now);
      synchronized (stateLock) {
        if (isStatusStaleLocked(now)) {
          failure = "status stale";
          recordFailureLocked(now, failure, false);
          leader = desiredFollower ? followerLeader : null;
        }

        boolean followerOutputAllowed = !desiredFollower
            || (diagnosticFollowerOutput
                && followerDiagnosticMode == FollowerDiagnosticMode.ACTIVE);
        boolean positionReferenceAllowed = controlType != ControlType.kPosition
            || positionReferenceGuard.isValid(positionReference);
        if (failure == null
            && dependenciesReady
            && isBaseReadyLocked(now)
            && followerOutputAllowed
            && positionReferenceAllowed) {
          REVLibError result = closedLoopController.setSetpoint(value, controlType);
          if (result == REVLibError.kOk) {
            outputGate.nonzeroSucceeded();
            accepted = true;
          } else {
            failure = "setpoint " + result;
            recordFailureLocked(now, failure, false);
            leader = desiredFollower ? followerLeader : null;
          }
        } else if (!accepted && outputGate.needsZeroCommand()) {
          outputGate.requireZero(now, false);
        }
      }
      revokeLeaderForDependentLocked(leader, port, failure, now, false);
    }
    if (failure != null) {
      logState(failure + " zero=queued");
    }
    return accepted;
  }

  /** Pauses configured follower mode and applies a diagnostic output capped at 10%. */
  public boolean beginFollowerDiagnostic(double output) {
    double safeOutput = Math.max(
        -MAX_DIAGNOSTIC_DUTY_CYCLE,
        Math.min(MAX_DIAGNOSTIC_DUTY_CYCLE, output));
    double now = Timer.getFPGATimestamp();
    boolean applyOutput = false;
    boolean accepted = false;
    String failure = null;
    SparkMAXContainer leader = null;
    synchronized (OUTPUT_ORDER_LOCK) {
      synchronized (stateLock) {
        if (!desiredFollower) {
          return false;
        }
        leader = followerLeader;
        if (followerDiagnosticMode == FollowerDiagnosticMode.ACTIVE) {
          applyOutput = true;
        } else if (followerDiagnosticMode == FollowerDiagnosticMode.PAUSE_PENDING) {
          accepted = true;
        } else if (followerDiagnosticMode == FollowerDiagnosticMode.LEADER_ZERO_PENDING) {
          accepted = true;
        } else if (followerDiagnosticMode == FollowerDiagnosticMode.NONE
            && isBaseReadyLocked(now)
            && !zeroInFlight) {
          followerDiagnosticMode = FollowerDiagnosticMode.LEADER_ZERO_PENDING;
          accepted = true;
        }
      }

      requestLeaderZeroLocked(leader, now, false);
      double leaderStoppedAt = getLeaderStoppedAtLocked(leader, now);
      synchronized (stateLock) {
        if (applyOutput && !Double.isFinite(leaderStoppedAt)) {
          applyOutput = false;
          accepted = true;
          if (outputGate.needsZeroCommand()) {
            outputGate.requireZero(now, false);
          }
        }
        if (followerDiagnosticMode == FollowerDiagnosticMode.LEADER_ZERO_PENDING
            && followerLeader == leader
            && Double.isFinite(leaderStoppedAt)
            && isBaseReadyLocked(now)
            && cachedFollower
            && lastSampleAt >= leaderStoppedAt
            && Math.abs(cachedAppliedOutput) <= 0.01
            && Math.abs(cachedVelocity) <= FOLLOWER_DIAGNOSTIC_STOPPED_RPM) {
          REVLibError pauseResult = motor.pauseFollowerModeAsync();
          if (pauseResult == REVLibError.kOk) {
            followerDiagnosticMode = FollowerDiagnosticMode.PAUSE_PENDING;
            followerTransitionDeadline = now + FOLLOWER_TRANSITION_TIMEOUT_SECONDS;
            nextSampleAt = now;
            accepted = true;
          } else {
            failure = "follower pause " + pauseResult;
            recordFailureLocked(now, failure, false);
          }
        }
      }
      if (failure != null) {
        revokeLeaderForDependentLocked(leader, port, failure, now, false);
      }
    }
    if (failure != null) {
      logState(failure + " zero=queued");
    }
    return applyOutput ? setDutyCycleInternal(safeOutput, true) : accepted;
  }

  /** Called only while OUTPUT_ORDER_LOCK is held. */
  private static double getLeaderStoppedAtLocked(SparkMAXContainer leader, double now) {
    if (leader == null) {
      return Double.NEGATIVE_INFINITY;
    }
    synchronized (leader.stateLock) {
      boolean stopped = leader.recoveryState.isConfigurationReady()
          && !leader.zeroInFlight
          && !leader.outputGate.needsZeroCommand()
          && Double.isFinite(leader.lastZeroConfirmedAt)
          && Double.isFinite(leader.lastSampleAt)
          && leader.lastSampleAt >= leader.lastZeroConfirmedAt
          && now - leader.lastSampleAt <= STATUS_FRESHNESS_SECONDS
          && Math.abs(leader.cachedAppliedOutput) <= 0.01
          && (leader.encoder == null
              || Math.abs(leader.cachedVelocity) <= FOLLOWER_DIAGNOSTIC_STOPPED_RPM);
      return stopped ? leader.lastZeroConfirmedAt : Double.NEGATIVE_INFINITY;
    }
  }

  /** Queues zero, then restores follower mode on the I/O worker. Idempotent while resuming. */
  public void endFollowerDiagnostic() {
    double now = Timer.getFPGATimestamp();
    SparkMAXContainer leader;
    synchronized (OUTPUT_ORDER_LOCK) {
      synchronized (stateLock) {
        if (followerDiagnosticMode == FollowerDiagnosticMode.NONE
            || followerDiagnosticMode == FollowerDiagnosticMode.RESUME_ZERO_PENDING
            || followerDiagnosticMode == FollowerDiagnosticMode.RESUME_PENDING) {
          return;
        }
        if (followerDiagnosticMode == FollowerDiagnosticMode.LEADER_ZERO_PENDING) {
          followerDiagnosticMode = FollowerDiagnosticMode.NONE;
          return;
        }
        followerDiagnosticMode = FollowerDiagnosticMode.RESUME_ZERO_PENDING;
        outputGate.requireZero(now, true);
        nextSampleAt = now;
        leader = desiredFollower ? followerLeader : null;
      }
      requestLeaderZeroLocked(leader, now, false);
    }
  }

  public boolean isFollowerDiagnosticActive() {
    synchronized (stateLock) {
      return followerDiagnosticMode == FollowerDiagnosticMode.ACTIVE;
    }
  }

  /**
   * Legacy encoder-zero API is intentionally blocked because it produced no continuity token.
   * Reference establishment will be added as worker-mediated homing after sensors are specified.
   */
  @Deprecated(forRemoval = false)
  public boolean setEncoderPosition(double position) {
    requestZeroOutput();
    return false;
  }

  public boolean isPositionReferenceValid(PositionReferenceGuard.Token reference) {
    double now = Timer.getFPGATimestamp();
    synchronized (OUTPUT_ORDER_LOCK) {
      synchronized (stateLock) {
        return positionReferenceGuard.isValid(reference) && isBaseReadyLocked(now);
      }
    }
  }

  public long getPositionContinuityEpoch() {
    return positionReferenceGuard.generation();
  }

  public String getPositionReferenceStatus(PositionReferenceGuard.Token reference) {
    double now = Timer.getFPGATimestamp();
    synchronized (OUTPUT_ORDER_LOCK) {
      synchronized (stateLock) {
        if (reference == null) {
          return "UNREFERENCED";
        }
        if (!positionReferenceGuard.isValid(reference)) {
          return "INVALIDATED";
        }
        return isBaseReadyLocked(now) ? "REFERENCED" : "CONTROLLER_NOT_READY";
      }
    }
  }

  /** Returns no value instead of conflating unavailable status with a real encoder zero. */
  public OptionalDouble getPositionIfReady() {
    if (!isReady()) {
      return OptionalDouble.empty();
    }
    synchronized (stateLock) {
      return OptionalDouble.of(cachedPosition);
    }
  }

  public OptionalDouble getReferencedPosition(PositionReferenceGuard.Token reference) {
    if (!isPositionReferenceValid(reference)) {
      return OptionalDouble.empty();
    }
    synchronized (stateLock) {
      return positionReferenceGuard.isValid(reference)
          ? OptionalDouble.of(cachedPosition)
          : OptionalDouble.empty();
    }
  }

  public double getPosition() {
    if (!isReady()) {
      return 0.0;
    }
    synchronized (stateLock) {
      return cachedPosition;
    }
  }

  public double getVelocity() {
    if (!isReady()) {
      return 0.0;
    }
    synchronized (stateLock) {
      return cachedVelocity;
    }
  }

  public double getMotorTemperatureInC() {
    if (!isReady()) {
      return 0.0;
    }
    synchronized (stateLock) {
      return cachedTemperatureCelsius;
    }
  }

  public double getMotorTemperatureInF() {
    return (getMotorTemperatureInC() * 9.0 / 5.0) + 32.0;
  }

  private boolean isBaseReadyLocked(double now) {
    return recoveryState.isConfigurationReady()
        && Double.isFinite(lastSampleAt)
        && now - lastSampleAt <= STATUS_FRESHNESS_SECONDS
        && !zeroInFlight
        && !outputGate.isZeroRequired();
  }

  private boolean isStatusStaleLocked(double now) {
    return recoveryState.isConfigurationReady()
        && Double.isFinite(lastSampleAt)
        && now - lastSampleAt > STATUS_FRESHNESS_SECONDS;
  }

  @Override
  public void reportMotor(String key) {
    String health = getHealthSummary();
    double position;
    double velocity;
    double current;
    double appliedOutput;
    double voltage;
    double smartLimit;
    double secondaryLimit;
    double temperatureF;
    synchronized (stateLock) {
      position = cachedPosition;
      velocity = cachedVelocity;
      current = cachedCurrent;
      appliedOutput = cachedAppliedOutput;
      voltage = cachedBusVoltage;
      smartLimit = desiredSmartCurrentLimit;
      secondaryLimit = desiredSecondaryCurrentLimit;
      temperatureF = (cachedTemperatureCelsius * 9.0 / 5.0) + 32.0;
    }
    SmartDashboard.putString(key + "/State", health);
    SmartDashboard.putNumber(key + "/Encoder Value", position);
    SmartDashboard.putNumber(key + "/Velocity", velocity);
    SmartDashboard.putNumber(key + "/Current", current);
    SmartDashboard.putNumber(key + "/Applied Output", appliedOutput);
    SmartDashboard.putNumber(key + "/Voltage", voltage);
    SmartDashboard.putNumber(key + "/CurrentLimit/Smart Limit", smartLimit);
    SmartDashboard.putNumber(key + "/CurrentLimit/Secondary Limit", secondaryLimit);
    SmartDashboard.putNumber(key + "/Motor Temp (F)", temperatureF);
  }

  @Override
  public void getPID(String key) {
    synchronized (stateLock) {
      SmartDashboard.putNumber(key + "P", desiredP);
      SmartDashboard.putNumber(key + "I", desiredI);
      SmartDashboard.putNumber(key + "D", desiredD);
    }
  }

  private void logState(String details) {
    System.out.printf(
        "SPARK_HEALTH id=%d state=%s details=[%s]%n",
        port,
        getHealthSummary(),
        details);
  }

  private static boolean allFinite(double... values) {
    for (double value : values) {
      if (!Double.isFinite(value)) {
        return false;
      }
    }
    return true;
  }

  private record ConfigurationWork(
      SparkMAXContainer device,
      ServiceAction action,
      SparkMaxConfig config,
      long revision,
      boolean expectedFollower,
      boolean persist) {}

  private record SampleWork(SparkMAXContainer device) {}

  private record ZeroWork(SparkMAXContainer device, long generation) {}
}
