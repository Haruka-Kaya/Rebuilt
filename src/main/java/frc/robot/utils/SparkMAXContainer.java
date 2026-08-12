package frc.robot.utils;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;
import java.util.TreeMap;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj.RobotBase;
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
  // REV's desktop backend reports firmware 0/kOk. Keep real firmware validation unchanged while
  // marking this one simulation-only value so the normal config/zero/fresh-sample gates can run.
  private static final int SIMULATION_FIRMWARE_MARKER = 0x7F260003;
  private static final SparkOutputStopEvaluator.Limits OUTPUT_STOP_LIMITS =
      new SparkOutputStopEvaluator.Limits(
          STATUS_FRESHNESS_SECONDS, 0.01, FOLLOWER_DIAGNOSTIC_STOPPED_RPM);

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
  private final Object simulationIoLock = new Object();
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController closedLoopController;
  private final SparkMaxConfig desiredConfig = new SparkMaxConfig();
  private final SparkRecoveryState recoveryState;
  private final SparkResetGuard resetGuard = new SparkResetGuard();
  private final SparkOutputGate outputGate = new SparkOutputGate();
  private final PositionReferenceGuard positionReferenceGuard = new PositionReferenceGuard();
  private final AtomicLong lastSetpointCallNanos = new AtomicLong();
  private final AtomicLong maximumSetpointCallNanos = new AtomicLong();
  private final List<SparkMAXContainer> requiredFollowers = new ArrayList<>();
  private final int port;
  private final SparkSimulationHandle simulationHandle;

  private long desiredRevision;
  private boolean desiredFollower;
  private boolean sampleInFlight;
  private boolean zeroInFlight;
  private int consecutiveSampleFailures;
  private double nextSampleAt;
  private double lastSampleAt = Double.NEGATIVE_INFINITY;
  private long lastSampleOutputEpoch = -1;
  private double lastZeroConfirmedAt = Double.NEGATIVE_INFINITY;
  private long outputEpoch;
  private long lastZeroedOutputEpoch = -1;
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
    simulationHandle = RobotBase.isSimulation()
        ? new SparkSimulationHandle(motor, simulationIoLock)
        : null;

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
   * Enqueues at most one asynchronous SPARK worker turn. A turn drains a device-count-bounded
   * batch of immediately due zero writes before releasing the worker, while blocking CAN reads and
   * configuration stay off the robot main thread.
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
      if (!IO_WORKER.submitDraining(
          () -> runZeroWork(zeroWork), SparkMAXContainer::runNextZeroWork, DEVICES.size())) {
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
        if (!IO_WORKER.submitDraining(
            () -> runConfigurationWork(work),
            SparkMAXContainer::runNextZeroWork,
            DEVICES.size())) {
          work.device().configurationCancelled();
        }
        return;
      }
    }

    SampleWork sample = selectSampleWork(now);
    if (sample != null) {
      if (!IO_WORKER.submitDraining(
          () -> runSampleWork(sample), SparkMAXContainer::runNextZeroWork, DEVICES.size())) {
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

  /** Drains one due zero on the existing single worker; false means backoff or no pending stop. */
  private static boolean runNextZeroWork() {
    ZeroWork work = selectZeroWork(Timer.getFPGATimestamp());
    if (work == null) {
      return false;
    }
    runZeroWork(work);
    return true;
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
    REVLibError result;
    String zeroException = null;
    try {
      result = device.sendSetpointTracked(0.0, ControlType.kDutyCycle);
      if (result == null) {
        result = REVLibError.kError;
        zeroException = "zero returned null";
      }
    } catch (RuntimeException exception) {
      result = REVLibError.kError;
      zeroException = "zero exception " + exception.getClass().getSimpleName();
    }
    REVLibError resumeResult = null;
    String resumeException = null;
    boolean resumeDeferredForLeader = false;
    if (result == REVLibError.kOk) {
      boolean resumeAfterZero;
      SparkMAXContainer resumeLeader;
      boolean leaderStoppedForResume;
      synchronized (OUTPUT_ORDER_LOCK) {
        synchronized (device.stateLock) {
          resumeAfterZero = device.followerDiagnosticMode
              == FollowerDiagnosticMode.RESUME_ZERO_PENDING;
          resumeLeader = resumeAfterZero && device.desiredFollower
              ? device.followerLeader : null;
        }
        leaderStoppedForResume = !resumeAfterZero
            || Double.isFinite(getLeaderStoppedAtLocked(
                resumeLeader, Timer.getFPGATimestamp()));
      }
      resumeDeferredForLeader = resumeAfterZero && !leaderStoppedForResume;
      if (resumeAfterZero && leaderStoppedForResume) {
        try {
          resumeResult = device.callVendorIo(device.motor::resumeFollowerMode);
          if (resumeResult == null) {
            resumeResult = REVLibError.kError;
            resumeException = "follower resume returned null";
          }
        } catch (RuntimeException exception) {
          resumeResult = REVLibError.kError;
          resumeException = "follower resume exception "
              + exception.getClass().getSimpleName();
        }
      }
    }

    String zeroFailure = result == REVLibError.kOk
        ? null : (zeroException != null ? zeroException : "zero " + result);
    String resumeFailure = resumeResult == null || resumeResult == REVLibError.kOk
        ? null : (resumeException != null ? resumeException : "follower resume " + resumeResult);

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
          if (zeroFailure != null) {
            device.recordFailureLocked(now, zeroFailure, false);
            device.outputGate.zeroFailed(now);
          } else if (resumeFailure != null) {
            device.recordFailureLocked(now, resumeFailure, false);
            device.outputGate.zeroFailed(now);
          }
        } else {
          device.completeZeroLocked(result, resumeResult, resumeDeferredForLeader, now);
        }
        if (zeroFailure != null) {
          leaderToRevoke = device.desiredFollower ? device.followerLeader : null;
          dependentFailure = zeroFailure;
        } else if (resumeFailure != null) {
          leaderToRevoke = device.desiredFollower ? device.followerLeader : null;
          dependentFailure = resumeFailure;
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
    if (zeroFailure != null) {
      device.logState(zeroFailure);
    } else if (resumeFailure != null) {
      device.logState(resumeFailure);
    }
  }

  private void completeZeroLocked(
      REVLibError zeroResult,
      REVLibError resumeResult,
      boolean resumeDeferredForLeader,
      double now) {
    zeroInFlight = false;
    if (zeroResult != REVLibError.kOk) {
      if (recoveryState.isConfigurationReady()) {
        recordFailureLocked(now, "zero " + zeroResult, false);
      }
      // recordFailureLocked requests an immediate protective zero. This attempt itself just failed,
      // so apply the bounded retry schedule after recording the failure.
      outputGate.zeroFailed(now);
      return;
    }

    outputGate.zeroSucceeded();
    lastZeroConfirmedAt = now;
    lastZeroedOutputEpoch = outputEpoch;
    if (followerDiagnosticMode == FollowerDiagnosticMode.RESUME_ZERO_PENDING) {
      if (resumeDeferredForLeader) {
        // Never reconnect a diagnostic follower until fresh telemetry proves its leader is zero.
        // Keep the follower at zero and retry on the bounded worker schedule.
        outputGate.requireZero(now, true);
        outputGate.zeroFailed(now);
      } else if (resumeResult == REVLibError.kOk) {
        followerDiagnosticMode = FollowerDiagnosticMode.RESUME_PENDING;
        followerTransitionDeadline = now + FOLLOWER_TRANSITION_TIMEOUT_SECONDS;
        nextSampleAt = now;
      } else if (resumeResult == null) {
        // The resume request arrived after this zero began; run a new ordered zero+resume item.
        outputGate.requireZero(now, true);
      } else {
        recordFailureLocked(now, "follower resume " + resumeResult, false);
        // The output is physically zero, but follower restoration is still unconfirmed. Reuse the
        // bounded zero retry schedule so a persistent REV error cannot spin zero+resume repeatedly
        // inside one worker drain and starve every other controller.
        outputGate.zeroFailed(now);
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
      return new SampleWork(this, outputEpoch);
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
        PreConfigurationProbe probe = device.readPreConfigurationProbe();
        if (probe.status0() == null) {
          device.configurationFailed("status0 timeout");
          return;
        }
        preConfigurationStatus = probe.status1();
        if (preConfigurationStatus == null) {
          device.configurationFailed("status1 timeout");
          return;
        }
        String preConfigurationFault = fatalFaultSummary(preConfigurationStatus);
        if (preConfigurationFault != null) {
          device.configurationFailed("pre-config " + preConfigurationFault);
          return;
        }

        int version = probe.firmwareVersion();
        REVLibError firmwareError = probe.lastError();
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
          REVLibError clearError = device.callVendorIo(device.motor::clearFaults);
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
      REVLibError configureError = device.callVendorIo(
          () -> device.motor.configure(work.config(), resetMode, persistMode));
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

      FollowerVerification followerVerification = device.readFollowerVerification();
      boolean actualFollower = followerVerification.actualFollower();
      REVLibError followerError = followerVerification.lastError();
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
    REVLibError result = sendSetpointTracked(0.0, ControlType.kDutyCycle);
    double now = Timer.getFPGATimestamp();
    synchronized (OUTPUT_ORDER_LOCK) {
      synchronized (stateLock) {
        if (result == REVLibError.kOk) {
          outputGate.zeroSucceeded();
          lastZeroConfirmedAt = now;
          lastZeroedOutputEpoch = outputEpoch;
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
      SampleStatuses statuses = device.readPeriodicStatuses();
      PeriodicStatus0 status0 = statuses.status0();
      if (status0 == null) {
        device.sampleFailed("periodic status 0 timeout");
        return;
      }
      PeriodicStatus1 status1 = statuses.status1();
      if (status1 == null) {
        device.sampleFailed("periodic status 1 timeout");
        return;
      }
      PeriodicStatus2 status2 = statuses.status2();
      if (device.encoder != null && status2 == null) {
        device.sampleFailed("periodic status 2 timeout");
        return;
      }
      device.sampleSucceeded(status0, status1, status2, work.outputEpoch());
    } catch (Exception exception) {
      device.sampleFailed(exception.getClass().getSimpleName() + ": " + exception.getMessage());
    }
  }

  private SampleStatuses readPeriodicStatuses() {
    if (simulationHandle == null) {
      return readPeriodicStatusesUnlocked();
    }
    synchronized (simulationIoLock) {
      return readPeriodicStatusesUnlocked();
    }
  }

  private SampleStatuses readPeriodicStatusesUnlocked() {
    return new SampleStatuses(
        motor.getPeriodicStatus0(),
        motor.getPeriodicStatus1(),
        encoder == null ? null : motor.getPeriodicStatus2());
  }

  private PreConfigurationProbe readPreConfigurationProbe() {
    return callVendorIo(() -> {
      PeriodicStatus0 status0 = motor.getPeriodicStatus0();
      PeriodicStatus1 status1 = motor.getPeriodicStatus1();
      int reportedFirmwareVersion = motor.getFirmwareVersion();
      REVLibError firmwareError = motor.getLastError();
      int effectiveFirmwareVersion = simulationHandle != null
              && reportedFirmwareVersion == 0
              && firmwareError == REVLibError.kOk
          ? SIMULATION_FIRMWARE_MARKER
          : reportedFirmwareVersion;
      return new PreConfigurationProbe(
          status0, status1, effectiveFirmwareVersion, firmwareError);
    });
  }

  private FollowerVerification readFollowerVerification() {
    return callVendorIo(() -> new FollowerVerification(motor.isFollower(), motor.getLastError()));
  }

  private <T> T callVendorIo(Supplier<T> call) {
    if (simulationHandle == null) {
      return call.get();
    }
    synchronized (simulationIoLock) {
      return call.get();
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
        lastZeroedOutputEpoch = outputEpoch;
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
      PeriodicStatus2 status2,
      long sampledOutputEpoch) {
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
          if (failure == null) {
            failure = SparkTelemetryValidator.failureReason(
                status0.appliedOutput,
                status0.voltage,
                status0.current,
                status0.motorTemperature,
                encoder != null,
                status2 == null ? Double.NaN : status2.primaryEncoderPosition,
                status2 == null ? Double.NaN : status2.primaryEncoderVelocity);
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
          // The general health cache remains fresh even when commands are being refreshed at
          // 50 Hz. A diagnostic that needs command-specific evidence separately compares this
          // captured epoch and therefore cannot mistake the frame for a newer setpoint.
          lastSampleOutputEpoch = sampledOutputEpoch;
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
    lastSampleOutputEpoch = -1;
    nextSampleAt = now;
  }

  private void recordFailureLocked(double now, String error, boolean reset) {
    positionReferenceGuard.invalidate();
    if (reset) {
      recoveryState.resetDetected(now);
    } else {
      recoveryState.operationFailed(now, error);
    }
    // Once pause follower mode has been attempted, its result is not trustworthy after a
    // communication error. Keep an ordered zero+resume pending until a fresh status frame proves
    // that follower mode has been restored. Clearing the mode here could leave a partially
    // successful pause latched in the controller with no recovery request queued.
    followerDiagnosticMode = switch (followerDiagnosticMode) {
      case PAUSE_PENDING, ACTIVE, RESUME_ZERO_PENDING, RESUME_PENDING ->
          FollowerDiagnosticMode.RESUME_ZERO_PENDING;
      default -> FollowerDiagnosticMode.NONE;
    };
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
        lastSampleOutputEpoch = -1;
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

  /** Cached readiness bit for each configured CAN ID, used to require release after recovery. */
  public static long getReadyCanIdMask() {
    long mask = 0L;
    for (SparkMAXContainer device : DEVICES) {
      if (device.port >= 0 && device.port < Long.SIZE - 1 && device.isReady()) {
        mask |= 1L << device.port;
      }
    }
    return mask;
  }

  /**
   * Requests zero from the selected controllers and captures their current nonzero-output epochs.
   * A returned batch can only confirm after fresh telemetry proves those exact epochs stopped.
   */
  public static OutputStopBatch requestOutputStops(int... canIds) {
    Map<Integer, SparkMAXContainer> devicesById = new TreeMap<>();
    for (SparkMAXContainer device : DEVICES) {
      devicesById.put(device.port, device);
    }

    Map<Integer, Boolean> requestedIds = new TreeMap<>();
    if (canIds != null) {
      for (int canId : canIds) {
        requestedIds.put(canId, true);
      }
    }

    for (int canId : requestedIds.keySet()) {
      SparkMAXContainer device = devicesById.get(canId);
      if (device != null) {
        device.stop();
      }
    }

    List<OutputStopTarget> targets = new ArrayList<>();
    synchronized (OUTPUT_ORDER_LOCK) {
      for (int canId : requestedIds.keySet()) {
        SparkMAXContainer device = devicesById.get(canId);
        if (device == null) {
          targets.add(new OutputStopTarget(canId, null, -1));
        } else {
          synchronized (device.stateLock) {
            targets.add(new OutputStopTarget(canId, device, device.outputEpoch));
          }
        }
      }
    }
    return new OutputStopBatch(List.copyOf(targets));
  }

  /** Read-only handle for one atomic group of stop requests. */
  public static final class OutputStopBatch {
    private final List<OutputStopTarget> targets;

    private OutputStopBatch(List<OutputStopTarget> targets) {
      this.targets = targets;
    }

    public OutputStopSnapshot snapshot() {
      Map<Integer, String> pending = new LinkedHashMap<>();
      double now = Timer.getFPGATimestamp();
      if (targets.isEmpty()) {
        pending.put(-1, "NO_TARGETS");
        return new OutputStopSnapshot(false, Map.copyOf(pending));
      }

      // Re-requesting stop is idempotent after a confirmed zero and repairs an unexpected
      // nonzero status observation without creating a new output epoch.
      for (OutputStopTarget target : targets) {
        if (target.device() != null) {
          target.device().stop();
        }
      }

      synchronized (OUTPUT_ORDER_LOCK) {
        for (OutputStopTarget target : targets) {
          SparkMAXContainer device = target.device();
          if (device == null) {
            pending.put(target.canId(), "DEVICE_NOT_REGISTERED");
            continue;
          }
          synchronized (device.stateLock) {
            var observation = new SparkOutputStopEvaluator.Observation(
                device.port,
                target.outputEpoch(),
                device.outputEpoch,
                device.lastZeroedOutputEpoch,
                device.recoveryState.isConfigurationReady(),
                device.zeroInFlight,
                device.outputGate.isZeroRequired(),
                device.outputGate.outputMayBeNonzero(),
                device.lastZeroConfirmedAt,
                device.lastSampleAt,
                now,
                device.cachedAppliedOutput,
                device.encoder != null,
                device.cachedVelocity,
                device.desiredFollower,
                device.followerDiagnosticMode == FollowerDiagnosticMode.NONE,
                device.cachedFollower);
            SparkOutputStopEvaluator.Evaluation evaluation =
                SparkOutputStopEvaluator.evaluate(observation, OUTPUT_STOP_LIMITS);
            if (!evaluation.confirmed()) {
              pending.put(device.port, evaluation.status().name());
            }
          }
        }
      }
      return new OutputStopSnapshot(pending.isEmpty(), Map.copyOf(pending));
    }
  }

  public record OutputStopSnapshot(boolean confirmed, Map<Integer, String> pendingByCanId) {
    public String summary() {
      return confirmed ? "CONFIRMED" : pendingByCanId.toString();
    }
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
      return state + String.format(
          "/SETPOINT_US=%d/%d",
          lastSetpointCallNanos.get() / 1_000L,
          maximumSetpointCallNanos.get() / 1_000L);
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
    return getTimedDiagnosticSnapshot(commandAccepted).snapshot();
  }

  /**
   * Returns telemetry together with the worker sample time and active output epoch.
   *
   * <p>The timestamp lets a diagnostic reject the cached zero frame that existed before a new
   * open-loop command was sent.
   */
  public TimedDiagnosticSnapshot getTimedDiagnosticSnapshot(boolean commandAccepted) {
    double now = Timer.getFPGATimestamp();
    synchronized (OUTPUT_ORDER_LOCK) {
      synchronized (stateLock) {
        boolean ready = followerDiagnosticMode == FollowerDiagnosticMode.NONE
            && isBaseReadyLocked(now)
            && requiredFollowersReadyLocked(now);
        return new TimedDiagnosticSnapshot(
            new Snapshot(
                port,
                ready,
                cachedAppliedOutput,
                cachedCurrent,
                cachedVelocity,
                cachedBusVoltage,
                commandAccepted),
            lastSampleAt,
            lastSampleOutputEpoch,
            outputEpoch);
      }
    }
  }

  /** Atomic cached diagnostic observation; no vendor call is made by this record. */
  public record TimedDiagnosticSnapshot(
      Snapshot snapshot,
      double sampledAtSeconds,
      long sampleOutputEpoch,
      long currentOutputEpoch) {}

  /** Finds a configured SPARK and returns one timestamped cached diagnostic observation. */
  public static Optional<TimedDiagnosticSnapshot> getTimedDiagnosticSnapshotForId(
      int canId, boolean commandAccepted) {
    for (SparkMAXContainer device : DEVICES) {
      if (device.port == canId) {
        return Optional.of(device.getTimedDiagnosticSnapshot(commandAccepted));
      }
    }
    return Optional.empty();
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

  /** Returns the simulation façade for exactly one registered CAN ID, never on the real robot. */
  public static Optional<SparkSimulationHandle> getSimulationHandleForId(int canId) {
    if (!RobotBase.isSimulation()) {
      return Optional.empty();
    }
    SparkSimulationHandle found = null;
    for (SparkMAXContainer device : DEVICES) {
      if (device.port == canId && device.simulationHandle != null) {
        if (found != null) {
          throw new IllegalStateException("duplicate simulated SPARK CAN ID " + canId);
        }
        found = device.simulationHandle;
      }
    }
    return Optional.ofNullable(found);
  }

  /** Test teardown for the process-static simulation registry. Never available on the robot. */
  static boolean cleanupSimulationDevicesForTesting() {
    if (!RobotBase.isSimulation() || !IO_WORKER.isIdle()) {
      return false;
    }
    synchronized (OUTPUT_ORDER_LOCK) {
      if (!IO_WORKER.isIdle()) {
        return false;
      }
      for (SparkMAXContainer device : DEVICES) {
        synchronized (device.stateLock) {
          if (device.sampleInFlight
              || device.zeroInFlight
              || device.recoveryState.isOperationInFlight()) {
            return false;
          }
        }
      }
      for (SparkMAXContainer device : DEVICES) {
        synchronized (device.simulationIoLock) {
          device.motor.close();
        }
      }
      DEVICES.clear();
      sampleCursor = 0;
      zeroCursor = 0;
      disabledSince = Double.NaN;
      FOLLOWER_TOPOLOGY_FROZEN.set(false);
      RECOVERY_COORDINATOR.resetForTesting();
      return true;
    }
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
      requestZeroOutput();
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

  public boolean setVelocity(double velocity) {
    if (!Double.isFinite(velocity)) {
      requestZeroOutput();
      return false;
    }
    if (Math.abs(velocity) <= 1e-9) {
      requestZeroOutput();
      return true;
    }
    return trySetpoint(velocity, ControlType.kVelocity, false, null);
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
    if (!Double.isFinite(value)) {
      requestZeroOutput();
      return false;
    }
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
          SparkVendorCall.Result result = SparkVendorCall.execute(
              "setpoint", () -> sendSetpointTracked(value, controlType));
          if (result.succeeded()) {
            outputGate.nonzeroSucceeded();
            outputEpoch++;
            accepted = true;
          } else {
            failure = result.failure();
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
          // Latch recovery before issuing the asynchronous pause. An error return does not prove
          // that the controller ignored the request, so every attempted pause must have a matching
          // ordered resume path.
          followerDiagnosticMode = FollowerDiagnosticMode.RESUME_ZERO_PENDING;
          SparkVendorCall.Result pauseResult = SparkVendorCall.execute(
              "follower pause", () -> callVendorIo(motor::pauseFollowerModeAsync));
          if (pauseResult.succeeded()) {
            followerDiagnosticMode = FollowerDiagnosticMode.PAUSE_PENDING;
            followerTransitionDeadline = now + FOLLOWER_TRANSITION_TIMEOUT_SECONDS;
            nextSampleAt = now;
            accepted = true;
          } else {
            failure = pauseResult.failure();
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
    AsyncDiagnosticSink.log(String.format(
        "SPARK_HEALTH id=%d state=%s details=[%s]",
        port,
        getHealthSummary(),
        details));
  }

  private static boolean allFinite(double... values) {
    for (double value : values) {
      if (!Double.isFinite(value)) {
        return false;
      }
    }
    return true;
  }

  private REVLibError sendSetpointTracked(double value, ControlType controlType) {
    long startedAt = System.nanoTime();
    try {
      if (simulationHandle == null) {
        return closedLoopController.setSetpoint(value, controlType);
      }
      synchronized (simulationIoLock) {
        return closedLoopController.setSetpoint(value, controlType);
      }
    } finally {
      long elapsed = Math.max(0L, System.nanoTime() - startedAt);
      lastSetpointCallNanos.set(elapsed);
      maximumSetpointCallNanos.accumulateAndGet(elapsed, Math::max);
    }
  }

  private record ConfigurationWork(
      SparkMAXContainer device,
      ServiceAction action,
      SparkMaxConfig config,
      long revision,
      boolean expectedFollower,
      boolean persist) {}

  private record SampleWork(SparkMAXContainer device, long outputEpoch) {}

  private record SampleStatuses(
      PeriodicStatus0 status0, PeriodicStatus1 status1, PeriodicStatus2 status2) {}

  private record PreConfigurationProbe(
      PeriodicStatus0 status0,
      PeriodicStatus1 status1,
      int firmwareVersion,
      REVLibError lastError) {}

  private record FollowerVerification(boolean actualFollower, REVLibError lastError) {}

  private record ZeroWork(SparkMAXContainer device, long generation) {}

  private record OutputStopTarget(
      int canId, SparkMAXContainer device, long outputEpoch) {}
}
