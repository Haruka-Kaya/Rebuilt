package frc.robot.utils;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Independent robot-loop heartbeat and confirmed global-output stop supervisor.
 *
 * <p>A healthy robot loop must renew the heartbeat. If the loop stalls, throws, or the robot is
 * disabled, authorization is revoked before this supervisor requests neutral outputs. A trip
 * cannot re-arm in the same enabled session; Disabled plus confirmed fresh stop evidence is
 * required before a later enable may authorize output again.
 */
public final class RobotOutputSafetySupervisor implements AutoCloseable {
  public static final String STATUS_KEY = "Runtime/Output Safety";
  public static final double DEFAULT_HEARTBEAT_TIMEOUT_SECONDS = 0.10;
  public static final double DEFAULT_WATCHDOG_PERIOD_SECONDS = 0.005;

  /** Stop evidence owned by RobotContainer and serviced without CommandScheduler. */
  public interface StopSession {
    void service();

    boolean confirmed();

    String summary();
  }

  public enum Phase {
    STARTUP_STOPPING,
    READY_DISABLED,
    ARMED,
    STOPPING,
    TRIPPED,
    CLOSED
  }

  private final Object lock = new Object();
  private final DoubleSupplier clock;
  private final BooleanSupplier enabled;
  private final Supplier<StopSession> stopSessionFactory;
  private final double heartbeatTimeoutSeconds;
  private final Notifier watchdog;

  private Phase phase = Phase.STARTUP_STOPPING;
  private double heartbeatDeadlineSeconds = Double.NEGATIVE_INFINITY;
  private long stopGeneration;
  private long authorizationGeneration;
  private String reason = "STARTUP_STOP_REQUIRED";
  private String stopSummary = "NOT_REQUESTED";
  private String schedulerFaultReason;
  private boolean irreversibleRuntimeFault;
  private StopSession stopSession;
  private boolean stopSessionFactoryInFlight;
  private boolean closed;

  /** Starts the production 5 ms watchdog. */
  public RobotOutputSafetySupervisor(Supplier<StopSession> stopSessionFactory) {
    this(
        Timer::getFPGATimestamp,
        DriverStation::isEnabled,
        stopSessionFactory,
        DEFAULT_HEARTBEAT_TIMEOUT_SECONDS,
        DEFAULT_WATCHDOG_PERIOD_SECONDS,
        true);
  }

  RobotOutputSafetySupervisor(
      DoubleSupplier clock,
      BooleanSupplier enabled,
      Supplier<StopSession> stopSessionFactory,
      double heartbeatTimeoutSeconds,
      double watchdogPeriodSeconds,
      boolean startWatchdog) {
    this.clock = Objects.requireNonNull(clock, "clock");
    this.enabled = Objects.requireNonNull(enabled, "enabled");
    this.stopSessionFactory = Objects.requireNonNull(stopSessionFactory, "stopSessionFactory");
    if (!Double.isFinite(heartbeatTimeoutSeconds) || heartbeatTimeoutSeconds <= 0.0
        || !Double.isFinite(watchdogPeriodSeconds) || watchdogPeriodSeconds <= 0.0) {
      throw new IllegalArgumentException("output safety periods must be finite and positive");
    }
    this.heartbeatTimeoutSeconds = heartbeatTimeoutSeconds;
    authorizationGeneration = ProcessOutputSafety.revoke("STARTUP_STOP_REQUIRED");
    watchdog = new Notifier(this::serviceSafetyNoThrow);
    if (startWatchdog) {
      watchdog.startPeriodic(watchdogPeriodSeconds);
    }
    publishNoThrow();
  }

  /**
   * Renews the main robot-loop heartbeat or arms a fresh enabled session after Disabled stop proof.
   */
  public void heartbeat() {
    final double now;
    final boolean outputsEnabled;
    try {
      now = clock.getAsDouble();
      outputsEnabled = enabled.getAsBoolean();
    } catch (RuntimeException exception) {
      forceTrip("HEARTBEAT_INPUT_EXCEPTION_" + exception.getClass().getSimpleName());
      return;
    }
    if (!Double.isFinite(now)) {
      forceTrip("HEARTBEAT_CLOCK_NONFINITE");
      return;
    }

    boolean startStop = false;
    synchronized (lock) {
      if (closed) {
        return;
      }
      if (!outputsEnabled) {
        if (phase == Phase.TRIPPED && irreversibleRuntimeFault) {
          // A RuntimeSafetyLatch is process-lifetime irreversible. Preserve its fault truth and
          // already-confirmed stop instead of cycling back through a normal Disabled re-arm path.
        } else if (phase != Phase.READY_DISABLED && phase != Phase.STARTUP_STOPPING
            && phase != Phase.STOPPING) {
          revokeLocked("ROBOT_DISABLED");
          phase = Phase.STOPPING;
          startStop = true;
        }
      } else if (phase == Phase.READY_DISABLED) {
        if (irreversibleRuntimeFault) {
          revokeLocked(schedulerFaultReason);
          phase = Phase.STOPPING;
          startStop = true;
        } else {
          heartbeatDeadlineSeconds = now + heartbeatTimeoutSeconds;
          authorizationGeneration = ProcessOutputSafety.snapshot().generation();
          if (ProcessOutputSafety.authorize(authorizationGeneration)) {
            phase = Phase.ARMED;
            reason = "AUTHORIZED_FRESH_ROBOT_LOOP_HEARTBEAT";
            schedulerFaultReason = null;
          } else {
            recordSchedulerFaultLocked("AUTHORIZATION_GENERATION_CHANGED");
            revokeLocked("AUTHORIZATION_GENERATION_CHANGED");
            phase = Phase.STOPPING;
            startStop = true;
          }
        }
      } else if (phase == Phase.ARMED) {
        heartbeatDeadlineSeconds = now + heartbeatTimeoutSeconds;
      }
    }
    if (startStop) {
      requestStopSessionNoThrow();
    }
    publishNoThrow();
  }

  /** Immediately revokes output and starts an independent confirmed stop. */
  public void forceTrip(String tripReason) {
    boolean startStop = false;
    synchronized (lock) {
      if (closed) {
        return;
      }
      recordSchedulerFaultLocked(tripReason);
      if (phase == Phase.STARTUP_STOPPING || phase == Phase.STOPPING) {
        reason = normalize(tripReason);
      } else if (phase == Phase.TRIPPED) {
        reason = normalize(tripReason);
      } else {
        revokeLocked(tripReason);
        phase = Phase.STOPPING;
        startStop = true;
      }
    }
    if (startStop) {
      requestStopSessionNoThrow();
    }
    publishNoThrow();
  }

  /** Returns current immutable supervisor and process authorization evidence. */
  public Snapshot snapshot() {
    synchronized (lock) {
      return new Snapshot(
          phase,
          reason,
          heartbeatDeadlineSeconds,
          stopGeneration,
          stopSession != null,
          stopSummary,
          schedulerFaultReason,
          irreversibleRuntimeFault,
          ProcessOutputSafety.snapshot());
    }
  }

  private void serviceSafetyNoThrow() {
    try {
      serviceSafety();
    } catch (RuntimeException exception) {
      synchronized (lock) {
        if (!closed) {
          String watchdogReason = "WATCHDOG_EXCEPTION_" + exception.getClass().getSimpleName();
          recordSchedulerFaultLocked(watchdogReason);
          revokeLocked(watchdogReason);
          phase = Phase.STOPPING;
          stopSession = null;
        }
      }
      requestStopSessionNoThrow();
      publishNoThrow();
    }
  }

  private void serviceSafety() {
    final double now;
    final boolean outputsEnabled;
    try {
      now = clock.getAsDouble();
      outputsEnabled = enabled.getAsBoolean();
    } catch (RuntimeException exception) {
      forceTrip("WATCHDOG_INPUT_EXCEPTION_" + exception.getClass().getSimpleName());
      return;
    }

    boolean startStop = false;
    synchronized (lock) {
      if (closed) {
        return;
      }
      if (phase == Phase.ARMED
          && (!outputsEnabled || !Double.isFinite(now)
              || !Double.isFinite(heartbeatDeadlineSeconds)
              || now > heartbeatDeadlineSeconds)) {
        String stopReason = !outputsEnabled ? "ROBOT_DISABLED" : "ROBOT_LOOP_HEARTBEAT_EXPIRED";
        if (outputsEnabled) {
          recordSchedulerFaultLocked(stopReason);
        }
        revokeLocked(stopReason);
        phase = Phase.STOPPING;
        startStop = true;
      } else if (phase == Phase.TRIPPED && !outputsEnabled && !irreversibleRuntimeFault) {
        revokeLocked("DISABLED_AFTER_OUTPUT_SAFETY_TRIP");
        phase = Phase.STOPPING;
        startStop = true;
      } else if ((phase == Phase.STARTUP_STOPPING || phase == Phase.STOPPING)
          && stopSession == null) {
        startStop = true;
      }
    }
    if (startStop) {
      requestStopSessionNoThrow();
    }

    StopSession current;
    long currentGeneration;
    synchronized (lock) {
      current = stopSession;
      currentGeneration = stopGeneration;
    }
    if (current == null) {
      publishNoThrow();
      return;
    }

    boolean confirmed = false;
    String stopSummary = "STOP_EVIDENCE_PENDING";
    try {
      current.service();
      confirmed = current.confirmed();
      stopSummary = normalize(current.summary());
    } catch (RuntimeException exception) {
      stopSummary = "STOP_SESSION_EXCEPTION_" + exception.getClass().getSimpleName();
    }

    synchronized (lock) {
      if (closed || currentGeneration != stopGeneration || current != stopSession) {
        return;
      }
      if (confirmed) {
        stopSession = null;
        heartbeatDeadlineSeconds = Double.NEGATIVE_INFINITY;
        if (irreversibleRuntimeFault) {
          phase = Phase.TRIPPED;
          reason = schedulerFaultReason + "_STOP_CONFIRMED_REBOOT_REQUIRED";
        } else if (outputsEnabled) {
          phase = Phase.TRIPPED;
          reason = reason + "_STOP_CONFIRMED_DISABLE_REQUIRED";
        } else {
          phase = Phase.READY_DISABLED;
          reason = "DISABLED_GLOBAL_STOP_CONFIRMED";
        }
      }
      this.stopSummary = stopSummary;
    }
    publishNoThrow();
  }

  private void requestStopSessionNoThrow() {
    synchronized (lock) {
      if (closed
          || (phase != Phase.STARTUP_STOPPING && phase != Phase.STOPPING)
          || stopSession != null
          || stopSessionFactoryInFlight) {
        return;
      }
      stopSessionFactoryInFlight = true;
    }
    StopSession created;
    try {
      created = Objects.requireNonNull(stopSessionFactory.get(), "stop session");
    } catch (RuntimeException exception) {
      synchronized (lock) {
        stopSessionFactoryInFlight = false;
        if (!closed) {
          stopSummary = "STOP_REQUEST_EXCEPTION_" + exception.getClass().getSimpleName();
        }
      }
      return;
    }
    synchronized (lock) {
      stopSessionFactoryInFlight = false;
      if (closed || (phase != Phase.STARTUP_STOPPING && phase != Phase.STOPPING)) {
        return;
      }
      if (stopSession == null) {
        stopSession = created;
        stopGeneration++;
      }
    }
  }

  private void revokeLocked(String revokeReason) {
    reason = normalize(revokeReason);
    heartbeatDeadlineSeconds = Double.NEGATIVE_INFINITY;
    authorizationGeneration = ProcessOutputSafety.revoke(reason);
    stopSession = null;
    stopSummary = "STOP_REQUEST_PENDING";
    stopGeneration++;
  }

  private void publishNoThrow() {
    Snapshot current = snapshot();
    try {
      SmartDashboard.putString(
          STATUS_KEY,
          current.phase() + " " + current.reason() + " process="
              + current.processOutputSafety().reason() + " stop=" + current.stopSummary());
      if (current.schedulerFaultReason() != null) {
        SmartDashboard.putBoolean("Runtime/Scheduler Healthy", false);
        SmartDashboard.putString("Runtime/Fault", current.schedulerFaultReason());
      } else if (current.phase() != Phase.CLOSED) {
        SmartDashboard.putBoolean("Runtime/Scheduler Healthy", true);
        SmartDashboard.putString("Runtime/Fault", "HEALTHY");
      }
    } catch (RuntimeException ignored) {
      // Output authorization and stop evidence remain authoritative if NT is unhealthy.
    }
  }

  static String normalize(String value) {
    if (value == null || value.isBlank()) {
      return "OUTPUT_SAFETY_STOP_REQUESTED";
    }
    return value.trim().replaceAll("\\s+", " ");
  }

  private void recordSchedulerFaultLocked(String value) {
    String normalized = normalize(value);
    if (normalized.contains("RUNTIME_FAULT")) {
      if (!irreversibleRuntimeFault) {
        schedulerFaultReason = normalized;
      }
      irreversibleRuntimeFault = true;
    } else if (!irreversibleRuntimeFault
        && (normalized.contains("HEARTBEAT_EXPIRED")
            || normalized.contains("HEARTBEAT_INPUT_EXCEPTION")
            || normalized.contains("HEARTBEAT_CLOCK_NONFINITE")
            || normalized.contains("WATCHDOG_")
            || normalized.contains("AUTHORIZATION_GENERATION_CHANGED"))) {
      schedulerFaultReason = normalized;
    }
  }

  void serviceOnceForTesting() {
    serviceSafetyNoThrow();
  }

  @Override
  public void close() {
    forceTrip("SUPERVISOR_CLOSED");
    synchronized (lock) {
      closed = true;
      phase = Phase.CLOSED;
      reason = "SUPERVISOR_CLOSED";
      ProcessOutputSafety.revoke(reason);
    }
    watchdog.close();
    publishNoThrow();
  }

  /** Immutable status for Dashboard and tests. */
  public record Snapshot(
      Phase phase,
      String reason,
      double heartbeatDeadlineSeconds,
      long stopGeneration,
      boolean stopSessionActive,
      String stopSummary,
      String schedulerFaultReason,
      boolean irreversibleRuntimeFault,
      ProcessOutputSafety.Snapshot processOutputSafety) {}
}
