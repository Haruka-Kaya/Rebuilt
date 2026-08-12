package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.constants.Constants.HardwareTestConstants;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

/**
 * Opaque authority for one bounded diagnostic-output session.
 *
 * <p>Only code in {@code frc.robot} can create a session. Subsystems receive only a pulse permit,
 * whose expected duty, absolute deadline, live interlock, and generation are checked again inside
 * the ordered SPARK output lock immediately before the vendor call.
 */
public final class DiagnosticOutputSession implements AutoCloseable {
  /** Longest individual pulse used by the isolated follower hardware test. */
  static final double MAX_PULSE_DURATION_SECONDS = 1.50;
  /** Bounded evaluation grace after the command's nominal low-output stage duration. */
  public static final double EVIDENCE_GRACE_SECONDS = 0.20;
  static final double WATCHDOG_PERIOD_SECONDS = 0.005;

  private final double absoluteSessionExpiresAtSeconds;
  private final double sessionCreatedAtSeconds;
  private final BooleanSupplier sessionInterlock;
  private final Consumer<String> expiryStopAction;
  private final DoubleSupplier clock;
  private final AtomicLong generation = new AtomicLong();
  private final AtomicBoolean revoked = new AtomicBoolean();
  private final Object alarmLock = new Object();
  private final Notifier expiryNotifier;
  private double alarmDeadlineSeconds;
  private double activePulseStartedAtSeconds = Double.NaN;
  private boolean pulseActive;

  /** Package-owned factory used by the robot composition root after it consumes an arm. */
  static DiagnosticOutputSession create(
      double absoluteSessionExpiresAtSeconds,
      BooleanSupplier sessionInterlock,
      Consumer<String> expiryStopAction) {
    return new DiagnosticOutputSession(
        absoluteSessionExpiresAtSeconds, sessionInterlock, expiryStopAction);
  }

  DiagnosticOutputSession(
      double absoluteSessionExpiresAtSeconds,
      BooleanSupplier sessionInterlock,
      Consumer<String> expiryStopAction) {
    this(
        absoluteSessionExpiresAtSeconds,
        sessionInterlock,
        expiryStopAction,
        Timer::getFPGATimestamp);
  }

  DiagnosticOutputSession(
      double absoluteSessionExpiresAtSeconds,
      BooleanSupplier sessionInterlock,
      Consumer<String> expiryStopAction,
      DoubleSupplier clock) {
    if (sessionInterlock == null || expiryStopAction == null || clock == null) {
      throw new IllegalArgumentException(
          "diagnostic session interlock, expiry stop, and clock are required");
    }
    double now = safeTime(clock);
    if (!Double.isFinite(now)
        || !Double.isFinite(absoluteSessionExpiresAtSeconds)
        || absoluteSessionExpiresAtSeconds <= now
        || absoluteSessionExpiresAtSeconds
            > now + HardwareTestConstants.HARDWARE_SELF_TEST_SESSION_LIFETIME_SECONDS) {
      throw new IllegalArgumentException("diagnostic session deadline is invalid or unbounded");
    }
    this.absoluteSessionExpiresAtSeconds = absoluteSessionExpiresAtSeconds;
    this.sessionCreatedAtSeconds = now;
    this.sessionInterlock = sessionInterlock;
    this.expiryStopAction = expiryStopAction;
    this.clock = clock;
    expiryNotifier = new Notifier(this::serviceExpiryAlarm);
    expiryNotifier.setName("diagnostic-output-session-expiry");
    scheduleAlarm(absoluteSessionExpiresAtSeconds);
    expiryNotifier.startPeriodic(WATCHDOG_PERIOD_SECONDS);
  }

  /**
   * Starts one pulse and invalidates every permit previously issued by this session.
   *
   * <p>An empty result is fail-closed: the request was invalid, the live interlock was false, or
   * either the pulse or the parent session had already expired.
   */
  public Optional<PulsePermit> beginPulse(
      double expectedDuty, double maximumDurationSeconds) {
    double now = now();
    boolean validRequest = Double.isFinite(expectedDuty)
        && Math.abs(expectedDuty) > 1e-9
        && Math.abs(expectedDuty) <= 1.0
        && Double.isFinite(maximumDurationSeconds)
        && maximumDurationSeconds > 0.0
        && maximumDurationSeconds <= MAX_PULSE_DURATION_SECONDS;
    if (!validRequest) {
      return Optional.empty();
    }
    double pulseExpiresAt = Math.min(
        absoluteSessionExpiresAtSeconds, now + maximumDurationSeconds);
    if (!Double.isFinite(pulseExpiresAt) || pulseExpiresAt <= now) {
      return Optional.empty();
    }
    boolean overlapping;
    long pulseGeneration;
    synchronized (alarmLock) {
      overlapping = pulseActive;
      if (overlapping || !sessionAllowsOutput(now)) {
        pulseGeneration = -1L;
      } else {
        pulseGeneration = generation.incrementAndGet();
        pulseActive = true;
        activePulseStartedAtSeconds = now;
        alarmDeadlineSeconds = pulseExpiresAt;
      }
    }
    if (overlapping) {
      revokeAndTrip("DIAGNOSTIC_OVERLAPPING_PULSE_REJECTED");
      return Optional.empty();
    }
    if (pulseGeneration < 0L) {
      return Optional.empty();
    }
    return Optional.of(new PulsePermit(
        this, pulseGeneration, expectedDuty, now, pulseExpiresAt));
  }

  /** Returns whether the session deadline and live interlock still allow a new pulse. */
  public boolean isValid() {
    return sessionAllowsOutput(now());
  }

  /** Permanently revokes this session and every permit created from it. */
  public void revoke() {
    revoked.set(true);
    generation.incrementAndGet();
    synchronized (alarmLock) {
      pulseActive = false;
      activePulseStartedAtSeconds = Double.NaN;
    }
    expiryNotifier.stop();
  }

  /** Revokes this session and escalates an unconfirmed diagnostic stop to process safety. */
  public void trip(String reason) {
    String normalized = reason == null || reason.isBlank()
        ? "DIAGNOSTIC_OUTPUT_SESSION_TRIPPED"
        : reason.trim();
    revokeAndTrip(normalized);
  }

  @Override
  public void close() {
    revoke();
    expiryNotifier.close();
  }

  private boolean permitAllowsOutput(
      long expectedGeneration,
      double expectedDuty,
      double requestedDuty,
      double pulseStartedAt,
      double pulseExpiresAt) {
    if (generation.get() != expectedGeneration
        || Double.compare(expectedDuty, requestedDuty) != 0) {
      return false;
    }
    double now = now();
    return sessionAllowsOutput(now)
        && now >= pulseStartedAt
        && Double.isFinite(pulseExpiresAt)
        && now < pulseExpiresAt;
  }

  private boolean sessionAllowsOutput(double now) {
    if (revoked.get()
        || !Double.isFinite(now)
        || now < sessionCreatedAtSeconds
        || now >= absoluteSessionExpiresAtSeconds) {
      return false;
    }
    try {
      return sessionInterlock.getAsBoolean();
    } catch (RuntimeException ignored) {
      return false;
    }
  }

  private void revokePulse(long expectedGeneration) {
    if (!generation.compareAndSet(expectedGeneration, expectedGeneration + 1)) {
      return;
    }
    synchronized (alarmLock) {
      pulseActive = false;
      activePulseStartedAtSeconds = Double.NaN;
    }
    if (!revoked.get()) {
      scheduleAlarm(absoluteSessionExpiresAtSeconds);
    }
  }

  private void scheduleAlarm(double deadlineSeconds) {
    synchronized (alarmLock) {
      alarmDeadlineSeconds = deadlineSeconds;
    }
  }

  private void serviceExpiryAlarm() {
    if (revoked.get()) {
      return;
    }
    double now = now();
    double deadline;
    double pulseStartedAt;
    boolean active;
    synchronized (alarmLock) {
      deadline = alarmDeadlineSeconds;
      pulseStartedAt = activePulseStartedAtSeconds;
      active = pulseActive;
    }
    boolean invalidClock = !Double.isFinite(now)
        || now < sessionCreatedAtSeconds
        || (active && (!Double.isFinite(pulseStartedAt) || now < pulseStartedAt));
    boolean interlockHeld = false;
    if (!invalidClock && Double.isFinite(deadline) && now < deadline) {
      try {
        interlockHeld = sessionInterlock.getAsBoolean();
      } catch (RuntimeException ignored) {
        interlockHeld = false;
      }
    }
    if (!invalidClock && Double.isFinite(deadline) && now < deadline && interlockHeld) {
      return;
    }
    String reason;
    if (!invalidClock && Double.isFinite(deadline) && now < deadline) {
      reason = "DIAGNOSTIC_SESSION_INTERLOCK_LOST";
    } else {
      reason = active
          ? "DIAGNOSTIC_PULSE_DEADLINE_EXPIRED"
          : "DIAGNOSTIC_SESSION_DEADLINE_EXPIRED";
    }
    revokeAndTrip(reason);
  }

  private void revokeAndTrip(String reason) {
    if (!revoked.compareAndSet(false, true)) {
      return;
    }
    generation.incrementAndGet();
    synchronized (alarmLock) {
      pulseActive = false;
      activePulseStartedAtSeconds = Double.NaN;
    }
    try {
      expiryStopAction.accept(reason);
    } catch (RuntimeException ignored) {
      // The process-wide authorization supplier still sees this session as revoked.
    }
  }

  private double now() {
    return safeTime(clock);
  }

  private static double safeTime(DoubleSupplier clock) {
    try {
      return clock.getAsDouble();
    } catch (RuntimeException ignored) {
      return Double.NaN;
    }
  }

  /** Unforgeable, signed authority for one exact-duty pulse. */
  public static final class PulsePermit implements AutoCloseable {
    private final DiagnosticOutputSession owner;
    private final long generation;
    private final double expectedDuty;
    private final double pulseStartedAtSeconds;
    private final double absolutePulseExpiresAtSeconds;
    private final AtomicBoolean revoked = new AtomicBoolean();

    private PulsePermit(
        DiagnosticOutputSession owner,
        long generation,
        double expectedDuty,
        double pulseStartedAtSeconds,
        double absolutePulseExpiresAtSeconds) {
      this.owner = owner;
      this.generation = generation;
      this.expectedDuty = expectedDuty;
      this.pulseStartedAtSeconds = pulseStartedAtSeconds;
      this.absolutePulseExpiresAtSeconds = absolutePulseExpiresAtSeconds;
    }

    /** Side-effect-free authorization check intended for a motor vendor-call supplier. */
    public boolean isValidFor(double requestedDuty) {
      return !revoked.get()
          && owner.permitAllowsOutput(
              generation,
              expectedDuty,
              requestedDuty,
              pulseStartedAtSeconds,
              absolutePulseExpiresAtSeconds);
    }

    /** Revokes this exact generation without making the parent session reusable by this permit. */
    public void revoke() {
      if (revoked.compareAndSet(false, true)) {
        owner.revokePulse(generation);
      }
    }

    @Override
    public void close() {
      revoke();
    }
  }
}
