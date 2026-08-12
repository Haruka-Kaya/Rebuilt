package frc.robot.utils;

import java.util.Optional;

/**
 * Pure one-shot authorization for a short, signed motor retest.
 *
 * <p>The opaque token is bound to one duty cycle and one session deadline. A successful consume
 * can happen only once, and output permission expires independently of the command scheduler.
 */
public final class OneShotMotorRetestLease {
  /** Identity-based capability; callers cannot construct a token. */
  public static final class Token {
    private final long generation;

    private Token(long generation) {
      this.generation = generation;
    }

    public long generation() {
      return generation;
    }
  }

  private long nextGeneration;
  private Token activeToken;
  private double authorizedDuty = Double.NaN;
  private double armedAt = Double.NEGATIVE_INFINITY;
  private double sessionExpiresAt = Double.NEGATIVE_INFINITY;
  private double consumedAt = Double.NEGATIVE_INFINITY;
  private double outputExpiresAt = Double.NEGATIVE_INFINITY;
  private boolean consumed;

  public synchronized Optional<Token> arm(
      double requestedDuty,
      double maximumAbsoluteDuty,
      double nowSeconds,
      double sessionExpiresAtSeconds,
      boolean sessionAllowed) {
    disarm();
    if (!sessionAllowed
        || !Double.isFinite(requestedDuty)
        || !Double.isFinite(maximumAbsoluteDuty)
        || maximumAbsoluteDuty <= 0.0
        || Math.abs(requestedDuty) <= 0.0
        || Math.abs(requestedDuty) > maximumAbsoluteDuty
        || !Double.isFinite(nowSeconds)
        || !Double.isFinite(sessionExpiresAtSeconds)
        || sessionExpiresAtSeconds <= nowSeconds) {
      return Optional.empty();
    }
    activeToken = new Token(++nextGeneration);
    authorizedDuty = requestedDuty;
    armedAt = nowSeconds;
    sessionExpiresAt = sessionExpiresAtSeconds;
    return Optional.of(activeToken);
  }

  public synchronized boolean consume(
      Token token,
      double requestedDuty,
      double nowSeconds,
      double maximumOutputDurationSeconds,
      boolean outputsAllowed) {
    if (consumed
        || !sessionValidInternal(token, requestedDuty, nowSeconds, outputsAllowed)
        || !Double.isFinite(maximumOutputDurationSeconds)
        || maximumOutputDurationSeconds <= 0.0) {
      return false;
    }
    consumed = true;
    consumedAt = nowSeconds;
    outputExpiresAt = Math.min(
        sessionExpiresAt, nowSeconds + maximumOutputDurationSeconds);
    return true;
  }

  public synchronized boolean sessionValid(
      Token token, double requestedDuty, double nowSeconds, boolean sessionAllowed) {
    return sessionValidInternal(token, requestedDuty, nowSeconds, sessionAllowed);
  }

  public synchronized boolean outputAllowed(double nowSeconds, boolean outputsAllowed) {
    boolean allowed = activeToken != null
        && consumed
        && outputsAllowed
        && Double.isFinite(nowSeconds)
        && nowSeconds >= consumedAt
        && nowSeconds <= outputExpiresAt;
    if (activeToken != null && consumed && !allowed) {
      // Mode loss, clock rollback, and timeout are irreversible for this one-shot capability.
      disarm();
    }
    return allowed;
  }

  /** Returns the consumed output deadline only to the exact active capability. */
  public synchronized double outputExpiresAtSeconds(Token token) {
    return activeToken != null && token == activeToken && consumed
        ? outputExpiresAt
        : Double.NEGATIVE_INFINITY;
  }

  /**
   * Bounded authorization view for the SPARK output-lock boundary.
   *
   * <p>The lease monitor is acquired while the caller holds the SPARK output lock. No code holds
   * this monitor while acquiring a SPARK or subsystem lock, so the lock order remains one-way.
   */
  public synchronized boolean outputAuthorizationSnapshot(
      Token token, double requestedDuty, double nowSeconds, boolean outputsAllowed) {
    return activeToken != null
        && token == activeToken
        && consumed
        && outputsAllowed
        && Double.isFinite(requestedDuty)
        && Double.compare(requestedDuty, authorizedDuty) == 0
        && Double.isFinite(nowSeconds)
        && nowSeconds >= consumedAt
        && nowSeconds <= outputExpiresAt;
  }

  public synchronized void disarm() {
    activeToken = null;
    authorizedDuty = Double.NaN;
    armedAt = Double.NEGATIVE_INFINITY;
    sessionExpiresAt = Double.NEGATIVE_INFINITY;
    consumedAt = Double.NEGATIVE_INFINITY;
    outputExpiresAt = Double.NEGATIVE_INFINITY;
    consumed = false;
  }

  private boolean sessionValidInternal(
      Token token, double requestedDuty, double nowSeconds, boolean sessionAllowed) {
    return activeToken != null
        && token == activeToken
        && sessionAllowed
        && Double.isFinite(requestedDuty)
        && Double.compare(requestedDuty, authorizedDuty) == 0
        && Double.isFinite(nowSeconds)
        && nowSeconds >= armedAt
        && nowSeconds <= sessionExpiresAt;
  }
}
