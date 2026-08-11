package frc.robot.utils;

/** Accepts one fresh false-to-true arm request while an external safety condition is valid. */
public final class OneShotTimedArmGate {
  private final double lifetimeSeconds;
  private boolean previousRequest;
  private double armedAtSeconds = Double.NaN;

  public OneShotTimedArmGate(double lifetimeSeconds) {
    if (!Double.isFinite(lifetimeSeconds) || lifetimeSeconds <= 0.0) {
      throw new IllegalArgumentException("lifetimeSeconds must be finite and positive");
    }
    this.lifetimeSeconds = lifetimeSeconds;
  }

  /** Observes the dashboard value and returns whether a fresh arm is currently available. */
  public synchronized boolean observe(
      boolean requested, boolean armingAllowed, double nowSeconds) {
    if (!Double.isFinite(nowSeconds)) {
      invalidate(requested);
      return false;
    }
    if (!armingAllowed) {
      invalidate(requested);
      return false;
    }
    if (!requested) {
      previousRequest = false;
      armedAtSeconds = Double.NaN;
      return false;
    }
    if (!previousRequest) {
      armedAtSeconds = nowSeconds;
    }
    previousRequest = true;
    if (!isFresh(nowSeconds)) {
      armedAtSeconds = Double.NaN;
      return false;
    }
    return true;
  }

  /** Consumes the arm exactly once. */
  public synchronized boolean consume(double nowSeconds) {
    boolean accepted = isFresh(nowSeconds);
    armedAtSeconds = Double.NaN;
    return accepted;
  }

  /** Invalidates any pending arm and remembers the current source level. */
  public synchronized void invalidate(boolean currentRequest) {
    previousRequest = currentRequest;
    armedAtSeconds = Double.NaN;
  }

  /** Invalidates the arm and requires a subsequently observed low level before another edge. */
  public synchronized void requireRelease() {
    previousRequest = true;
    armedAtSeconds = Double.NaN;
  }

  private boolean isFresh(double nowSeconds) {
    if (!Double.isFinite(nowSeconds) || !Double.isFinite(armedAtSeconds)) {
      return false;
    }
    double age = nowSeconds - armedAtSeconds;
    return age >= 0.0 && age <= lifetimeSeconds;
  }
}
