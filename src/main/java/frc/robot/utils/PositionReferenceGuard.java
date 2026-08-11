package frc.robot.utils;

/**
 * Issues opaque position-reference tokens and invalidates every previously issued token when
 * encoder continuity is lost.
 */
public final class PositionReferenceGuard {
  /** A token can only be created by the guard that owns it. */
  public static final class Token {
    private final PositionReferenceGuard owner;
    private final long generation;

    private Token(PositionReferenceGuard owner, long generation) {
      this.owner = owner;
      this.generation = generation;
    }
  }

  private long generation = 1;

  /** Creates a token after the caller has independently proven and written a known position. */
  public synchronized Token establish() {
    return new Token(this, generation);
  }

  /** Invalidates all tokens issued before this call. */
  public synchronized void invalidate() {
    generation++;
  }

  /** Returns whether the supplied token belongs to this guard and the current continuity epoch. */
  public synchronized boolean isValid(Token token) {
    return token != null && token.owner == this && token.generation == generation;
  }

  /** Monotonic diagnostic value; it is not itself authorization for position control. */
  public synchronized long generation() {
    return generation;
  }
}
