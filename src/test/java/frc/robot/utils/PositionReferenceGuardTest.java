package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class PositionReferenceGuardTest {
  @Test
  void rejectsMissingAndForeignTokens() {
    PositionReferenceGuard first = new PositionReferenceGuard();
    PositionReferenceGuard second = new PositionReferenceGuard();

    assertFalse(first.isValid(null));
    assertFalse(first.isValid(second.establish()));
  }

  @Test
  void invalidationPermanentlyRejectsAnOldToken() {
    PositionReferenceGuard guard = new PositionReferenceGuard();
    PositionReferenceGuard.Token oldToken = guard.establish();
    long oldGeneration = guard.generation();

    assertTrue(guard.isValid(oldToken));
    guard.invalidate();

    assertFalse(guard.isValid(oldToken));
    assertNotEquals(oldGeneration, guard.generation());
    assertTrue(guard.isValid(guard.establish()));
  }
}
