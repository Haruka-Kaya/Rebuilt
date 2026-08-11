package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.utils.SparkOutputGate.ZeroDecision;

class SparkOutputGateTest {
  @Test
  void startupIsUnknownUntilAZeroIsConfirmed() {
    SparkOutputGate gate = new SparkOutputGate();

    assertTrue(gate.outputMayBeNonzero());
    assertEquals(ZeroDecision.ATTEMPT, gate.decideZero(0.0));

    gate.zeroSucceeded();

    assertFalse(gate.outputMayBeNonzero());
    assertEquals(ZeroDecision.NOT_NEEDED, gate.decideZero(0.1));
  }

  @Test
  void failedZeroRetriesAreRateLimitedButSafetyRevocationCanForceAnAttempt() {
    SparkOutputGate gate = new SparkOutputGate();
    gate.zeroFailed(1.0);

    assertEquals(ZeroDecision.RETRY_LATER, gate.decideZero(1.24));
    gate.requireZero(1.24, true);
    assertEquals(ZeroDecision.ATTEMPT, gate.decideZero(1.24));
    assertEquals(ZeroDecision.ATTEMPT, gate.decideZero(1.25));
    assertEquals(0.25, SparkOutputGate.zeroRetryDelaySeconds(1));
    assertEquals(0.5, SparkOutputGate.zeroRetryDelaySeconds(2));
    assertEquals(0.5, SparkOutputGate.zeroRetryDelaySeconds(3));
    assertEquals(0.5, SparkOutputGate.zeroRetryDelaySeconds(99));
  }

  @Test
  void aNewerNonzeroCancelsAnOlderPendingStop() {
    SparkOutputGate gate = new SparkOutputGate();
    gate.zeroSucceeded();
    long stoppedGeneration = gate.requireZero(2.0, true);

    assertEquals(ZeroDecision.ATTEMPT, gate.decideZero(2.0));
    gate.nonzeroSucceeded();

    assertFalse(gate.isZeroRequired());
    assertTrue(gate.generation() > stoppedGeneration);
    assertEquals(ZeroDecision.NOT_NEEDED, gate.decideZero(2.0));
  }

  @Test
  void repeatedStopRequestsDoNotInvalidateAnInFlightZero() {
    SparkOutputGate gate = new SparkOutputGate();
    long generation = gate.generation();

    gate.requireZero(0.0, false);
    gate.requireZero(0.1, false);

    assertEquals(generation, gate.generation());
  }

  @Test
  void callersCanDeduplicateStopsAfterZeroIsConfirmed() {
    SparkOutputGate gate = new SparkOutputGate();
    gate.zeroSucceeded();
    long stoppedGeneration = gate.generation();

    gate.requireZero(1.0, false);

    assertFalse(gate.outputMayBeNonzero());
    assertFalse(gate.isZeroRequired());
    assertFalse(gate.needsZeroCommand());
    assertEquals(stoppedGeneration, gate.generation());
    assertEquals(ZeroDecision.NOT_NEEDED, gate.decideZero(1.0));

    gate.requireZero(2.0, true);
    assertTrue(gate.isZeroRequired());
    assertTrue(gate.generation() > stoppedGeneration);
    assertEquals(ZeroDecision.ATTEMPT, gate.decideZero(2.0));
  }
}
