package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class OneShotTimedArmGateTest {
  @Test
  void freshDisabledRisingEdgeCanBeConsumedOnlyOnce() {
    OneShotTimedArmGate gate = new OneShotTimedArmGate(15.0);

    assertFalse(gate.observe(false, true, 1.0));
    assertTrue(gate.observe(true, true, 2.0));
    assertTrue(gate.consume(3.0));
    assertFalse(gate.consume(3.1));
    assertFalse(gate.observe(true, true, 3.2));
  }

  @Test
  void requestMadeInAnotherModeRequiresReleaseBeforeArming() {
    OneShotTimedArmGate gate = new OneShotTimedArmGate(15.0);

    assertFalse(gate.observe(true, false, 1.0));
    assertFalse(gate.observe(true, true, 2.0));
    assertFalse(gate.observe(false, true, 3.0));
    assertTrue(gate.observe(true, true, 4.0));
  }

  @Test
  void expiredArmDoesNotRearmWhileSourceRemainsHigh() {
    OneShotTimedArmGate gate = new OneShotTimedArmGate(5.0);

    assertFalse(gate.observe(false, true, 0.0));
    assertTrue(gate.observe(true, true, 1.0));
    assertFalse(gate.observe(true, true, 6.01));
    assertFalse(gate.observe(true, true, 7.0));
    assertFalse(gate.observe(false, true, 8.0));
    assertTrue(gate.observe(true, true, 9.0));
  }

  @Test
  void programmaticDashboardClearCannotImpersonateAUserRelease() {
    OneShotTimedArmGate gate = new OneShotTimedArmGate(15.0);

    assertFalse(gate.observe(true, false, 1.0));
    gate.requireRelease();
    assertFalse(gate.observe(true, true, 2.0));
    assertFalse(gate.observe(false, true, 3.0));
    assertTrue(gate.observe(true, true, 4.0));
  }
}
