package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class OneShotMotorRetestLeaseTest {
  @Test
  void bindsOneOpaqueTokenToOneSignedDutyAndOneConsume() {
    OneShotMotorRetestLease gate = new OneShotMotorRetestLease();
    var first = gate.arm(0.03, 0.03, 10.0, 25.0, true).orElseThrow();

    assertFalse(gate.consume(first, -0.03, 10.1, 0.35, true));
    assertTrue(gate.consume(first, 0.03, 10.1, 0.35, true));
    assertTrue(Math.abs(gate.outputExpiresAtSeconds(first) - 10.45) < 1e-9);
    assertTrue(gate.outputAuthorizationSnapshot(first, 0.03, 10.2, true));
    assertFalse(gate.outputAuthorizationSnapshot(first, -0.03, 10.2, true));
    assertFalse(gate.consume(first, 0.03, 10.2, 0.35, true));

    var replacement = gate.arm(-0.03, 0.03, 11.0, 20.0, true).orElseThrow();
    assertFalse(gate.sessionValid(first, 0.03, 11.0, true));
    assertTrue(Double.isInfinite(gate.outputExpiresAtSeconds(first))
        && gate.outputExpiresAtSeconds(first) < 0.0);
    assertTrue(gate.sessionValid(replacement, -0.03, 11.0, true));
  }

  @Test
  void outputPermissionExpiresIndependentlyAndModeLossFailsClosed() {
    OneShotMotorRetestLease gate = new OneShotMotorRetestLease();
    var token = gate.arm(0.03, 0.03, 1.0, 20.0, true).orElseThrow();
    assertTrue(gate.consume(token, 0.03, 2.0, 0.35, true));

    assertTrue(gate.outputAllowed(2.35, true));
    assertFalse(gate.outputAllowed(2.1, false));
    assertFalse(gate.outputAllowed(2.1, true), "mode loss must irreversibly consume the lease");

    var timeoutToken = gate.arm(0.03, 0.03, 3.0, 20.0, true).orElseThrow();
    assertTrue(gate.consume(timeoutToken, 0.03, 3.1, 0.35, true));
    assertFalse(gate.outputAllowed(3.450_001, true));
    assertFalse(gate.outputAllowed(3.2, true), "timeout must irreversibly consume the lease");

    var rollbackToken = gate.arm(0.03, 0.03, 4.0, 20.0, true).orElseThrow();
    assertTrue(gate.consume(rollbackToken, 0.03, 4.1, 0.35, true));
    assertFalse(gate.outputAllowed(4.09, true));
    assertFalse(gate.sessionValid(rollbackToken, 0.03, 4.2, true));
  }

  @Test
  void invalidArmAndExplicitDisarmNeverLeaveACapability() {
    OneShotMotorRetestLease gate = new OneShotMotorRetestLease();
    assertTrue(gate.arm(0.04, 0.03, 0.0, 10.0, true).isEmpty());
    assertTrue(gate.arm(0.03, 0.03, 0.0, 10.0, false).isEmpty());

    var token = gate.arm(0.03, 0.03, 0.0, 10.0, true).orElseThrow();
    gate.disarm();
    assertFalse(gate.sessionValid(token, 0.03, 1.0, true));
    assertFalse(gate.consume(token, 0.03, 1.0, 0.35, true));
    assertFalse(gate.outputAuthorizationSnapshot(token, 0.03, 1.0, true));
  }

  @Test
  void outputCanNeverOutliveTheShorterSessionDeadline() {
    OneShotMotorRetestLease gate = new OneShotMotorRetestLease();
    var token = gate.arm(0.03, 0.03, 1.0, 2.1, true).orElseThrow();
    assertTrue(gate.consume(token, 0.03, 2.0, 0.35, true));
    assertTrue(gate.outputAllowed(2.1, true));
    assertFalse(gate.outputAllowed(2.100_001, true));
  }
}
