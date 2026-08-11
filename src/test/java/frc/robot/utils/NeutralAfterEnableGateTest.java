package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class NeutralAfterEnableGateTest {
  @Test
  void heldInputCannotStartUntilReleasedAfterEnable() {
    NeutralAfterEnableGate gate = new NeutralAfterEnableGate();

    assertFalse(gate.allow(false, true));
    assertFalse(gate.allow(true, true));
    assertFalse(gate.allow(true, false));
    assertTrue(gate.allow(true, true));
  }

  @Test
  void sourceChangeAndExplicitBlockBothRequireAnotherRelease() {
    NeutralAfterEnableGate gate = new NeutralAfterEnableGate();

    assertFalse(gate.allow(true, 1, false));
    assertTrue(gate.allow(true, 1, true));
    assertFalse(gate.allow(true, 2, true));
    assertFalse(gate.allow(true, 2, false));
    assertTrue(gate.allow(true, 2, true));

    gate.blockUntilNeutral();
    assertFalse(gate.allow(true, 2, true));
    assertFalse(gate.allow(true, 2, false));
    assertTrue(gate.allow(true, 2, true));
  }
}
