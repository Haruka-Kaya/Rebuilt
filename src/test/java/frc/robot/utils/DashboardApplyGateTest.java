package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.utils.DashboardApplyGate.Decision;

class DashboardApplyGateTest {
  @Test
  void requiresNeutralThenDisabledNonFmsRisingEdge() {
    DashboardApplyGate gate = new DashboardApplyGate();

    assertEquals(Decision.RELEASE_REQUIRED, gate.evaluate(true, true, false));
    assertEquals(Decision.NONE, gate.evaluate(true, true, false));
    assertEquals(Decision.NONE, gate.evaluate(false, true, false));
    assertEquals(Decision.REJECT_ENABLED, gate.evaluate(true, false, false));
    assertEquals(Decision.NONE, gate.evaluate(true, true, false));
    assertEquals(Decision.NONE, gate.evaluate(false, true, false));
    assertEquals(Decision.REJECT_FMS, gate.evaluate(true, true, true));
    assertEquals(Decision.NONE, gate.evaluate(true, true, false));
    assertEquals(Decision.NONE, gate.evaluate(false, true, false));
    assertEquals(Decision.APPLY, gate.evaluate(true, true, false));
    assertEquals(Decision.NONE, gate.evaluate(true, true, false));
    assertEquals(Decision.NONE, gate.evaluate(true, true, false));
  }

  @Test
  void validatesFinitenessBoundsAndMatchingShapes() {
    assertTrue(DashboardApplyGate.allFiniteInRange(
        new double[] {0.0, 0.2, -0.2},
        new double[] {0.0, 0.0, -0.2},
        new double[] {1.0, 0.2, 0.0}));
    assertFalse(DashboardApplyGate.allFiniteInRange(
        new double[] {Double.NaN}, new double[] {0.0}, new double[] {1.0}));
    assertFalse(DashboardApplyGate.allFiniteInRange(
        new double[] {Double.POSITIVE_INFINITY}, new double[] {0.0}, new double[] {1.0}));
    assertFalse(DashboardApplyGate.allFiniteInRange(
        new double[] {-0.01}, new double[] {0.0}, new double[] {1.0}));
    assertFalse(DashboardApplyGate.allFiniteInRange(
        new double[] {0.0, 1.0}, new double[] {0.0}, new double[] {1.0}));
  }
}
