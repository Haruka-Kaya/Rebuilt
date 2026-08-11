package frc.robot.containers;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class AutonomousReadinessTest {
  @Test
  void allDependenciesMustBeReady() {
    var allReady = new AutonomousReadiness.Inputs(
        true, true, true, true, true, true, true, true, true);
    assertTrue(AutonomousReadiness.evaluate(allReady).ready());

    for (int unavailable = 0; unavailable < 9; unavailable++) {
      boolean[] ready = {true, true, true, true, true, true, true, true, true};
      ready[unavailable] = false;
      var result = AutonomousReadiness.evaluate(new AutonomousReadiness.Inputs(
          ready[0], ready[1], ready[2], ready[3], ready[4], ready[5], ready[6], ready[7],
          ready[8]));
      assertFalse(result.ready(), "dependency " + unavailable + " must block autonomous");
      assertFalse(result.reason().isBlank());
    }
  }

  @Test
  void feederKnownFaultHasSpecificOperatorReason() {
    var result = AutonomousReadiness.evaluate(
        new AutonomousReadiness.Inputs(
            true, true, true, true, true, true, true, true, false));

    assertFalse(result.ready());
    assertTrue(result.reason().contains("known fault"));
  }
}
