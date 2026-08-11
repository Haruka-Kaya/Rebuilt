package frc.robot.containers;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class AutonomousSelectionPolicyTest {
  @Test
  void healthyDependenciesStillRequireAnExplicitReviewedSelection() {
    var healthy = new AutonomousReadiness.Result(true, "dependencies ready");

    assertFalse(AutonomousSelectionPolicy.evaluate(healthy, null).ready());
    assertFalse(AutonomousSelectionPolicy.evaluate(healthy, "  ").ready());
    assertTrue(AutonomousSelectionPolicy.evaluate(healthy, "Example Auto").ready());
  }

  @Test
  void aSelectionCannotOverrideFailedDependencies() {
    var blocked = new AutonomousReadiness.Result(false, "swerve unavailable");

    var result = AutonomousSelectionPolicy.evaluate(blocked, "Example Auto");

    assertFalse(result.ready());
    assertTrue(result.reason().contains("swerve unavailable"));
  }

  @Test
  void nullReadinessFailsClosed() {
    assertFalse(AutonomousSelectionPolicy.evaluate(null, "Example Auto").ready());
  }
}
