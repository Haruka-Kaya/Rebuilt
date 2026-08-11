package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class FlywheelPairReadinessTest {
  @Test
  void acceptsOnlyTheConfiguredLeaderAndInvertedFollowerDirections() {
    assertTrue(FlywheelPairReadiness.atRequestedSpeed(500.0, 500.0, -500.0, 50.0));
    assertTrue(FlywheelPairReadiness.atRequestedSpeed(500.0, 455.0, -545.0, 50.0));

    assertFalse(FlywheelPairReadiness.atRequestedSpeed(500.0, 500.0, 500.0, 50.0));
    assertFalse(FlywheelPairReadiness.atRequestedSpeed(500.0, -500.0, -500.0, 50.0));
    assertFalse(FlywheelPairReadiness.atRequestedSpeed(500.0, -500.0, 500.0, 50.0));
  }

  @Test
  void rejectsStoppedOutOfToleranceAndNonfiniteMeasurements() {
    assertFalse(FlywheelPairReadiness.atRequestedSpeed(500.0, 0.0, 0.0, 50.0));
    assertFalse(FlywheelPairReadiness.atRequestedSpeed(500.0, 440.0, -500.0, 50.0));
    assertFalse(FlywheelPairReadiness.atRequestedSpeed(500.0, 500.0, -440.0, 50.0));
    assertFalse(FlywheelPairReadiness.atRequestedSpeed(500.0, Double.NaN, -500.0, 50.0));
    assertFalse(FlywheelPairReadiness.atRequestedSpeed(500.0, 500.0, Double.NaN, 50.0));
  }

  @Test
  void rejectsInvalidRequestAndTolerance() {
    assertFalse(FlywheelPairReadiness.atRequestedSpeed(50.0, 50.0, -50.0, 50.0));
    assertFalse(FlywheelPairReadiness.atRequestedSpeed(Double.NaN, 500.0, -500.0, 50.0));
    assertFalse(FlywheelPairReadiness.atRequestedSpeed(500.0, 500.0, -500.0, -1.0));
  }
}
