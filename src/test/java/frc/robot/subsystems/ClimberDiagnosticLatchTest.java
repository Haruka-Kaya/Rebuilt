package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.ClimberDiagnosticLatch.MotorSide;
import org.junit.jupiter.api.Test;

class ClimberDiagnosticLatchTest {
  private final ClimberDiagnosticLatch latch = new ClimberDiagnosticLatch();

  @Test
  void requiresEverySafetyInterlock() {
    assertTrue(latch.accept(MotorSide.LEFT, 0.03, 0.03, false, false, true, true).isEmpty());
    assertTrue(latch.accept(MotorSide.LEFT, 0.03, 0.03, true, true, true, true).isEmpty());
    assertTrue(latch.accept(MotorSide.LEFT, 0.03, 0.03, true, false, false, true).isEmpty());
    assertTrue(latch.accept(MotorSide.LEFT, 0.03, 0.03, true, false, true, false).isEmpty());
    assertTrue(latch.accept(MotorSide.LEFT, Double.NaN, 0.03, true, false, true, true).isEmpty());
  }

  @Test
  void clampsAndLatchesSideAndDirectionUntilDisabled() {
    assertEquals(
        0.03,
        latch.accept(MotorSide.LEFT, 0.50, 0.03, true, false, true, true).orElseThrow(),
        1e-9);
    assertTrue(latch.accept(MotorSide.RIGHT, 0.03, 0.03, true, false, true, true).isEmpty());
    assertTrue(latch.accept(MotorSide.LEFT, -0.03, 0.03, true, false, true, true).isEmpty());

    assertTrue(latch.accept(MotorSide.LEFT, 0.03, 0.03, false, false, true, true).isEmpty());
    assertEquals(
        -0.02,
        latch.accept(MotorSide.RIGHT, -0.02, 0.03, true, false, true, true).orElseThrow(),
        1e-9);
  }
}
