package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class HeadingSnapControllerTest {
  @Test
  void resetMakesRepeatedActivationStartFromCurrentHeading() {
    HeadingSnapController controller = new HeadingSnapController();

    assertTrue(controller.reset(0.0, Math.PI / 4.0));
    assertTrue(controller.calculate(0.0) > 0.0);

    double secondHeading = Math.PI / 2.0;
    assertTrue(controller.reset(secondHeading, secondHeading));
    assertEquals(0.0, controller.calculate(secondHeading), 1e-9);
  }

  @Test
  void rejectsNonFiniteHeadings() {
    HeadingSnapController controller = new HeadingSnapController();

    assertFalse(controller.reset(Double.NaN, 0.0));
    assertFalse(controller.reset(0.0, Double.NEGATIVE_INFINITY));
    assertTrue(Double.isNaN(controller.calculate(Double.POSITIVE_INFINITY)));
  }

  @Test
  void targetRemainsFixedWhileRobotCrossesAnotherSnapBoundary() {
    HeadingSnapController controller = new HeadingSnapController();

    assertTrue(controller.reset(0.0, Math.PI / 4.0));
    assertTrue(controller.calculate(Math.toRadians(70.0)) < 0.0);
  }

  @Test
  void bumpTargetsAreAlwaysTheNearestDiagonal() {
    assertEquals(Math.PI / 4.0, HeadingSnapController.closestBumpHeading(0.0), 1e-9);
    assertEquals(3.0 * Math.PI / 4.0,
        HeadingSnapController.closestBumpHeading(Math.PI / 2.0), 1e-9);
    assertEquals(-3.0 * Math.PI / 4.0,
        HeadingSnapController.closestBumpHeading(Math.PI), 1e-9);
    assertEquals(-Math.PI / 4.0,
        HeadingSnapController.closestBumpHeading(-Math.PI / 2.0), 1e-9);
    assertTrue(Double.isNaN(HeadingSnapController.closestBumpHeading(Double.NaN)));
  }
}
