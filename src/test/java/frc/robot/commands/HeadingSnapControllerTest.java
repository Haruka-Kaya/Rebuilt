package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class HeadingSnapControllerTest {
  @Test
  void resetMakesRepeatedActivationStartFromCurrentHeading() {
    HeadingSnapController controller = new HeadingSnapController();

    assertTrue(controller.reset(0.0));
    assertEquals(0.0, controller.calculate(0.0), 1e-9);

    double secondHeading = Math.PI / 2.0;
    assertTrue(controller.reset(secondHeading));
    assertEquals(0.0, controller.calculate(secondHeading), 1e-9);
  }

  @Test
  void rejectsNonFiniteHeadings() {
    HeadingSnapController controller = new HeadingSnapController();

    assertFalse(controller.reset(Double.NaN));
    assertTrue(Double.isNaN(controller.calculate(Double.POSITIVE_INFINITY)));
  }
}
