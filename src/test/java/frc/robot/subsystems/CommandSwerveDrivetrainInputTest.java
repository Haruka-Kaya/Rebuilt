package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class CommandSwerveDrivetrainInputTest {
  @Test
  void acceptsOnlyFiniteDriveInputsAndNonnegativeScale() {
    assertTrue(CommandSwerveDrivetrain.driveInputsAreFinite(0.1, -0.2, 0.3, 1.0));

    assertFalse(CommandSwerveDrivetrain.driveInputsAreFinite(Double.NaN, 0.0, 0.0, 1.0));
    assertFalse(CommandSwerveDrivetrain.driveInputsAreFinite(
        0.0, Double.POSITIVE_INFINITY, 0.0, 1.0));
    assertFalse(CommandSwerveDrivetrain.driveInputsAreFinite(
        0.0, 0.0, Double.NEGATIVE_INFINITY, 1.0));
    assertFalse(CommandSwerveDrivetrain.driveInputsAreFinite(0.0, 0.0, 0.0, -1.0));
  }
}
