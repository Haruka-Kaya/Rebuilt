package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Resettable 45-degree heading snap used by {@link JumpBumpCommand}. */
final class HeadingSnapController {
  private static final double SNAP_STEP_RADIANS = Math.PI / 4.0;

  private final ProfiledPIDController controller = new ProfiledPIDController(
      4.0,
      0.0,
      0.0,
      new TrapezoidProfile.Constraints(Math.PI * 4.0, Math.PI * 8.0));

  HeadingSnapController() {
    controller.enableContinuousInput(-Math.PI, Math.PI);
  }

  boolean reset(double currentHeadingRadians) {
    if (!Double.isFinite(currentHeadingRadians)) {
      return false;
    }
    controller.reset(currentHeadingRadians);
    return true;
  }

  double calculate(double currentHeadingRadians) {
    if (!Double.isFinite(currentHeadingRadians)) {
      return Double.NaN;
    }
    double targetAngle = Math.round(currentHeadingRadians / SNAP_STEP_RADIANS)
        * SNAP_STEP_RADIANS;
    double output = controller.calculate(currentHeadingRadians, targetAngle);
    return Double.isFinite(output) ? output : Double.NaN;
  }
}
