package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Resettable diagonal heading snap used by {@link JumpBumpCommand}. */
final class HeadingSnapController {
  private final ProfiledPIDController controller = new ProfiledPIDController(
      4.0,
      0.0,
      0.0,
      new TrapezoidProfile.Constraints(Math.PI * 4.0, Math.PI * 8.0));
  private double targetHeadingRadians = Double.NaN;

  HeadingSnapController() {
    controller.enableContinuousInput(-Math.PI, Math.PI);
  }

  static double closestBumpHeading(double currentHeadingRadians) {
    if (!Double.isFinite(currentHeadingRadians)) {
      return Double.NaN;
    }
    double shifted = currentHeadingRadians - Math.PI / 4.0;
    double snapped = Math.round(shifted / (Math.PI / 2.0)) * (Math.PI / 2.0)
        + Math.PI / 4.0;
    return Math.IEEEremainder(snapped, 2.0 * Math.PI);
  }

  boolean reset(double currentHeadingRadians, double requestedTargetHeadingRadians) {
    if (!Double.isFinite(currentHeadingRadians)
        || !Double.isFinite(requestedTargetHeadingRadians)) {
      targetHeadingRadians = Double.NaN;
      return false;
    }
    targetHeadingRadians = Math.IEEEremainder(requestedTargetHeadingRadians, 2.0 * Math.PI);
    controller.reset(currentHeadingRadians);
    return true;
  }

  double calculate(double currentHeadingRadians) {
    if (!Double.isFinite(currentHeadingRadians) || !Double.isFinite(targetHeadingRadians)) {
      return Double.NaN;
    }
    double output = controller.calculate(currentHeadingRadians, targetHeadingRadians);
    return Double.isFinite(output) ? output : Double.NaN;
  }
}
