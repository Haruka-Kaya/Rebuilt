package frc.robot.utils;

/** Pure validation boundary for cached REV periodic-status telemetry. */
final class SparkTelemetryValidator {
  private static final double MAX_ABSOLUTE_APPLIED_OUTPUT = 1.05;
  private static final double MAX_REASONABLE_BUS_VOLTAGE = 30.0;

  private SparkTelemetryValidator() {}

  static String failureReason(
      double appliedOutput,
      double busVoltage,
      double outputCurrent,
      double motorTemperatureCelsius,
      boolean encoderExpected,
      double encoderPosition,
      double encoderVelocity) {
    if (!Double.isFinite(appliedOutput)
        || !Double.isFinite(busVoltage)
        || !Double.isFinite(outputCurrent)
        || !Double.isFinite(motorTemperatureCelsius)) {
      return "nonfinite periodic status 0 telemetry";
    }
    if (Math.abs(appliedOutput) > MAX_ABSOLUTE_APPLIED_OUTPUT) {
      return "applied output outside expected range";
    }
    if (busVoltage <= 0.0 || busVoltage > MAX_REASONABLE_BUS_VOLTAGE) {
      return "bus voltage outside expected range";
    }
    if (outputCurrent < 0.0) {
      return "output current outside expected range";
    }
    if (encoderExpected
        && (!Double.isFinite(encoderPosition) || !Double.isFinite(encoderVelocity))) {
      return "nonfinite periodic status 2 telemetry";
    }
    return null;
  }
}
