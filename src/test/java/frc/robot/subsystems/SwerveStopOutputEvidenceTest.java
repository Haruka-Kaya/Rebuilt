package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.utils.CtreDeviceEvidence.Metric;
import frc.robot.utils.CtreDeviceEvidence.SignalObservation;
import frc.robot.utils.CtreDeviceEvidence.Snapshot;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class SwerveStopOutputEvidenceTest {
  @Test
  void requiresFreshPostNeutralDutyAndVoltageNearZero() {
    Map<Integer, Map<String, Double>> baseline = Map.of(
        50, Map.of("duty-cycle", 9.0, "motor-voltage", 9.0));

    assertEquals(
        "OK",
        CommandSwerveDrivetrain.postNeutralOutputReason(
            snapshot(0.0, 0.0, 2.0, 10.0), 1.5, baseline));
    assertEquals(
        "duty-cycle/NONZERO_REPORTED_OUTPUT",
        CommandSwerveDrivetrain.postNeutralOutputReason(
            snapshot(0.02, 0.0, 2.0, 10.0), 1.5, baseline));
    assertEquals(
        "motor-voltage/NONZERO_REPORTED_OUTPUT",
        CommandSwerveDrivetrain.postNeutralOutputReason(
            snapshot(0.0, 0.30, 2.0, 10.0), 1.5, baseline));
    assertEquals(
        "POST_NEUTRAL_OUTPUT_SAMPLE_PENDING",
        CommandSwerveDrivetrain.postNeutralOutputReason(
            snapshot(0.0, 0.0, 1.4, 10.0), 1.5, baseline));
    assertEquals(
        "POST_NEUTRAL_OUTPUT_SAMPLE_PENDING",
        CommandSwerveDrivetrain.postNeutralOutputReason(
            snapshot(0.0, 0.0, 2.0, 9.0), 1.5, baseline));
  }

  @Test
  void missingOutputSignalsCannotConfirmStop() {
    Snapshot missingVoltage = new Snapshot(
        50,
        "front-left steer",
        "SWERVE_STEER_MOTOR",
        true,
        "OK",
        true,
        "OK",
        2.0,
        List.of(observation("duty-cycle", Metric.DUTY_CYCLE, 0.0, 2.0, 10.0)));

    assertEquals(
        "OUTPUT_SIGNALS_MISSING",
        CommandSwerveDrivetrain.postNeutralOutputReason(
            missingVoltage,
            1.5,
            Map.of(50, Map.of("duty-cycle", 9.0))));
  }

  private static Snapshot snapshot(
      double duty, double voltage, double sampleTimestamp, double progressTimestamp) {
    return new Snapshot(
        50,
        "front-left steer",
        "SWERVE_STEER_MOTOR",
        true,
        "OK",
        true,
        "OK",
        sampleTimestamp,
        List.of(
            observation(
                "duty-cycle", Metric.DUTY_CYCLE, duty, sampleTimestamp, progressTimestamp),
            observation(
                "motor-voltage",
                Metric.MOTOR_VOLTAGE_VOLTS,
                voltage,
                sampleTimestamp,
                progressTimestamp)));
  }

  private static SignalObservation observation(
      String name,
      Metric metric,
      double value,
      double sampleTimestamp,
      double progressTimestamp) {
    return new SignalObservation(
        name, metric, true, "OK", value, sampleTimestamp, progressTimestamp, 0.01);
  }
}
