package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.commands.ManualUnhomedActuatorDiagnosticCommand.Direction;
import frc.robot.commands.ManualUnhomedActuatorDiagnosticCommand.Target;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.constants.Constants.HardwareTestConstants;
import frc.robot.diagnostics.HardwareDiagnosticEvaluator.Snapshot;
import frc.robot.utils.SparkMAXContainer.TimedDiagnosticSnapshot;

class ManualUnhomedActuatorDiagnosticCommandTest {
  private static final Snapshot READY_ZERO =
      new Snapshot(30, true, 0.0, 0.0, 0.0, 12.0, true);

  @Test
  void onlyANewerSampleFromTheCommandOutputEpochCanBeMotionEvidence() {
    long epoch = 7;
    double commandCompletedAt = 10.0;

    assertFalse(ManualUnhomedActuatorDiagnosticCommand.isCurrentPostCommandSample(
        new TimedDiagnosticSnapshot(READY_ZERO, 9.99, epoch, epoch),
        epoch, commandCompletedAt, Double.NEGATIVE_INFINITY));
    assertFalse(ManualUnhomedActuatorDiagnosticCommand.isCurrentPostCommandSample(
        new TimedDiagnosticSnapshot(READY_ZERO, 10.01, epoch - 1, epoch),
        epoch, commandCompletedAt, Double.NEGATIVE_INFINITY));
    assertFalse(ManualUnhomedActuatorDiagnosticCommand.isCurrentPostCommandSample(
        new TimedDiagnosticSnapshot(READY_ZERO, Double.NaN, epoch, epoch),
        epoch, commandCompletedAt, Double.NEGATIVE_INFINITY));
    assertTrue(ManualUnhomedActuatorDiagnosticCommand.isCurrentPostCommandSample(
        new TimedDiagnosticSnapshot(READY_ZERO, 10.01, epoch, epoch),
        epoch, commandCompletedAt, Double.NEGATIVE_INFINITY));
    assertFalse(ManualUnhomedActuatorDiagnosticCommand.isCurrentPostCommandSample(
        new TimedDiagnosticSnapshot(READY_ZERO, 10.01, epoch, epoch),
        epoch, commandCompletedAt, 10.01));
  }

  @Test
  void disabledSelectionMapsToExactlyTheBoundedSignedDuty() {
    assertEquals(
        -HardwareTestConstants.UNHOMED_DIAGNOSTIC_MAX_DUTY_CYCLE,
        Direction.NEGATIVE.duty());
    assertEquals(
        HardwareTestConstants.UNHOMED_DIAGNOSTIC_MAX_DUTY_CYCLE,
        Direction.POSITIVE.duty());
  }

  @Test
  void climberPulseStopsBothControllersAndUsesTheConservativeCurrentLimit() {
    assertArrayEquals(
        new int[] {ClimberConstants.LEFT_MOTOR_CAN_ID, ClimberConstants.RIGHT_MOTOR_CAN_ID},
        Target.CLIMBER_LEFT.stopCanIds());
    assertArrayEquals(
        new int[] {ClimberConstants.LEFT_MOTOR_CAN_ID, ClimberConstants.RIGHT_MOTOR_CAN_ID},
        Target.CLIMBER_RIGHT.stopCanIds());
    assertEquals(
        ClimberConstants.DIAGNOSTIC_CURRENT_LIMIT_AMPS,
        Target.CLIMBER_LEFT.maximumCurrentAmps());
    assertEquals(
        ClimberConstants.DIAGNOSTIC_CURRENT_LIMIT_AMPS,
        Target.CLIMBER_RIGHT.maximumCurrentAmps());
  }
}
