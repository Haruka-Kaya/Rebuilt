package frc.robot.constants;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
import frc.robot.constants.Constants.ManipulatorConstants;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.Constants.TurretConstants;
import org.junit.jupiter.api.Test;

class HardwareSafetyConfigurationTest {
  @Test
  void feederKnownStallRemainsBlockedUntilPhysicalRetest() {
    assertTrue(ManipulatorConstants.FEEDER_MOTION_BLOCKED_KNOWN_STALL);
    assertFalse(ManipulatorConstants.FEEDER_CONTROLLED_RETEST_ENABLED);
  }

  @Test
  void sparkCanIdsRemainUniqueAndOutsideTheTeleopSignatureReservedBits() {
    int[] sparkCanIds = {
      IntakeConstants.INTAKE_ACTUATOR_CAN_ID,
      IntakeConstants.INTAKE_ROLLER_CAN_ID,
      ManipulatorConstants.FEEDER_CAN_ID,
      ManipulatorConstants.CONVEYOR_CAN_ID,
      ClimberConstants.LEFT_MOTOR_CAN_ID,
      ClimberConstants.RIGHT_MOTOR_CAN_ID,
      ShooterConstants.SHOOTER_1_CAN_ID,
      ShooterConstants.SHOOTER_2_CAN_ID,
      ShooterConstants.ACTUATOR_CAN_ID,
      TurretConstants.TURRET_CAN_ID
    };

    assertEquals(
        sparkCanIds.length,
        Arrays.stream(sparkCanIds).distinct().count(),
        "Every SPARK must have a unique CAN ID");
    assertTrue(
        Arrays.stream(sparkCanIds).allMatch(id -> id >= 30 && id <= 62),
        "SPARK readiness uses CAN ID as a bit; signature bits 0..29 are already reserved");
  }
}
