package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.utils.SparkMAXContainer.PositionCommandStatus;
import org.junit.jupiter.api.Test;

class PositionCommandStatusMappingTest {
  @Test
  void shooterPreparationPreservesExactActuatorStatusAndBooleanCompatibility() {
    ShooterSubsystem.PreparationStatus moving = new ShooterSubsystem.PreparationStatus(
        true, PositionCommandStatus.MOVING, true, false);
    ShooterSubsystem.PreparationStatus atTarget = new ShooterSubsystem.PreparationStatus(
        true, PositionCommandStatus.AT_TARGET, true, true);
    ShooterSubsystem.PreparationStatus rejected = new ShooterSubsystem.PreparationStatus(
        false, PositionCommandStatus.REJECTED, true, false);

    assertEquals(PositionCommandStatus.MOVING, moving.actuatorPositionStatus());
    assertFalse(moving.actuatorAtTarget());
    assertEquals(PositionCommandStatus.AT_TARGET, atTarget.actuatorPositionStatus());
    assertTrue(atTarget.actuatorAtTarget());
    assertEquals(PositionCommandStatus.REJECTED, rejected.actuatorPositionStatus());
    assertFalse(rejected.actuatorAtTarget());
  }

  @Test
  void turretAimDoesNotClassifyAcceptedMovementAsRejected() {
    assertEquals(
        TurretSubsystem.AimStatus.COMMANDING_CORRECTION,
        TurretSubsystem.aimStatusForPositionCommand(PositionCommandStatus.MOVING));
    assertEquals(
        TurretSubsystem.AimStatus.CORRECTION_AT_TARGET,
        TurretSubsystem.aimStatusForPositionCommand(PositionCommandStatus.AT_TARGET));
    assertEquals(
        TurretSubsystem.AimStatus.COMMAND_REJECTED,
        TurretSubsystem.aimStatusForPositionCommand(PositionCommandStatus.REJECTED));
  }
}
