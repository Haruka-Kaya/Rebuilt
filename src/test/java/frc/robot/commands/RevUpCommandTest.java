package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.SparkMAXContainer.PositionCommandStatus;
import java.util.List;
import org.junit.jupiter.api.Test;

class RevUpCommandTest {
  @Test
  void bothRejectedPreservesBothComponentReasonsAndReferenceDetail() {
    var unreferenced = RevUpCommand.classifyPreparation(new ShooterSubsystem.PreparationStatus(
        false, PositionCommandStatus.REJECTED, false, false));
    var referenced = RevUpCommand.classifyPreparation(new ShooterSubsystem.PreparationStatus(
        true, PositionCommandStatus.REJECTED, false, false));

    assertTrue(unreferenced.blocked());
    assertEquals(
        List.of(
            "FLYWHEEL_PAIR_COMMAND_REJECTED",
            "HOOD_UNREFERENCED",
            "NO_SHOT_PREPARATION_COMMAND_ACCEPTED"),
        unreferenced.reasons());
    assertEquals("HOOD_COMMAND_REJECTED", referenced.reasons().get(1));
  }

  @Test
  void oneAcceptedComponentRemainsActiveWhileNamingTheRejectedComponent() {
    var hoodOnly = RevUpCommand.classifyPreparation(new ShooterSubsystem.PreparationStatus(
        true, PositionCommandStatus.MOVING, false, false));
    var flywheelOnly = RevUpCommand.classifyPreparation(new ShooterSubsystem.PreparationStatus(
        false, PositionCommandStatus.REJECTED, true, false));

    assertFalse(hoodOnly.blocked());
    assertEquals(
        List.of(
            "HOOD_POSITION_COMMAND_ACCEPTED_NOT_MOTION_PROOF",
            "FLYWHEEL_PAIR_COMMAND_REJECTED"),
        hoodOnly.reasons());
    assertFalse(flywheelOnly.blocked());
    assertEquals(
        List.of("FLYWHEEL_COMMAND_ACCEPTED_HOOD_UNREFERENCED"), flywheelOnly.reasons());
  }

  @Test
  void acceptedPreparationDistinguishesMovingSpeedAndReadyStates() {
    var hoodMoving = RevUpCommand.classifyPreparation(new ShooterSubsystem.PreparationStatus(
        true, PositionCommandStatus.MOVING, true, false));
    var speedPending = RevUpCommand.classifyPreparation(new ShooterSubsystem.PreparationStatus(
        true, PositionCommandStatus.AT_TARGET, true, false));
    var ready = RevUpCommand.classifyPreparation(new ShooterSubsystem.PreparationStatus(
        true, PositionCommandStatus.AT_TARGET, true, true));

    assertEquals(
        List.of("FLYWHEEL_AND_HOOD_COMMANDS_ACCEPTED_HOOD_NOT_AT_TARGET_NOT_MOTION_PROOF"),
        hoodMoving.reasons());
    assertEquals(
        List.of("FLYWHEEL_COMMAND_ACCEPTED_SPEED_NOT_READY_NOT_MOTION_PROOF"),
        speedPending.reasons());
    assertEquals(List.of("SHOT_PREPARATION_READY"), ready.reasons());
  }
}
