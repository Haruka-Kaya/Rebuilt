package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.SparkMAXContainer.PositionCommandStatus;
import frc.robot.constants.ConfiguredOperatorActions.Action;
import frc.robot.utils.OperatorActionEvidence;

public class OutputCommand extends Command {
    private final IntakeSubsystem m_intake;
    private final ConveyorSubsystem m_conveyor;
    private final OperatorActionEvidence evidence;
    
    public OutputCommand(IntakeSubsystem intake, ConveyorSubsystem conveyor) {
        this(intake, conveyor, null);
    }

    public OutputCommand(
            IntakeSubsystem intake,
            ConveyorSubsystem conveyor,
            OperatorActionEvidence evidence) {
        this.m_intake = intake;
        this.m_conveyor = conveyor;
        this.evidence = evidence;

        addRequirements(intake);
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        if (evidence != null) {
            evidence.requested(Action.OUTPUT);
        }
        runOnlyAfterExtension();
    }

    @Override
    public void execute() {
        runOnlyAfterExtension();
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopAll();
        m_conveyor.stop();
        if (evidence != null) {
            evidence.commandEnded(Action.OUTPUT, interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void runOnlyAfterExtension() {
        PositionCommandStatus actuatorStatus = m_intake.extendIntake();
        if (actuatorStatus == PositionCommandStatus.REJECTED) {
            if (evidence != null) {
                evidence.blocked(
                    Action.OUTPUT,
                    m_intake.isActuatorReferenced()
                        ? "ACTUATOR_COMMAND_REJECTED"
                        : "ACTUATOR_UNREFERENCED");
            }
        } else if (actuatorStatus == PositionCommandStatus.MOVING) {
            if (evidence != null) {
                evidence.active(
                    Action.OUTPUT, "ACTUATOR_POSITION_COMMAND_ACCEPTED_NOT_AT_TARGET");
            }
        } else if (!m_intake.runIntake(false)) {
            if (evidence != null) {
                evidence.blocked(Action.OUTPUT, "ROLLER_COMMAND_REJECTED");
            }
        } else if (!m_conveyor.backfeedConveyor()) {
            if (evidence != null) {
                evidence.blocked(Action.OUTPUT, "CONVEYOR_REJECTED_ROLLER_STOP_REQUESTED");
            }
        } else {
            if (evidence != null) {
                evidence.active(
                    Action.OUTPUT,
                    "ACTUATOR_AT_TARGET_ROLLER_AND_CONVEYOR_COMMANDS_ACCEPTED_NOT_MOTION_PROOF");
            }
            return;
        }
        m_intake.stopRoller();
        m_conveyor.stop();
    }
}

