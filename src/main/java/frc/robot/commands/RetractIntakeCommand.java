package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.constants.ConfiguredOperatorActions.Action;
import frc.robot.utils.OperatorActionEvidence;
import frc.robot.utils.SparkMAXContainer.PositionCommandStatus;

public class RetractIntakeCommand extends Command {
    private final IntakeSubsystem m_intake;
    private final OperatorActionEvidence evidence;

    public RetractIntakeCommand(IntakeSubsystem intake) {
        this(intake, null);
    }

    public RetractIntakeCommand(IntakeSubsystem intake, OperatorActionEvidence evidence) {
        this.m_intake = intake;
        this.evidence = evidence;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (evidence != null) {
            evidence.requested(Action.RETRACT);
        }
        runRetraction();
    }

    @Override
    public void execute() {
        runRetraction();
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopAll();
        if (evidence != null) {
            evidence.commandEnded(Action.RETRACT, interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void runRetraction() {
        PositionCommandStatus status = m_intake.retractIntake();
        if (evidence == null) {
            return;
        }
        switch (status) {
            case AT_TARGET -> evidence.active(Action.RETRACT, "ACTUATOR_AT_RETRACT_TARGET");
            case MOVING -> evidence.active(
                Action.RETRACT, "ACTUATOR_POSITION_COMMAND_ACCEPTED_NOT_AT_TARGET");
            case REJECTED -> evidence.blocked(
                Action.RETRACT,
                m_intake.isActuatorReferenced()
                    ? "ACTUATOR_COMMAND_REJECTED"
                    : "ACTUATOR_UNREFERENCED");
        }
    }
}
