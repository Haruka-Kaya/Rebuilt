package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.SparkMAXContainer.PositionCommandStatus;

public class OutputCommand extends Command {
    private final IntakeSubsystem m_intake;
    private final ConveyorSubsystem m_conveyor;
    
    public OutputCommand(IntakeSubsystem intake, ConveyorSubsystem conveyor) {
        this.m_intake = intake;
        this.m_conveyor = conveyor;

        addRequirements(intake);
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
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
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void runOnlyAfterExtension() {
        PositionCommandStatus actuatorStatus = m_intake.extendIntake();
        if (actuatorStatus == PositionCommandStatus.AT_TARGET && m_intake.runIntake(false)) {
            if (m_conveyor.backfeedConveyor()) {
                return;
            }
        }
        m_intake.stopRoller();
        m_conveyor.stop();
    }
}

