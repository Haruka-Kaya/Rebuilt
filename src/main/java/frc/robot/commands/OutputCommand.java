package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

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
        m_intake.extendIntake();
    }

    @Override
    public void execute() {
        m_intake.runIntake(false);
        m_conveyor.backfeedConveyor();
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
        m_conveyor.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

