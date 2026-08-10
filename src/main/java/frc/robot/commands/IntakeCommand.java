package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem m_intake;
    private final ConveyorSubsystem m_conveyor;
    
    public IntakeCommand(IntakeSubsystem intake, ConveyorSubsystem conveyor) {
        this.m_intake = intake;
        this.m_conveyor = conveyor;

        addRequirements(intake);
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        m_intake.extendIntake();
        m_intake.runIntake(true);
        m_conveyor.runConveyor();
    }

    @Override
    public void execute() {
        // remove in prod
        m_intake.extendIntake();
        m_intake.runIntake(true);
        m_conveyor.runConveyor();
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
