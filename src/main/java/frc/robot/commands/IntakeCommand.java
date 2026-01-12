package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private boolean trueForIn;
    private final IntakeSubsystem m_intake;
    
    public IntakeCommand(boolean trueForIn, IntakeSubsystem intake) {
        this.trueForIn = trueForIn;
        this.m_intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.runIntake(trueForIn);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
    }
}
