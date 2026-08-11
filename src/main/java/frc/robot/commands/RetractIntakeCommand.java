package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RetractIntakeCommand extends Command {
    private final IntakeSubsystem m_intake;

    public RetractIntakeCommand(IntakeSubsystem intake) {
        this.m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.retractIntake();
    }

    @Override
    public void execute() {
        m_intake.retractIntake();
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopAll();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
