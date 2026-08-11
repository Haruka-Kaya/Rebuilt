package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FeederSubsystem;

public class FireCommand extends Command {
    private final FeederSubsystem m_feeder;
    private final ConveyorSubsystem m_conveyer;

    public FireCommand(FeederSubsystem feeder, ConveyorSubsystem conveyor) {
        this.m_feeder = feeder;
        this.m_conveyer = conveyor;
        addRequirements(feeder, conveyor);
    }

    @Override
    public void initialize() {
        m_feeder.feed();
        m_conveyer.runConveyor();
    }

    @Override
    public void execute() {
        // remove this in prod
        m_feeder.feed();
        m_conveyer.runConveyor();
    }

    @Override
    public void end(boolean interrupted) {
        m_feeder.stop();
        m_conveyer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
