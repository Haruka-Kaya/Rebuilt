package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FireCommand extends Command {
    private final FeederSubsystem m_feeder;
    private final ConveyorSubsystem m_conveyer;
    private final ShooterSubsystem m_shooter;

    public FireCommand(
            FeederSubsystem feeder,
            ConveyorSubsystem conveyor,
            ShooterSubsystem shooter) {
        this.m_feeder = feeder;
        this.m_conveyer = conveyor;
        this.m_shooter = shooter;
        addRequirements(feeder, conveyor);
    }

    @Override
    public void initialize() {
        feedOnlyWhenShooterIsReady();
    }

    @Override
    public void execute() {
        feedOnlyWhenShooterIsReady();
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

    private void feedOnlyWhenShooterIsReady() {
        boolean pathReady = m_feeder.isReady() && m_conveyer.isReady();
        if (m_shooter.isFlywheelReady() && pathReady) {
            boolean conveyorStarted = m_conveyer.runConveyor();
            boolean feederStarted = conveyorStarted && m_feeder.feed();
            if (conveyorStarted && feederStarted) {
                return;
            }
        }
        m_feeder.stop();
        m_conveyer.stop();
    }
}
