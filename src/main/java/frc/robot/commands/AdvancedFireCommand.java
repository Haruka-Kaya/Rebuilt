package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Telemetry;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AdvancedFireCommand extends Command {
    private final TurretSubsystem m_turret;
    private final ShooterSubsystem m_shooter;
    private final FeederSubsystem m_feeder;
    private final ConveyorSubsystem m_conveyer;

    public AdvancedFireCommand(TurretSubsystem turret, ShooterSubsystem shooter, FeederSubsystem feedeer, ConveyorSubsystem conveyor) {
        this.m_turret = turret;
        this.m_shooter = shooter;
        this.m_feeder = feedeer;
        this.m_conveyer = conveyor;
        addRequirements(turret, shooter, feedeer, conveyor);
    }

    @Override
    public void initialize() {
        m_shooter.setShooterSpeed();
    }

    @Override
    public void execute() {
        m_turret.autoAimWithLimelight();
        m_shooter.setShooterSpeed();    // remove in prod

        boolean pathReady = m_feeder.isReady() && m_conveyer.isReady();
        if(m_shooter.isFlywheelReady()
                && m_turret.onTarget
                && Telemetry.isHubActive()
                && pathReady) {
            boolean conveyorStarted = m_conveyer.runConveyor();
            boolean feederStarted = conveyorStarted && m_feeder.feed();
            if (conveyorStarted && feederStarted) {
                return;
            }
        }
        m_feeder.stop();
        m_conveyer.stop();
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.stop();
        m_shooter.stop();
        m_conveyer.stop();
        m_feeder.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
