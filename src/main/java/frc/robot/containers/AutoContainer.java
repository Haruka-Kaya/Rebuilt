package frc.robot.containers;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AdvancedFireCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoContainer {
    private SendableChooser<Command> autoChooser;
    // private final CommandSwerveDrivetrain drivetrain;

    private final TurretSubsystem m_turret;
    private final ShooterSubsystem m_shooter;
    private final FeederSubsystem m_feeder;
    private final ConveyorSubsystem m_conveyor;
    private final IntakeSubsystem m_intake;

    public AutoContainer(CommandSwerveDrivetrain drivetrain, TurretSubsystem turret,
                            ShooterSubsystem shooter, FeederSubsystem feeder,
                            ConveyorSubsystem conveyor, IntakeSubsystem intake) {
        // this.drivetrain = drivetrain;
        // this.drivetrain.configureAutoBuilder();

        this.m_turret = turret;
        this.m_shooter = shooter;
        this.m_feeder = feeder;
        this.m_conveyor = conveyor;
        this.m_intake = intake;

        this.configureAutoBindings();
    }

    private void configureAutoBindings() {
        NamedCommands.registerCommand("Advanced Fire", new AdvancedFireCommand(m_turret, m_shooter, m_feeder, m_conveyor));
        NamedCommands.registerCommand("Slurp", new IntakeCommand(m_intake, m_conveyor));

        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}