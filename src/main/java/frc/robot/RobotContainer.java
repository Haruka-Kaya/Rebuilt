// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.commands.FireCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JumpBumpCommand;
import frc.robot.commands.RevUpCommand;
import frc.robot.commands.OutputCommand;
import frc.robot.commands.RetractIntakeCommand;
import frc.robot.constants.Constants.LimelightConstants;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.containers.DriveBaseContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The driver's controller
  private final CommandPS5Controller m_driverController = new CommandPS5Controller(OIConstants.kDriverControllerPort);
  private final CommandPS5Controller m_operatorController = new CommandPS5Controller(OIConstants.kOperatorControllerPort);
  private final CommandPS5Controller m_maintenanceController = new CommandPS5Controller(OIConstants.kMaintenanceControllerPort);

  // The robot's subsystems
  private final VisionSubsystem m_driveVision = new VisionSubsystem(LimelightConstants.DRIVE_LIMELIGHT_NAME);

  private final CommandSwerveDrivetrain drivetrain;

  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final TurretSubsystem m_turret = new TurretSubsystem(m_driveVision);
  private final ConveyorSubsystem m_conveyor = new ConveyorSubsystem();
  private final FeederSubsystem m_feeder = new FeederSubsystem();

  // The robot's commands
  private final JumpBumpCommand jumpBump;

  private final IntakeCommand slurp = new IntakeCommand(m_intake, m_conveyor);
  private final OutputCommand spit = new OutputCommand(m_intake, m_conveyor);

  private final RetractIntakeCommand back_in_shell = new RetractIntakeCommand(m_intake);

  private final FireCommand fire = new FireCommand(m_feeder, m_conveyor);
  private final RevUpCommand revWheel = new RevUpCommand(m_shooter);


  // Something?
  private final DriveBaseContainer m_DriveBaseContainer; 

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_DriveBaseContainer = new DriveBaseContainer(m_driverController, m_turret, m_shooter, m_feeder, m_conveyor, m_intake);
    drivetrain = m_DriveBaseContainer.drivetrain;

    jumpBump = new JumpBumpCommand(drivetrain, m_driverController);

    // Configure the button bindings (put this last)
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link PS5Controller}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Probably change (all of) this

    // Note;
    /*
     * Driver controls driving, intake, aiming, reving, and shooting
     * 
     */

    m_driverController.L1().whileTrue(slurp);
    m_driverController.R1().whileTrue(spit);

    m_driverController.R3().whileTrue(jumpBump);

    m_driverController.L2().whileTrue(revWheel);
    m_driverController.R2().whileTrue(fire);

    m_operatorController.L1().whileTrue(back_in_shell);

    m_maintenanceController.L1().whileTrue(new RunCommand(() -> m_turret.autoAimWithLimelight(), m_turret));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return this.m_DriveBaseContainer.GetAutonCommand();
  }

  public void stopAll() {
    m_intake.stop();
    m_conveyor.stop();
    m_feeder.stop();
    m_shooter.stop();
  }
}
