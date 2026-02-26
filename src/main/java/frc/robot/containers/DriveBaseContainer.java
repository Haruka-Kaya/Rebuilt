package frc.robot.containers;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Telemetry;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class DriveBaseContainer {
    public AutoContainer autoContainer;
    public static double speedFactor = .2;
    public static double rotationFactor = .2;
    
    static {
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Speed Factor", speedFactor);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Rotation Factor", rotationFactor);
    }

    public static DoubleSupplier MaxSpeed = () -> speedFactor * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static DoubleSupplier MaxAngularRate = () -> RotationsPerSecond.of(rotationFactor).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed.getAsDouble() * OIConstants.kDriveDeadband).withRotationalDeadband(MaxAngularRate.getAsDouble() * OIConstants.kDriveDeadband) // Add deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed.getAsDouble());
    CommandPS5Controller joystick;
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // private final TurretSubsystem m_turret;
    // private final ShooterSubsystem m_shooter;
    // private final FeederSubsystem m_feeder;
    // private final ConveyorSubsystem m_conveyor;
    // private final IntakeSubsystem m_intake;

    public DriveBaseContainer(CommandPS5Controller driverController, TurretSubsystem turret, 
                            ShooterSubsystem shooter, FeederSubsystem feeder, 
                            ConveyorSubsystem conveyor, IntakeSubsystem intake) {
        joystick = driverController;
        configureBindings();
        SmartDashboard.putBoolean("DriveBase Running",true);

        SmartDashboard.putString("MESSAGE", "we are at autoSetup");

        // this.m_turret = turret;
        // this.m_shooter = shooter;
        // this.m_feeder = feeder;
        // this.m_conveyor = conveyor;
        // this.m_intake = intake;

        autoContainer = new AutoContainer(drivetrain, turret, shooter, feeder, conveyor, intake);
    }

    public Command driveHider(){
            return drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed.getAsDouble()) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed.getAsDouble()) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate.getAsDouble()) // Drive counterclockwise with negative X (left)
            );
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            driveHider()
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

        joystick.R1().whileTrue(drivetrain.applyRequest(() -> brake));

        // Reset the field-centric heading on left bumper press.
        joystick.cross().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command GetAutonCommand(){
        return this.autoContainer.getAutonomousCommand();
    }
}