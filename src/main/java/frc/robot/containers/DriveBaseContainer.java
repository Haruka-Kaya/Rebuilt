package frc.robot.containers;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Telemetry;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.Constants.DebugConstants;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.NeutralAfterEnableGate;

public class DriveBaseContainer {
    public AutoContainer autoContainer;
    public static double speedFactor = .05;
    public static double rotationFactor = .05;
    
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
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed.getAsDouble() * OIConstants.kDriveDeadband)
            .withRotationalDeadband(MaxAngularRate.getAsDouble() * OIConstants.kDriveDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();
    private final NeutralAfterEnableGate driveInputGate = new NeutralAfterEnableGate();
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
        SmartDashboard.putBoolean("Swerve Output Enabled", DebugConstants.ALLOW_SWERVE_OUTPUT);

        SmartDashboard.putString("MESSAGE", "we are at autoSetup");

        // this.m_turret = turret;
        // this.m_shooter = shooter;
        // this.m_feeder = feeder;
        // this.m_conveyor = conveyor;
        // this.m_intake = intake;

        autoContainer = new AutoContainer(drivetrain, turret, shooter, feeder, conveyor, intake);
    }

    public Command driveHider(){
            return drivetrain.applyRequest(() -> {
                if (!driveInputsAllowed()) {
                    return idle;
                }
                double velocityX = -availableAxis(1) * MaxSpeed.getAsDouble();
                double velocityY = -availableAxis(0) * MaxSpeed.getAsDouble();
                double rotation = -availableAxis(2) * MaxAngularRate.getAsDouble();
                if (drivetrain.isGyroConnected()) {
                    return drive.withVelocityX(velocityX)
                        .withVelocityY(velocityY)
                        .withRotationalRate(rotation);
                }
                return robotCentricDrive.withVelocityX(velocityX)
                    .withVelocityY(velocityY)
                    .withRotationalRate(rotation);
            });
    }

    private double availableAxis(int axis) {
        return DriverStation.getStickAxisCount(OIConstants.kDriverControllerPort) > axis
            ? joystick.getHID().getRawAxis(axis)
            : 0.0;
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
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

        // Touchpad is reserved for wheel-lock so it does not conflict with R1/intake output.
        Trigger brakeButton = availableButton(14);
        brakeButton.whileTrue(drivetrain.applyRequest(() -> brake));

        // Create resets the field-centric heading without colliding with mechanism controls.
        availableButton(9)
            .and(brakeButton.negate())
            .and(availableButton(12).negate())
            .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private Trigger availableButton(int button) {
        return new Trigger(() -> driveInputsAllowed()
            && DriverStation.getStickButtonCount(OIConstants.kDriverControllerPort) >= button
            && joystick.getHID().getRawButton(button));
    }

    private boolean driveInputsAllowed() {
        boolean anyAxisActive = Math.abs(availableAxis(0)) > OIConstants.kDriveDeadband
            || Math.abs(availableAxis(1)) > OIConstants.kDriveDeadband
            || Math.abs(availableAxis(2)) > OIConstants.kDriveDeadband;
        boolean anyControlButton = rawButtonPressed(9)
            || rawButtonPressed(12)
            || rawButtonPressed(14);
        int sourceSignature = DriverStation.getStickButtonCount(
            OIConstants.kDriverControllerPort)
            | (DriverStation.getStickAxisCount(OIConstants.kDriverControllerPort) << 8);
        return driveInputGate.allow(
            DriverStation.isTeleopEnabled(), sourceSignature, anyAxisActive || anyControlButton);
    }

    private boolean rawButtonPressed(int button) {
        return DriverStation.getStickButtonCount(OIConstants.kDriverControllerPort) >= button
            && joystick.getHID().getRawButton(button);
    }

    public Command GetAutonCommand(){
        return this.autoContainer.getAutonomousCommand();
    }
}
