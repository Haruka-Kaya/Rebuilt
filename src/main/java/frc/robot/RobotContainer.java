// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooterFull;

public class RobotContainer {
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private final DriveSettings driveSettings = new DriveSettings(MaxSpeed, MaxAngularRate);
    private final NeutralSwerveRequest neutral = new NeutralSwerveRequest();

    private boolean invertControllerLeftX = true;
    private boolean invertControllerLeftY = false;


    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentric aimRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    @SuppressWarnings("unused") // Reason: This is used in the commented-out code below, but is left here for future use.
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt(); 

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    public final shooterFull shooter = new shooterFull();

    @SuppressWarnings("unused")
    private Command aimAtAprilTag() {
        return drivetrain.applyRequest(() -> {

            // 1. Check if a target is visible
            if (!LimelightHelpers.getTV("limelight")) {
                return aimRequest
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0);
            }

            // 2. Fetch Limelight values
            double tx = LimelightHelpers.getTX("limelight"); // Horizontal offset in degrees
            double ta = LimelightHelpers.getTA("limelight"); // Target area (0% to 100% of image)

            // 3. Define target parameters & proportional gains (kP)
            double targetArea = 3.0; // Desired area percentage when at the ideal distance (adjust for your setup)
            
            double kP_Rot = 0.03;   // Proportional gain for rotation
            double kP_Drive = 0.5;  // Proportional gain for forward/backward driving

            // 4. Calculate error and output speeds
            
            // Rotational output (Aiming)
            double omega = -tx * kP_Rot;
            omega = Math.max(-1.5, Math.min(1.5, omega)); // Clamp rotation speed

            if (Math.abs(tx) < 1.0) {
                omega = 0; // Deadband for rotation
            }

            // Forward/Backward output (Ranging)
            double areaError = targetArea - ta;
            double velocityX = areaError * kP_Drive; // Positive moves forward towards tag
            velocityX = Math.max(-MaxSpeed, Math.min(MaxSpeed, velocityX)); // Clamp drive speed

            if (Math.abs(areaError) < 0.2) {
                velocityX = 0; // Deadband for distance
            }

            // 5. Apply velocities to FieldCentric request
            // Note: Set velocityY to 0 unless you also want to strafe alignment
            return aimRequest
                    .withVelocityX(velocityX)
                    .withVelocityY(0)
                    .withRotationalRate(omega);
        });
    }

    @SuppressWarnings("unused") // Reason: This is used in the commented-out code below, but is left here for future use.
    private Command changeControllerInversionLeftCommand(boolean invertX, boolean invertY) {
        if (invertX) {
            invertControllerLeftX = !invertControllerLeftX;
        }
        if (invertY) {
            invertControllerLeftY = !invertControllerLeftY;
        }
        return Commands.runOnce(() -> {
            int invertLeftX = invertControllerLeftX ? -1 : 1;
            int invertLeftY = invertControllerLeftY ? -1 : 1;
            drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() ->
                    teleopRequest(invertLeftX, invertLeftY)
                )
            );
        });
    }



    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                teleopRequest(1, 1)
            )
        );

        // Explicitly replace drive and steer outputs with the configured
        // neutral mode; Phoenix Idle does not change the module state.
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> neutral).ignoringDisable(true)
        );

        joystick.a().and(RobotModeTriggers.teleop()).whileTrue(drivetrain.applyRequest(() -> brake));

        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).and(RobotModeTriggers.test()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).and(RobotModeTriggers.test()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).and(RobotModeTriggers.test()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).and(RobotModeTriggers.test()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().and(RobotModeTriggers.teleop()).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        joystick.rightBumper().onTrue(shooter.on());
        joystick.rightBumper().onFalse(shooter.off());

        // joystick.y().whileTrue(aimAtAprilTag());


        // joystick.povUp().onTrue(changeControllerInversionLeftCommand(true, false));
        // joystick.povDown().onTrue(changeControllerInversionLeftCommand(false, true));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private SwerveRequest teleopRequest(int invertLeftX, int invertLeftY) {
        if (!DriverStation.isTeleopEnabled()) {
            return neutral;
        }
        double speed = driveSettings.maxSpeedMps();
        double angularRate = driveSettings.maxAngularRateRadPerSec();
        return drive.withDeadband(speed * 0.1)
            .withRotationalDeadband(angularRate * 0.1)
            .withVelocityX(-joystick.getLeftY() * speed * invertLeftY)
            .withVelocityY(-joystick.getLeftX() * speed * invertLeftX)
            .withRotationalRate(-joystick.getRightX() * angularRate);
    }

    public Command getAutonomousCommand() {
        // Autoの速度は開始時に読み、不感帯のない専用requestを使う。
        return Commands.defer(() -> {
            var autoDrive = new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(driveSettings.autoSpeedMps())
                .withVelocityY(0)
                .withRotationalRate(0);
            return Commands.sequence(
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                drivetrain.applyRequest(() -> autoDrive).withTimeout(5.0),
                drivetrain.applyRequest(() -> neutral)
            );
        }, java.util.Set.of(drivetrain))
            .finallyDo(() -> drivetrain.setControl(neutral));
    }
}
