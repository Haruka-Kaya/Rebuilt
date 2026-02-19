package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class JumpBumpCommand extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final CommandPS5Controller m_controller; // may be used

    private final ProfiledPIDController m_rotationController = new ProfiledPIDController(4.0, 0.0, 0.0,
            new TrapezoidProfile.Constraints(
                    Math.PI * 4, // Max velocity (rad/s)
                    Math.PI * 8 // Max acceleration (rad/s^2)
            ));


    public JumpBumpCommand(CommandSwerveDrivetrain m_drive, CommandPS5Controller m_controller){
        this.m_drivetrain = m_drive;
        this.m_controller = m_controller;

        m_rotationController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        Pose2d currentPose = m_drivetrain.getState().Pose;
        double currentRotationRadians = currentPose.getRotation().getRadians();

        double step = Math.PI / 4.0;

        double targetAngle = Math.round(currentRotationRadians / step) * step;

        double rotationSpeed = m_rotationController.calculate(
            currentRotationRadians,
            targetAngle
        );

        m_drivetrain.drive(
            -MathUtil.applyDeadband(m_controller.getLeftX(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_controller.getLeftY(), OIConstants.kDriveDeadband), 
            rotationSpeed,
            true
        );
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
