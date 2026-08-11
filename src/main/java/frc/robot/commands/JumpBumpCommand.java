package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class JumpBumpCommand extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final CommandPS5Controller m_controller; // may be used
    private boolean moduleFaulted;

    private final HeadingSnapController m_rotationController = new HeadingSnapController();


    public JumpBumpCommand(CommandSwerveDrivetrain m_drive, CommandPS5Controller m_controller){
        this.m_drivetrain = m_drive;
        this.m_controller = m_controller;

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        moduleFaulted = !m_drivetrain.areAllDevicesConnected();
        if (moduleFaulted) {
            m_drivetrain.requestIdle();
            return;
        }
        if (!m_rotationController.reset(
                m_drivetrain.getState().Pose.getRotation().getRadians())) {
            moduleFaulted = true;
            m_drivetrain.requestIdle();
        }
    }

    @Override
    public void execute() {
        if (moduleFaulted || !m_drivetrain.areAllDevicesConnected()) {
            moduleFaulted = true;
            m_drivetrain.requestIdle();
            return;
        }
        Pose2d currentPose = m_drivetrain.getState().Pose;
        double currentRotationRadians = currentPose.getRotation().getRadians();

        double rotationSpeed = m_rotationController.calculate(currentRotationRadians);
        if (!Double.isFinite(rotationSpeed)) {
            moduleFaulted = true;
            m_drivetrain.requestIdle();
            return;
        }

        m_drivetrain.drive(
            -MathUtil.applyDeadband(m_controller.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_controller.getLeftX(), OIConstants.kDriveDeadband),
            rotationSpeed,
            true
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.requestIdle();
    }

    @Override
    public boolean isFinished() {
        return moduleFaulted;
    }
}
