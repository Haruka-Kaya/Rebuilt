package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import java.util.function.BooleanSupplier;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class JumpBumpCommand extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final CommandPS5Controller m_controller; // may be used
    private final BooleanSupplier m_driverInputsAllowed;
    private boolean moduleFaulted;

    private final HeadingSnapController m_rotationController = new HeadingSnapController();


    public JumpBumpCommand(
            CommandSwerveDrivetrain m_drive,
            CommandPS5Controller m_controller,
            BooleanSupplier driverInputsAllowed) {
        this.m_drivetrain = m_drive;
        this.m_controller = m_controller;
        this.m_driverInputsAllowed = driverInputsAllowed;

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        // Always poll the shared gate so a device fault also forces a new neutral sample.
        boolean inputsAllowed = m_driverInputsAllowed.getAsBoolean();
        moduleFaulted = !inputsAllowed || !m_drivetrain.areAllDevicesConnected();
        if (moduleFaulted) {
            m_drivetrain.requestIdle();
            return;
        }
        double currentHeading;
        try {
            currentHeading = m_drivetrain.getStateCopy().Pose.getRotation().getRadians();
        } catch (RuntimeException exception) {
            moduleFaulted = true;
            m_drivetrain.requestIdle();
            return;
        }
        if (!m_rotationController.reset(
                currentHeading,
                HeadingSnapController.closestBumpHeading(currentHeading))) {
            moduleFaulted = true;
            m_drivetrain.requestIdle();
        }
    }

    @Override
    public void execute() {
        // Do not short-circuit this poll; it records fault/enable transitions in the shared gate.
        boolean inputsAllowed = m_driverInputsAllowed.getAsBoolean();
        if (moduleFaulted
                || !inputsAllowed
                || !m_drivetrain.areAllDevicesConnected()) {
            moduleFaulted = true;
            m_drivetrain.requestIdle();
            return;
        }
        Pose2d currentPose;
        try {
            currentPose = m_drivetrain.getStateCopy().Pose;
        } catch (RuntimeException exception) {
            moduleFaulted = true;
            m_drivetrain.requestIdle();
            return;
        }
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
