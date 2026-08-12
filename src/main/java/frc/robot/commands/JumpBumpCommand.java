package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import java.util.function.BooleanSupplier;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.constants.ConfiguredOperatorActions.Action;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain.ControlResult;
import frc.robot.utils.OperatorActionEvidence;

public class JumpBumpCommand extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final CommandPS5Controller m_controller; // may be used
    private final BooleanSupplier m_driverInputsAllowed;
    private final Runnable m_rearmDriverInputs;
    private final OperatorActionEvidence evidence;
    private boolean moduleFaulted;
    private String failureReason;

    private final HeadingSnapController m_rotationController = new HeadingSnapController();


    public JumpBumpCommand(
            CommandSwerveDrivetrain m_drive,
            CommandPS5Controller m_controller,
            BooleanSupplier driverInputsAllowed) {
        this(m_drive, m_controller, driverInputsAllowed, null);
    }

    public JumpBumpCommand(
            CommandSwerveDrivetrain m_drive,
            CommandPS5Controller m_controller,
            BooleanSupplier driverInputsAllowed,
            OperatorActionEvidence evidence) {
        this(m_drive, m_controller, driverInputsAllowed, () -> {}, evidence);
    }

    public JumpBumpCommand(
            CommandSwerveDrivetrain m_drive,
            CommandPS5Controller m_controller,
            BooleanSupplier driverInputsAllowed,
            Runnable rearmDriverInputs,
            OperatorActionEvidence evidence) {
        this.m_drivetrain = m_drive;
        this.m_controller = m_controller;
        this.m_driverInputsAllowed = driverInputsAllowed;
        this.m_rearmDriverInputs = rearmDriverInputs;
        this.evidence = evidence;

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        failureReason = null;
        if (evidence != null) {
            evidence.requested(Action.JUMP_BUMP);
        }
        // Always poll the shared gate so a device fault also forces a new neutral sample.
        boolean inputsAllowed = m_driverInputsAllowed.getAsBoolean();
        boolean devicesConnected = m_drivetrain.areAllDevicesConnected();
        moduleFaulted = !inputsAllowed || !devicesConnected;
        if (moduleFaulted) {
            fail(devicesConnected
                ? "RELEASE_TO_ARM_OR_INPUT_UNAVAILABLE"
                : "DRIVETRAIN_UNHEALTHY");
            m_drivetrain.requestIdle();
            return;
        }
        double currentHeading;
        try {
            currentHeading = m_drivetrain.getStateCopy().Pose.getRotation().getRadians();
        } catch (RuntimeException exception) {
            fail("DRIVETRAIN_STATE_READ_FAILED");
            m_drivetrain.requestIdle();
            return;
        }
        if (!m_rotationController.reset(
                currentHeading,
                HeadingSnapController.closestBumpHeading(currentHeading))) {
            fail("HEADING_RESET_REJECTED");
            m_drivetrain.requestIdle();
        }
    }

    @Override
    public void execute() {
        // Do not short-circuit this poll; it records fault/enable transitions in the shared gate.
        boolean inputsAllowed = m_driverInputsAllowed.getAsBoolean();
        boolean devicesConnected = m_drivetrain.areAllDevicesConnected();
        if (moduleFaulted) {
            m_drivetrain.requestIdle();
            return;
        }
        if (!inputsAllowed || !devicesConnected) {
            fail(devicesConnected
                ? "RELEASE_TO_ARM_OR_INPUT_UNAVAILABLE"
                : "DRIVETRAIN_UNHEALTHY");
            m_drivetrain.requestIdle();
            return;
        }
        Pose2d currentPose;
        try {
            currentPose = m_drivetrain.getStateCopy().Pose;
        } catch (RuntimeException exception) {
            fail("DRIVETRAIN_STATE_READ_FAILED");
            m_drivetrain.requestIdle();
            return;
        }
        double currentRotationRadians = currentPose.getRotation().getRadians();

        double rotationSpeed = m_rotationController.calculate(currentRotationRadians);
        if (!Double.isFinite(rotationSpeed)) {
            fail("ROTATION_CONTROLLER_INVALID_OUTPUT");
            m_drivetrain.requestIdle();
            return;
        }

        ControlResult result = m_drivetrain.drive(
            -MathUtil.applyDeadband(m_controller.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_controller.getLeftX(), OIConstants.kDriveDeadband),
            rotationSpeed,
            true
        );
        if (result == ControlResult.REQUEST_SUBMITTED) {
            if (evidence != null) {
                evidence.active(
                    Action.JUMP_BUMP,
                    "BUMP_HEADING_API_RETURNED_NOT_REQUEST_OR_MOTION_PROOF");
            }
        } else {
            fail("SWERVE_" + result.name());
            m_drivetrain.requestIdle();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.requestIdle();
        if (evidence != null && failureReason == null) {
            evidence.commandEnded(Action.JUMP_BUMP, interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        return moduleFaulted;
    }

    private void fail(String reason) {
        moduleFaulted = true;
        failureReason = reason;
        m_rearmDriverInputs.run();
        if (evidence != null) {
            evidence.blocked(Action.JUMP_BUMP, reason);
        }
    }
}
