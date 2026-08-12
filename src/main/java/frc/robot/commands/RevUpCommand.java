package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ConfiguredOperatorActions.Action;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.OperatorActionEvidence;
import frc.robot.utils.SparkMAXContainer.PositionCommandStatus;
import java.util.List;
import java.util.Objects;

public class RevUpCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final OperatorActionEvidence evidence;

    public RevUpCommand(ShooterSubsystem shooter) {
        this(shooter, null);
    }

    public RevUpCommand(ShooterSubsystem shooter, OperatorActionEvidence evidence) {
        this.m_shooter = shooter;
        this.evidence = evidence;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        if (evidence != null) {
            evidence.requested(Action.REV);
        }
        prepare();
    }

    @Override
    public void execute() {
        prepare();
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        if (evidence != null) {
            evidence.commandEnded(Action.REV, interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void prepare() {
        ShooterSubsystem.PreparationStatus status = m_shooter.prepareToFire();
        if (evidence == null) {
            return;
        }
        PreparationEvidence preparationEvidence = classifyPreparation(status);
        if (preparationEvidence.blocked()) {
            evidence.blocked(Action.REV, preparationEvidence.reasons());
        } else {
            evidence.active(Action.REV, preparationEvidence.reasons());
        }
    }

    static PreparationEvidence classifyPreparation(ShooterSubsystem.PreparationStatus status) {
        Objects.requireNonNull(status, "status");
        PositionCommandStatus actuatorStatus = status.actuatorPositionStatus();
        boolean hoodAccepted = actuatorStatus != PositionCommandStatus.REJECTED;
        if (!status.flywheelAccepted() && !hoodAccepted) {
            return PreparationEvidence.blocked(
                "FLYWHEEL_PAIR_COMMAND_REJECTED",
                status.actuatorReferenced() ? "HOOD_COMMAND_REJECTED" : "HOOD_UNREFERENCED",
                "NO_SHOT_PREPARATION_COMMAND_ACCEPTED");
        }
        if (!status.flywheelAccepted()) {
            return PreparationEvidence.active(
                "HOOD_POSITION_COMMAND_ACCEPTED_NOT_MOTION_PROOF",
                "FLYWHEEL_PAIR_COMMAND_REJECTED");
        }
        if (actuatorStatus == PositionCommandStatus.REJECTED) {
            return PreparationEvidence.active(
                status.actuatorReferenced()
                    ? "FLYWHEEL_COMMAND_ACCEPTED_HOOD_COMMAND_REJECTED"
                    : "FLYWHEEL_COMMAND_ACCEPTED_HOOD_UNREFERENCED");
        }
        if (actuatorStatus == PositionCommandStatus.MOVING) {
            return PreparationEvidence.active(
                "FLYWHEEL_AND_HOOD_COMMANDS_ACCEPTED_HOOD_NOT_AT_TARGET_NOT_MOTION_PROOF");
        }
        if (!status.flywheelReady()) {
            return PreparationEvidence.active(
                "FLYWHEEL_COMMAND_ACCEPTED_SPEED_NOT_READY_NOT_MOTION_PROOF");
        }
        return PreparationEvidence.active("SHOT_PREPARATION_READY");
    }

    record PreparationEvidence(boolean blocked, List<String> reasons) {
        PreparationEvidence {
            reasons = List.copyOf(Objects.requireNonNull(reasons, "reasons"));
            if (reasons.isEmpty()) {
                throw new IllegalArgumentException("at least one evidence reason is required");
            }
        }

        static PreparationEvidence blocked(String... reasons) {
            return new PreparationEvidence(true, List.of(reasons));
        }

        static PreparationEvidence active(String... reasons) {
            return new PreparationEvidence(false, List.of(reasons));
        }
    }
}
