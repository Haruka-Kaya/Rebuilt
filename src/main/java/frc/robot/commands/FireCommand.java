package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Telemetry;
import frc.robot.constants.ConfiguredOperatorActions.Action;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.OperatorActionEvidence;
import java.util.stream.Collectors;
import java.util.List;

public class FireCommand extends Command {
    private final FeederSubsystem m_feeder;
    private final ConveyorSubsystem m_conveyer;
    private final ShooterSubsystem m_shooter;
    private final OperatorActionEvidence evidence;

    public FireCommand(
            FeederSubsystem feeder,
            ConveyorSubsystem conveyor,
            ShooterSubsystem shooter) {
        this(feeder, conveyor, shooter, null);
    }

    public FireCommand(
            FeederSubsystem feeder,
            ConveyorSubsystem conveyor,
            ShooterSubsystem shooter,
            OperatorActionEvidence evidence) {
        this.m_feeder = feeder;
        this.m_conveyer = conveyor;
        this.m_shooter = shooter;
        this.evidence = evidence;
        addRequirements(feeder, conveyor);
    }

    @Override
    public void initialize() {
        if (evidence != null) {
            evidence.requested(Action.FIRE);
        }
        feedOnlyWhenShooterIsReady();
    }

    @Override
    public void execute() {
        feedOnlyWhenShooterIsReady();
    }

    @Override
    public void end(boolean interrupted) {
        m_feeder.stop();
        m_conveyer.stop();
        if (evidence != null) {
            evidence.commandEnded(Action.FIRE, interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void feedOnlyWhenShooterIsReady() {
        boolean feederKnownStall = m_feeder.isMotionBlockedByKnownStall();
        ShotReleaseInterlock.Decision release = ShotReleaseInterlock.evaluate(
            m_shooter.isReadyToFeed(),
            m_conveyer.isReady(),
            m_feeder.isReady(),
            Telemetry.isHubActive(),
            true);
        if (!release.allowed()) {
            publishBlocked(releaseBlockReasons(release, feederKnownStall));
            stopFeedPath();
            return;
        }

        boolean conveyorStarted = m_conveyer.runConveyor();
        boolean feederStarted = conveyorStarted && m_feeder.feed();
        if (conveyorStarted && feederStarted) {
            if (evidence != null) {
                evidence.active(Action.FIRE, "CONVEYOR_AND_FEEDER_COMMANDS_ACCEPTED");
            }
            return;
        }

        publishBlocked(conveyorStarted
            ? "FEEDER_COMMAND_REJECTED_FEED_PATH_STOP_REQUESTED"
            : "CONVEYOR_COMMAND_REJECTED");
        stopFeedPath();
    }

  static String releaseBlockReason(
            ShotReleaseInterlock.Decision release,
            boolean feederKnownStall) {
        return releaseBlockReasons(release, feederKnownStall).stream()
            .collect(Collectors.joining("+"));
    }

    static List<String> releaseBlockReasons(
            ShotReleaseInterlock.Decision release,
            boolean feederKnownStall) {
        return release.blockers().stream()
            .map(reason -> feederKnownStall && reason == ShotReleaseInterlock.Reason.FEEDER_NOT_READY
                ? "FEEDER_KNOWN_STALL"
                : reason.name())
            .toList();
    }

    private void publishBlocked(String reason) {
        if (evidence != null) {
            evidence.blocked(Action.FIRE, reason);
        }
    }

    private void publishBlocked(List<String> reasons) {
        if (evidence != null) {
            evidence.blocked(Action.FIRE, reasons);
        }
    }

    private void stopFeedPath() {
        m_feeder.stop();
        m_conveyer.stop();
    }
}
