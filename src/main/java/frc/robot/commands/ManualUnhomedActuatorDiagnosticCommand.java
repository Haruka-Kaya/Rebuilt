package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants.HardwareTestConstants;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.constants.Constants.ShooterConstants;
import frc.robot.constants.Constants.TurretConstants;
import frc.robot.diagnostics.HardwareDiagnosticEvaluator;
import frc.robot.diagnostics.HardwareDiagnosticEvaluator.MotionResult;
import frc.robot.diagnostics.HardwareDiagnosticEvaluator.Snapshot;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.AsyncDiagnosticSink;
import frc.robot.utils.SparkMAXContainer;
import frc.robot.utils.SparkMAXContainer.TimedDiagnosticSnapshot;

/**
 * One manually held polarity pulse for an actuator whose mechanical reference is not configured.
 * This command never creates or updates a position-reference token.
 */
public final class ManualUnhomedActuatorDiagnosticCommand extends Command {
  public static final String PREFIX = "Unhomed Actuator Diagnostic/";
  public static final String ARM_KEY = PREFIX + "Armed";
  public static final String ARM_VALID_KEY = PREFIX + "Arm Valid";
  public static final String PHYSICAL_CLEARANCE_KEY = PREFIX + "Physical Clearance Verified";
  public static final String MOTOR_TYPE_VERIFIED_KEY = PREFIX + "Brushless Motor Type Verified";
  public static final String TARGET_INTAKE_KEY = PREFIX + "Target ID30 Intake";
  public static final String TARGET_SHOOTER_KEY = PREFIX + "Target ID38 Shooter";
  public static final String TARGET_TURRET_KEY = PREFIX + "Target ID39 Turret";
  public static final String DIRECTION_NEGATIVE_KEY = PREFIX + "Direction Negative";
  public static final String DIRECTION_POSITIVE_KEY = PREFIX + "Direction Positive";
  public static final String STATUS_KEY = PREFIX + "Status";
  public static final String STOP_EVIDENCE_KEY = PREFIX + "Stop Evidence";

  public enum Target {
    INTAKE_ACTUATOR(IntakeConstants.INTAKE_ACTUATOR_CAN_ID, "ID30_INTAKE_ACTUATOR"),
    SHOOTER_ACTUATOR(ShooterConstants.ACTUATOR_CAN_ID, "ID38_SHOOTER_ACTUATOR"),
    TURRET(TurretConstants.TURRET_CAN_ID, "ID39_TURRET");

    private final int canId;
    private final String label;

    Target(int canId, String label) {
      this.canId = canId;
      this.label = label;
    }

    public int canId() {
      return canId;
    }

    public String label() {
      return label;
    }
  }

  /** Direction is selected and snapshotted while Disabled, never inferred after motion starts. */
  public enum Direction {
    NEGATIVE(-1.0),
    POSITIVE(1.0);

    private final double sign;

    Direction(double sign) {
      this.sign = sign;
    }

    public double duty() {
      return sign * HardwareTestConstants.UNHOMED_DIAGNOSTIC_MAX_DUTY_CYCLE;
    }
  }

  private enum Phase {
    PRE_STOP,
    PULSE,
    POST_STOP,
    DONE
  }

  private final Target target;
  private final double requestedDuty;
  private final BooleanSupplier interlocksHeld;
  private final BooleanSupplier runAction;
  private final Runnable stopAction;

  private Phase phase = Phase.DONE;
  private SparkMAXContainer.OutputStopBatch stopBatch;
  private double phaseDeadlineSeconds;
  private MotionResult latestMotionResult = MotionResult.NOT_RUN;
  private boolean motionObserved;
  private String terminalStatus = MotionResult.NOT_RUN.name();
  private double pulseCommandCompletedAt = Double.NaN;
  private double lastEvaluatedSampleAt = Double.NEGATIVE_INFINITY;
  private long pulseOutputEpoch = -1;

  public ManualUnhomedActuatorDiagnosticCommand(
      Target target,
      double requestedDuty,
      BooleanSupplier interlocksHeld,
      IntakeSubsystem intake,
      ShooterSubsystem shooter,
      TurretSubsystem turret,
      Subsystem... exclusiveRequirements) {
    if (target == null
        || !Double.isFinite(requestedDuty)
        || Math.abs(requestedDuty) <= 0.0
        || Math.abs(requestedDuty) > HardwareTestConstants.UNHOMED_DIAGNOSTIC_MAX_DUTY_CYCLE) {
      throw new IllegalArgumentException("manual unhomed diagnostic request is invalid");
    }
    this.target = target;
    this.requestedDuty = requestedDuty;
    this.interlocksHeld = interlocksHeld;

    switch (target) {
      case INTAKE_ACTUATOR -> {
        runAction = () -> intake.runUnhomedActuatorDiagnostic(requestedDuty);
        stopAction = intake::stopActuatorDiagnostic;
      }
      case SHOOTER_ACTUATOR -> {
        runAction = () -> shooter.runUnhomedActuatorDiagnostic(requestedDuty);
        stopAction = shooter::stopActuatorDiagnostic;
      }
      case TURRET -> {
        runAction = () -> turret.runUnhomedDiagnostic(requestedDuty);
        stopAction = turret::stopUnhomedDiagnostic;
      }
      default -> throw new IllegalStateException("unhandled diagnostic target " + target);
    }

    addRequirements(exclusiveRequirements);
    setName("ManualUnhomedDiagnostic-" + target.label());
  }

  public static void initializeDashboard() {
    SmartDashboard.putBoolean(ARM_KEY, false);
    SmartDashboard.putBoolean(ARM_VALID_KEY, false);
    SmartDashboard.putBoolean(PHYSICAL_CLEARANCE_KEY, false);
    SmartDashboard.putBoolean(MOTOR_TYPE_VERIFIED_KEY, false);
    SmartDashboard.putBoolean(TARGET_INTAKE_KEY, false);
    SmartDashboard.putBoolean(TARGET_SHOOTER_KEY, false);
    SmartDashboard.putBoolean(TARGET_TURRET_KEY, false);
    SmartDashboard.putBoolean(DIRECTION_NEGATIVE_KEY, false);
    SmartDashboard.putBoolean(DIRECTION_POSITIVE_KEY, false);
    SmartDashboard.putString(STATUS_KEY, "DISARMED");
    SmartDashboard.putString(STOP_EVIDENCE_KEY, "NOT_RUN");
  }

  /** Returns a target only when exactly one dashboard selection is active. */
  public static Optional<Target> readExactlyOneTarget() {
    boolean intake = SmartDashboard.getBoolean(TARGET_INTAKE_KEY, false);
    boolean shooter = SmartDashboard.getBoolean(TARGET_SHOOTER_KEY, false);
    boolean turret = SmartDashboard.getBoolean(TARGET_TURRET_KEY, false);
    int count = (intake ? 1 : 0) + (shooter ? 1 : 0) + (turret ? 1 : 0);
    if (count != 1) {
      return Optional.empty();
    }
    if (intake) {
      return Optional.of(Target.INTAKE_ACTUATOR);
    }
    if (shooter) {
      return Optional.of(Target.SHOOTER_ACTUATOR);
    }
    return Optional.of(Target.TURRET);
  }

  /** Returns a direction only when exactly one Disabled-mode direction selection is active. */
  public static Optional<Direction> readExactlyOneDirection() {
    boolean negative = SmartDashboard.getBoolean(DIRECTION_NEGATIVE_KEY, false);
    boolean positive = SmartDashboard.getBoolean(DIRECTION_POSITIVE_KEY, false);
    if (negative == positive) {
      return Optional.empty();
    }
    return Optional.of(negative ? Direction.NEGATIVE : Direction.POSITIVE);
  }

  @Override
  public void initialize() {
    latestMotionResult = MotionResult.NOT_RUN;
    motionObserved = false;
    terminalStatus = MotionResult.NOT_RUN.name();
    pulseCommandCompletedAt = Double.NaN;
    lastEvaluatedSampleAt = Double.NEGATIVE_INFINITY;
    pulseOutputEpoch = -1;
    stopAction.run();
    stopBatch = SparkMAXContainer.requestOutputStops(target.canId());
    phase = Phase.PRE_STOP;
    phaseDeadlineSeconds = Timer.getFPGATimestamp()
        + HardwareTestConstants.STOP_CONFIRM_TIMEOUT_SECONDS;
    publish("PRE_STOP_PENDING", "PRE_STOP_PENDING");
  }

  @Override
  public void execute() {
    if (phase == Phase.DONE) {
      return;
    }
    if (phase != Phase.POST_STOP && !interlocksHeld.getAsBoolean()) {
      beginPostStop("INTERLOCK_RELEASED");
      return;
    }

    double now = Timer.getFPGATimestamp();
    switch (phase) {
      case PRE_STOP -> executePreStop(now);
      case PULSE -> executePulse(now);
      case POST_STOP -> executePostStop(now);
      case DONE -> {
        // Handled above.
      }
      default -> throw new IllegalStateException("unhandled diagnostic phase " + phase);
    }
  }

  private void executePreStop(double now) {
    SparkMAXContainer.OutputStopSnapshot stop = stopBatch.snapshot();
    if (stop.confirmed()) {
      SmartDashboard.putString(STOP_EVIDENCE_KEY, "PRE_STOP_CONFIRMED");
      Snapshot readySnapshot = timedSnapshot(false).map(TimedDiagnosticSnapshot::snapshot)
          .orElse(null);
      if (readySnapshot == null || !readySnapshot.ready()) {
        latestMotionResult = MotionResult.FAIL_NOT_READY;
        beginPostStop(latestMotionResult.name());
        return;
      }
      boolean accepted = runAction.getAsBoolean();
      if (!accepted) {
        latestMotionResult = MotionResult.FAIL_COMMAND_REJECTED;
        beginPostStop(latestMotionResult.name());
        return;
      }
      TimedDiagnosticSnapshot sent = timedSnapshot(true).orElse(null);
      if (sent == null) {
        latestMotionResult = MotionResult.FAIL_NOT_READY;
        beginPostStop(latestMotionResult.name());
        return;
      }
      pulseOutputEpoch = sent.currentOutputEpoch();
      pulseCommandCompletedAt = Timer.getFPGATimestamp();
      phase = Phase.PULSE;
      phaseDeadlineSeconds = now + HardwareTestConstants.UNHOMED_DIAGNOSTIC_PULSE_SECONDS;
      publish("PULSING_" + signedDirection(), "PRE_STOP_CONFIRMED_PULSE_ACTIVE");
      return;
    }
    if (now >= phaseDeadlineSeconds) {
      latestMotionResult = MotionResult.FAIL_NOT_READY;
      beginPostStop("PRE_STOP_UNCONFIRMED_" + stop.summary());
    }
  }

  private void executePulse(double now) {
    TimedDiagnosticSnapshot timed = timedSnapshot(true).orElse(null);
    if (timed == null || timed.currentOutputEpoch() != pulseOutputEpoch) {
      latestMotionResult = MotionResult.FAIL_NOT_READY;
      beginPostStop(latestMotionResult.name());
      return;
    }

    // Status frames are cached asynchronously. Do not mistake the pre-command zero frame for a
    // rejected duty-cycle command; only a frame sampled after the command completed is evidence.
    if (!isCurrentPostCommandSample(
        timed, pulseOutputEpoch, pulseCommandCompletedAt, lastEvaluatedSampleAt)) {
      if (now >= phaseDeadlineSeconds) {
        if (Double.isFinite(lastEvaluatedSampleAt)) {
          beginPostStop(
              (motionObserved ? MotionResult.PASS_OBSERVED : latestMotionResult).name());
        } else {
          latestMotionResult = MotionResult.FAIL_NOT_READY;
          beginPostStop("NO_POST_COMMAND_SAMPLE");
        }
      }
      return;
    }
    lastEvaluatedSampleAt = timed.sampledAtSeconds();
    Snapshot snapshot = timed.snapshot();
    MotionResult sample = HardwareDiagnosticEvaluator.evaluateOpenLoop(
        snapshot,
        requestedDuty,
        HardwareTestConstants.UNHOMED_DIAGNOSTIC_MAX_CURRENT_AMPS);
    latestMotionResult = sample;
    if (sample == MotionResult.PASS_OBSERVED) {
      motionObserved = true;
      beginPostStop(MotionResult.PASS_OBSERVED.name());
      return;
    } else if (sample == MotionResult.FAIL_NOT_READY
        || sample == MotionResult.FAIL_COMMAND_REJECTED
        || sample == MotionResult.FAIL_DIRECTION_MISMATCH
        || sample == MotionResult.STALL_SUSPECTED) {
      beginPostStop(sample.name());
      return;
    }

    if (now >= phaseDeadlineSeconds) {
      beginPostStop((motionObserved ? MotionResult.PASS_OBSERVED : latestMotionResult).name());
    }
  }

  private Optional<TimedDiagnosticSnapshot> timedSnapshot(boolean commandAccepted) {
    return SparkMAXContainer.getTimedDiagnosticSnapshotForId(target.canId(), commandAccepted);
  }

  static boolean isCurrentPostCommandSample(
      TimedDiagnosticSnapshot sample,
      long expectedOutputEpoch,
      double commandCompletedAt,
      double lastEvaluatedAt) {
    return sample != null
        && sample.sampleOutputEpoch() == expectedOutputEpoch
        && sample.currentOutputEpoch() == expectedOutputEpoch
        && Double.isFinite(sample.sampledAtSeconds())
        && Double.isFinite(commandCompletedAt)
        && sample.sampledAtSeconds() > commandCompletedAt
        && sample.sampledAtSeconds() > lastEvaluatedAt;
  }

  private void beginPostStop(String result) {
    if (phase == Phase.POST_STOP || phase == Phase.DONE) {
      return;
    }
    stopAction.run();
    terminalStatus = result;
    stopBatch = SparkMAXContainer.requestOutputStops(target.canId());
    phase = Phase.POST_STOP;
    phaseDeadlineSeconds = Timer.getFPGATimestamp()
        + HardwareTestConstants.STOP_CONFIRM_TIMEOUT_SECONDS;
    publish(result + "_POST_STOP_PENDING", "POST_STOP_PENDING");
  }

  private void executePostStop(double now) {
    SparkMAXContainer.OutputStopSnapshot stop = stopBatch.snapshot();
    if (stop.confirmed()) {
      phase = Phase.DONE;
      publish(terminalStatus + "_STOP_CONFIRMED", "POST_STOP_CONFIRMED");
      AsyncDiagnosticSink.log(
          "UNHOMED DIAGNOSTIC target=" + target.label() + " duty=" + requestedDuty
              + " result=" + terminalStatus + " stop=CONFIRMED");
      return;
    }
    if (now >= phaseDeadlineSeconds) {
      phase = Phase.DONE;
      publish(terminalStatus + "_STOP_UNCONFIRMED", "POST_STOP_UNCONFIRMED_"
          + stop.summary());
    }
  }

  @Override
  public boolean isFinished() {
    return phase == Phase.DONE;
  }

  @Override
  public void end(boolean interrupted) {
    stopAction.run();
    if (interrupted || phase != Phase.DONE) {
      SparkMAXContainer.requestOutputStops(target.canId());
      phase = Phase.DONE;
      publish("INTERRUPTED_STOP_REQUESTED", "STOP_REQUESTED");
      AsyncDiagnosticSink.log(
          "UNHOMED DIAGNOSTIC target=" + target.label() + " interrupted stop=REQUESTED");
    }
  }

  private String signedDirection() {
    return requestedDuty < 0.0 ? "NEGATIVE" : "POSITIVE";
  }

  private void publish(String status, String stopEvidence) {
    SmartDashboard.putString(STATUS_KEY, target.label() + " " + status);
    SmartDashboard.putString(STOP_EVIDENCE_KEY, target.label() + " " + stopEvidence);
  }
}
