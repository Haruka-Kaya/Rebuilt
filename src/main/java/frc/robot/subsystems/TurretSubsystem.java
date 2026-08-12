package frc.robot.subsystems;


import java.util.OptionalDouble;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.TurretConstants;
import frc.robot.constants.Constants.HardwareTestConstants;
import frc.robot.diagnostics.HardwareDiagnosticEvaluator.Snapshot;
import frc.robot.subsystems.VisionSubsystem.TargetObservation;
import frc.robot.utils.DashboardApplyGate;
import frc.robot.utils.DashboardApplyGate.Decision;
import frc.robot.utils.PositionReferenceGuard.Token;
import frc.robot.utils.SparkMAXContainer;
import frc.robot.utils.SparkMAXContainer.PositionCommandStatus;
import frc.robot.utils.HubTagFilter;

public class TurretSubsystem extends SubsystemBase {
    private static final String TUNING_APPLY_KEY = "Tuning/Turret/Apply";
    private static final String TUNING_STATUS_KEY = "Tuning/Turret/Status";
    private static final double MAX_PID_GAIN = 10.0;

    private final SparkMAXContainer m_motor = new SparkMAXContainer(TurretConstants.TURRET_CAN_ID);
    private final DashboardApplyGate tuningApplyGate = new DashboardApplyGate();
    // Assigned only after future, sensor-validated homing or absolute reference succeeds.
    private Token positionReference;
    private boolean unhomedDiagnosticActive;
    private String lastBlockedPositionCommand = "startup: homing/absolute reference未実装";

    private final VisionSubsystem m_vision;

    private double turret_kP = 2.4;
    private double turret_kI = 0.0;
    private double turret_kD = 0.1;
    private double lastAimFrameTimestamp = -1.0;
    private int alignedFrameCount = 0;

    private boolean onTarget = false;

    public TurretSubsystem(VisionSubsystem vision) {
        this.m_vision = vision;

        m_motor.setBreakMode(true);
        m_motor.setCurrentLimit(15);
        m_motor.assignPIDValues(turret_kP, turret_kI, turret_kD);
        m_motor.setMaxSpeed(TurretConstants.MAX_CLOSED_LOOP_OUTPUT);
        
        SmartDashboard.putNumber("Set turret_kP", 2.4);
        SmartDashboard.putNumber("Set turret_kI", 0);
        SmartDashboard.putNumber("Set turret_kD", 0.1);
        SmartDashboard.putBoolean(TUNING_APPLY_KEY, false);
        SmartDashboard.putString(TUNING_STATUS_KEY, "ACTIVE_DEFAULTS");
    }

    public boolean setTurretAngle(double angleDegrees) {
        return commandTurretAngle(angleDegrees) == PositionCommandStatus.AT_TARGET;
    }

    public PositionCommandStatus commandTurretAngle(double angleDegrees) {
        if (!Double.isFinite(angleDegrees)
            || angleDegrees < TurretConstants.MIN_ANGLE_DEGREES
            || angleDegrees > TurretConstants.MAX_ANGLE_DEGREES) {
            lastBlockedPositionCommand = "angle outside configured limits";
            stop();
            return PositionCommandStatus.REJECTED;
        }
        if (!m_motor.isPositionReferenceValid(positionReference)) {
            lastBlockedPositionCommand = "set angle: UNREFERENCED";
            stop();
            return PositionCommandStatus.REJECTED;
        }
        // convert turret degrees -> motor rotations before commanding
        PositionCommandStatus status = m_motor.commandReferencedPosition(
            degreesToMotorRotations(angleDegrees),
            degreesToMotorRotations(TurretConstants.AIM_DEADBAND_DEG),
            positionReference);
        if (status == PositionCommandStatus.REJECTED) {
            lastBlockedPositionCommand = "set angle: command rejected";
        }
        return status;
    }

    public void stop() {
        unhomedDiagnosticActive = false;
        stopMotorOutput();
        onTarget = false;
        alignedFrameCount = 0;
    }

    private void stopMotorOutput() {
        m_motor.stop();
    }

    /** Test-only low-output polarity evidence; this does not establish an angular reference. */
    public boolean runUnhomedDiagnostic(double requestedDuty) {
        if (!unhomedDiagnosticAllowed(requestedDuty)) {
            stopUnhomedDiagnostic();
            return false;
        }
        unhomedDiagnosticActive = true;
        boolean accepted = m_motor.setDutyCycle(requestedDuty);
        if (!accepted) {
            stopUnhomedDiagnostic();
        }
        return accepted;
    }

    public Snapshot getUnhomedDiagnosticSnapshot(boolean commandAccepted) {
        return m_motor.getDiagnosticSnapshot(commandAccepted);
    }

    public void stopUnhomedDiagnostic() {
        unhomedDiagnosticActive = false;
        m_motor.stop();
        onTarget = false;
        alignedFrameCount = 0;
    }

    private double degreesToMotorRotations(double degrees) {
        // turret degrees -> turret rotations -> motor rotations
        // turret rotations = degrees / 360
        // motor rotations = turret rotations * GEAR_RATIO
        return (degrees / 360.0) * TurretConstants.GEAR_RATIO;
    }

    public OptionalDouble getTurretAngle() {
        OptionalDouble motorRotations = m_motor.getReferencedPosition(positionReference);
        if (motorRotations.isEmpty()) {
            return OptionalDouble.empty();
        }
        // motor rotations -> turret degrees
        return OptionalDouble.of(motorRotationsToTurretDegrees(motorRotations.getAsDouble()));
    }

    private double motorRotationsToTurretDegrees(double motorRotations) {
        // turret rotations = motorRotations / GEAR_RATIO
        // degrees = turret rotations * 360
        return (motorRotations / TurretConstants.GEAR_RATIO) * 360.0;
    }
    
    public AimStatus autoAimWithLimelight() {
        if (!m_motor.isPositionReferenceValid(positionReference)) {
            lastBlockedPositionCommand = "auto aim: UNREFERENCED";
            stop();
            return AimStatus.UNREFERENCED;
        }

        var observation = m_vision.getLatestTargetObservation();
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            stop();
            return AimStatus.ALLIANCE_UNKNOWN;
        }
        if (observation.isEmpty()) {
            stop();
            return m_vision.isTargetingPipelineReady()
                ? AimStatus.NO_VALID_TARGET
                : AimStatus.VISION_NOT_READY;
        }

        TargetObservation target = observation.get();
        if (!HubTagFilter.isHubTagForAlliance(alliance.get(), target.tagId())) {
            stop();
            return AimStatus.WRONG_ALLIANCE_OR_NON_HUB_TAG;
        }

        // Apply at most one correction per camera frame.
        if (target.timestampSeconds() <= lastAimFrameTimestamp + 1e-6) {
            return AimStatus.WAITING_FOR_NEW_FRAME;
        }
        lastAimFrameTimestamp = target.timestampSeconds();

        double tx = target.txDegrees();
        if (Math.abs(tx) < TurretConstants.AIM_DEADBAND_DEG) {
            stopMotorOutput();
            alignedFrameCount++;
            onTarget = alignedFrameCount >= TurretConstants.REQUIRED_ON_TARGET_FRAMES;
            return onTarget ? AimStatus.ALIGNED : AimStatus.CONFIRMING_ALIGNMENT;
        }

        onTarget = false;
        alignedFrameCount = 0;

        // compute new turret setpoint: add camera offset to current turret angle
        OptionalDouble currentAngle = getTurretAngle();
        if (currentAngle.isEmpty()) {
            lastBlockedPositionCommand = "auto aim: position unavailable";
            stop();
            return AimStatus.POSITION_UNAVAILABLE;
        }
        double correctionDegrees = MathUtil.clamp(
            -tx * TurretConstants.SAFE_KP,
            -TurretConstants.MAX_AIM_STEP_DEGREES,
            TurretConstants.MAX_AIM_STEP_DEGREES);
        double commandedAngle = currentAngle.getAsDouble() + correctionDegrees;
        PositionCommandStatus commandStatus = commandTurretAngle(commandedAngle);
        if (commandStatus == PositionCommandStatus.REJECTED) {
            onTarget = false;
        }
        return aimStatusForPositionCommand(commandStatus);
    }

    static AimStatus aimStatusForPositionCommand(PositionCommandStatus commandStatus) {
        return switch (commandStatus) {
            case REJECTED -> AimStatus.COMMAND_REJECTED;
            case MOVING -> AimStatus.COMMANDING_CORRECTION;
            case AT_TARGET -> AimStatus.CORRECTION_AT_TARGET;
        };
    }

    public enum AimStatus {
        UNREFERENCED,
        ALLIANCE_UNKNOWN,
        VISION_NOT_READY,
        NO_VALID_TARGET,
        WRONG_ALLIANCE_OR_NON_HUB_TAG,
        WAITING_FOR_NEW_FRAME,
        CONFIRMING_ALIGNMENT,
        ALIGNED,
        POSITION_UNAVAILABLE,
        COMMAND_REJECTED,
        COMMANDING_CORRECTION,
        CORRECTION_AT_TARGET
    }

    public boolean isOnTarget() {
        return onTarget && m_motor.isPositionReferenceValid(positionReference);
    }

    public boolean isPositionControlReadyForAutonomousAim() {
        return m_motor.isReady() && m_motor.isPositionReferenceValid(positionReference);
    }

    public boolean isVisionReadyForAutonomousAim() {
        return m_vision.isTargetingPipelineReady();
    }

    @Override
    public void periodic() {
        boolean motorConnected = m_motor.isAvailable();
        SmartDashboard.putBoolean("Turret motor connected", motorConnected);
        boolean referenced = m_motor.isPositionReferenceValid(positionReference);
        if (!motorConnected || (!referenced && !unhomedDiagnosticActive)) {
            m_motor.stop();
            onTarget = false;
        }

        processDashboardTuning();

        OptionalDouble turretAngle = getTurretAngle();
        OptionalDouble rawMotorRotations = m_motor.getPositionIfReady();
        SmartDashboard.putString(
            "Turret/Reference State", m_motor.getPositionReferenceStatus(positionReference));
        SmartDashboard.putNumber(
            "Turret/Continuity Epoch", m_motor.getPositionContinuityEpoch());
        SmartDashboard.putBoolean("Turret/Position Valid", turretAngle.isPresent());
        SmartDashboard.putNumber(
            "Turret/Raw Encoder Rotations", rawMotorRotations.orElse(Double.NaN));
        SmartDashboard.putNumber("Real turret angle", turretAngle.orElse(Double.NaN));
        SmartDashboard.putString(
            "Turret/Last Blocked Command", lastBlockedPositionCommand);

        SmartDashboard.putBoolean("On target", isOnTarget());
    }

    private static boolean unhomedDiagnosticAllowed(double requestedDuty) {
        return DriverStation.isTestEnabled()
            && !DriverStation.isFMSAttached()
            && Double.isFinite(requestedDuty)
            && Math.abs(requestedDuty)
                <= HardwareTestConstants.UNHOMED_DIAGNOSTIC_MAX_DUTY_CYCLE;
    }

    private void processDashboardTuning() {
        Decision decision = tuningApplyGate.poll(TUNING_APPLY_KEY);
        if (decision == Decision.NONE) {
            return;
        }
        if (decision != Decision.APPLY
                || !DriverStation.isDisabled()
                || DriverStation.isFMSAttached()) {
            publishActiveTuning();
            SmartDashboard.putString(TUNING_STATUS_KEY, "REJECTED_" + decision.name());
            return;
        }

        double requestedP = SmartDashboard.getNumber("Set turret_kP", turret_kP);
        double requestedI = SmartDashboard.getNumber("Set turret_kI", turret_kI);
        double requestedD = SmartDashboard.getNumber("Set turret_kD", turret_kD);
        if (!DashboardApplyGate.allFiniteInRange(
                new double[] {requestedP, requestedI, requestedD},
                new double[] {0.0, 0.0, 0.0},
                new double[] {MAX_PID_GAIN, MAX_PID_GAIN, MAX_PID_GAIN})) {
            publishActiveTuning();
            SmartDashboard.putString(TUNING_STATUS_KEY, "REJECTED_INVALID_OR_OUT_OF_RANGE");
            return;
        }

        if (Double.compare(turret_kP, requestedP) != 0
                || Double.compare(turret_kI, requestedI) != 0
                || Double.compare(turret_kD, requestedD) != 0) {
            turret_kP = requestedP;
            turret_kI = requestedI;
            turret_kD = requestedD;
            m_motor.assignPIDValues(turret_kP, turret_kI, turret_kD);
        }
        SmartDashboard.putString(TUNING_STATUS_KEY, "QUEUED_DISABLED");
    }

    private void publishActiveTuning() {
        SmartDashboard.putNumber("Set turret_kP", turret_kP);
        SmartDashboard.putNumber("Set turret_kI", turret_kI);
        SmartDashboard.putNumber("Set turret_kD", turret_kD);
    }
}
