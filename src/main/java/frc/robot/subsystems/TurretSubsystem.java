package frc.robot.subsystems;


import java.util.Arrays;
import java.util.OptionalDouble;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.TurretConstants;
import frc.robot.constants.Constants.AprilTagConstants;
import frc.robot.subsystems.VisionSubsystem.TargetObservation;
import frc.robot.utils.PositionReferenceGuard.Token;
import frc.robot.utils.SparkMAXContainer;

public class TurretSubsystem extends SubsystemBase {
    private final SparkMAXContainer m_motor = new SparkMAXContainer(TurretConstants.TURRET_CAN_ID);
    // Assigned only after future, sensor-validated homing or absolute reference succeeds.
    private Token positionReference;
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
    }

    public boolean setTurretAngle(double angleDegrees) {
        if (!Double.isFinite(angleDegrees)
            || angleDegrees < TurretConstants.MIN_ANGLE_DEGREES
            || angleDegrees > TurretConstants.MAX_ANGLE_DEGREES) {
            lastBlockedPositionCommand = "angle outside configured limits";
            stop();
            return false;
        }
        if (!m_motor.isPositionReferenceValid(positionReference)) {
            lastBlockedPositionCommand = "set angle: UNREFERENCED";
            stop();
            return false;
        }
        // convert turret degrees -> motor rotations before commanding
        boolean atTarget = m_motor.goToReferencedPosition(
            degreesToMotorRotations(angleDegrees),
            degreesToMotorRotations(TurretConstants.AIM_DEADBAND_DEG),
            positionReference);
        if (!atTarget) {
            lastBlockedPositionCommand = "set angle: not at target or command rejected";
        }
        return atTarget;
    }

    public void stop() {
        stopMotorOutput();
        onTarget = false;
        alignedFrameCount = 0;
    }

    private void stopMotorOutput() {
        m_motor.stop();
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
    
    public void autoAimWithLimelight() {
        if (!m_motor.isPositionReferenceValid(positionReference)) {
            lastBlockedPositionCommand = "auto aim: UNREFERENCED";
            stop();
            return;
        }

        var observation = m_vision.getLatestTargetObservation();
        var alliance = DriverStation.getAlliance();
        if (observation.isEmpty() || alliance.isEmpty()) {
            stop();
            return;
        }

        TargetObservation target = observation.get();
        int[] validTagIds = alliance.get() == Alliance.Red
            ? AprilTagConstants.VALID_RED_HUB_TAG_IDS
            : AprilTagConstants.VALID_BLUE_HUB_TAG_IDS;
        if (Arrays.stream(validTagIds).noneMatch(id -> id == target.tagId())) {
            stop();
            return;
        }

        // Apply at most one correction per camera frame.
        if (target.timestampSeconds() <= lastAimFrameTimestamp + 1e-6) {
            return;
        }
        lastAimFrameTimestamp = target.timestampSeconds();

        double tx = target.txDegrees();
        if (Math.abs(tx) < TurretConstants.AIM_DEADBAND_DEG) {
            stopMotorOutput();
            alignedFrameCount++;
            onTarget = alignedFrameCount >= TurretConstants.REQUIRED_ON_TARGET_FRAMES;
            return;
        }

        onTarget = false;
        alignedFrameCount = 0;

        // compute new turret setpoint: add camera offset to current turret angle
        OptionalDouble currentAngle = getTurretAngle();
        if (currentAngle.isEmpty()) {
            lastBlockedPositionCommand = "auto aim: position unavailable";
            stop();
            return;
        }
        double correctionDegrees = MathUtil.clamp(
            -tx * TurretConstants.SAFE_KP,
            -TurretConstants.MAX_AIM_STEP_DEGREES,
            TurretConstants.MAX_AIM_STEP_DEGREES);
        double commandedAngle = currentAngle.getAsDouble() + correctionDegrees;
        if (!setTurretAngle(commandedAngle)) {
            onTarget = false;
        }
    }

    public boolean isOnTarget() {
        return onTarget && m_motor.isPositionReferenceValid(positionReference);
    }

    @Override
    public void periodic() {
        boolean motorConnected = m_motor.isAvailable();
        SmartDashboard.putBoolean("Turret motor connected", motorConnected);
        boolean referenced = m_motor.isPositionReferenceValid(positionReference);
        if (!motorConnected || !referenced) {
            m_motor.stop();
            onTarget = false;
        }

        double requestedTurretKp = SmartDashboard.getNumber("Set turret_kP", 2.4);
        double requestedTurretKi = SmartDashboard.getNumber("Set turret_kI", 0);
        double requestedTurretKd = SmartDashboard.getNumber("Set turret_kD", 0.1);

        if (Double.compare(turret_kP, requestedTurretKp) != 0
                || Double.compare(turret_kI, requestedTurretKi) != 0
                || Double.compare(turret_kD, requestedTurretKd) != 0) {
            turret_kP = requestedTurretKp;
            turret_kI = requestedTurretKi;
            turret_kD = requestedTurretKd;
            m_motor.assignPIDValues(turret_kP, turret_kI, turret_kD);
        }

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
}
