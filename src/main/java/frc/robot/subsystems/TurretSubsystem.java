package frc.robot.subsystems;


import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.TurretConstants;
import frc.robot.constants.Constants.AprilTagConstants;
import frc.robot.subsystems.VisionSubsystem.TargetObservation;
import frc.robot.utils.SparkMAXContainer;

public class TurretSubsystem extends SubsystemBase {
    private final SparkMAXContainer m_motor = new SparkMAXContainer(TurretConstants.TURRET_CAN_ID);

    private final VisionSubsystem m_vision;

    private double turret_kP = 2.4;
    private double turret_kI = 0.0;
    private double turret_kD = 0.1;
    private boolean motorConfigured = false;
    private double lastAimFrameTimestamp = -1.0;
    private int alignedFrameCount = 0;

    public boolean onTarget = false;

    public TurretSubsystem(VisionSubsystem vision) {
        this.m_vision = vision;
        
        SmartDashboard.putNumber("Set turret_kP", 2.4);
        SmartDashboard.putNumber("Set turret_kI", 0);
        SmartDashboard.putNumber("Set turret_kD", 0.1);
    }

    public void setTurretAngle(double angleDegrees) {
        if (!m_motor.isAvailable()) return;
        // convert turret degrees -> motor rotations before commanding
        m_motor.goToPostion(degreesToMotorRotations(angleDegrees));
    }

    public void stop() {
        stopMotorOutput();
        onTarget = false;
        alignedFrameCount = 0;
    }

    private void stopMotorOutput() {
        if (m_motor.isAvailable()) {
            m_motor.motor.stopMotor();
        }
    }

    private double degreesToMotorRotations(double degrees) {
        // turret degrees -> turret rotations -> motor rotations
        // turret rotations = degrees / 360
        // motor rotations = turret rotations * GEAR_RATIO
        return (degrees / 360.0) * TurretConstants.GEAR_RATIO;
    }

    public double getTurretAngle() {
        if (!m_motor.isAvailable()) return 0;
        // motor rotations -> turret degrees
        double motorRotations = m_motor.getPosition();
        return motorRotationsToTurretDegrees(motorRotations);
    }

    private double motorRotationsToTurretDegrees(double motorRotations) {
        // turret rotations = motorRotations / GEAR_RATIO
        // degrees = turret rotations * 360
        return (motorRotations / TurretConstants.GEAR_RATIO) * 360.0;
    }
    
    public void autoAimWithLimelight() {
        if (!m_motor.isAvailable()) {
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
        double currentAngle = getTurretAngle();
        double correctionDegrees = MathUtil.clamp(
            -tx * TurretConstants.SAFE_KP,
            -TurretConstants.MAX_AIM_STEP_DEGREES,
            TurretConstants.MAX_AIM_STEP_DEGREES);
        double commandedAngle = currentAngle + correctionDegrees;

        // clamp to mechanical limits
        commandedAngle = MathUtil.clamp(commandedAngle, TurretConstants.MIN_ANGLE_DEGREES, TurretConstants.MAX_ANGLE_DEGREES);

        setTurretAngle(commandedAngle);
    }

    @Override
    public void periodic() {
        boolean motorConnected = m_motor.isAvailable();
        SmartDashboard.putBoolean("Turret motor connected", motorConnected);
        if (!motorConnected) {
            motorConfigured = false;
            onTarget = false;
            SmartDashboard.putBoolean("On target", false);
            return;
        }

        if (!motorConfigured) {
            m_motor.setBreakMode(true);
            m_motor.setCurrentLimit(15);
            m_motor.assignPIDValues(turret_kP, turret_kI, turret_kD);
            m_motor.setMaxSpeed(TurretConstants.MAX_CLOSED_LOOP_OUTPUT);
            motorConfigured = true;
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

        SmartDashboard.putNumber("Real turret angle", getTurretAngle());

        SmartDashboard.putBoolean("On target", onTarget);
    }
}
